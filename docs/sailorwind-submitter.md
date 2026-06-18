# Sailorwind Submitter — firmware design

Turning the SH-wg (rebranded **Sailorwind Gateway**) from a generic NMEA 2000 →
WiFi gateway into a device that *also* reads weather off the N2K bus and submits
observations directly to **sailorwind.net** — no phone, no laptop, no SignalK
server in the loop.

Status: **design / not yet implemented.** This doc is the canonical shape; the
Pad PLAN tracks the work. Authored 2026-06-16.

---

## 1. Principles

1. **Additive, not a fork.** The gateway stays exactly as-is (TCP/UDP servers,
   NMEA0183/YDWG/SeaSmart translation, Hat Labs OTA). The submitter is a new
   module gated behind one config checkbox — `translate_to_seasmart` is the
   pattern to copy. One firmware, two modes: pure gateway, or gateway **+**
   submitter.
2. **Reuse what the binary already has.** SensESP gives WiFi provisioning + a
   web config UI + persistent config. The NMEA2000-library already decodes the
   nav/wind PGNs we need into SI floats. `ota_update_task.cpp` already proves
   `WiFiClientSecure` + `setCACert(ISRG_Root_X1)` HTTPS on this hardware — and
   ISRG Root X1 is exactly the Let's Encrypt root sailorwind.net serves under,
   so the same pinned CA works unchanged.
3. **Feature parity with the SignalK plugin, without SignalK.** The reference
   implementation is the `sailorwind-integration-core` TS package
   (`clients/shared/src/` in the sailorwind repo: `aggregator.ts`,
   `wind-direction.ts`, `submitter.ts`, `units.ts`, `types.ts`). Field names and
   wire shape MUST match `POST /v1/observations` exactly — drift = 400s.
4. **Never fabricate wind direction.** If a true bearing can't be resolved for a
   window, submit nothing for that window. Never borrow a forecast direction —
   that manufactures false obs-vs-forecast agreement, which is the exact signal
   sailorwind exists to measure.
5. **Unattended-device discipline.** These ship and may never be touched again.
   Self-update must be safe (hash-verified + auto-rollback), and device
   provisioning must need zero per-unit factory steps.

## 2. Module layout

```
src/sailorwind/
  sw_config.h/.cpp        SensESP config nodes (token, server, field toggles, precise, channel)
  sw_n2k_tap.h/.cpp       single N2K message handler → feeds the aggregator
  sw_aggregator.h/.cpp    streaming accumulators (port of aggregator.ts)
  sw_wind_direction.h/.cpp 130306-reference resolver (port of wind-direction.ts)
  sw_submitter.h/.cpp     HTTPS POST + persistent ring queue (port of submitter.ts)
  sw_json.h               ArduinoJson body builder (matches ObservationInput)
  sw_provision.h/.cpp     self-registration + claim-code (section 6)
  sw_selfupdate.h/.cpp    manifest check + hashed OTA + rollback (section 7)
  sailorwind.h            init(app, nmea2000) entry, called from main.cpp behind one checkbox
```

`main.cpp` gains one gated block, next to the existing translate toggles:

```cpp
if (checkbox_config_enable_sailorwind->get_value()) {
  sailorwind::init(sensesp_app, nmea2000);
}
```

## 3. Data tap + PGNs

Register a **sibling** N2K message handler alongside `n2k_nmea0183_transform`
(the NMEA2000-library supports multiple handlers) — don't edit the transform.
The handler parses each PGN with the library's `ParseN2k*` helpers (all return
SI) and pushes timestamped samples into the aggregator.

| Need | PGN | Parser | Already received? |
|---|---|---|---|
| position | 129025 | `ParseN2kPGN129025` | ✅ in `ReceiveMessages` |
| wind speed + angle + reference | 130306 | `ParseN2kWindSpeed` | ✅ |
| heading (true/mag) + variation | 127250 | `ParseN2kHeading` | ✅ |
| magnetic variation | 127258 | `ParseN2kMagneticVariation` | ✅ |
| COG/SOG (apparent-wind solve, v2) | 129026 | `ParseN2kCOGSOGRapid` | ✅ |
| **pressure** | 130314 / 130311 | `ParseN2kPGN130314` / `…130311` | ➕ add |
| **air temp** | 130312 / 130311 / 130316 | `ParseN2kPGN130312` / `…316` | ➕ add |
| **water temp** | 130316 / 130311 | `ParseN2kPGN130316` | ➕ add |
| **humidity** | 130313 / 130311 | `ParseN2kPGN130313` | ➕ add |

The four env PGNs need adding to `ReceiveMessages[]` + a parse case each. Note
130311 (Environmental) is a combined frame carrying temp+humidity+pressure with
source enums — decode the source instances to route the right value.

Unit conversions match `units.ts`: N2K wind is already m/s; pressure Pa → hPa
(÷100); temp K → °C (−273.15); humidity is a 0..1 ratio → % (×100). Angles are
radians → degrees, wrapped to [0,360).

## 4. Streaming aggregator (memory-bounded)

The TS aggregator retains every sample for the 10-minute window. On a 520 KB
SRAM part with TLS + gateway buffers resident we don't — everything except the
sustained-gust ladder is computed incrementally (O(1) state, no retention):

```cpp
struct ScalarAccum {        // pressure / airTemp / waterTemp / humidity
  float    latest = NAN;    // we submit "latest in window"
  uint32_t latest_t = 0;
  bool has() const { return !isnan(latest); }
};

struct WindAccum {
  // running mean+variance (Welford) — no sample retention
  uint32_t n = 0;
  double   mean = 0, m2 = 0;       // population var = m2/n; std = sqrt(var)
  float    peak = 0;               // windGustMs (instantaneous max)
  uint16_t hist[40] = {0};         // windSpeedHistogram; bin i=[i,i+1), 39=[39,∞)

  // sustained-gust ladder = the ONLY retained samples: a ~60 s ring
  struct S { float v; uint32_t t; };
  S        ring[768];              // 60 s @ ~12 Hz ≈ 5 KB
  uint16_t head = 0, count = 0;
};
```

- **mean/std/histogram/peak** → streaming, ~80 bytes regardless of window length.
- **`windGust10s/30s/60sMs`** → `peakSustained()` (port verbatim) run over the
  60 s ring at flush.
- **`windGustCrossings`** (upward crossings of `mean+1σ`) → the threshold isn't
  known until the window closes. v1 computes it over the 60 s ring only; this
  diverges slightly from the full-window TS semantics — **document the
  divergence, don't hide it.**

Steady-state submitter footprint **< 10 KB**; transient ~45 KB during the TLS
handshake (already proven by OTA). This is the single assumption to validate on
real hardware before building the rest.

`observedAt` = latest sample timestamp across the measured-weather streams (not
flush wall-clock — submissions can fire late), exactly as `aggregator.ts`
documents. Time source: N2K (129029 GNSS / system-time PGN — the firmware
already tracks `elapsed_since_last_system_time_update`) or SNTP. A valid clock
is also required for TLS cert validation, so gate the first POST + the
self-update check on the clock being set.

## 5. Wind-direction resolver (the crux)

N2K **PGN 130306** carries a `WindReference` enum. The resolver is a priority
cascade evaluated at flush; it ports `wind-direction.ts` onto the N2K source:

| ref | maps to SignalK path | → compass FROM (true) | v1 |
|---|---|---|---|
| `0` North/True (ground) | `directionTrue` | `wrap(angle)` — used directly as FROM | ✅ direct |
| `1` Magnetic (ground) | `directionMagnetic` | `wrap(angle + variation)` | ✅ if variation |
| `3` True (boat ref) | `angleTrueWater` | `wrap(headingTrue + angle)` | ✅ if heading |
| `4` True (water ref) | `angleTrueWater` | `wrap(headingTrue + angle)` | ✅ if heading |
| `2` **Apparent** | (apparent) | needs SOG/COG vector solve | ⛔ v1 skip → **fast-follow** |

> **Convention:** mirror the SignalK `@signalk/n2k-signalk` mapping exactly —
> ref 0 is used as a FROM bearing directly (no `+180`), and the obs already in
> the sailorwind DB follow that convention. The exact FROM-vs-TO sense per
> reference MUST be verified against canboat / n2k-signalk during TASK-158
> before trusting output; do not assume.

```
resolve(window):
  prefer ref 0 → 1 → 3/4  (most-trusted first; mirrors directionTrue-first)
  variation: 127258 explicit → 127250's variation field → WMM(lat,lon,date)  [WMM deferred to v2]
  heading:   headingTrue → headingMagnetic + variation
  if a usable (angle, ref, +context) combo exists:
     vector-mean the per-sample FROM bearings: atan2(Σsin, Σcos)  → windFromDeg
     record method string (debug/telemetry only — never wired to the barb)
  else: leave windFromDeg unset → window is SKIPPED (no fabrication)
```

`vectorMeanDegFromRadians` + `wrapDeg` port verbatim from `units.ts`.

**v1 scope:** refs 0/1/3/4, requiring the boat to publish variation or magnetic
heading (most N2K compasses do). **Apparent wind (ref 2) is a deliberate
fast-follow, not a someday** — the vector solve combines apparent (angle,speed)
with boat motion (SOG/COG from 129026, already parsed) to recover true wind.
The resolver and the obs schema should leave an obvious seam for it (a
`solveApparent()` branch + the SOG/COG samples already buffered) so adding it is
a localized change, not a re-architecture. The 325-line WMM variation model
(`wmm.ts`) is the genuinely-deferrable v2 piece.

## 6. Provisioning — self-register + claim, zero factory steps

**Constraint (Adam):** no factory-flashed tokens, no per-unit sticker/secret, no
unique packaging step. The device must self-register and a user must be able to
claim it using only what's already in the box.

**The proof-of-possession is the device's own config web UI.** The user already
opens it to set WiFi (SensESP captive portal / LAN page); we put the claim
affordance right there. Being able to see the config page *is* the proof you
physically possess the device — no shared secret needed.

**Identity:** the ESP32 MAC (unique, burned in, already used as `?mac=` by Hat
Labs OTA) + a random `deviceSecret` the device generates on first boot and keeps
in NVS. The secret authenticates subsequent calls so the device never
re-registers; it is never displayed and never leaves the device except as a
bearer on its own calls.

**Phase 1 — self-registration (device, first boot, WiFi up):**

```
POST /v1/devices/register
  { hardwareId: <mac>, deviceSecret: <random, NVS>, model: "shwg", fw: <hexver> }
→ 200 { deviceId, deviceToken, claimCode }     # deviceToken is provisional (unclaimed)
```

The device stores `deviceId` + `deviceToken` in NVS and **can begin submitting
immediately as an unclaimed device** — the obs are still valuable; they're just
not yet attributed to a user account. Re-registration is idempotent on
`(hardwareId, deviceSecret)`.

**Phase 2 — claim (user, in the sailorwind web app):** the device config page
shows a short `claimCode` (e.g. 6–8 chars, generated by the device, refreshed
each boot) plus a deep link `https://sailorwind.net/claim?code=…`. The
signed-in user enters/opens it; the server binds `deviceId → userId`. Now obs
are attributed and per-user settings (publishPrecise, vessel name) flow down on
the device's next check-in.

**Unclaimed obs are used, not withheld** (decision 2026-06-18): a device shipping
data is valuable even if its owner never claims it — served coarse (no user → not
precise) + provenance-flagged. **Abuse controls (server):** rate-limit `register`
by IP; throttle unclaimed-device submits with a dedicated (non-punitive) policy;
claim binds to a real authenticated account.

This needs three sailorwind-side routes (`/v1/devices/register`,
`/v1/devices/claim`, and a device-settings pull `GET /v1/devices/me`) plus
device-token auth on `POST /v1/observations` (the token path already exists for
the SignalK plugin — reuse it). **Full server contract:**
`sailorwind` repo `docs/device-registration-api.md`. Key decision there: the
device mints its bearer token *once* at registration with `user_id NULL` and
submits immediately as unclaimed; claiming just *attaches* a user to that same
token row (no provisional→full rotation, token never transits the browser).

## 7. Self-update — manifest + hash + rollback

**Not required to be functional in the first release, but the code shape must
accommodate it from day one** (unattended fleet → updates have to be safe and
hands-off). Evolve `ota_update_task.cpp` rather than add a parallel task. Three
upgrades over the Hat Labs scheme (which GETs a bare hex version and builds
`firmware_<ver>.bin`):

**(a) A JSON manifest** so the server owns naming, channels, and integrity:

```
GET https://sailorwind.net/v1/firmware/manifest?hw=shwg&channel=stable&ver=<hexver>&mac=<mac>
→ 200 { versionCode, version, url, sha256, size, mandatory }   # server returns the full URL
→ 204                                                          # already current
```

The server returning `url` is the "assemble an update URL from filename + hash"
ask: the device hard-codes only the manifest path; filenames, channels
(`stable`/`beta`), and CDN layout live server-side forever. `versionCode` keeps
Hat Labs' packed-hex scheme (`kFirmwareHexVersion`) to avoid churning their
bump-version tooling.

**(b) SHA-256 verification beyond TLS.** TLS authenticates the *connection*; the
hash authenticates the *artifact* — catching a truncated download or a swapped
CDN object, and letting the device skip re-downloading a version it already has.
The Arduino `Update` lib only does MD5, so stream the OTA bytes through
`mbedtls_sha256` (mbedTLS already linked for TLS) during the write and refuse to
commit unless the digest matches the manifest. No new dependency.

**(c) Rollback — non-negotiable for unattended units.** Use ESP32 dual-OTA +
bootloader rollback (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`): write to the
inactive partition, set pending, reboot, and **only call
`esp_ota_mark_app_valid_cancel_rollback()` after the submitter successfully
reaches sailorwind.net once** post-update. If the new image panics or can't get
online, the bootloader auto-reverts on the next reset. The current
`min_spiffs.csv` already has dual 1.96 MB `app0/app1` slots, so the layout is
there; rollback just needs the sdkconfig flag.

**Cadence:** check on boot (after WiFi + clock valid) **and** on a long periodic
timer (reuse `kDelayBetweenFirmwareUpdateChecksMs`) — an always-on boat would
never update on a boot-only policy. Applying drops the gateway for the ~15 s
reboot; add a config to choose download-and-stage vs. download-and-reboot so a
user can opt into "notify, don't auto-reboot."

**Server side:** a `GET /v1/firmware/manifest` route, a static `/fw/*.bin` dir
(Caddy serves it), and a release step that records `{versionCode, sha256, size}`
per build. The `mac=` param enables staged rollouts (hash the MAC → percentage
gate) almost for free.

## 8. Config surface (SensESP web UI — free)

Same `CheckboxConfig` / `StringConfig` / `UIOutput` machinery already in
`main.cpp`:

- `enable_sailorwind` (checkbox) — the master gate
- `server_url` (string, default `https://sailorwind.net`)
- device token (password widget) — auto-filled after claim; manual override allowed
- field toggles: windSpeed / windDirection / pressure / airTemp / waterTemp / humidity
- `publishPrecise` (checkbox, default off)
- update channel (`stable` / `beta`) + auto-apply mode (stage vs. reboot)
- read-only status: claim code, claim state, last submit, queue depth

## 9. Submitter + queue

Direct port of `submitter.ts`:

- `POST {server}/v1/observations`, `Authorization: Bearer <deviceToken>`,
  `Idempotency-Key: <uuidv4 from esp_random()>`, body from ArduinoJson matching
  `ObservationInput` (`types.ts`).
- Reuse OTA's `WiFiClientSecure` + `setCACert(ISRG_Root_X1)`.
- Queue persisted to NVS or a SPIFFS file, bounded drop-oldest at 100 entries
  (~16 h buffer at the 10-min cadence). 2xx → drop+success; 408/429/5xx →
  retain+retry; other 4xx → drop (permanent). Persist on every enqueue and after
  any drain that changed the queue.

## 10. Build / verify

- New module behind a `-D SW_SAILORWIND` build flag *and* the runtime checkbox,
  so a pure-gateway build is still possible.
- Port the `aggregator.test.ts` / `wind-direction.test.ts` fixtures to native
  unit tests (PlatformIO `test/`) so the two data planes provably agree.
- **De-risk first:** a throwaway branch that just stands up the N2K tap + a
  stub submitter to measure heap during a real TLS POST with the gateway
  running. That single measurement validates the whole approach.
- **Hardware caveat for the spike (Adam, 2026-06-18):** the unit on hand for
  testing is *similar to but not the exact* board we'll manufacture (both
  ESP32-WROOM-32, but PSRAM/flash/board revision may differ). So read the spike
  asymmetrically: a **red** result (heap exhaustion) is definitive and blocks;
  a **green** result is a strong positive signal but NOT a guarantee for the
  production board — re-confirm on a real production unit before shipping, and
  size the design for comfortable margin rather than a thin pass.

## 11. Open items / deferred

- **WMM variation model** (`wmm.ts`, 325 lines) — v2; v1 requires published
  variation/magnetic.
- **Apparent-wind true solve** (ref 2 + SOG/COG) — fast-follow, seam left in §5.
- **Token rotation policy** at claim time — provisional → full.
- **Multi-source arbitration** (two wind sensors on one bus) — the SK server
  does this for the plugin; on-device we'd pick by source instance/priority.
  v1: take all and vector-mean; revisit if it causes trouble.
```
