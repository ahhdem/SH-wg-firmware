# Sailorwind Gateway — device setup & end-to-end flow

How a user sets up the device and links it to their sailorwind account, the
end-to-end data path, and an **honest readiness status** for shipping / e2e
testing. Revise freely — this tracks reality, not aspiration.

Status legend: ✅ built & verified · 🟡 partial/uncommitted · ⛔ not built

---

## 1. What the device does (one paragraph)

The Sailorwind Gateway plugs into the boat's NMEA 2000 bus, reads weather
(wind, pressure, air/water temp, humidity) and position directly off the wire,
aggregates ~10 minutes of data into one rich observation (mean/gust/std/gust-
ladder/histogram + true wind direction), and POSTs it to sailorwind.net over
HTTPS, tagged `source=auto_swgw`. It keeps doing the gateway's normal job
(NMEA0183/SignalK over WiFi) at the same time — the submitter is additive.

---

## 2. User setup — two flows

### Flow A — Manual token (the near-term / first-e2e path)

Uses only endpoints that **already ship**. This is how we'll do the first real
end-to-end test.

1. **Get a token.** Sign in at sailorwind.net → Settings (gear) → "Create
   device token" → copy the `slw_dev_…` value (shown once).
2. **Flash + power the device** on the N2K bus (or bench with an N2K source).
3. **Join the device's WiFi config portal** (SensESP captive portal) → set your
   boat WiFi credentials.
4. **Open the device config page** → Sailorwind section → paste the token, tick
   "Enable sailorwind submission", choose fields (wind speed + direction on by
   default), optionally "publish precise position".
5. Done. Within ~1 minute the device sends its first observation; it appears on
   the map under your account.

What this skips: no per-device account linking — the token *is* the link
(it's already tied to your user). Good enough to validate the whole data path.

### Flow B — Self-register + claim (the target, plug-n-play)

Zero typing of secrets. This is the shipping UX. **Server side built** (2026-06-18);
the firmware provisioning client + module wiring is the remaining half.

1. **Flash + power** the device. On first boot it generates a secret and
   **self-registers** with sailorwind.net, getting a provisional token; it
   starts submitting immediately as an *unclaimed* device (data is still used).
2. The device's config page shows a short **claim code** (e.g. `K7QP-2M9X`).
3. **Claim it:** signed in at sailorwind.net/claim, enter the code (or follow
   the link on the config page). The device is now bound to your account; its
   observations are attributed to you and your settings flow down.
4. The device never needs a hand-pasted token.

Proof-of-possession = you can see the claim code on the device's own config
page (you're physically/network present). Full design:
`sailorwind` repo `docs/device-registration-api.md`.

---

## 3. End-to-end data path (after setup)

```
N2K bus ──130306/129025/130311…──▶ SwN2kTap ──▶ SwAggregator (10-min window)
                                                      │ flush
                                          wind-dir resolver + rich stats
                                                      ▼
                                          SwObservation ──▶ sw_json (ArduinoJson)
                                                      ▼
   WiFi/TLS ◀── SwSubmitter (Bearer token, Idempotency-Key, retry queue) ──▶
        POST https://sailorwind.net/v1/observations  (source=auto_swgw)
                                                      ▼
                              server QC + coarsening ──▶ public map
```

Clock: set from N2K PGN 126992, **plus an SNTP fallback** (a bench without a GPS
has no 126992, and TLS needs a valid clock). No valid clock → device holds off
(no observation has a trustworthy time).

---

## 4. Readiness status (the honest part)

### Firmware
- ✅ Data plane compiles against the real toolchain: aggregator, wind-dir
  resolver, JSON builder, TLS submitter, N2K tap. Pure logic covered by a
  native host test.
- ✅ Config UI nodes (`sw_config.cpp`) + full firmware builds green (~69.5% flash).
- ⛔ **Module not wired into `main.cpp`** — there is no `sailorwind::init`, no
  `-D SW_SAILORWIND` block. **Flashed today, the device does nothing sailorwind.**
  This is the single gating item for any e2e test.
- ⛔ SNTP fallback not added yet.
- ⛔ `sw_provision.cpp` (self-register/claim/check-in) — interface only, no impl.
- ⛔ `sw_selfupdate.cpp` (OTA) — interface only; not needed for first release.
- 🟡 Queue store is RAM-only (no persistence across reboot); LittleFS = follow-up.
- ℹ️ SH-ESP32 test board needs `config.h` CAN pins → TX `GPIO_NUM_32`, RX `GPIO_NUM_34`.

### Server / API
- ✅ `auto_swgw` source accepted by `POST /v1/observations` (shipped to main).
- ✅ Manual device-token mint (`POST /v1/me/tokens`) + device-token auth — the
  Flow A path. Already in production (SignalK plugin uses it).
- ✅ **Self-register/claim/check-in endpoints BUILT** (2026-06-18):
  `POST /v1/devices/register`, `POST /v1/devices/claim`, `GET /v1/devices/me`
  (`apps/api/src/routes/devices.ts`). 12 integration tests pass; full API suite
  (133) green. Contract: sailorwind `docs/device-registration-api.md`.
- ✅ `session.ts` userless-device context, `observations.ts` unclaimed→coarse +
  device-keyed rate-limit + `device_id` attribution + Turnstile exemption,
  `rate-limit.ts` policies (`deviceRegister`/`deviceClaim`/`unclaimedDevice`).
- ⛔ Firmware-update manifest endpoint — not built (not needed for first release).

### So: is the API done / are we ready to e2e test?
- **All API work done?** No. The `auto_swgw` + manual-token path is shipped;
  the registration/claim/manifest endpoints are not.
- **Ready to e2e test now?** No — the **firmware isn't wired**. But the gate is
  small and **server-free**: once the module is wired into `main.cpp` (+ SNTP),
  **Flow A** works end-to-end against the already-shipped endpoints.

---

## 5. Minimum path to a first end-to-end test

In order; nothing here needs new server work:

1. **Wire the module** — `sailorwind::init(app, nmea2000)` + a `-D SW_SAILORWIND`
   gated block in `main.cpp`: construct config/aggregator/tap/store/submitter,
   `AttachMsgHandler` the tap, `ExtendReceiveMessages` the env PGNs, start the
   10-min submit timer, kick off SNTP.
2. **Build green** (`pio run`) and confirm flash headroom.
3. **Set `config.h` CAN pins** for the SH-ESP32 (TX 32 / RX 34).
4. **Flash** the SH-ESP32; join its config portal; set WiFi.
5. **Mint a token** in the sailorwind web app; paste into the device config.
6. **Feed it N2K** (real bus, or a simulator/recording on the bench) and watch
   for the observation on the map (and in the device's serial log).

After Flow A is proven, build **Flow B** (registration endpoints + provisioning)
for the plug-n-play shipping experience.
