#include "sw_app.h"

#include <Arduino.h>
#include <WiFi.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "N2kMessages.h"  // tN2kMsg

#include "sw_aggregator.h"
#include "sw_ca_cert.h"
#include "sw_config.h"
#include "sw_json.h"
#include "sw_mem_queue_store.h"
#include "sw_n2k_tap.h"
#include "sw_provision.h"
#include "sw_submitter.h"

namespace sailorwind {

namespace {

// ── Module state (single instance; the device runs one submitter) ────────────
SwConfig g_config;
SwAggregator g_agg;
SwMemQueueStore<16> g_store;  // RAM ring (bring-up); LittleFS persistence = TODO
SwN2kTap* g_tap = nullptr;
SwProvisioner* g_prov = nullptr;
SemaphoreHandle_t g_agg_mtx = nullptr;

constexpr uint32_t kFlushIntervalMs = 10 * 60 * 1000;     // 10-min observation window
// Check-in (claim state + settings) poll. Fast while UNCLAIMED so a fresh claim
// shows within ~20 s; slow (15 min) once claimed — it's just a passive backstop
// then, because an enable toggle forces an immediate re-sync (see SailorwindTask).
constexpr uint32_t kCheckInUnclaimedMs = 20 * 1000;       // 20 s
constexpr uint32_t kCheckInClaimedMs = 15 * 60 * 1000;    // 15 min
constexpr TickType_t kTaskTick = pdMS_TO_TICKS(1000);

// UTC epoch milliseconds, or 0 when the clock isn't trustworthy yet. The system
// clock is set by N2K PGN 126992 (main.cpp SetSystemTime) and/or SNTP; before
// either lands, tv_sec sits near 0. Gate on ~2020-09-13 so we never timestamp an
// observation — or start a TLS session — without a real clock.
int64_t SwEpochMs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1600000000L) return 0;
  return static_cast<int64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
}

// Build submitter deps from current config. The String args must outlive the
// SwSubmitter use (deps hold const char* into them) — callers keep them on the
// stack across the Enqueue/Drain call.
SwSubmitterDeps MakeDeps(const String& server, const String& token) {
  SwSubmitterDeps deps;
  deps.server_url = server.c_str();
  deps.api_token = token.c_str();
  deps.ca_cert_pem = kSwIsrgRootX1Pem;
  deps.store = &g_store;
  return deps;
}

void DrainQueue() {
  String server = g_config.serverUrl();
  String token = g_config.apiToken();
  SwSubmitter sub(MakeDeps(server, token));
  sub.Drain();
}

// Close the current 10-min window: flush+reset the aggregator (under lock),
// then enqueue iff the window yielded a COMPLETE wind obs (speed AND direction —
// the server requires both; we never fabricate a direction). Returns false when
// the window is skipped.
bool FlushAndEnqueue() {
  SwObservation obs;
  bool got = false;
  if (g_agg_mtx && xSemaphoreTake(g_agg_mtx, portMAX_DELAY) == pdTRUE) {
    got = g_agg.Flush(&obs);
    g_agg.Reset();
    xSemaphoreGive(g_agg_mtx);
  }
  if (!got) return false;

  const SwFieldMask mask = g_config.fieldMask();
  const bool wind_ok = mask.has(SwField::WindSpeed) &&
                       mask.has(SwField::WindDirection) &&
                       obs.has(obs.wind_speed_ms) && obs.has(obs.wind_from_deg);
  if (!wind_ok) return false;

  char body[kMaxBodyBytes];
  const size_t n = BuildObservationJson(obs, mask, g_config.publishPrecise(),
                                        body, sizeof(body));
  if (n == 0) return false;

  String server = g_config.serverUrl();
  String token = g_config.apiToken();
  SwSubmitter sub(MakeDeps(server, token));
  return sub.Enqueue(body);
}

void DiscardWindow() {
  if (g_agg_mtx && xSemaphoreTake(g_agg_mtx, portMAX_DELAY) == pdTRUE) {
    g_agg.Reset();
    xSemaphoreGive(g_agg_mtx);
  }
}

// Dedicated task: all blocking network I/O lives here, off the main loop.
void SailorwindTask(void*) {
  bool registered = false;
  bool prev_enabled = false;
  uint32_t last_flush_ms = 0;
  uint32_t last_checkin_ms = 0;

  for (;;) {
    vTaskDelay(kTaskTick);
    const bool enabled = g_config.enabled();
    // A deliberate enable toggle (off -> on) is the natural "re-sync now"
    // gesture: force the next check-in so a claim/unclaim/settings change made
    // in the web app reflects immediately, instead of waiting for the periodic
    // poll. (Setting last_checkin_ms = 0 makes the interval check below fire.)
    if (enabled && !prev_enabled) last_checkin_ms = 0;
    prev_enabled = enabled;
    if (!enabled) continue;
    // TLS needs a real clock + a network; wait for both.
    if (!WiFi.isConnected() || SwEpochMs() == 0) continue;

    if (!registered) {
      if (!g_prov->EnsureRegistered()) continue;  // retry next tick
      registered = true;
      g_prov->CheckIn();
      last_checkin_ms = millis();
      // Start the first observation window clean from the moment we're live.
      DiscardWindow();
      last_flush_ms = millis();
    }

    const uint32_t now = millis();
    const uint32_t checkin_interval =
        (g_prov->claimState() == ClaimState::Claimed) ? kCheckInClaimedMs
                                                      : kCheckInUnclaimedMs;
    if (now - last_checkin_ms >= checkin_interval) {
      g_prov->CheckIn();
      last_checkin_ms = now;
    }
    if (now - last_flush_ms >= kFlushIntervalMs) {
      FlushAndEnqueue();
      last_flush_ms = now;
    }
    DrainQueue();
  }
}

}  // namespace

void SailorwindBegin() {
  g_config.Begin();
  g_agg_mtx = xSemaphoreCreateMutex();
  g_tap = new SwN2kTap(&g_agg, &SwEpochMs);
  g_prov = new SwProvisioner(&g_config, kSwIsrgRootX1Pem, &SwEpochMs);
  g_prov->Begin();
  // SNTP fallback so a bench unit with no GPS (hence no N2K 126992) still gets a
  // clock for TLS. Harmless next to the N2K system-time path — whichever lands
  // first sets the clock.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  // Generous stack: a WiFiClientSecure TLS handshake is stack-hungry (mbedtls).
  xTaskCreate(SailorwindTask, "SailorwindTask", 12288, nullptr, 1, nullptr);
}

String ClaimStatusForUi() {
  if (!g_config.enabled())
    return "Disabled - tick 'Enable sailorwind submission' below";
  if (!WiFi.isConnected()) return "Waiting for WiFi...";
  if (SwEpochMs() == 0) return "Waiting for clock (GPS time or SNTP)...";
  if (!g_prov) return "Starting...";
  switch (g_prov->claimState()) {
    case ClaimState::Claimed:
      return "Claimed - submitting as your account";
    case ClaimState::Unclaimed:
      return "Registered, UNCLAIMED - enter the claim code at sailorwind.net/claim";
    default:
      return "Registering...";
  }
}

String ClaimCodeForUi() {
  if (!g_prov || g_prov->claimState() != ClaimState::Unclaimed) return "";
  return g_prov->claimCode();
}

// Stable state tokens for the app's provisioning logic (NOT user-facing prose;
// ClaimStatusForUi() carries the human copy). Order mirrors the lifecycle.
static const char* LifecycleStateToken() {
  if (!g_config.enabled()) return "DISABLED";
  if (!WiFi.isConnected()) return "NO_WIFI";
  if (SwEpochMs() == 0) return "CONNECTING";
  if (!g_prov) return "REGISTERING";
  switch (g_prov->claimState()) {
    case ClaimState::Claimed:
      return "CLAIMED";
    case ClaimState::Unclaimed:
      return "REGISTERED_UNCLAIMED";
    default:
      return "REGISTERING";
  }
}

String MachineStatusJson() {
  const bool claimed = g_prov && g_prov->claimState() == ClaimState::Claimed;
  const String code =
      (g_prov && g_prov->claimState() == ClaimState::Unclaimed)
          ? g_prov->claimCode()
          : String("");
  const String deviceId = g_prov ? g_prov->deviceId() : String("");

  // Values are controlled (state tokens, XXXX-XXXX claim code, UUID deviceId) —
  // no characters that need JSON escaping — so build the object by hand and keep
  // ArduinoJson out of this translation unit.
  String j = "{\"state\":\"";
  j += LifecycleStateToken();
  j += "\",\"claimed\":";
  j += claimed ? "true" : "false";
  j += ",\"claimCode\":";
  if (code.length()) {
    j += "\"";
    j += code;
    j += "\"";
  } else {
    j += "null";
  }
  j += ",\"deviceId\":";
  if (deviceId.length()) {
    j += "\"";
    j += deviceId;
    j += "\"";
  } else {
    j += "null";
  }
  j += "}";
  return j;
}

void FeedN2k(const tN2kMsg& msg) {
  if (!g_tap || !g_agg_mtx) return;
  // Short timeout, NOT portMAX_DELAY: the task holds this lock only for a
  // microsecond-scale flush/reset every 10 min, so 2 ms is ample; if it ever
  // can't acquire, drop this one sample rather than stall the N2K pump.
  if (xSemaphoreTake(g_agg_mtx, pdMS_TO_TICKS(2)) == pdTRUE) {
    g_tap->HandleMsg(msg);
    xSemaphoreGive(g_agg_mtx);
  }
}

}  // namespace sailorwind
