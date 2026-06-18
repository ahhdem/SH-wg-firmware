#pragma once
//
// sw_provision.h — device side of the self-register + claim flow.
//
// Implements the device half of the contract in the sailorwind repo
// docs/device-registration-api.md. Zero factory steps: on first boot the device
// generates a secret, self-registers, and starts submitting immediately as an
// unclaimed device; a user later claims it via a short code shown on the config
// UI. The bearer token is minted once at registration and never rotates — the
// claim just attaches a user to it server-side.
//
// HTTP uses the same proven path as ota_update_task.cpp: WiFiClientSecure +
// setCACert(ISRG Root X1) + HTTPClient, with ArduinoJson bodies. Compiles on
// the ESP32 target only. The .cpp is the on-hardware compile-loop work.
//
// Persistent state (NVS, e.g. Preferences namespace "sw_prov"):
//   device_id   — server-assigned UUID (returned by /register)
//   secret      — device-generated, base64url; proves hardware continuity on
//                 re-register after an NVS wipe / re-flash (recovery + anti-spoof)
//   (the bearer token lives in SwConfig /sailorwind/token so it's UI-visible)

#include <Arduino.h>

#include "sw_clock.h"
#include "sw_config.h"

namespace sailorwind {

enum class ClaimState : uint8_t { Unknown, Unclaimed, Claimed };

class SwProvisioner {
 public:
  SwProvisioner(SwConfig* config, const char* ca_cert_pem, EpochClockMs clock)
      : config_(config), ca_cert_pem_(ca_cert_pem), clock_(clock) {}

  // Load NVS state (device_id, secret); generate the secret on first ever boot.
  // Does NOT do network I/O — call EnsureRegistered() once WiFi + clock are up.
  void Begin();

  // Register if we don't yet hold a device_id/token (first boot or NVS wipe):
  //   POST {server}/v1/devices/register {hardwareId(MAC), model, secret, fw}
  //   → { deviceId, token, claimCode, claimUrl, ... }
  // Stores device_id (NVS) + token (SwConfig::setApiToken) + claimCode (RAM).
  // Idempotent server-side on (hardwareId, secret), so a wiped unit recovers
  // its claim. Safe to call when already registered (no-op). Returns true if
  // the device holds a valid token afterward.
  bool EnsureRegistered();

  // Periodic check-in: GET {server}/v1/devices/me (Bearer token).
  //   → { claimed, claimCode, user, vessel, settings:{publishPrecise} }
  // Updates claim_state_ + claim_code_ and reflects pulled settings. Call on
  // boot (after EnsureRegistered) and on a long timer. No-op without a token.
  void CheckIn();

  ClaimState claimState() const { return claim_state_; }
  // Shown on the config UI while unclaimed so the user can claim at /claim?code=.
  const String& claimCode() const { return claim_code_; }

 private:
  bool Register();   // POST /v1/devices/register
  bool GenerateSecretIfMissing();  // esp_random → base64url, persist to NVS

  SwConfig* config_;
  const char* ca_cert_pem_;
  EpochClockMs clock_;

  String device_id_;   // NVS
  String secret_;      // NVS (raw; server stores only its hash)
  String claim_code_;  // RAM (from /register or /me while unclaimed)
  ClaimState claim_state_ = ClaimState::Unknown;
};

}  // namespace sailorwind
