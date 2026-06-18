#pragma once
//
// sw_selfupdate.h — manifest-driven, hash-verified, auto-rollback self-update.
//
// Evolves ota_update_task.cpp (which GETs a bare hex version from Hat Labs and
// streams firmware_<ver>.bin). Full design: sailorwind repo
// docs/sailorwind-submitter.md §7. Three upgrades over the stock scheme:
//
//   1. JSON manifest — the server owns filename + channel + integrity:
//        GET {server}/v1/firmware/manifest?hw=shwg&channel=<ch>&ver=<hex>&mac=<mac>
//          → 200 { versionCode, version, url, sha256, size, mandatory }
//          → 204  (already current)
//      The device hard-codes only the manifest path; the returned `url` is the
//      "assemble an update URL from filename" piece.
//   2. SHA-256 verification beyond TLS — stream the OTA bytes through
//      mbedtls_sha256 (already linked for TLS) and refuse to commit unless the
//      digest matches the manifest. Catches a truncated/swapped artifact and
//      lets the device skip re-downloading a version it already has.
//   3. Rollback — write to the inactive OTA partition, set pending, reboot, and
//      call esp_ota_mark_app_valid_cancel_rollback() ONLY after the submitter
//      reaches sailorwind.net once post-update. A bad image that panics or can't
//      get online auto-reverts on next reset (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE;
//      the current min_spiffs dual app0/app1 layout already supports it).
//
// versionCode keeps Hat Labs' packed-hex scheme (kFirmwareHexVersion) to avoid
// churning their bump-version tooling. Reuses WiFiClientSecure + ISRG Root X1.
// NOT required functional in the first release — this is the shape so the code
// can carry it from day one (unattended fleet → updates must be safe + hands-off).
// Compiles on the ESP32 target only; .cpp is the on-hardware work.

#include <Arduino.h>

#include "sw_clock.h"

namespace sailorwind {

// How an available update is applied. download-and-stage writes + verifies the
// inactive partition but does NOT reboot (surfaced to the user); reboot applies
// immediately (drops the gateway ~15 s). Driven by SwConfig::autoApplyUpdates().
enum class UpdateApply : uint8_t { StageOnly, Reboot };

struct UpdateManifest {
  uint32_t version_code = 0;
  char version[16] = {0};
  char url[256] = {0};
  char sha256_hex[65] = {0};
  uint32_t size = 0;
  bool mandatory = false;
};

class SwSelfUpdate {
 public:
  SwSelfUpdate(const char* server_url, const char* ca_cert_pem,
               const char* channel, EpochClockMs clock)
      : server_url_(server_url),
        ca_cert_pem_(ca_cert_pem),
        channel_(channel),
        clock_(clock) {}

  // After a successful post-update boot, confirm the image is good so the
  // bootloader won't roll it back. Call once the submitter has reached
  // sailorwind.net (or another liveness signal) — NOT merely on boot.
  void MarkRunningImageValid();

  // Check on boot (clock valid) and on a long periodic timer. Fetches the
  // manifest; if a newer versionCode is offered, downloads to the inactive
  // partition with streaming SHA-256 verification and applies per `apply`.
  // No-op when already current (204) or clock unsynced. Returns true if an
  // update was staged/applied.
  bool CheckAndMaybeApply(UpdateApply apply);

 private:
  bool FetchManifest(UpdateManifest* out);  // GET …/v1/firmware/manifest
  bool DownloadVerifyApply(const UpdateManifest& m, UpdateApply apply);

  const char* server_url_;
  const char* ca_cert_pem_;
  const char* channel_;
  EpochClockMs clock_;
};

}  // namespace sailorwind
