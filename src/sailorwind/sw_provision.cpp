#include "sw_provision.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_random.h>

#include "firmware_info.h"  // kFirmwareVersion

namespace sailorwind {

namespace {

constexpr const char* kNvsNamespace = "sw_prov";
constexpr const char* kNvsDeviceId = "device_id";
constexpr const char* kNvsSecret = "secret";

// base64url alphabet (RFC 4648 §5, no padding) — URL-safe, matches what the
// server's DeviceRegisterRequest.secret accepts.
const char kB64Url[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

// Encode `len` bytes as base64url (no padding) into `out` (NUL-terminated).
void Base64UrlEncode(const uint8_t* in, size_t len, String* out) {
  out->clear();
  size_t i = 0;
  while (i + 3 <= len) {
    const uint32_t n = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
    *out += kB64Url[(n >> 18) & 63];
    *out += kB64Url[(n >> 12) & 63];
    *out += kB64Url[(n >> 6) & 63];
    *out += kB64Url[n & 63];
    i += 3;
  }
  const size_t rem = len - i;
  if (rem == 1) {
    const uint32_t n = in[i] << 16;
    *out += kB64Url[(n >> 18) & 63];
    *out += kB64Url[(n >> 12) & 63];
  } else if (rem == 2) {
    const uint32_t n = (in[i] << 16) | (in[i + 1] << 8);
    *out += kB64Url[(n >> 18) & 63];
    *out += kB64Url[(n >> 12) & 63];
    *out += kB64Url[(n >> 6) & 63];
  }
}

// Trim trailing slashes so `${base}/v1/...` is stable.
String TrimBase(const String& url) {
  String base(url);
  while (base.endsWith("/")) base.remove(base.length() - 1);
  return base;
}

}  // namespace

void SwProvisioner::Begin() {
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/true)) {
    device_id_ = prefs.getString(kNvsDeviceId, "");
    secret_ = prefs.getString(kNvsSecret, "");
    prefs.end();
  }
  GenerateSecretIfMissing();
  // If we already hold a device_id + token, we're registered (possibly claimed —
  // CheckIn() resolves that). Leave claim_state_ Unknown until the first CheckIn.
}

bool SwProvisioner::GenerateSecretIfMissing() {
  if (secret_.length() >= 16) return true;  // already have one
  uint8_t buf[32];
  for (int i = 0; i < 8; i++) {
    const uint32_t r = esp_random();
    buf[i * 4 + 0] = static_cast<uint8_t>(r);
    buf[i * 4 + 1] = static_cast<uint8_t>(r >> 8);
    buf[i * 4 + 2] = static_cast<uint8_t>(r >> 16);
    buf[i * 4 + 3] = static_cast<uint8_t>(r >> 24);
  }
  Base64UrlEncode(buf, sizeof(buf), &secret_);
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
    prefs.putString(kNvsSecret, secret_);
    prefs.end();
  }
  return secret_.length() >= 16;
}

bool SwProvisioner::EnsureRegistered() {
  // Already have a token + device id → registered. (Recovery after an NVS wipe
  // is handled server-side: same hardwareId + secret re-registers idempotently;
  // we only reach Register() when we have no token.)
  if (config_ && config_->hasToken() && device_id_.length() > 0) return true;
  return Register();
}

bool SwProvisioner::Register() {
  if (!config_ || secret_.length() < 16) return false;

  WiFiClientSecure client;
  client.setCACert(ca_cert_pem_);
  const String url = TrimBase(config_->serverUrl()) + "/v1/devices/register";

  HTTPClient https;
  if (!https.begin(client, url)) return false;
  https.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> req;
  req["hardwareId"] = WiFi.macAddress();
  req["model"] = "shwg";
  req["secret"] = secret_;
  req["fw"] = kFirmwareVersion;
  String body;
  serializeJson(req, body);

  const int code = https.POST(body);
  if (code != 201) {
    https.end();
    return false;  // 409 (secret mismatch) / 429 / network — caller retries
  }
  String resp = https.getString();
  https.end();

  StaticJsonDocument<768> rd;
  if (deserializeJson(rd, resp)) return false;

  const String device_id = rd["deviceId"] | "";
  const String token = rd["token"] | "";
  if (device_id.length() == 0 || token.length() == 0) return false;

  device_id_ = device_id;
  claim_code_ = rd["claimCode"] | "";
  claim_state_ = (rd["claimed"] | false) ? ClaimState::Claimed
                                         : ClaimState::Unclaimed;

  // Persist device_id (NVS) and token (SwConfig → SPIFFS, UI-visible).
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
    prefs.putString(kNvsDeviceId, device_id_);
    prefs.end();
  }
  config_->setApiToken(token);
  return true;
}

void SwProvisioner::CheckIn() {
  if (!config_ || !config_->hasToken()) return;

  WiFiClientSecure client;
  client.setCACert(ca_cert_pem_);
  const String url = TrimBase(config_->serverUrl()) + "/v1/devices/me";

  HTTPClient https;
  if (!https.begin(client, url)) return;
  https.addHeader("Authorization", String("Bearer ") + config_->apiToken());

  const int code = https.GET();
  if (code != 200) {
    https.end();
    return;  // keep last known state; retry next check-in
  }
  String resp = https.getString();
  https.end();

  StaticJsonDocument<768> rd;
  if (deserializeJson(rd, resp)) return;

  const bool claimed = rd["claimed"] | false;
  claim_state_ = claimed ? ClaimState::Claimed : ClaimState::Unclaimed;
  // While unclaimed the server echoes a live code; once claimed it's null.
  claim_code_ = claimed ? String("") : (rd["claimCode"] | "");
}

}  // namespace sailorwind
