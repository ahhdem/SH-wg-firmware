#include "sw_submitter.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_random.h>

#include <cstdio>
#include <cstring>

namespace sailorwind {

SwSubmitter::SwSubmitter(const SwSubmitterDeps& deps)
    : deps_(deps), store_(deps.store) {}

void SwSubmitter::MakeIdempotencyKey(char out[kIdempotencyKeyLen + 1]) {
  uint8_t b[16];
  for (int i = 0; i < 4; i++) {
    const uint32_t r = esp_random();  // hardware RNG
    b[i * 4 + 0] = static_cast<uint8_t>(r);
    b[i * 4 + 1] = static_cast<uint8_t>(r >> 8);
    b[i * 4 + 2] = static_cast<uint8_t>(r >> 16);
    b[i * 4 + 3] = static_cast<uint8_t>(r >> 24);
  }
  b[6] = (b[6] & 0x0F) | 0x40;  // version 4
  b[8] = (b[8] & 0x3F) | 0x80;  // variant 1
  snprintf(out, kIdempotencyKeyLen + 1,
           "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10],
           b[11], b[12], b[13], b[14], b[15]);
}

bool SwSubmitter::Enqueue(const char* body_json) {
  if (!store_ || !body_json) return false;
  const size_t len = strlen(body_json);
  if (len == 0 || len >= kMaxBodyBytes) return false;  // too big → drop (would 413)
  SwQueueEntry e;
  MakeIdempotencyKey(e.idempotency_key);
  memcpy(e.body, body_json, len + 1);
  e.body_len = len;
  return store_->Push(e);
}

SwDrainResult SwSubmitter::Drain() {
  SwDrainResult r;
  if (!store_) return r;
  // Send in FIFO order; stop at the first Retain so we don't hammer a server
  // that's already complaining. Drop permanent 4xx.
  while (store_->Count() > 0) {
    SwQueueEntry e;
    if (!store_->PeekOldest(&e)) break;
    const SendOutcome o = Send(e);
    if (o == SendOutcome::Success) {
      store_->PopOldest();
      r.sent++;
      r.had_success = true;
      last_success_ms_ = millis();
      r.last_success_ms = last_success_ms_;
    } else if (o == SendOutcome::Drop) {
      store_->PopOldest();
    } else {
      break;  // Retain — leave at head, retry next drain
    }
  }
  r.queue_depth = store_->Count();
  return r;
}

SendOutcome SwSubmitter::Send(const SwQueueEntry& entry) {
  WiFiClientSecure client;
  client.setCACert(deps_.ca_cert_pem);  // ISRG Root X1 (Let's Encrypt)

  // Trim one trailing slash so `${url}/v1/observations` is stable.
  String base(deps_.server_url);
  while (base.endsWith("/")) base.remove(base.length() - 1);
  const String url = base + "/v1/observations";

  HTTPClient https;
  if (!https.begin(client, url)) return SendOutcome::Retain;
  https.addHeader("Content-Type", "application/json");
  https.addHeader("Idempotency-Key", entry.idempotency_key);
  if (deps_.api_token && deps_.api_token[0]) {
    // NEVER log this value.
    https.addHeader("Authorization", String("Bearer ") + deps_.api_token);
  }
  const int code =
      https.POST(reinterpret_cast<uint8_t*>(const_cast<char*>(entry.body)),
                 entry.body_len);
  https.end();

  if (code <= 0) return SendOutcome::Retain;  // network/TLS layer failure
  return ClassifyStatus(code);
}

}  // namespace sailorwind
