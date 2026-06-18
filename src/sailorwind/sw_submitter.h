#pragma once
//
// sw_submitter.h — HTTPS POST + persistent retry queue for the on-device
// sailorwind submitter.
//
// Port of clients/shared/src/submitter.ts, re-shaped for the ESP32:
//   - One idempotency key (uuidv4) per enqueue, kept across retries so the
//     server dedupes replays of an unacked send (PGN bursts + flaky marine WiFi
//     make this real).
//   - Bounded queue, drop-oldest: a multi-day offline stretch must not eat
//     memory or flash. 100 entries @ the 10-min cadence ≈ 16 h of buffer.
//   - FLASH is the source of truth, not RAM. submitter.ts holds the whole queue
//     array in memory; on a 520 KB part with TLS resident we can't (100×~400 B
//     ≈ 40 KB). Instead the queue lives in a flash-backed store (LittleFS/NVS)
//     and the submitter holds only counts + lastSuccess; bodies are streamed
//     from the store one at a time at send.
//   - Reuses the OTA path's proven TLS: WiFiClientSecure + setCACert(ISRG Root
//     X1) — the same root sailorwind.net serves under (see ota_update_task.cpp).
//
// Heavy networking headers (WiFiClientSecure / HTTPClient) stay out of this
// header; the send()/drain() bodies live in sw_submitter.cpp.

#include <cstddef>
#include <cstdint>

namespace sailorwind {

constexpr int kMaxQueueEntries = 100;     // ~16 h @ 10-min cadence
constexpr size_t kMaxBodyBytes = 1024;    // one ObservationInput JSON, generous
constexpr size_t kIdempotencyKeyLen = 36; // uuidv4 canonical form

// Per-entry outcome of a single POST attempt. Mirrors submitter.ts send().
enum class SendOutcome : uint8_t {
  Success,  // 2xx — drop from queue, advance
  Drop,     // permanent 4xx (bad token, schema reject) — drop, don't retry
  Retain,   // 408/429/5xx/network — keep at head, stop draining this pass
};

// One queued observation. The body is already-serialized JSON (built by
// sw_json from an SwObservation), so the submitter is agnostic to field shape.
struct SwQueueEntry {
  char idempotency_key[kIdempotencyKeyLen + 1];
  char body[kMaxBodyBytes];  // NUL-terminated JSON
  size_t body_len;
};

// ── Persistence backend (flash-backed; the submitter is otherwise stateless) ──
//
// Contract: append-ordered, drop-oldest at the cap, survives reboot. An entry
// is removed only after a confirmed Success/Drop. Implementations:
//   - SwLittleFsQueueStore — one file per entry under /sw/queue/ (production)
//   - SwMemQueueStore      — RAM-only ring (host/native unit tests)
// Mirrors the QueueStore interface in clients/shared/src/queue/types.ts.
class SwQueueStore {
 public:
  virtual ~SwQueueStore() = default;
  // Append; enforce the cap by dropping the oldest. Returns false on flash error.
  virtual bool Push(const SwQueueEntry& entry) = 0;
  // Load the oldest entry (FIFO head) into `out`. False if empty / read error.
  virtual bool PeekOldest(SwQueueEntry* out) = 0;
  // Remove the oldest entry (after a confirmed send/drop). False on error.
  virtual bool PopOldest() = 0;
  virtual int Count() = 0;
};

// ── Dependencies (injectable — keeps send() testable on host) ──────────────────
struct SwSubmitterDeps {
  const char* server_url;   // e.g. "https://sailorwind.net" (trailing slash trimmed)
  const char* api_token;    // device bearer token (slw_dev_…); NEVER logged
  const char* ca_cert_pem;  // ISRG Root X1 (reuse ota's server_ca_certificate)
  SwQueueStore* store;      // not owned
};

struct SwDrainResult {
  int sent = 0;          // entries POSTed OK this pass
  int queue_depth = 0;   // entries still queued after the pass
  bool had_success = false;
  uint32_t last_success_ms = 0;  // millis() of the most recent success
};

class SwSubmitter {
 public:
  explicit SwSubmitter(const SwSubmitterDeps& deps);

  // Serialize-and-enqueue happens in the caller (sw_json builds the body);
  // Enqueue stamps a fresh idempotency key, pushes to the store, and returns.
  // `body_json` must be NUL-terminated and < kMaxBodyBytes.
  bool Enqueue(const char* body_json);

  // Attempt every queued entry in FIFO order. Stops at the first Retain so we
  // don't hammer a server that's already complaining. Drops permanent 4xx.
  // Safe to call when offline (every send Retains → no-op, queue intact).
  SwDrainResult Drain();

  int QueueDepth() { return store_->Count(); }
  uint32_t LastSuccessMs() const { return last_success_ms_; }

 private:
  // POST one entry. Status classification mirrors submitter.ts exactly:
  //   2xx → Success; 408/429/5xx → Retain; other 4xx → Drop; network err → Retain.
  SendOutcome Send(const SwQueueEntry& entry);

  // uuidv4 from the ESP32 hardware RNG (esp_random()), canonical 8-4-4-4-12.
  static void MakeIdempotencyKey(char out[kIdempotencyKeyLen + 1]);

  // Map an HTTP status to an outcome (inline; pure — unit-tested on host).
  static SendOutcome ClassifyStatus(int http_status) {
    if (http_status >= 200 && http_status < 300) return SendOutcome::Success;
    if (http_status == 408 || http_status == 429) return SendOutcome::Retain;
    if (http_status >= 500) return SendOutcome::Retain;
    return SendOutcome::Drop;  // other 4xx — permanent
  }

  SwSubmitterDeps deps_;
  SwQueueStore* store_;
  uint32_t last_success_ms_ = 0;
};

}  // namespace sailorwind
