#pragma once
//
// sw_mem_queue_store.h — RAM-backed FIFO queue store for the submitter.
//
// A small fixed-capacity ring (NOT the full 100-entry cap — 100×1KB would be
// ~100KB RAM, too much alongside TLS). This is the bring-up store: it does not
// survive reboot. The production store is LittleFS-backed (queue survives power
// cycles) — see docs/sailorwind-submitter.md §9 / the submitter TASK.
//
// Capacity 16 × ~1KB ≈ 17KB RAM. Drop-oldest at the cap (newest data is the
// most useful — old positions are stale anyway).

#include "sw_submitter.h"

namespace sailorwind {

template <int Capacity = 16>
class SwMemQueueStore : public SwQueueStore {
 public:
  bool Push(const SwQueueEntry& entry) override {
    if (count_ == Capacity) {  // drop oldest
      head_ = (head_ + 1) % Capacity;
      count_--;
    }
    buf_[(head_ + count_) % Capacity] = entry;
    count_++;
    return true;
  }
  bool PeekOldest(SwQueueEntry* out) override {
    if (count_ == 0 || !out) return false;
    *out = buf_[head_];
    return true;
  }
  bool PopOldest() override {
    if (count_ == 0) return false;
    head_ = (head_ + 1) % Capacity;
    count_--;
    return true;
  }
  int Count() override { return count_; }

 private:
  SwQueueEntry buf_[Capacity];
  int head_ = 0;
  int count_ = 0;
};

}  // namespace sailorwind
