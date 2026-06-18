#pragma once
//
// sw_n2k_tap.h — bridges decoded NMEA 2000 PGNs into the SwAggregator.
//
// Registered as a SIBLING N2K consumer alongside n2k_nmea0183_transform (do NOT
// edit the transform). Each incoming tN2kMsg is parsed with the ttlappalainen
// NMEA2000 library's ParseN2k* helpers (all return SI) and pushed into the
// aggregator, stamped with the current epoch-ms clock (GNSS / SNTP) so
// observedAt reflects wall-clock, not uptime.
//
// Compiles against the NMEA2000 library on the ESP32 target only (N2kMessages.h
// / N2kTypes.h). It is NOT part of the host native-test build — that exercises
// the pure SwAggregator directly. The exact ParseN2k* signatures + enum names
// below match what n2k_nmea0183_transform.cpp already calls; re-confirm against
// the pinned NMEA2000-library version when this first compiles on hardware.

#include "sw_aggregator.h"
#include "sw_clock.h"  // EpochClockMs

class tN2kMsg;  // forward decl — defined by the NMEA2000 library

namespace sailorwind {

// The tap DROPS weather samples while the clock returns 0 (unsynced): an
// observation with no trustworthy observedAt is worse than none. See sw_clock.h.

class SwN2kTap {
 public:
  SwN2kTap(SwAggregator* agg, EpochClockMs clock) : agg_(agg), clock_(clock) {}

  // Route one decoded N2K message into the aggregator. Call from the N2K
  // message pump (e.g. an NMEA2000.AttachMsgHandler shim, or a hook in the
  // existing transform). Unknown PGNs are ignored.
  void HandleMsg(const tN2kMsg& msg);

  // 0-terminated list of PGNs this tap consumes — add them to the firmware's
  // ReceiveMessages[] (129025/130306/127250/127258/129026 are already there;
  // 130311–130316 are the new environmental ones).
  static const unsigned long* WantedPgns();

 private:
  SwAggregator* agg_;
  EpochClockMs clock_;
};

}  // namespace sailorwind
