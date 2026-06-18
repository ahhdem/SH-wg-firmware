#pragma once
//
// sw_app.h — entry point that wires the sailorwind submitter into the SH-wg
// firmware. Additive + config-gated: the gateway's NMEA0183/SignalK paths are
// untouched. Compiled only when -D SW_SAILORWIND is set (see platformio.ini).
//
// Integration (main.cpp, inside #ifdef SW_SAILORWIND):
//   1. Construct everything + start the network task:   sailorwind::SailorwindBegin();
//   2. Feed every decoded N2K message into the tap from the existing N2K
//      consumer on the main loop:                        sailorwind::FeedN2k(msg);
//   3. Advertise the extra environmental PGNs:           ExtendReceiveMessages(SwN2kTap::WantedPgns())
//
// All blocking TLS (register / check-in / submit) runs on a dedicated FreeRTOS
// task — never on the ReactESP main loop, which runs the 50 µs N2K pump. The
// only cross-task state is the aggregator, guarded by a short-held mutex.

#include <Arduino.h>  // String

class tN2kMsg;  // defined by the NMEA2000 library

namespace sailorwind {

// Build config + aggregator + tap + provisioner, kick SNTP, and start the
// submit/provision task. Call once from setup(), after the SensESP app +
// filesystem are up (SwConfig loads from SPIFFS) and before app start.
void SailorwindBegin();

// Route one decoded N2K message into the aggregator. Call from the main-loop
// N2K consumer. Cheap + non-stalling (brief mutex); safe before/after claim.
void FeedN2k(const tN2kMsg& msg);

// Human-readable status + claim code for the SensESP web UI status page. Safe to
// call any time (null-safe before SailorwindBegin runs). ClaimCodeForUi returns
// the formatted code only while unclaimed, otherwise "".
String ClaimStatusForUi();
String ClaimCodeForUi();

}  // namespace sailorwind
