#pragma once
//
// sw_clock.h — shared clock type for the sailorwind submitter modules.

#include <cstdint>

namespace sailorwind {

// Returns current UTC epoch milliseconds, or 0 when time isn't known yet (no
// GNSS fix / no SNTP sync). Consumers MUST treat 0 as "no trustworthy clock"
// and hold off: don't timestamp observations, and don't start a TLS submit or
// self-update (cert validation needs a real clock). The firmware already tracks
// system time (N2K 129029 / system-time PGN; see main.cpp's
// elapsed_since_last_system_time_update) — wire that source into one function
// of this shape and pass it to the modules below.
using EpochClockMs = int64_t (*)();

}  // namespace sailorwind
