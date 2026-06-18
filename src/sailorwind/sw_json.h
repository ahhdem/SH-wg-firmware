#pragma once
//
// sw_json.h — serialize an SwObservation into the POST /v1/observations wire
// body, applying the user's field-toggle selection.
//
// Port of buildObservation() in clients/shared/signalk-sailorwind-plugin and
// the ObservationInput contract in clients/shared/src/types.ts. Field NAMES and
// units must match the canonical Zod schema
// (packages/shared/src/schemas/observation.ts) exactly — drift = 400 from the
// server. Position is ALWAYS included; everything else is gated by SwFieldMask.
//
// ArduinoJson (header-only) does the encoding in sw_json.cpp; this header is the
// reviewable contract: the field enum, the mask, and the builder signature.

#include <cstddef>
#include <cstdint>

#include "sw_aggregator.h"  // SwObservation

namespace sailorwind {

// Constant wire values for the on-device (N2K) path. The server forces device-
// token submissions to an auto_* source and gps position; we send the honest
// pair. `auto_swgw` (Sailorwind Gateway) is this device's own provenance tag:
// it parses N2K PGNs off the wire directly and POSTs — it never emits NMEA0183
// over a TCP socket (auto_nmea0183) and isn't the SignalK plugin (auto_signalk).
constexpr const char* kSourceSwgw = "auto_swgw";
constexpr const char* kPositionSourceGps = "gps";

// Which optional fields the user has enabled in the device config (sw_config).
// Position is always sent and has no flag. Mirrors SubmittableField in the
// plugin's types.ts. The rich wind distribution rides the WindSpeed opt-in
// (it's all derived from speed), exactly as buildObservation() does.
enum class SwField : uint8_t {
  WindSpeed = 1 << 0,
  WindDirection = 1 << 1,
  Pressure = 1 << 2,
  AirTemp = 1 << 3,
  WaterTemp = 1 << 4,
  Humidity = 1 << 5,
};

struct SwFieldMask {
  uint8_t bits = 0;
  bool has(SwField f) const { return bits & static_cast<uint8_t>(f); }
  void set(SwField f) { bits |= static_cast<uint8_t>(f); }
};

// Format an epoch-millisecond timestamp as ISO-8601 UTC (e.g.
// "2026-06-18T14:03:21.000Z") into `out` (needs >= 25 bytes). Returns false on
// a bad buffer. Pure — unit-tested on host. observedAt precision is ms to match
// the JS Date().toISOString() the rest of the pipeline emits.
bool FormatIso8601Utc(int64_t epoch_ms, char* out, size_t cap);

// Build the JSON body from one flushed observation, gated by `fields`.
//
//   - position {lat,lon}, positionSource, source, observedAt  → always
//   - windSpeedMs + windGustMs + the rich distribution (std, crossings,
//     histogram, 10/30/60s gust ladder)                       → SwField::WindSpeed
//   - windDirDegFrom                                          → SwField::WindDirection
//   - pressureHpa / airTempC / sstC / relativeHumidityPct     → respective flags
//   - publishPrecise: emitted ONLY when true (server defaults coarse; sending
//     false is equivalent to omitting it — matches the plugin)
//
// Each gated field is written only when BOTH its flag is set AND the value is
// present in `obs` (SwObservation::has()). The CALLER is responsible for the
// completeness check the server requires — windSpeedMs AND windDirDegFrom must
// both be present or the POST will 400; if either is missing, skip the window
// rather than enqueue (see submit() in the plugin's index.ts). We deliberately
// never fabricate a direction.
//
// Writes a NUL-terminated body into `out`; returns the byte length, or 0 on
// error (buffer too small / no position).
size_t BuildObservationJson(const SwObservation& obs, SwFieldMask fields,
                            bool publish_precise, char* out, size_t cap);

}  // namespace sailorwind
