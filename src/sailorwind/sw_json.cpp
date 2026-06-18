#include "sw_json.h"

#include <ArduinoJson.h>
#include <time.h>

#include <cstdio>

namespace sailorwind {

bool FormatIso8601Utc(int64_t epoch_ms, char* out, size_t cap) {
  if (!out || cap < 25) return false;
  const time_t secs = static_cast<time_t>(epoch_ms / 1000);
  int ms = static_cast<int>(epoch_ms % 1000);
  if (ms < 0) ms += 1000;  // defensive: epoch_ms should be >= 0
  struct tm tmv;
  gmtime_r(&secs, &tmv);
  const int n = snprintf(out, cap, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                         tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                         tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
  return n > 0 && static_cast<size_t>(n) < cap;
}

size_t BuildObservationJson(const SwObservation& obs, SwFieldMask fields,
                            bool publish_precise, char* out, size_t cap) {
  if (!out || !obs.has(static_cast<float>(obs.lat_deg))) return 0;

  // Capacity covers ~15 scalar fields + position + the 40-element histogram
  // (≈600 B of tree); the serialized OUTPUT is bounded by `cap` (kMaxBodyBytes).
  StaticJsonDocument<1024> doc;

  char ts[28];
  if (!FormatIso8601Utc(obs.observed_at_ms, ts, sizeof(ts))) return 0;
  doc["observedAt"] = ts;

  JsonObject pos = doc.createNestedObject("position");
  pos["lat"] = obs.lat_deg;
  pos["lon"] = obs.lon_deg;
  doc["positionSource"] = kPositionSourceGps;
  doc["source"] = kSourceSwgw;
  // Only emit the flag when opted in — the server defaults coarse, so an omitted
  // field and `false` are equivalent (matches the SignalK plugin).
  if (publish_precise) doc["publishPrecise"] = true;

  if (fields.has(SwField::WindSpeed)) {
    if (obs.has(obs.wind_speed_ms)) doc["windSpeedMs"] = obs.wind_speed_ms;
    if (obs.has(obs.wind_gust_ms)) doc["windGustMs"] = obs.wind_gust_ms;
    // Rich distribution rides the windSpeed opt-in (all derived from speed).
    if (obs.has(obs.wind_gust_10s_ms)) doc["windGust10sMs"] = obs.wind_gust_10s_ms;
    if (obs.has(obs.wind_gust_30s_ms)) doc["windGust30sMs"] = obs.wind_gust_30s_ms;
    if (obs.has(obs.wind_gust_60s_ms)) doc["windGust60sMs"] = obs.wind_gust_60s_ms;
    if (obs.has(obs.wind_speed_std_ms)) doc["windSpeedStdMs"] = obs.wind_speed_std_ms;
    if (obs.wind_gust_crossings >= 0)
      doc["windGustCrossings"] = obs.wind_gust_crossings;
    if (obs.has_histogram) {
      JsonArray hist = doc.createNestedArray("windSpeedHistogram");
      for (int i = 0; i < kWindHistogramBins; i++) hist.add(obs.wind_speed_histogram[i]);
    }
  }
  if (fields.has(SwField::WindDirection) && obs.has(obs.wind_from_deg))
    doc["windDirDegFrom"] = obs.wind_from_deg;
  if (fields.has(SwField::Pressure) && obs.has(obs.pressure_hpa))
    doc["pressureHpa"] = obs.pressure_hpa;
  if (fields.has(SwField::AirTemp) && obs.has(obs.air_temp_c))
    doc["airTempC"] = obs.air_temp_c;
  if (fields.has(SwField::WaterTemp) && obs.has(obs.sst_c))
    doc["sstC"] = obs.sst_c;
  if (fields.has(SwField::Humidity) && obs.has(obs.rel_humidity_pct))
    doc["relativeHumidityPct"] = obs.rel_humidity_pct;

  const size_t n = serializeJson(doc, out, cap);
  // serializeJson writes a NUL and never overruns `cap`; a 0 return means the
  // buffer was too small (truncated → caller drops the window rather than POST
  // malformed JSON).
  return n;
}

}  // namespace sailorwind
