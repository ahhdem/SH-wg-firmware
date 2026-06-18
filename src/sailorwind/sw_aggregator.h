#pragma once
//
// sw_aggregator.h — windowed observation aggregator for the on-device
// sailorwind submitter.
//
// Port of clients/shared/src/aggregator.ts, re-shaped for a 520 KB-SRAM part:
// where the TS version retains every sample for the 10-min window, we keep
// O(1) STREAMING accumulators for everything except the sustained-gust ladder,
// which needs the last ~60 s and so keeps a single bounded ring (~5 KB). Net
// steady-state footprint is well under 10 KB.
//
// The plugin entrypoint owns the 10-min timer; it feeds samples via Add*() and
// calls Flush() at the boundary, then Reset(). Pure data + math — no IO, no
// timers, no N2K types (the tap converts to SI before calling in).
//
// Field names + units in SwObservation MUST match ObservationInput in
// clients/shared/src/types.ts exactly (drift = 400 from the server). All SI:
// m/s, degrees-true (meteorological FROM), hPa, C, % , WGS84 degrees.

#include <cmath>
#include <cstdint>
#include <cstring>

#include "sw_wind_direction.h"

namespace sailorwind {

constexpr int kWindHistogramBins = 40;   // bin i = [i, i+1) m/s; bin 39 = [39, inf)
constexpr int kWindRingCapacity = 768;   // ~60 s @ ~12 Hz worst case
constexpr int64_t kGustWindow10Ms = 10000;
constexpr int64_t kGustWindow30Ms = 30000;
constexpr int64_t kGustWindow60Ms = 60000;

// ── Flush output → straight into the JSON body builder ────────────────────────
// Optional fields use NAN (floats) / -1 (counts) / has_* (arrays) as "absent".
struct SwObservation {
  int64_t observed_at_ms = 0;  // latest weather-sample time; JSON formats ISO-8601
  double lat_deg = NAN;
  double lon_deg = NAN;

  float wind_speed_ms = NAN;   // window mean
  float wind_gust_ms = NAN;    // instantaneous max
  float wind_from_deg = NAN;   // true, meteorological FROM
  float pressure_hpa = NAN;
  float air_temp_c = NAN;
  float sst_c = NAN;
  float rel_humidity_pct = NAN;

  // Rich per-window wind distribution (N2K path sees the full high-rate stream).
  float wind_gust_10s_ms = NAN;
  float wind_gust_30s_ms = NAN;
  float wind_gust_60s_ms = NAN;
  float wind_speed_std_ms = NAN;
  int32_t wind_gust_crossings = -1;
  bool has_histogram = false;
  uint16_t wind_speed_histogram[kWindHistogramBins] = {0};

  WindDirMethod wind_from_method = WindDirMethod::None;  // diagnostic only

  bool has(float v) const { return std::isfinite(v); }
};

// ── Streaming scalar accumulator (pressure/temp/humidity) ─────────────────────
// We submit "latest in window" for these (matches aggregator.ts latestNonNull).
// Stored in N2K-native units (Pa / K / %); converted at Flush().
struct ScalarAccum {
  double latest = NAN;
  int64_t latest_t = 0;
  void Add(double v, int64_t t) {
    if (!std::isfinite(v)) return;
    latest = v;
    latest_t = t;
  }
  bool has() const { return std::isfinite(latest); }
  void Reset() { latest = NAN; latest_t = 0; }
};

// ── Bounded time-ordered ring for the sustained-gust ladder ───────────────────
struct WindRing {
  struct S { float v; int64_t t; };
  S buf[kWindRingCapacity];
  int head = 0;      // index of oldest element
  int count = 0;
  void Push(float v, int64_t t) {
    int idx = (head + count) % kWindRingCapacity;
    if (count < kWindRingCapacity) {
      count++;
    } else {
      head = (head + 1) % kWindRingCapacity;  // overwrite oldest
      idx = (head + count - 1) % kWindRingCapacity;
    }
    buf[idx] = {v, t};
  }
  int size() const { return count; }
  // Oldest-first logical access (k = 0..count-1).
  const S& at(int k) const { return buf[(head + k) % kWindRingCapacity]; }
  void Reset() { head = 0; count = 0; }
};

// Peak rolling-mean over any fully-supported window of width window_ms.
// Port of peakSustained() in aggregator.ts: only windows whose right edge stays
// within the data span count; if the whole span is shorter than the window we
// fall back to the overall mean (can't claim 60 s-sustained from 20 s of data).
// Two-pointer running sum — O(n), O(1) extra (no prefix array → small stack).
inline double PeakSustained(const WindRing& r, int64_t window_ms) {
  const int n = r.size();
  if (n == 0) return NAN;
  const int64_t last_t = r.at(n - 1).t;
  double overall = 0;
  for (int i = 0; i < n; i++) overall += r.at(i).v;
  overall /= n;

  double peak = -INFINITY;
  int j = 0;
  double sum = 0;
  for (int i = 0; i < n; i++) {
    const int64_t start = r.at(i).t;
    if (start + window_ms > last_t) break;  // window would overrun the span
    if (j < i) { j = i; sum = 0; }
    while (j < n && r.at(j).t < start + window_ms) { sum += r.at(j).v; j++; }
    const double m = sum / (j - i);
    if (m > peak) peak = m;
    sum -= r.at(i).v;  // slide the left edge for the next i
  }
  return std::isfinite(peak) ? peak : overall;
}

// ── The aggregator ────────────────────────────────────────────────────────────
class SwAggregator {
 public:
  // Position — only the latest is needed (matches TS, which buffers but uses
  // the last). No retention.
  void AddPosition(double lat, double lon, int64_t t) {
    if (!std::isfinite(lat) || !std::isfinite(lon)) return;
    lat_ = lat; lon_ = lon; has_pos_ = true;
    BumpWeatherTime(t);
  }

  // One PGN 130306 sample carries both speed and a direction sample.
  void AddWind(double speed_ms, double angle_rad, WindRef ref, int64_t t) {
    if (std::isfinite(speed_ms)) {
      AddWindSpeed(speed_ms, t);
    }
    wind_dir_.AddSample(angle_rad, ref, t);  // routes by reference; may skip
    BumpWeatherTime(t);
  }

  // Speed-only path (if an installation publishes speed without 130306 angle).
  void AddWindSpeed(double v, int64_t t);

  // Nav context for the direction resolver (not weather → does not move
  // observed_at; see BumpWeatherTime).
  void SetHeadingTrue(double rad, uint32_t t) { wind_dir_.SetHeadingTrue(rad, t); }
  void SetHeadingMagnetic(double rad, uint32_t t) { wind_dir_.SetHeadingMagnetic(rad, t); }
  void SetVariation(double rad, uint32_t t) { wind_dir_.SetVariation(rad, t); }
  void SetBoatMotion(double sog_ms, double cog_rad, uint32_t t) {
    wind_dir_.SetBoatMotion(sog_ms, cog_rad, t);  // v2 apparent-wind seam
  }

  // Scalars — stored N2K-native, converted at Flush(). These ARE weather.
  void AddPressurePa(double pa, int64_t t)   { pressure_.Add(pa, t); BumpWeatherTime(t); }
  void AddAirTempK(double k, int64_t t)      { air_temp_.Add(k, t);  BumpWeatherTime(t); }
  void AddWaterTempK(double k, int64_t t)    { water_temp_.Add(k, t); BumpWeatherTime(t); }
  void AddHumidityPct(double pct, int64_t t) { humidity_.Add(pct, t); BumpWeatherTime(t); }

  // True only when no usable position has been seen this window — submission
  // requires a fix (matches aggregator.ts isEmpty()).
  bool IsEmpty() const { return !has_pos_; }

  // Compute the window's observation. Returns false when there's no position.
  // Caller applies the field-toggle gating, then Reset().
  bool Flush(SwObservation* out) const;

  void Reset();

 private:
  void BumpWeatherTime(int64_t t) {
    if (t > latest_weather_t_) latest_weather_t_ = t;
  }

  // Latest position.
  double lat_ = NAN, lon_ = NAN;
  bool has_pos_ = false;

  // Streaming wind-speed stats (Welford mean/variance + peak + histogram) —
  // no sample retention.
  uint32_t wind_n_ = 0;
  double wind_mean_ = 0, wind_m2_ = 0;  // population var = m2/n
  float wind_peak_ = 0;
  uint16_t hist_[kWindHistogramBins] = {0};
  WindRing ring_;  // only retained samples — the 60 s gust ladder

  SwWindDirection wind_dir_;
  ScalarAccum pressure_, air_temp_, water_temp_, humidity_;

  // observed_at = latest sample time across MEASURED-WEATHER streams (not flush
  // wall-clock, not nav context). See the long note in aggregator.ts.
  int64_t latest_weather_t_ = 0;
};

// ── Inline implementation ────────────────────────────────────────────────────

inline void SwAggregator::AddWindSpeed(double v, int64_t t) {
  if (!std::isfinite(v) || v < 0) return;
  // Welford streaming mean/variance.
  wind_n_++;
  const double delta = v - wind_mean_;
  wind_mean_ += delta / wind_n_;
  wind_m2_ += delta * (v - wind_mean_);
  if (v > wind_peak_) wind_peak_ = static_cast<float>(v);
  // Histogram: bin i = [i, i+1), top bin clamps [39, inf).
  int bin = static_cast<int>(std::floor(v));
  if (bin >= kWindHistogramBins) bin = kWindHistogramBins - 1;
  hist_[bin]++;
  ring_.Push(static_cast<float>(v), t);
  BumpWeatherTime(t);
}

inline bool SwAggregator::Flush(SwObservation* out) const {
  if (!has_pos_ || out == nullptr) return false;
  *out = SwObservation{};
  out->observed_at_ms = latest_weather_t_;
  out->lat_deg = lat_;
  out->lon_deg = lon_;

  if (wind_n_ > 0) {
    const double std_ms = std::sqrt(wind_m2_ / wind_n_);  // population std
    out->wind_speed_ms = static_cast<float>(wind_mean_);
    out->wind_gust_ms = wind_peak_;
    out->wind_speed_std_ms = static_cast<float>(std_ms);
    out->has_histogram = true;
    std::memcpy(out->wind_speed_histogram, hist_, sizeof(hist_));
    out->wind_gust_10s_ms = static_cast<float>(PeakSustained(ring_, kGustWindow10Ms));
    out->wind_gust_30s_ms = static_cast<float>(PeakSustained(ring_, kGustWindow30Ms));
    out->wind_gust_60s_ms = static_cast<float>(PeakSustained(ring_, kGustWindow60Ms));
    // windGustCrossings: upward crossings of (mean + 1σ). DIVERGENCE FROM TS:
    // the TS version counts over the whole window; we count over the 60 s ring
    // against the full-window threshold (we don't retain the whole window).
    // Documented in docs/sailorwind-submitter.md §4.
    const double level = wind_mean_ + std_ms;
    int32_t crossings = 0;
    for (int i = 1; i < ring_.size(); i++) {
      if (ring_.at(i - 1).v < level && ring_.at(i).v >= level) crossings++;
    }
    out->wind_gust_crossings = crossings;
  }

  double dir_deg;
  WindDirMethod method;
  if (wind_dir_.Resolve(&dir_deg, &method)) {
    out->wind_from_deg = static_cast<float>(dir_deg);
    out->wind_from_method = method;
  }

  // Scalars: convert N2K-native → sailorwind canonical at the edge.
  if (pressure_.has())   out->pressure_hpa = static_cast<float>(pressure_.latest / 100.0);  // Pa → hPa
  if (air_temp_.has())   out->air_temp_c = static_cast<float>(air_temp_.latest - 273.15);   // K → C
  if (water_temp_.has()) out->sst_c = static_cast<float>(water_temp_.latest - 273.15);      // K → C
  if (humidity_.has())   out->rel_humidity_pct = static_cast<float>(humidity_.latest);      // N2K already %

  return true;
}

inline void SwAggregator::Reset() {
  lat_ = lon_ = NAN; has_pos_ = false;
  wind_n_ = 0; wind_mean_ = 0; wind_m2_ = 0; wind_peak_ = 0;
  std::memset(hist_, 0, sizeof(hist_));
  ring_.Reset();
  wind_dir_.Reset();  // keeps nav context; clears accumulators
  pressure_.Reset(); air_temp_.Reset(); water_temp_.Reset(); humidity_.Reset();
  latest_weather_t_ = 0;
}

}  // namespace sailorwind
