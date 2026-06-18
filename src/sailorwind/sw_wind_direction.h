#pragma once
//
// sw_wind_direction.h — true-wind-direction resolver for the on-device
// sailorwind submitter.
//
// Port of clients/shared/src/wind-direction.ts (the SignalK plugin's resolver,
// which is our parity target). The map renders a wind barb, which needs the
// true compass direction the wind blows FROM (`windDirDegFrom`). Boats expose
// that in different shapes; we resolve in a fixed priority order and stop at the
// first source that yields an answer. If none does, we emit nothing for the
// window — we NEVER fabricate a direction or borrow the forecast's (that would
// manufacture false obs-vs-forecast agreement, the exact signal sailorwind
// exists to measure).
//
// ── On-device divergence from the TS reference (intentional, documented) ──
// The TS resolver buffers every direction sample for the 10-min window and
// pairs each wind angle with the NEAREST-IN-TIME heading at flush. On a 520 KB
// part we instead resolve each incoming wind sample to a true bearing
// IMMEDIATELY, against the most-recent heading/variation seen on the bus, and
// fold it into an O(1) vector accumulator. Resolving with the heading valid at
// that instant is equivalent-or-better than nearest-in-time and costs no
// retention. Priority across sources is preserved (see resolve()).
//
// ── CONVENTION NOTE (verify in TASK-158 before trusting output) ──
// We mirror the SignalK n2k→delta mapping exactly, because the obs already in
// the sailorwind DB were produced that way:
//   - N2K ref 0 (True/North-referenced) → SignalK directionTrue, used as a FROM
//     bearing directly (NO +180).
//   - N2K ref 1 (Magnetic)              → directionMagnetic + variation → true.
//   - N2K ref 3/4 (True boat/water)     → angle off bow + heading.
// The exact FROM-vs-TO sense per reference MUST be confirmed against
// canboat / @signalk/n2k-signalk during implementation; do not assume.

#include <cmath>
#include <cstdint>

namespace sailorwind {

// N2K PGN 130306 wind reference enum (matches tN2kWindReference ordering).
enum class WindRef : uint8_t {
  TrueNorth = 0,  // referenced to true north → directionTrue
  Magnetic = 1,   // referenced to magnetic north → directionMagnetic
  Apparent = 2,   // apparent, angle off bow → v1 SKIP (see solveApparent seam)
  TrueBoat = 3,   // true wind, boat-referenced angle off bow
  TrueWater = 4,  // true wind, water-referenced angle off bow
};

// How the direction was obtained — diagnostic/telemetry only, never wired to
// the barb. Mirrors WindDirectionMethod in wind-direction.ts.
enum class WindDirMethod : uint8_t {
  None = 0,
  DirectionTrue,
  DirectionMagneticVariation,
  DirectionMagneticWmm,            // v2 (WMM) — not reachable in v1
  AngleTrueWaterHeadingTrue,
  AngleTrueWaterHeadingMagVariation,
  AngleTrueWaterHeadingMagWmm,     // v2 (WMM) — not reachable in v1
  Apparent,                        // v2 fast-follow — not reachable in v1
};

const char* WindDirMethodName(WindDirMethod m);  // for debug logging

// Wrap an angle in degrees to a compass bearing in [0, 360).
// The trailing guard handles the FP edge where a tiny-negative input (e.g. the
// atan2 of mean(350,10), which rounds just below 0) takes the +360 branch and
// lands on exactly 360.0 — violating the documented [0,360) contract. Not a
// server-reject (windDirDegFrom is min(0).max(360) inclusive, so 360 is
// accepted) — purely a correctness/cleanliness fix: 360° == 0° as a bearing.
// NOTE: the TS wrapDeg in clients/shared/src/units.ts shares this latent edge
// (its own docstring claims mean(350,10)→0 but it actually yields 360); worth
// the same one-line guard there to keep the two data planes identical.
inline double WrapDeg(double deg) {
  double w = std::fmod(deg, 360.0);
  if (w < 0) w += 360.0;
  if (w >= 360.0) w -= 360.0;
  return w;
}

// ── The resolver ────────────────────────────────────────────────────────────
//
// Usage per window:
//   1. As heading / variation PGNs arrive, call SetHeadingTrue/Magnetic/
//      Variation to keep the live nav context current.
//   2. As 130306 wind PGNs arrive, call AddSample(angleRad, ref, t).
//   3. At flush, call Resolve(&deg, &method). Reset() before the next window.
//
// Internally we keep three priority-tiered circular-mean accumulators, each
// holding bearings already lifted to true. Resolve() returns the highest tier
// that saw any sample — preserving the TS "directionTrue > directionMagnetic >
// angle+heading" precedence without retaining samples.
class SwWindDirection {
 public:
  void SetHeadingTrue(double rad, uint32_t /*t*/) {
    heading_true_rad_ = rad;
    has_heading_true_ = std::isfinite(rad);
  }
  void SetHeadingMagnetic(double rad, uint32_t /*t*/) {
    heading_mag_rad_ = rad;
    has_heading_mag_ = std::isfinite(rad);
  }
  // East-positive magnetic variation (declination), radians.
  void SetVariation(double rad, uint32_t /*t*/) {
    variation_rad_ = rad;
    has_variation_ = std::isfinite(rad);
  }
  // For the v2 apparent-wind vector solve (already parsed from PGN 129026).
  void SetBoatMotion(double sog_ms, double cog_rad, uint32_t /*t*/) {
    sog_ms_ = sog_ms;
    cog_rad_ = cog_rad;
    has_boat_motion_ = std::isfinite(sog_ms) && std::isfinite(cog_rad);
  }

  // Classify one 130306 sample, lift it to a true FROM bearing using the
  // current nav context, and accumulate it into its priority tier. Samples we
  // can't lift (e.g. magnetic with no variation in v1, apparent in v1) are
  // dropped and counted in skipped_.
  void AddSample(double angle_rad, WindRef ref, uint32_t t);

  // Pick the best available tier. Returns false when no usable direction was
  // accumulated this window → caller submits no direction → window skipped.
  bool Resolve(double* out_deg, WindDirMethod* out_method) const;

  void Reset();

  uint32_t skipped() const { return skipped_; }

 private:
  struct VecAccum {
    double sum_sin = 0, sum_cos = 0;
    uint32_t count = 0;
    WindDirMethod method = WindDirMethod::None;  // last method folded into tier
    void Add(double bearing_rad, WindDirMethod m) {
      sum_sin += std::sin(bearing_rad);
      sum_cos += std::cos(bearing_rad);
      count++;
      method = m;
    }
    double MeanDeg() const {
      return WrapDeg(std::atan2(sum_sin, sum_cos) * 180.0 / M_PI);
    }
    void Reset() { sum_sin = sum_cos = 0; count = 0; method = WindDirMethod::None; }
  };

  // v2 fast-follow seam: recover true wind FROM-bearing from an apparent angle
  // + boat motion (SOG/COG). Returns false in v1 (always skipped).
  // bool SolveApparent(double angle_rad, double speed_ms, double* out_bearing_rad) const;

  VecAccum tier_true_;   // ref 0 — directionTrue (highest priority)
  VecAccum tier_mag_;    // ref 1 — directionMagnetic + variation
  VecAccum tier_angle_;  // ref 3/4 — angle off bow + heading

  // Live nav context (latest seen on the bus).
  double heading_true_rad_ = NAN, heading_mag_rad_ = NAN, variation_rad_ = NAN;
  double sog_ms_ = NAN, cog_rad_ = NAN;
  bool has_heading_true_ = false, has_heading_mag_ = false;
  bool has_variation_ = false, has_boat_motion_ = false;

  uint32_t skipped_ = 0;  // samples we couldn't lift (diagnostic)
};

// ── Inline implementation ────────────────────────────────────────────────────

inline void SwWindDirection::AddSample(double angle_rad, WindRef ref,
                                       uint32_t /*t*/) {
  if (!std::isfinite(angle_rad)) return;
  switch (ref) {
    case WindRef::TrueNorth:
      // Already a true FROM bearing (mirror SK directionTrue — see header note).
      tier_true_.Add(angle_rad, WindDirMethod::DirectionTrue);
      return;
    case WindRef::Magnetic:
      if (has_variation_) {
        tier_mag_.Add(angle_rad + variation_rad_,
                      WindDirMethod::DirectionMagneticVariation);
      } else {
        // v1: no published variation → drop. v2: WMM(lat,lon,date) fallback.
        skipped_++;
      }
      return;
    case WindRef::TrueBoat:
    case WindRef::TrueWater:
      if (has_heading_true_) {
        tier_angle_.Add(heading_true_rad_ + angle_rad,
                        WindDirMethod::AngleTrueWaterHeadingTrue);
      } else if (has_heading_mag_ && has_variation_) {
        tier_angle_.Add(heading_mag_rad_ + variation_rad_ + angle_rad,
                        WindDirMethod::AngleTrueWaterHeadingMagVariation);
      } else {
        skipped_++;  // v1: no heading (or mag heading w/o variation) → drop
      }
      return;
    case WindRef::Apparent:
      // v1 SKIP. v2 fast-follow: SolveApparent(angle_rad, speed, ...) using
      // has_boat_motion_ (SOG/COG). See TASK-163.
      skipped_++;
      return;
  }
}

inline bool SwWindDirection::Resolve(double* out_deg,
                                     WindDirMethod* out_method) const {
  const VecAccum* tier = nullptr;
  if (tier_true_.count > 0) {
    tier = &tier_true_;
  } else if (tier_mag_.count > 0) {
    tier = &tier_mag_;
  } else if (tier_angle_.count > 0) {
    tier = &tier_angle_;
  } else {
    return false;
  }
  if (out_deg) *out_deg = tier->MeanDeg();
  if (out_method) *out_method = tier->method;
  return true;
}

inline void SwWindDirection::Reset() {
  tier_true_.Reset();
  tier_mag_.Reset();
  tier_angle_.Reset();
  // Nav context intentionally NOT reset — heading/variation carry across
  // windows (they reflect the boat's current state, not per-window data).
  skipped_ = 0;
}

}  // namespace sailorwind
