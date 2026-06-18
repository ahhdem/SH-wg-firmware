// Native (host) unit tests for the pure data-plane headers — no Arduino, no
// hardware. Mirrors the TS fixtures in clients/shared/src/__tests__/
// (aggregator.test.ts, wind-direction.test.ts) so the C++ port provably agrees
// with the SignalK plugin's semantics.
//
// Build + run (no PlatformIO needed):
//   g++ -std=c++17 -I ../../src/sailorwind test_dataplane.cpp -o /tmp/sw_test && /tmp/sw_test
//
// This is the partial of TASK-164 we can run without the ESP32 toolchain.

#include <cmath>
#include <cstdio>

#include "sw_aggregator.h"
#include "sw_wind_direction.h"

using namespace sailorwind;

static int g_failures = 0;
static int g_checks = 0;

static void check(bool cond, const char* what) {
  g_checks++;
  if (!cond) {
    g_failures++;
    std::printf("  FAIL: %s\n", what);
  }
}
static void close_to(double got, double want, double tol, const char* what) {
  g_checks++;
  if (std::fabs(got - want) > tol) {
    g_failures++;
    std::printf("  FAIL: %s (got %.6f, want %.6f, tol %g)\n", what, got, want, tol);
  }
}
static double d2r(double deg) { return deg * M_PI / 180.0; }

// ── wind-direction resolver (mirrors wind-direction.test.ts) ──────────────────
static void test_wind_direction() {
  std::printf("wind-direction resolver:\n");
  double deg;
  WindDirMethod m;

  // 1. directionTrue used directly, ignores a magnetic decoy (strict priority).
  {
    SwWindDirection wd;
    wd.SetVariation(d2r(0), 0);
    wd.AddSample(d2r(200), WindRef::Magnetic, 0);   // decoy
    wd.AddSample(d2r(120), WindRef::TrueNorth, 0);
    wd.AddSample(d2r(124), WindRef::TrueNorth, 0);
    check(wd.Resolve(&deg, &m), "1 resolves");
    check(m == WindDirMethod::DirectionTrue, "1 method=directionTrue");
    close_to(deg, 122, 0.1, "1 windFromDeg=122");
  }
  // 2. magnetic + published variation (+10) -> true 210.
  {
    SwWindDirection wd;
    wd.SetVariation(d2r(10), 0);
    wd.AddSample(d2r(200), WindRef::Magnetic, 0);
    check(wd.Resolve(&deg, &m), "2 resolves");
    check(m == WindDirMethod::DirectionMagneticVariation, "2 method");
    close_to(deg, 210, 1e-3, "2 windFromDeg=210");
  }
  // 4. true wind angle (+40) + true heading (30) -> wind FROM 070.
  {
    SwWindDirection wd;
    wd.SetHeadingTrue(d2r(30), 0);
    wd.AddSample(d2r(40), WindRef::TrueWater, 0);
    check(wd.Resolve(&deg, &m), "4 resolves");
    check(m == WindDirMethod::AngleTrueWaterHeadingTrue, "4 method");
    close_to(deg, 70, 1e-3, "4 windFromDeg=70");
  }
  // 5. angle (+40) + magnetic heading (30) + variation (+10) -> 080.
  {
    SwWindDirection wd;
    wd.SetHeadingMagnetic(d2r(30), 0);
    wd.SetVariation(d2r(10), 0);
    wd.AddSample(d2r(40), WindRef::TrueWater, 0);
    check(wd.Resolve(&deg, &m), "5 resolves");
    check(m == WindDirMethod::AngleTrueWaterHeadingMagVariation, "5 method");
    close_to(deg, 80, 1e-3, "5 windFromDeg=80");
  }
  // priority: magnetic wins over the angle path.
  {
    SwWindDirection wd;
    wd.SetVariation(d2r(0), 0);
    wd.SetHeadingTrue(d2r(30), 0);
    wd.AddSample(d2r(200), WindRef::Magnetic, 0);
    wd.AddSample(d2r(40), WindRef::TrueWater, 0);
    check(wd.Resolve(&deg, &m), "priority resolves");
    check(m == WindDirMethod::DirectionMagneticVariation, "priority method=magnetic");
    close_to(deg, 200, 1e-3, "priority windFromDeg=200");
  }
  // undefined: an angle with no heading to anchor it.
  {
    SwWindDirection wd;
    wd.AddSample(d2r(40), WindRef::TrueWater, 0);
    check(!wd.Resolve(&deg, &m), "angle-no-heading -> unresolved");
    check(wd.skipped() == 1, "angle-no-heading skipped++");
  }
  // undefined: nothing at all.
  {
    SwWindDirection wd;
    check(!wd.Resolve(&deg, &m), "empty -> unresolved");
  }
  // apparent (ref 2) skipped in v1.
  {
    SwWindDirection wd;
    wd.SetHeadingTrue(d2r(30), 0);
    wd.AddSample(d2r(40), WindRef::Apparent, 0);
    check(!wd.Resolve(&deg, &m), "apparent -> unresolved (v1 skip)");
    check(wd.skipped() == 1, "apparent skipped++");
  }
  // circular mean across the 0/360 wrap: 350 & 10 -> 0, not 180.
  {
    SwWindDirection wd;
    wd.AddSample(d2r(350), WindRef::TrueNorth, 0);
    wd.AddSample(d2r(10), WindRef::TrueNorth, 0);
    check(wd.Resolve(&deg, &m), "wrap resolves");
    close_to(deg, 0, 1e-6, "wrap mean(350,10)=0");
  }
}

// ── aggregator (mirrors aggregator.ts semantics) ──────────────────────────────
static void test_aggregator() {
  std::printf("aggregator:\n");

  // empty until a position arrives.
  {
    SwAggregator agg;
    check(agg.IsEmpty(), "empty before position");
    SwObservation o;
    check(!agg.Flush(&o), "flush fails with no position");
  }

  // streaming mean/std/peak/histogram + observedAt = latest weather sample.
  {
    SwAggregator agg;
    agg.AddPosition(10.0, -60.0, 1000);
    check(!agg.IsEmpty(), "not empty after position");
    agg.AddWindSpeed(5.0, 2000);
    agg.AddWindSpeed(7.0, 3000);
    agg.AddWindSpeed(9.0, 4000);
    SwObservation o;
    check(agg.Flush(&o), "flush ok");
    close_to(o.lat_deg, 10.0, 1e-9, "lat");
    close_to(o.lon_deg, -60.0, 1e-9, "lon");
    check(o.observed_at_ms == 4000, "observedAt = latest weather sample");
    close_to(o.wind_speed_ms, 7.0, 1e-6, "mean=7");
    close_to(o.wind_gust_ms, 9.0, 1e-6, "gust(peak)=9");
    // population std of [5,7,9]: sqrt(((4+0+4)/3)) = 1.632993
    close_to(o.wind_speed_std_ms, 1.632993, 1e-4, "population std");
    check(o.has_histogram, "histogram present");
    check(o.wind_speed_histogram[5] == 1 && o.wind_speed_histogram[7] == 1 &&
              o.wind_speed_histogram[9] == 1,
          "histogram bins 5/7/9 each = 1");
    // span (2 s) < 10 s window -> peakSustained falls back to overall mean (7).
    close_to(o.wind_gust_10s_ms, 7.0, 1e-6, "gust10s fallback = overall mean");
  }

  // histogram top-bin clamp: 39.5 and 50 both land in bin 39.
  {
    SwAggregator agg;
    agg.AddPosition(0, 0, 100);
    agg.AddWindSpeed(0.5, 100);
    agg.AddWindSpeed(39.5, 100);
    agg.AddWindSpeed(50.0, 100);
    SwObservation o;
    agg.Flush(&o);
    check(o.wind_speed_histogram[0] == 1, "bin0=1");
    check(o.wind_speed_histogram[39] == 2, "bin39 clamps [39,inf) = 2");
  }

  // a fully-supported sustained window: constant 10 m/s over >10 s -> 10.
  {
    SwAggregator agg;
    agg.AddPosition(0, 0, 0);
    for (int i = 0; i <= 20; i++) agg.AddWindSpeed(10.0, i * 1000);  // 0..20 s
    SwObservation o;
    agg.Flush(&o);
    close_to(o.wind_gust_10s_ms, 10.0, 1e-6, "gust10s sustained = 10");
    close_to(o.wind_gust_60s_ms, 10.0, 1e-6, "gust60s fallback (span<60s) = 10");
  }

  // scalar conversions: Pa->hPa, K->C, humidity passthrough.
  {
    SwAggregator agg;
    agg.AddPosition(0, 0, 0);
    agg.AddPressurePa(101325.0, 0);
    agg.AddAirTempK(293.15, 0);
    agg.AddWaterTempK(283.15, 0);
    agg.AddHumidityPct(64.0, 0);
    SwObservation o;
    agg.Flush(&o);
    close_to(o.pressure_hpa, 1013.25, 1e-3, "Pa->hPa");
    close_to(o.air_temp_c, 20.0, 1e-3, "K->C air");
    close_to(o.sst_c, 10.0, 1e-3, "K->C water");
    close_to(o.rel_humidity_pct, 64.0, 1e-6, "humidity passthrough");
  }

  // reset clears the window (but Resolve nav-context persistence is separate).
  {
    SwAggregator agg;
    agg.AddPosition(1, 2, 5);
    agg.Reset();
    check(agg.IsEmpty(), "empty after reset");
  }
}

int main() {
  test_wind_direction();
  test_aggregator();
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
