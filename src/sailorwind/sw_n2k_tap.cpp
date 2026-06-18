#include "sw_n2k_tap.h"

#include <N2kMessages.h>  // ParseN2k*, tN2k* enums, N2kIsNA — NMEA2000 library

namespace sailorwind {

// PGNs consumed. The first five are already in the firmware's ReceiveMessages[]
// (the gateway/transform path); 130311–130316 are added for atmospherics.
static const unsigned long kWantedPgns[] = {
    127250UL,  // Vessel Heading (+ variation)
    127258UL,  // Magnetic Variation
    129025UL,  // Position, Rapid Update
    129026UL,  // COG & SOG, Rapid Update (v2 apparent-wind solve)
    130306UL,  // Wind Data
    130311UL,  // Environmental Parameters (temp + humidity + pressure)
    130312UL,  // Temperature
    130313UL,  // Humidity
    130314UL,  // Actual Pressure
    130316UL,  // Temperature, Extended Range
    0,
};

const unsigned long* SwN2kTap::WantedPgns() { return kWantedPgns; }

// Route an N2K temperature by its source: outside air vs. sea water. Other
// sources (engine room, cabin, …) are weather-irrelevant and dropped.
static void RouteTemp(SwAggregator* agg, tN2kTempSource src, double tempK,
                      int64_t t) {
  if (src == N2kts_OutsideTemperature) {
    agg->AddAirTempK(tempK, t);
  } else if (src == N2kts_SeaTemperature) {
    agg->AddWaterTempK(tempK, t);
  }
}

void SwN2kTap::HandleMsg(const tN2kMsg& msg) {
  const int64_t t = clock_ ? clock_() : 0;
  if (t == 0) return;  // clock not synced → no trustworthy observedAt → drop

  unsigned char SID;
  switch (msg.PGN) {
    case 129025UL: {  // Position
      double lat, lon;
      if (ParseN2kPGN129025(msg, lat, lon) && !N2kIsNA(lat) && !N2kIsNA(lon)) {
        agg_->AddPosition(lat, lon, t);
      }
      break;
    }
    case 130306UL: {  // Wind: speed (m/s) + angle (rad) + reference
      double windSpeed, windAngle;
      tN2kWindReference ref;
      if (ParseN2kWindSpeed(msg, SID, windSpeed, windAngle, ref)) {
        // Speed-only sensors leave the angle NA; the resolver then simply finds
        // no direction and the window is skipped (never fabricated). The N2K
        // WindReference enum maps 1:1 onto sailorwind::WindRef (same ordering).
        const double spd = N2kIsNA(windSpeed) ? NAN : windSpeed;
        const double ang = N2kIsNA(windAngle) ? NAN : windAngle;
        agg_->AddWind(spd, ang, static_cast<WindRef>(ref), t);
      }
      break;
    }
    case 127250UL: {  // Heading (+ variation)
      double heading, deviation, variation;
      tN2kHeadingReference ref;
      if (ParseN2kHeading(msg, SID, heading, deviation, variation, ref)) {
        if (!N2kIsNA(heading)) {
          if (ref == N2khr_true) {
            agg_->SetHeadingTrue(heading, t);
          } else if (ref == N2khr_magnetic) {
            agg_->SetHeadingMagnetic(heading, t);
          }
        }
        if (!N2kIsNA(variation)) agg_->SetVariation(variation, t);
      }
      break;
    }
    case 127258UL: {  // Magnetic Variation
      double variation;
      tN2kMagneticVariation source;
      uint16_t daysSince1970;
      if (ParseN2kMagneticVariation(msg, SID, source, daysSince1970, variation) &&
          !N2kIsNA(variation)) {
        agg_->SetVariation(variation, t);
      }
      break;
    }
    case 129026UL: {  // COG & SOG (boat motion — v2 apparent-wind seam)
      tN2kHeadingReference ref;
      double cog, sog;
      if (ParseN2kCOGSOGRapid(msg, SID, ref, cog, sog) && !N2kIsNA(cog) &&
          !N2kIsNA(sog)) {
        agg_->SetBoatMotion(sog, cog, t);
      }
      break;
    }
    case 130311UL: {  // Environmental (combined temp + humidity + pressure)
      tN2kTempSource tsrc;
      tN2kHumiditySource hsrc;
      double temp, humidity, pressure;
      if (ParseN2kPGN130311(msg, SID, tsrc, temp, hsrc, humidity, pressure)) {
        if (!N2kIsNA(temp)) RouteTemp(agg_, tsrc, temp, t);
        if (!N2kIsNA(humidity) && hsrc == N2khs_OutsideHumidity) {
          agg_->AddHumidityPct(humidity, t);
        }
        if (!N2kIsNA(pressure)) agg_->AddPressurePa(pressure, t);
      }
      break;
    }
    case 130312UL: {  // Temperature
      unsigned char inst;
      tN2kTempSource src;
      double actual, set;
      if (ParseN2kPGN130312(msg, SID, inst, src, actual, set) &&
          !N2kIsNA(actual)) {
        RouteTemp(agg_, src, actual, t);
      }
      break;
    }
    case 130316UL: {  // Temperature, Extended Range
      unsigned char inst;
      tN2kTempSource src;
      double actual, set;
      if (ParseN2kPGN130316(msg, SID, inst, src, actual, set) &&
          !N2kIsNA(actual)) {
        RouteTemp(agg_, src, actual, t);
      }
      break;
    }
    case 130313UL: {  // Humidity
      unsigned char inst;
      tN2kHumiditySource src;
      double actual, set;
      if (ParseN2kPGN130313(msg, SID, inst, src, actual, set) &&
          !N2kIsNA(actual) && src == N2khs_OutsideHumidity) {
        agg_->AddHumidityPct(actual, t);
      }
      break;
    }
    case 130314UL: {  // Actual Pressure (Pa)
      unsigned char inst;
      tN2kPressureSource src;
      double pressure;
      if (ParseN2kPGN130314(msg, SID, inst, src, pressure) &&
          !N2kIsNA(pressure) && src == N2kps_Atmospheric) {
        agg_->AddPressurePa(pressure, t);
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace sailorwind
