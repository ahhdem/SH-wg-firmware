#include "sw_config.h"

namespace sailorwind {

// StringConfig's constructor takes String& (lvalues), so each needs local
// String holders; the config object copies them, so the locals can go out of
// scope after construction.
static StringConfig* MakeString(const char* path, const char* desc,
                                const char* initial, int sort_order) {
  String v(initial), p(path), d(desc);
  return new StringConfig(v, p, d, sort_order);
}

void SwConfig::Begin() {
  // Default ON: this is the sailorwind-first firmware (gateway second), so a
  // freshly-flashed unit self-registers and starts submitting as soon as it has
  // WiFi + a clock — no toggle hunt. Users who want gateway-only can untick it.
  enable_ = new CheckboxConfig(true, "Enable sailorwind submission",
                               "/sailorwind/enable",
                               "Read N2K weather and submit it to sailorwind.net",
                               300);
  server_url_ = MakeString("/sailorwind/server_url",
                           "sailorwind API base URL (blank = https://sailorwind.net)",
                           "", 305);
  token_ = MakeString("/sailorwind/token",
                      "Device token (slw_dev_…) — auto-filled after you claim "
                      "the device; treat as a secret",
                      "", 310);
  publish_precise_ = new CheckboxConfig(
      false, "Publish my precise (uncoarsened) position",
      "/sailorwind/publish_precise",
      "Off by default — positions are coarsened to a ~1nm grid for privacy", 315);

  field_wind_speed_ = new CheckboxConfig(true, "Submit wind speed",
                                         "/sailorwind/field/wind_speed", "", 320);
  field_wind_dir_ = new CheckboxConfig(true, "Submit wind direction",
                                       "/sailorwind/field/wind_dir", "", 321);
  field_pressure_ = new CheckboxConfig(false, "Submit barometric pressure",
                                       "/sailorwind/field/pressure", "", 322);
  field_air_temp_ = new CheckboxConfig(false, "Submit air temperature",
                                       "/sailorwind/field/air_temp", "", 323);
  field_water_temp_ = new CheckboxConfig(false, "Submit water temperature",
                                         "/sailorwind/field/water_temp", "", 324);
  field_humidity_ = new CheckboxConfig(false, "Submit humidity",
                                       "/sailorwind/field/humidity", "", 325);

  update_channel_ = MakeString("/sailorwind/update_channel",
                               "Firmware channel: stable or beta", "stable", 330);
  auto_apply_updates_ = new CheckboxConfig(
      false, "Auto-apply firmware updates (else download + notify)",
      "/sailorwind/auto_apply", "", 335);
}

void SwConfig::setApiToken(const String& token) {
  if (token_) token_->set_value(token);
}

}  // namespace sailorwind
