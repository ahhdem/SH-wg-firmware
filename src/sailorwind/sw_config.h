#pragma once
//
// sw_config.h — SensESP web-UI configuration for the sailorwind submitter.
//
// Mirrors the existing main.cpp pattern: ui_controls.h Configurable subclasses
// (CheckboxConfig / StringConfig), constructed in Begin() and persisted to
// SPIFFS by SensESP. Compiles on the ESP32 target only (SensESP + ui_controls.h).
//
// Config-node spec (config_path → UI title, default, sort_order):
//   /sailorwind/enable          "Enable sailorwind submission"   false   300
//   /sailorwind/server_url      "sailorwind API base URL"        ""(*)   305
//   /sailorwind/token           "Device token (slw_dev_…)"       ""      310
//   /sailorwind/publish_precise "Publish my precise position"    false   315
//   /sailorwind/field/wind_speed  …/wind_dir …/pressure …/air_temp …/water_temp …/humidity  320..325
//   /sailorwind/update_channel  "Firmware channel (stable/beta)" ""(*)   330
//   /sailorwind/auto_apply      "Auto-apply firmware updates"    false   335
//   (*) blank means use the built-in default (serverUrl→https://sailorwind.net,
//       channel→stable) — applied by the accessor, since SensESP doesn't
//       re-apply schema defaults to a blanked field.
//
// Status read-outs (UILambdaOutput, populated by the submitter/provisioner):
//   claim code + claim state, last-submit age, queue depth.

#include <Arduino.h>

#include "../ui_controls.h"  // CheckboxConfig, StringConfig
#include "sw_json.h"         // SwFieldMask, SwField

namespace sailorwind {

class SwConfig {
 public:
  // Construct + load all config nodes from SPIFFS. Call once after the SensESP
  // app is initialized (config nodes load_configuration() in their ctors).
  void Begin();

  bool enabled() const { return enable_ && enable_->get_value(); }

  String serverUrl() const {
    if (!server_url_) return "https://sailorwind.net";
    const String s = server_url_->get_value();
    return s.length() ? s : "https://sailorwind.net";
  }

  String apiToken() const { return token_ ? token_->get_value() : ""; }
  bool hasToken() const { return apiToken().length() > 0; }

  bool publishPrecise() const {
    return publish_precise_ && publish_precise_->get_value();
  }

  String updateChannel() const {
    if (!update_channel_) return "stable";
    const String c = update_channel_->get_value();
    return c.length() ? c : "stable";
  }

  bool autoApplyUpdates() const {
    return auto_apply_updates_ && auto_apply_updates_->get_value();
  }

  // Build the field mask the JSON body builder gates on, from the six toggles.
  SwFieldMask fieldMask() const {
    SwFieldMask m;
    if (field_wind_speed_ && field_wind_speed_->get_value()) m.set(SwField::WindSpeed);
    if (field_wind_dir_ && field_wind_dir_->get_value()) m.set(SwField::WindDirection);
    if (field_pressure_ && field_pressure_->get_value()) m.set(SwField::Pressure);
    if (field_air_temp_ && field_air_temp_->get_value()) m.set(SwField::AirTemp);
    if (field_water_temp_ && field_water_temp_->get_value()) m.set(SwField::WaterTemp);
    if (field_humidity_ && field_humidity_->get_value()) m.set(SwField::Humidity);
    return m;
  }

  // Persist a token obtained at self-registration so it survives reboots and is
  // visible/overridable in the UI. NOTE: ui_controls.h StringConfig has no
  // public setter today — implementing this needs either a small
  // `StringConfig::set_value(const String&)` (sets value_ + save_configuration())
  // or a direct SensESP config write. [impl in sw_config.cpp]
  void setApiToken(const String& token);

 private:
  CheckboxConfig* enable_ = nullptr;
  StringConfig* server_url_ = nullptr;
  StringConfig* token_ = nullptr;
  CheckboxConfig* publish_precise_ = nullptr;
  StringConfig* update_channel_ = nullptr;
  CheckboxConfig* auto_apply_updates_ = nullptr;
  CheckboxConfig* field_wind_speed_ = nullptr;
  CheckboxConfig* field_wind_dir_ = nullptr;
  CheckboxConfig* field_pressure_ = nullptr;
  CheckboxConfig* field_air_temp_ = nullptr;
  CheckboxConfig* field_water_temp_ = nullptr;
  CheckboxConfig* field_humidity_ = nullptr;
};

}  // namespace sailorwind
