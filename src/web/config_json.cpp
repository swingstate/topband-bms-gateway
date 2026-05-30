#include "config_json.h"

namespace web {

void config_to_json(const Config& c, JsonDocument& doc) {
  doc["schema_version"]     = c.schema_version;
  doc["board_preset"]       = (uint8_t)c.board_preset;

  JsonObject pins = doc["pins"].to<JsonObject>();
  pins["rs485_tx"]  = c.pins.rs485_tx;
  pins["rs485_rx"]  = c.pins.rs485_rx;
  pins["rs485_dir"] = c.pins.rs485_dir;
  pins["can_tx"]    = c.pins.can_tx;
  pins["can_rx"]    = c.pins.can_rx;
  pins["led"]       = c.pins.led;
  doc["rs485_enabled"] = c.rs485_enabled;

  doc["bms_count"]             = c.bms_count;
  doc["force_cell_count"]      = c.force_cell_count;
  doc["can_protocol"]          = (uint8_t)c.can_protocol;
  doc["can_enabled"]           = c.can_enabled;
  doc["charge_amps_per_pack"]  = c.charge_amps_per_pack;
  doc["discharge_amps_per_pack"] = c.discharge_amps_per_pack;
  doc["cvl_voltage"]           = c.cvl_voltage;
  doc["safe_pack_volt"]        = c.safe_pack_volt;
  doc["safe_cell_volt"]        = c.safe_cell_volt;
  doc["safe_cell_drift"]       = c.safe_cell_drift;
  doc["spike_volt_max"]        = c.spike_volt_max;
  doc["spike_curr_max"]        = c.spike_curr_max;
  doc["spike_soc_max"]         = c.spike_soc_max;
  doc["charge_temp_min"]       = c.charge_temp_min;
  doc["charge_temp_max"]       = c.charge_temp_max;
  doc["discharge_temp_min"]    = c.discharge_temp_min;
  doc["discharge_temp_max"]    = c.discharge_temp_max;
  doc["temp_soft_zone"]        = c.temp_soft_zone;
  doc["temp_mode"]             = (uint8_t)c.temp_mode;
  doc["soc_mode"]              = (uint8_t)c.soc_mode;
  doc["setup_mode"]            = (uint8_t)c.setup_mode;
  doc["auto_from_bms_applied"] = c.auto_from_bms_applied;
  doc["maint_charge_enabled"]  = c.maint_charge_enabled;
  doc["maint_target_voltage"]  = c.maint_target_voltage;
  doc["auto_balance_enabled"]  = c.auto_balance_enabled;
  doc["auto_balance_last_ts"]  = c.auto_balance_last_ts;
  doc["wifi_ssid"]             = c.wifi_ssid;
  doc["ntp_server"]            = c.ntp_server;
  doc["timezone_offset_h"]     = c.timezone_offset_h;
  doc["mqtt_enabled"]          = c.mqtt_enabled;
  doc["mqtt_host"]             = c.mqtt_host;
  doc["mqtt_port"]             = c.mqtt_port;
  doc["mqtt_user"]             = c.mqtt_user;
  doc["mqtt_pass_obf"]         = "";   // always redacted
  doc["mqtt_base_topic"]       = c.mqtt_base_topic;
  doc["mqtt_level"]            = (uint8_t)c.mqtt_level;
  doc["mqtt_diag_enabled"]     = c.mqtt_diag_enabled;
  doc["ha_discovery_enabled"]  = c.ha_discovery_enabled;
  doc["mqtt_full_publish"]     = c.mqtt_full_publish;
  doc["auth_enabled"]          = c.auth_enabled;
  doc["auth_user"]             = c.auth_user;
  doc["auth_hash"]             = "";   // always redacted
  doc["theme_id"]              = c.theme_id;
  doc["chart_series_a"]        = c.chart_series_a;
  doc["chart_series_b"]        = c.chart_series_b;
  doc["ui_poll_live_ms"]       = c.ui_poll_live_ms;
  doc["ui_poll_diag_ms"]       = c.ui_poll_diag_ms;
  doc["ui_poll_alerts_ms"]     = c.ui_poll_alerts_ms;
  doc["last_reset_ts"]         = c.last_reset_ts;
  doc["serial_debug_enabled"]  = c.serial_debug_enabled;
  doc["spy_persist_default"]   = c.spy_persist_default;
}

// Helper: safely copy a JSON string field into a fixed char array.
template<size_t N>
static void copy_str(const JsonVariantConst& v, char (&dst)[N]) {
  if (v.is<const char*>()) {
    snprintf(dst, N, "%s", v.as<const char*>());
  }
}

bool json_to_config(const JsonDocument& doc, Config& c) {
  // board / pins
  if (doc["board_preset"].is<uint8_t>()) {
    uint8_t raw = doc["board_preset"].as<uint8_t>();
    // Guard: only accept known enum values (Waveshare=0, Manual=1).
    // Values 2+ (old "Custom") are silently mapped to Manual.
    c.board_preset = (raw == 0) ? Config::BoardPreset::Waveshare
                                : Config::BoardPreset::Manual;
  }
  if (doc["rs485_enabled"].is<bool>())
    c.rs485_enabled = doc["rs485_enabled"];
  if (doc["pins"].is<JsonObjectConst>()) {
    JsonObjectConst p = doc["pins"].as<JsonObjectConst>();
    if (p["rs485_tx"].is<int8_t>())  c.pins.rs485_tx  = p["rs485_tx"];
    if (p["rs485_rx"].is<int8_t>())  c.pins.rs485_rx  = p["rs485_rx"];
    if (p["rs485_dir"].is<int8_t>()) c.pins.rs485_dir = p["rs485_dir"];
    if (p["can_tx"].is<int8_t>())    c.pins.can_tx    = p["can_tx"];
    if (p["can_rx"].is<int8_t>())    c.pins.can_rx    = p["can_rx"];
    if (p["led"].is<int8_t>())       c.pins.led       = p["led"];
  }
  if (doc["bms_count"].is<uint8_t>())
    c.bms_count = doc["bms_count"];
  if (doc["force_cell_count"].is<uint8_t>())
    c.force_cell_count = doc["force_cell_count"];
  if (doc["can_protocol"].is<uint8_t>())
    c.can_protocol = (Config::CanProtocol)doc["can_protocol"].as<uint8_t>();
  if (doc["can_enabled"].is<bool>())
    c.can_enabled = doc["can_enabled"];
  if (doc["charge_amps_per_pack"].is<float>())
    c.charge_amps_per_pack = doc["charge_amps_per_pack"];
  if (doc["discharge_amps_per_pack"].is<float>())
    c.discharge_amps_per_pack = doc["discharge_amps_per_pack"];
  if (doc["cvl_voltage"].is<float>())
    c.cvl_voltage = doc["cvl_voltage"];
  if (doc["safe_pack_volt"].is<float>())
    c.safe_pack_volt = doc["safe_pack_volt"];
  if (doc["safe_cell_volt"].is<float>())
    c.safe_cell_volt = doc["safe_cell_volt"];
  if (doc["safe_cell_drift"].is<float>())
    c.safe_cell_drift = doc["safe_cell_drift"];
  if (doc["spike_volt_max"].is<float>())
    c.spike_volt_max = doc["spike_volt_max"];
  if (doc["spike_curr_max"].is<float>())
    c.spike_curr_max = doc["spike_curr_max"];
  if (doc["spike_soc_max"].is<uint8_t>())
    c.spike_soc_max = doc["spike_soc_max"];
  if (doc["charge_temp_min"].is<float>())
    c.charge_temp_min = doc["charge_temp_min"];
  if (doc["charge_temp_max"].is<float>())
    c.charge_temp_max = doc["charge_temp_max"];
  if (doc["discharge_temp_min"].is<float>())
    c.discharge_temp_min = doc["discharge_temp_min"];
  if (doc["discharge_temp_max"].is<float>())
    c.discharge_temp_max = doc["discharge_temp_max"];
  if (doc["temp_soft_zone"].is<float>())
    c.temp_soft_zone = doc["temp_soft_zone"];
  if (doc["temp_mode"].is<uint8_t>())
    c.temp_mode = (Config::TempMode)doc["temp_mode"].as<uint8_t>();
  if (doc["soc_mode"].is<uint8_t>())
    c.soc_mode = (Config::SocMode)doc["soc_mode"].as<uint8_t>();
  if (doc["setup_mode"].is<uint8_t>())
    c.setup_mode = (Config::SetupMode)doc["setup_mode"].as<uint8_t>();
  if (doc["auto_from_bms_applied"].is<bool>())
    c.auto_from_bms_applied = doc["auto_from_bms_applied"];
  if (doc["maint_charge_enabled"].is<bool>())
    c.maint_charge_enabled = doc["maint_charge_enabled"];
  if (doc["maint_target_voltage"].is<float>())
    c.maint_target_voltage = doc["maint_target_voltage"];
  if (doc["auto_balance_enabled"].is<bool>())
    c.auto_balance_enabled = doc["auto_balance_enabled"];
  copy_str(doc["wifi_ssid"], c.wifi_ssid);
  copy_str(doc["ntp_server"], c.ntp_server);
  if (doc["timezone_offset_h"].is<int8_t>())
    c.timezone_offset_h = doc["timezone_offset_h"];
  if (doc["mqtt_enabled"].is<bool>())
    c.mqtt_enabled = doc["mqtt_enabled"];
  copy_str(doc["mqtt_host"], c.mqtt_host);
  if (doc["mqtt_port"].is<uint16_t>())
    c.mqtt_port = doc["mqtt_port"];
  copy_str(doc["mqtt_user"], c.mqtt_user);
  // mqtt_pass_obf: only update if non-empty (not the "redacted" placeholder)
  if (doc["mqtt_pass_obf"].is<const char*>()) {
    const char* p = doc["mqtt_pass_obf"].as<const char*>();
    if (p && p[0] != '\0') snprintf(c.mqtt_pass_obf, sizeof(c.mqtt_pass_obf), "%s", p);
  }
  copy_str(doc["mqtt_base_topic"], c.mqtt_base_topic);
  if (doc["mqtt_level"].is<uint8_t>())
    c.mqtt_level = (Config::MqttLevel)doc["mqtt_level"].as<uint8_t>();
  if (doc["mqtt_diag_enabled"].is<bool>())
    c.mqtt_diag_enabled = doc["mqtt_diag_enabled"];
  if (doc["ha_discovery_enabled"].is<bool>())
    c.ha_discovery_enabled = doc["ha_discovery_enabled"];
  if (doc["mqtt_full_publish"].is<bool>())
    c.mqtt_full_publish = doc["mqtt_full_publish"];
  if (doc["auth_enabled"].is<bool>())
    c.auth_enabled = doc["auth_enabled"];
  copy_str(doc["auth_user"], c.auth_user);
  // auth_hash: only update if non-empty
  if (doc["auth_hash"].is<const char*>()) {
    const char* h = doc["auth_hash"].as<const char*>();
    if (h && h[0] != '\0') snprintf(c.auth_hash, sizeof(c.auth_hash), "%s", h);
  }
  if (doc["theme_id"].is<uint8_t>())
    c.theme_id = doc["theme_id"];
  if (doc["chart_series_a"].is<uint8_t>())
    c.chart_series_a = doc["chart_series_a"];
  if (doc["chart_series_b"].is<uint8_t>())
    c.chart_series_b = doc["chart_series_b"];
  if (doc["ui_poll_live_ms"].is<uint16_t>())
    c.ui_poll_live_ms = doc["ui_poll_live_ms"];
  if (doc["ui_poll_diag_ms"].is<uint16_t>())
    c.ui_poll_diag_ms = doc["ui_poll_diag_ms"];
  if (doc["ui_poll_alerts_ms"].is<uint16_t>())
    c.ui_poll_alerts_ms = doc["ui_poll_alerts_ms"];
  if (doc["serial_debug_enabled"].is<bool>())
    c.serial_debug_enabled = doc["serial_debug_enabled"];
  if (doc["spy_persist_default"].is<bool>())
    c.spy_persist_default = doc["spy_persist_default"];
  return true;
}

}  // namespace web
