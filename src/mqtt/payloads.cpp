#include "mqtt/payloads.h"
#include <ArduinoJson.h>

namespace mqtt::payloads {

static const char* event_name(SafetyState::SafetyEvent type) {
  switch (type) {
    case SafetyState::SafetyEvent::BmsWentOffline:       return "BmsWentOffline";
    case SafetyState::SafetyEvent::BmsCameOnline:        return "BmsCameOnline";
    case SafetyState::SafetyEvent::PackOvervoltStart:    return "PackOvervoltStart";
    case SafetyState::SafetyEvent::PackOvervoltClear:    return "PackOvervoltClear";
    case SafetyState::SafetyEvent::CellOvervoltStart:    return "CellOvervoltStart";
    case SafetyState::SafetyEvent::CellOvervoltClear:    return "CellOvervoltClear";
    case SafetyState::SafetyEvent::PackUndervoltStart:   return "PackUndervoltStart";
    case SafetyState::SafetyEvent::PackUndervoltClear:   return "PackUndervoltClear";
    case SafetyState::SafetyEvent::TempChargeStop:       return "TempChargeStop";
    case SafetyState::SafetyEvent::TempChargeResume:     return "TempChargeResume";
    case SafetyState::SafetyEvent::TempDischargeStop:    return "TempDischargeStop";
    case SafetyState::SafetyEvent::TempDischargeResume:  return "TempDischargeResume";
    case SafetyState::SafetyEvent::CellImbalanceStart:   return "CellImbalanceStart";
    case SafetyState::SafetyEvent::CellImbalanceClear:   return "CellImbalanceClear";
    case SafetyState::SafetyEvent::BmsReportedAlarm:     return "BmsReportedAlarm";
    case SafetyState::SafetyEvent::NoPacksOnline:        return "NoPacksOnline";
    case SafetyState::SafetyEvent::PacksOnlineRecovered: return "PacksOnlineRecovered";
    default:                                              return "Unknown";
  }
}

size_t build_data(const BmsSystemSnapshot& snap, const SafetyState& safety,
                  uint64_t ts_ms, uint32_t uptime_s,
                  char* out, size_t out_size) {
  JsonDocument doc;

  doc["ts_ms"]                = ts_ms;
  doc["uptime_s"]             = uptime_s;
  doc["bms_count_online"]     = safety.packs_online;
  doc["bms_count_configured"] = safety.packs_configured;
  doc["soc_avg"]              = safety.soc_avg;
  doc["soh_avg"]              = safety.soh_avg;
  doc["pack_voltage_avg"]     = safety.pack_voltage_avg;
  doc["pack_current_total"]   = safety.pack_current_total;
  doc["pack_power_w"]         = safety.pack_voltage_avg * safety.pack_current_total;
  doc["temp_avg"]             = safety.temp_avg;
  doc["cvl_v"]                = safety.cvl_volts;
  doc["ccl_a"]                = safety.ccl_amps;
  doc["dcl_a"]                = safety.dcl_amps;
  doc["alarm_flags"]          = safety.alarm_flags;
  doc["sys_message"]          = safety.sys_message;

  // Aggregate cell min/max/drift and temp_max across all online packs.
  float cell_min = 0.0f, cell_max = 0.0f, cell_drift = 0.0f, temp_max = 0.0f;
  bool  have_cells = false;
  for (uint8_t i = 0; i < snap.pack_count_configured && i < 16; ++i) {
    const BmsPackSnapshot& p = snap.pack[i];
    if (!p.online) continue;
    if (!have_cells) {
      cell_min   = p.cell_min_v;
      cell_max   = p.cell_max_v;
      cell_drift = p.cell_drift_v;
      temp_max   = p.temp_max_c;
      have_cells = true;
    } else {
      if (p.cell_min_v  < cell_min)  cell_min  = p.cell_min_v;
      if (p.cell_max_v  > cell_max)  cell_max  = p.cell_max_v;
      if (p.cell_drift_v > cell_drift) cell_drift = p.cell_drift_v;
      if (p.temp_max_c  > temp_max)  temp_max  = p.temp_max_c;
    }
  }

  if (have_cells) {
    doc["cell_v_min"]   = cell_min;
    doc["cell_v_max"]   = cell_max;
    doc["cell_v_drift"] = cell_drift;
    doc["temp_max"]     = temp_max;
  } else {
    doc["cell_v_min"]   = nullptr;
    doc["cell_v_max"]   = nullptr;
    doc["cell_v_drift"] = nullptr;
    doc["temp_max"]     = nullptr;
  }

  size_t n = serializeJson(doc, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

size_t build_diag(const BmsSystemSnapshot& snap, const SafetyState& safety,
                  const bms::poller::PollerStats& ps, const can::tx::CanStats& cs,
                  uint64_t ts_ms, uint32_t uptime_s,
                  char* out, size_t out_size) {
  (void)snap;
  JsonDocument doc;

  doc["ts_ms"]         = ts_ms;
  doc["uptime_s"]      = uptime_s;
  doc["cycles"]        = ps.cycles_completed;
  doc["analog_ok"]     = ps.analog_polls_ok;
  doc["analog_timeout"]= ps.analog_polls_timeout;
  doc["analog_err"]    = ps.analog_polls_parse_err;
  doc["alarm_ok"]      = ps.alarm_polls_ok;
  doc["alarm_err"]     = ps.alarm_polls_err;
  doc["sysparam_ok"]   = ps.sysparam_polls_ok;
  doc["sysparam_err"]  = ps.sysparam_polls_err;
  doc["cycle_avg_ms"]  = ps.cycle_avg_ms;
  doc["cycle_max_ms"]  = ps.cycle_max_ms;
  doc["can_tx_ok"]     = cs.tx_ok;
  doc["can_tx_fail"]   = cs.tx_fail;
  doc["can_bus_off"]   = cs.bus_off_count;
  doc["can_restarts"]  = cs.driver_restart_count;
  doc["alarm_flags"]   = safety.alarm_flags;
  doc["packs_online"]  = safety.packs_online;

  size_t n = serializeJson(doc, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

size_t build_cells(const BmsPackSnapshot& pack, uint64_t ts_ms,
                   char* out, size_t out_size) {
  JsonDocument doc;

  doc["ts_ms"]      = ts_ms;
  doc["bms_id"]     = pack.bms_id;
  doc["online"]     = pack.online;
  doc["cell_count"] = pack.cell_count;

  JsonArray cells = doc["cells_v"].to<JsonArray>();
  for (uint8_t i = 0; i < pack.cell_count && i < 16; ++i) {
    cells.add(pack.cell_v[i]);
  }

  doc["temp_count"] = pack.temp_count;
  JsonArray temps = doc["temps_c"].to<JsonArray>();
  for (uint8_t i = 0; i < pack.temp_count && i < 8; ++i) {
    temps.add(pack.temp_c[i]);
  }

  doc["cell_v_min"]   = pack.cell_min_v;
  doc["cell_v_max"]   = pack.cell_max_v;
  doc["cell_v_drift"] = pack.cell_drift_v;
  doc["alarm_bits"]   = pack.alarm_bits;

  size_t n = serializeJson(doc, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

size_t build_alarm_event(const SafetyState::EventEntry& evt,
                          uint64_t ts_ms, char* out, size_t out_size) {
  JsonDocument doc;

  doc["ts_ms"]      = ts_ms;
  doc["event"]      = event_name(evt.type);
  doc["bms_id"]     = evt.bms_id;
  doc["alarm_bits"] = evt.alarm_bits;

  size_t n = serializeJson(doc, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

}  // namespace mqtt::payloads
