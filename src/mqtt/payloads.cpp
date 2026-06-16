#include "mqtt/payloads.h"
#include <ArduinoJson.h>

// Module-level JsonDocument pools: reused across calls to eliminate per-call
// DRAM alloc/free cycles. Pool grows to peak size on first use then stays.
// HousekeepingTask is the sole caller of all three functions; no mutex needed.
// Per docs/diag-mqtt-crash-review.md Finding 1.
static JsonDocument s_doc_data;
static JsonDocument s_doc_diag;
static JsonDocument s_doc_cells;

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
  s_doc_data.clear();

  s_doc_data["ts_ms"]                = ts_ms;
  s_doc_data["uptime_s"]             = uptime_s;
  s_doc_data["bms_count_online"]     = safety.packs_online;
  s_doc_data["bms_count_configured"] = safety.packs_configured;
  s_doc_data["soc_avg"]              = safety.soc_avg;
  s_doc_data["soh_avg"]              = safety.soh_avg;
  s_doc_data["pack_voltage_avg"]     = safety.pack_voltage_avg;
  s_doc_data["pack_current_total"]   = safety.pack_current_total;
  s_doc_data["pack_power_w"]         = safety.pack_voltage_avg * safety.pack_current_total;
  s_doc_data["temp_avg"]             = safety.temp_avg;
  s_doc_data["cvl_v"]                = safety.cvl_volts;
  s_doc_data["ccl_a"]                = safety.ccl_amps;
  s_doc_data["dcl_a"]                = safety.dcl_amps;
  s_doc_data["alarm_flags"]          = safety.alarm_flags;
  s_doc_data["sys_message"]          = safety.sys_message;

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
    s_doc_data["cell_v_min"]   = cell_min;
    s_doc_data["cell_v_max"]   = cell_max;
    s_doc_data["cell_v_drift"] = cell_drift;
    s_doc_data["temp_max"]     = temp_max;
  } else {
    s_doc_data["cell_v_min"]   = nullptr;
    s_doc_data["cell_v_max"]   = nullptr;
    s_doc_data["cell_v_drift"] = nullptr;
    s_doc_data["temp_max"]     = nullptr;
  }

  size_t n = serializeJson(s_doc_data, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

size_t build_diag(const BmsSystemSnapshot& snap, const SafetyState& safety,
                  const bms::poller::PollerStats& ps, const can::tx::CanStats& cs,
                  uint64_t ts_ms, uint32_t uptime_s,
                  char* out, size_t out_size) {
  (void)snap;
  s_doc_diag.clear();

  s_doc_diag["ts_ms"]         = ts_ms;
  s_doc_diag["uptime_s"]      = uptime_s;
  s_doc_diag["cycles"]        = ps.cycles_completed;
  s_doc_diag["analog_ok"]     = ps.analog_polls_ok;
  s_doc_diag["analog_timeout"]= ps.analog_polls_timeout;
  s_doc_diag["analog_err"]    = ps.analog_polls_parse_err;
  s_doc_diag["alarm_ok"]      = ps.alarm_polls_ok;
  s_doc_diag["alarm_err"]     = ps.alarm_polls_err;
  s_doc_diag["sysparam_ok"]   = ps.sysparam_polls_ok;
  s_doc_diag["sysparam_err"]  = ps.sysparam_polls_err;
  s_doc_diag["cycle_avg_ms"]  = ps.cycle_avg_ms;
  s_doc_diag["cycle_max_ms"]  = ps.cycle_max_ms;
  s_doc_diag["can_tx_ok"]     = cs.tx_ok;
  s_doc_diag["can_tx_fail"]   = cs.tx_fail;
  s_doc_diag["can_bus_off"]   = cs.bus_off_count;
  s_doc_diag["can_restarts"]  = cs.driver_restart_count;
  s_doc_diag["alarm_flags"]   = safety.alarm_flags;
  s_doc_diag["packs_online"]  = safety.packs_online;

  size_t n = serializeJson(s_doc_diag, out, out_size);
  return (n > 0 && n < out_size) ? n : 0;
}

size_t build_cells(const BmsPackSnapshot& pack, uint64_t ts_ms,
                   char* out, size_t out_size) {
  s_doc_cells.clear();

  s_doc_cells["ts_ms"]      = ts_ms;
  s_doc_cells["bms_id"]     = pack.bms_id;
  s_doc_cells["online"]     = pack.online;
  s_doc_cells["cell_count"] = pack.cell_count;

  JsonArray cells = s_doc_cells["cells_v"].to<JsonArray>();
  for (uint8_t i = 0; i < pack.cell_count && i < 16; ++i) {
    cells.add(pack.cell_v[i]);
  }

  s_doc_cells["temp_count"] = pack.temp_count;
  JsonArray temps = s_doc_cells["temps_c"].to<JsonArray>();
  for (uint8_t i = 0; i < pack.temp_count && i < 8; ++i) {
    temps.add(pack.temp_c[i]);
  }

  s_doc_cells["cell_v_min"]   = pack.cell_min_v;
  s_doc_cells["cell_v_max"]   = pack.cell_max_v;
  s_doc_cells["cell_v_drift"] = pack.cell_drift_v;
  s_doc_cells["alarm_bits"]   = pack.alarm_bits;

  size_t n = serializeJson(s_doc_cells, out, out_size);
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
