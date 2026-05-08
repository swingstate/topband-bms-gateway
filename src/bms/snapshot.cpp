#include "bms/snapshot.h"
#include <cstring>
#include <cstdlib>   // for abs() on floats via fabsf
#include <cmath>     // for fabsf

// ── Constants ────────────────────────────────────────────────────────────────
// Hold-last-value window for pack_current (architecture §5.3, V2.66 H1).
// TopBand BMS firmware reports 0A for ~90 s sample windows; we hold the most
// recent non-zero value for up to this window so the inverter sees stable data.
static constexpr uint32_t CURRENT_HOLD_MS = 120'000u;

namespace bms {

void init_pack_snapshot_offline(BmsPackSnapshot& pack, uint8_t bms_id) {
  memset(&pack, 0, sizeof(BmsPackSnapshot));
  pack.bms_id = bms_id;
  pack.online  = false;
  pack.sysparam_valid = false;
}

void fill_from_analog(const bms::protocol::tb_analog_values_fixed_point& parsed,
                      uint32_t now_ms,
                      BmsPackSnapshot& out) {
  out.online       = true;
  out.last_seen_ms = now_ms;

  // Raw analog fields
  out.pack_voltage = parsed.pack_voltage;
  out.soc          = parsed.soc;
  out.soh          = parsed.soh;
  out.cycles       = parsed.cycles;
  out.rem_ah       = parsed.rem_ah;
  out.full_ah      = parsed.full_ah;
  out.cell_count   = parsed.cell_count;
  out.temp_count   = parsed.temp_count;

  // Cell voltages
  uint8_t n = (parsed.cell_count <= 16) ? parsed.cell_count : 16;
  for (uint8_t i = 0; i < n; ++i)  out.cell_v[i] = parsed.cells[i];
  for (uint8_t i = n; i < 16; ++i) out.cell_v[i] = 0.0f;

  // Temperatures (cell temps only; balancer/env/MOS are separate)
  uint8_t cell_temp_count = parsed.temp_count;
  // Last three entries of parsed.temps are balancer/env/MOS when temp_count >= 3.
  uint8_t special = (parsed.temp_count >= 3) ? 3u : 0u;
  uint8_t cell_t  = (parsed.temp_count >= special) ? (parsed.temp_count - special) : 0u;
  // Store all temps in temp_c (including special ones for direct access)
  uint8_t t_total = (cell_temp_count <= 8) ? cell_temp_count : 8;
  for (uint8_t i = 0; i < t_total; ++i)  out.temp_c[i] = parsed.temps[i];
  for (uint8_t i = t_total; i < 8; ++i)  out.temp_c[i] = 0.0f;

  out.balancer_temp_c     = parsed.balancer_temp_c;
  out.environment_temp_c  = parsed.environment_temp_c;
  out.mosfet_temp_c       = parsed.mosfet_temp_c;

  // Derived: cell min/max/avg/drift
  if (n > 0) {
    float v_min = parsed.cells[0];
    float v_max = parsed.cells[0];
    uint8_t min_idx = 0;
    uint8_t max_idx = 0;
    float   sum     = parsed.cells[0];
    for (uint8_t i = 1; i < n; ++i) {
      float v = parsed.cells[i];
      sum += v;
      if (v < v_min) { v_min = v; min_idx = i; }
      if (v > v_max) { v_max = v; max_idx = i; }
    }
    out.cell_min_v   = v_min;
    out.cell_max_v   = v_max;
    out.cell_avg_v   = sum / static_cast<float>(n);
    out.cell_min_idx = min_idx;
    out.cell_max_idx = max_idx;
    out.cell_drift_v = v_max - v_min;
  } else {
    out.cell_min_v   = 0.0f;
    out.cell_max_v   = 0.0f;
    out.cell_avg_v   = 0.0f;
    out.cell_min_idx = 0;
    out.cell_max_idx = 0;
    out.cell_drift_v = 0.0f;
  }

  // Derived: temp max/avg over cell temps only (special sensors excluded)
  if (cell_t > 0) {
    float t_max = parsed.temps[0];
    float t_sum = parsed.temps[0];
    for (uint8_t i = 1; i < cell_t; ++i) {
      float t = parsed.temps[i];
      t_sum += t;
      if (t > t_max) t_max = t;
    }
    out.temp_max_c = t_max;
    out.temp_avg_c = t_sum / static_cast<float>(cell_t);
  } else {
    out.temp_max_c = 0.0f;
    out.temp_avg_c = 0.0f;
  }

  // Hold-last-value for pack_current (V2.66 H1 workaround)
  if (fabsf(parsed.pack_current) > 0.01f) {
    // Non-zero current: update hold value and extend window.
    out.pack_current         = parsed.pack_current;
    out.current_held_value   = parsed.pack_current;
    out.current_held_until_ms = now_ms + CURRENT_HOLD_MS;
    out.current_held          = false;
  } else {
    // BMS reports 0A.
    if (now_ms < out.current_held_until_ms && out.current_held_until_ms != 0) {
      // Still within the hold window: display the last non-zero value.
      out.pack_current = out.current_held_value;
      out.current_held = true;
    } else {
      // Window expired (or never set): show 0.
      out.pack_current = 0.0f;
      out.current_held = false;
    }
  }
}

void fill_from_alarm(const bms::protocol::tb_alarm_info& parsed,
                     uint32_t now_ms,
                     BmsPackSnapshot& inout) {
  inout.last_alarm_ms = now_ms;
  inout.alarm_bits    = parsed.alarm_bits;

  uint8_t nc = (parsed.cell_count <= 16) ? parsed.cell_count : 16;
  for (uint8_t i = 0; i < nc; ++i)  inout.cell_v_alarm[i] = parsed.cell_v_alarm[i];
  for (uint8_t i = nc; i < 16; ++i) inout.cell_v_alarm[i] = 0;

  // cell_t_alarm covers cell temps only (not the balancer/env/MOS sensors)
  uint8_t special    = (parsed.temp_count >= 3) ? 3u : 0u;
  uint8_t cell_tcount = (parsed.temp_count >= special) ? (parsed.temp_count - special) : 0u;
  uint8_t nt = (cell_tcount <= 16) ? cell_tcount : 16;
  for (uint8_t i = 0; i < nt; ++i)  inout.cell_t_alarm[i] = parsed.cell_t_alarm[i];
  for (uint8_t i = nt; i < 16; ++i) inout.cell_t_alarm[i] = 0;

  inout.module_v_alarm    = parsed.module_v_alarm;
  inout.charge_curr_alarm = parsed.charge_curr_alarm;
}

void fill_from_sysparam(const bms::protocol::tb_system_parameter& parsed,
                        uint32_t now_ms,
                        BmsPackSnapshot& inout) {
  inout.last_sysparam_ms   = now_ms;
  inout.sysparam_valid     = true;
  inout.sys_cell_high_v    = parsed.cell_high_v;
  inout.sys_cell_low_v     = parsed.cell_low_v;
  inout.sys_module_high_v  = parsed.module_high_v;
  inout.sys_module_low_v   = parsed.module_low_v;
  inout.sys_module_under_v = parsed.module_under_v;
  inout.sys_charge_high_t  = parsed.charge_high_t;
  inout.sys_charge_low_t   = parsed.charge_low_t;
  inout.sys_discharge_high_t = parsed.discharge_high_t;
  inout.sys_discharge_low_t  = parsed.discharge_low_t;
  inout.sys_charge_max_a     = parsed.charge_max_a;
  inout.sys_discharge_max_a  = parsed.discharge_max_a;
}

bool decay_online_status(BmsPackSnapshot& inout, uint32_t now_ms,
                         uint32_t offline_threshold_ms) {
  if (!inout.online) return false;  // Already offline
  if (inout.last_seen_ms == 0)      return false;  // Never seen

  // Wrap-safe comparison: if now_ms overflowed last_seen_ms, elapsed may look huge.
  uint32_t elapsed = now_ms - inout.last_seen_ms;
  if (elapsed >= offline_threshold_ms) {
    inout.online = false;
    return true;  // Transition: online → offline
  }
  return false;
}

void update_system_aggregates(BmsSystemSnapshot& sys) {
  uint8_t online = 0;
  for (uint8_t i = 0; i < sys.pack_count_configured; ++i) {
    if (sys.pack[i].online) ++online;
  }
  sys.pack_count_online = online;
}

}  // namespace bms
