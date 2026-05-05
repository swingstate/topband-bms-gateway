#pragma once
#include <cstdint>

// ── BmsPackSnapshot ─────────────────────────────────────────────────────────
// Per-pack state snapshot. Updated by bms/poller via bms/snapshot fill helpers.
// Architecture §5.3 canonical definition.
struct BmsPackSnapshot {
  uint8_t  bms_id;
  bool     online;               // true if last_seen_ms within offline threshold
  uint32_t last_seen_ms;
  uint32_t last_alarm_ms;
  uint32_t last_sysparam_ms;

  // Analog data (cid2 = 0x42)
  float    pack_voltage;
  float    pack_current;         // A, signed; + = charge (V2.67 convention)
  uint8_t  soc;
  uint8_t  soh;
  uint16_t cycles;
  float    rem_ah;
  float    full_ah;
  uint8_t  cell_count;
  float    cell_v[16];           // V; unused slots = 0.0
  uint8_t  temp_count;
  float    temp_c[8];            // °C
  float    balancer_temp_c;
  float    environment_temp_c;
  float    mosfet_temp_c;

  // Derived fields (computed in bms/snapshot.cpp at parse time)
  float    cell_min_v;
  float    cell_max_v;
  float    cell_avg_v;
  uint8_t  cell_min_idx;
  uint8_t  cell_max_idx;
  float    cell_drift_v;         // cell_max_v - cell_min_v
  float    temp_max_c;
  float    temp_avg_c;

  // Alarm bitmap (cid2 = 0x44)
  uint64_t alarm_bits;
  uint8_t  cell_v_alarm[16];
  uint8_t  cell_t_alarm[16];
  uint8_t  module_v_alarm;
  uint8_t  charge_curr_alarm;

  // System parameters (cid2 = 0x47; valid for ~5 min, check sysparam_valid)
  bool     sysparam_valid;
  float    sys_cell_high_v;
  float    sys_cell_low_v;
  float    sys_module_high_v;
  float    sys_module_low_v;
  float    sys_module_under_v;
  float    sys_charge_high_t;
  float    sys_charge_low_t;
  float    sys_discharge_high_t;
  float    sys_discharge_low_t;
  float    sys_charge_max_a;
  float    sys_discharge_max_a;

  // Hold-last-value workaround (architecture §5.3; V2.66 H1)
  // TopBand firmware reports 0A for ~90 s sample windows; we hold the last
  // non-zero value for up to 120 s so the inverter sees a stable current.
  bool     current_held;
  float    current_held_value;
  uint32_t current_held_until_ms;
};

// ── BmsSystemSnapshot ───────────────────────────────────────────────────────
// Full-system snapshot. Two instances live in PSRAM, managed by the seqlock
// double-buffer in bus/snapshot_bus.cpp. Only ControlTask writes; any Core 1
// task reads via bus::snapshot_bus::read(). Size ~8 KB per slot.
struct BmsSystemSnapshot {
  uint32_t seq;               // seqlock counter: odd = write in progress, even = stable
  uint32_t cycle_id;          // monotonically incremented per publish
  uint32_t produced_ms;       // esp_timer_get_time()/1000 at publish time
  uint8_t  pack_count_configured;
  uint8_t  pack_count_online;
  BmsPackSnapshot pack[16];
};
