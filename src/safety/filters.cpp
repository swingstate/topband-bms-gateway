// TopBand BMS Gateway — safety/filters.cpp
//
// Re-implementation of V2.67's alarm sanity filter (tbFilterCriticalAlarmBits,
// lines 1409-1426) and its helper functions (lines 1373-1407).
// Host-portable: no IDF headers, no FreeRTOS, no Arduino.

#include "safety/filters.h"
#include <cmath>

namespace safety::filters {

float under_volt_sanity_cap(int cells) {
  // V2.67 tbUnderVoltSanityCap (line 1373).
  // Returns implausibility cap: if pack_v exceeds this, UV bits are suppressed.
  if (cells <= 0 || cells > 32) return 57.0f;
  float cap = static_cast<float>(cells) * 3.20f;
  if (cap < 40.0f) cap = 40.0f;
  if (cap > 57.0f) cap = 57.0f;
  return cap;
}

bool should_flag_proto_under_volt(float pack_v, float module_under_v, int cells) {
  // V2.67 tbShouldFlagProtoUnderVolt (line 1381).
  if (module_under_v <= 1.0f) return false;
  if (cells <= 0 || cells > 32) return false;
  float trigger_v   = module_under_v + 0.05f;
  float sanity_cap  = under_volt_sanity_cap(cells);
  if (trigger_v > sanity_cap) return false;
  return pack_v <= trigger_v;
}

float over_volt_cell_sanity_floor(float cell_high_v) {
  // V2.67 tbOverVoltCellSanityFloor (line 1390).
  float floor_v = cell_high_v - 0.12f;
  if (cell_high_v < 3.2f || cell_high_v > 4.5f) floor_v = 3.58f;
  if (floor_v < 3.50f) floor_v = 3.50f;
  if (floor_v > 4.05f) floor_v = 4.05f;
  return floor_v;
}

float over_volt_pack_sanity_floor(float module_high_v, int cells) {
  // V2.67 tbOverVoltPackSanityFloor (line 1398).
  float floor_v = module_high_v - 0.80f;
  if (module_high_v < 30.0f || module_high_v > 70.0f) {
    if (cells > 0 && cells <= 32) floor_v = static_cast<float>(cells) * 3.55f;
    else floor_v = 50.0f;
  }
  if (floor_v < 40.0f) floor_v = 40.0f;
  if (floor_v > 61.0f) floor_v = 61.0f;
  return floor_v;
}

uint64_t filter_alarm_bits(uint64_t raw_bits,
                            float pack_v,
                            float max_temp_c,
                            int   cells,
                            float temp_limit_c,
                            float max_cell_v,
                            float cell_high_v,
                            float module_high_v) {
  // V2.67 tbFilterCriticalAlarmBits (line 1409).
  uint64_t bits = raw_bits & CRITICAL_ALARM_MASK;

  // ── Undervolt bits (bits 12, 13) ─────────────────────────────────────────
  // Suppress if cell count unknown or pack voltage is above the sanity cap
  // (i.e. the pack clearly is not in undervolt territory).
  if (cells <= 0 || cells > 32) {
    bits &= ~UV_ALARM_BITS;
  } else if (pack_v > under_volt_sanity_cap(cells)) {
    bits &= ~UV_ALARM_BITS;
  }

  // ── Overvolt bits (bits 0, 58) ───────────────────────────────────────────
  // Suppress if both max cell voltage and pack voltage are below their
  // respective sanity floors (i.e. the pack is clearly not overvolt).
  {
    float ov_cell_floor = over_volt_cell_sanity_floor(cell_high_v);
    float ov_pack_floor = over_volt_pack_sanity_floor(module_high_v, cells);
    if (max_cell_v < ov_cell_floor && pack_v < ov_pack_floor) {
      bits &= ~OV_ALARM_BITS;
    }
  }

  // ── Temperature bits (bits 5, 6, 59) ────────────────────────────────────
  // Suppress if max temp is comfortably below the trip threshold.
  // V2.67 uses temp_guard = max(temp_limit - 3, 15).
  {
    float temp_guard = temp_limit_c - 3.0f;
    if (temp_guard < 15.0f) temp_guard = 15.0f;
    if (max_temp_c < temp_guard) bits &= ~TEMP_ALARM_BITS;
  }

  return bits;
}

}  // namespace safety::filters
