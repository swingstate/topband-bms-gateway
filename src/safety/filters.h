#pragma once
#include <cstdint>

// ── Critical alarm bit filter ─────────────────────────────────────────────────
// Re-implementation of V2.67's tbFilterCriticalAlarmBits() (lines 1409-1426).
// Suppresses implausible BMS alarm bits based on measured pack state and
// sysparam sanity floors.  Host-portable: no IDF, no FreeRTOS.

namespace safety::filters {

// Mask of bits that the filter considers "critical". Other bits are stripped
// first; the caller only needs to check the returned value != 0.
// V2.67 line 1367: TB_CRITICAL_ALARM_MASK
constexpr uint64_t CRITICAL_ALARM_MASK =
    (1ULL << 0)  | (1ULL << 2)  | (1ULL << 4)  | (1ULL << 5)  |
    (1ULL << 6)  | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) |
    (1ULL << 50) | (1ULL << 51) | (1ULL << 52) | (1ULL << 53) |
    (1ULL << 54) | (1ULL << 55) | (1ULL << 58) | (1ULL << 59);

// Bit positions for undervolt alarm bits (V2.67 line 1411)
constexpr uint64_t UV_ALARM_BITS = (1ULL << 12) | (1ULL << 13);

// Bit positions for overvolt alarm bits (V2.67 line 1415)
constexpr uint64_t OV_ALARM_BITS = (1ULL << 0) | (1ULL << 58);

// Bit positions for temperature alarm bits (V2.67 line 1420)
constexpr uint64_t TEMP_ALARM_BITS = (1ULL << 5) | (1ULL << 6) | (1ULL << 59);

// ── Helper functions (V2.67 lines 1373-1407) ─────────────────────────────────

// Per V2.67 tbUnderVoltSanityCap: returns an implausibility cap for pack UV.
// If pack_v exceeds this cap, under-volt alarm bits are suppressed.
float under_volt_sanity_cap(int cells);

// Returns true if the pack should be flagged for proto-undervolt based on
// the sysparam module_under_v threshold. V2.67 tbShouldFlagProtoUnderVolt.
bool should_flag_proto_under_volt(float pack_v, float module_under_v, int cells);

// V2.67 tbOverVoltCellSanityFloor: minimum plausible cell voltage floor.
// OV cell alarm bits suppressed when max_cell_v is below this.
float over_volt_cell_sanity_floor(float cell_high_v);

// V2.67 tbOverVoltPackSanityFloor: minimum plausible pack voltage floor.
// OV pack alarm bits suppressed when pack_v is below this.
float over_volt_pack_sanity_floor(float module_high_v, int cells);

// ── Main filter ───────────────────────────────────────────────────────────────
// V2.67 tbFilterCriticalAlarmBits (lines 1409-1426).
// Applies CRITICAL_ALARM_MASK, then suppresses UV/OV/temp bits that are
// implausible given the current measured pack state and sysparam floors.
//
// Parameters:
//   raw_bits    — alarm_bits from the parsed 0x44 response
//   pack_v      — measured pack voltage
//   max_temp_c  — measured max cell temperature
//   cells       — cell count (0 = unknown)
//   temp_limit_c — min(charge_temp_max, discharge_temp_max) from Config
//   max_cell_v  — measured max cell voltage
//   cell_high_v — sys_cell_high_v from sysparam (0 if unknown)
//   module_high_v — sys_module_high_v from sysparam (0 if unknown)
uint64_t filter_alarm_bits(uint64_t raw_bits,
                            float pack_v,
                            float max_temp_c,
                            int   cells,
                            float temp_limit_c,
                            float max_cell_v,
                            float cell_high_v,
                            float module_high_v);

}  // namespace safety::filters
