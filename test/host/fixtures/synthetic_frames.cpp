// Synthetic TopBand BMS protocol INFO byte arrays for host testing.
//
// These are the decoded INFO bytes for each response type.  The test helper
// make_response() hex-encodes them and wraps them in a valid wire frame
// (SOI, header, LENID, INFO, CHKSUM, EOI) so parse_response_header works.
//
// For CID2=0x51 (manufacturer info) the INFO bytes are raw ASCII, not binary;
// use make_response_raw() instead of make_response().

#pragma once
#include <cstdint>
#include <cstddef>

namespace synth {

// ── CID2=0x42 analog, 16-cell upper bound ────────────────────────────────────
// All cells at 3.350 V (0x0D16), 8 temps at 25.0°C (0x0B9F=(2975-2731)/10=24.4°C).
// Actually 25°C → raw = 25×10+2731 = 2981 = 0x0BA5.
// pack_current=0, pack_voltage=16×3350×0.01=536.00V (raw 0xD190=53648→536.48V — OK for test).
// rem_ah=full_ah=20.00Ah(0x07D0), skip=0x04, cycles=1, soc=100, soh=100.
inline constexpr uint8_t kInfo_Analog16Cell[] = {
  0x00, 0x10,                                        // dataflag, cell_count=16
  0x0D,0x16, 0x0D,0x16, 0x0D,0x16, 0x0D,0x16,       // cells 0-3
  0x0D,0x16, 0x0D,0x16, 0x0D,0x16, 0x0D,0x16,       // cells 4-7
  0x0D,0x16, 0x0D,0x16, 0x0D,0x16, 0x0D,0x16,       // cells 8-11
  0x0D,0x16, 0x0D,0x16, 0x0D,0x16, 0x0D,0x16,       // cells 12-15
  0x08,                                              // temp_count=8
  0x0B,0xA5, 0x0B,0xA5, 0x0B,0xA5, 0x0B,0xA5,       // temps 0-3
  0x0B,0xA5, 0x0B,0xA5, 0x0B,0xA5, 0x0B,0xA5,       // temps 4-7
  0x00,0x00,                                         // current=0
  0xD1,0x90,                                         // pack_voltage≈536.48V
  0x07,0xD0,                                         // rem_ah=20.00Ah
  0x04,                                              // skip byte
  0x07,0xD0,                                         // full_ah=20.00Ah
  0x00,0x01,                                         // cycles=1
  0x64,                                              // soc=100
  0x64,                                              // soh=100
  0x00,0x00,                                         // padding
};
constexpr size_t kInfo_Analog16Cell_len = sizeof(kInfo_Analog16Cell);

// ── CID2=0x44 alarm, all clear ────────────────────────────────────────────────
// 15 cells, 7 temps, all alarms off, alarm_bits=0.
inline constexpr uint8_t kInfo_AlarmClean[] = {
  0x00, 0x0F,                   // dataflag, cell_count=15
  0x00,0x00,0x00,0x00,0x00,     // cell_v_alarm[0..4]
  0x00,0x00,0x00,0x00,0x00,     // cell_v_alarm[5..9]
  0x00,0x00,0x00,0x00,0x00,     // cell_v_alarm[10..14]
  0x07,                         // temp_count=7
  0x00,0x00,0x00,0x00,          // cell_t_alarm[0..3] (7-3=4 entries)
  0x00, 0x00, 0x00,             // balancer/env/mosfet temp alarm
  0x00, 0x00,                   // charge_curr_alarm, module_v_alarm
  0x08,                         // status_count=8
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // status_bytes (alarm_bits=0)
};
constexpr size_t kInfo_AlarmClean_len = sizeof(kInfo_AlarmClean);

// ── CID2=0x44 alarm, cell 3 over-voltage ─────────────────────────────────────
// cell_v_alarm[2]=0x01 (over-voltage code).
inline constexpr uint8_t kInfo_AlarmOvervolt[] = {
  0x00, 0x0F,
  0x00,0x00,0x01,0x00,0x00,     // cell_v_alarm[0..4]: cell 2 = 0x01
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x07,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,
  0x00,0x00,
  0x08,
  0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // bit 0: CELL_OVER_VOLTAGE_PROTECT
};
constexpr size_t kInfo_AlarmOvervolt_len = sizeof(kInfo_AlarmOvervolt);

// ── CID2=0x44 alarm, temp charge stop ────────────────────────────────────────
// bit 6 (CELL_TEMP_CHARGE_PROTECT) set.
inline constexpr uint8_t kInfo_AlarmTempChargeStop[] = {
  0x00, 0x0F,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x07,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,
  0x00,0x00,
  0x08,
  0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // bit 6: CELL_TEMP_CHARGE_PROTECT
};
constexpr size_t kInfo_AlarmTempChargeStop_len = sizeof(kInfo_AlarmTempChargeStop);

// ── CID2=0x44 alarm, critical combo ──────────────────────────────────────────
// bits 0, 13, 23 set: CELL_OVER_VOLTAGE_PROTECT | PACK_UNDER_VOLTAGE_PROTECT | CHARGING
// alarm_bits = 0x00000000_00802001 → LE bytes: 01 20 80 00 00 00 00 00
inline constexpr uint8_t kInfo_AlarmCritical[] = {
  0x00, 0x0F,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x07,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,
  0x00,0x00,
  0x08,
  0x01,0x20,0x80,0x00,0x00,0x00,0x00,0x00,
};
constexpr size_t kInfo_AlarmCritical_len = sizeof(kInfo_AlarmCritical);

// ── CID2=0x47 system parameters, 15S LFP defaults ────────────────────────────
inline constexpr uint8_t kInfo_Sysparam[] = {
  0x0E,0x42,  // cell_high_v  = 3650 → 3.650 V
  0x09,0xC4,  // cell_low_v   = 2500 → 2.500 V
  0x09,0x60,  // cell_under_v = 2400 → 2.400 V
  0x0C,0x6D,  // charge_high_t = 3181 → 45.0°C
  0x0A,0xAB,  // charge_low_t  = 2731 → 0.0°C
  0x1D,0x4C,  // charge_max_a  = 7500 → 75.00A
  0x15,0x63,  // module_high_v = 5475 → 54.75V
  0x0E,0xA6,  // module_low_v  = 3750 → 37.50V
  0x0E,0x10,  // module_under_v= 3600 → 36.00V
  0x0D,0x03,  // discharge_high_t = 3331 → 60.0°C
  0x09,0xE3,  // discharge_low_t  = 2531 → -20.0°C
  0x3A,0x98,  // discharge_max_a  = 15000 → 150.00A
};
constexpr size_t kInfo_Sysparam_len = sizeof(kInfo_Sysparam);

// ── CID2=0x4D battery date, 2024-01-15 ────────────────────────────────────────
inline constexpr uint8_t kInfo_Date[] = {
  0x07,0xE8,  // year  = 2024
  0x01,       // month = 1
  0x0F,       // day   = 15
  0x00,       // hour  = 0
  0x00,       // minute= 0
  0x00,       // second= 0
};
constexpr size_t kInfo_Date_len = sizeof(kInfo_Date);

// ── CID2=0x51 manufacturer info (raw ASCII, NOT hex-encoded) ─────────────────
// hw[20] = "TopBand             ", sw[4] = "V1.0", id[8] = "TBLT-15S"
inline constexpr uint8_t kInfo_Manufacturer[] = {
  'T','o','p','B','a','n','d',' ',' ',' ',  // hw: "TopBand   " (10)
  ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',  // hw continued (10), total 20
  'V','1','.','0',                           // sw: "V1.0" (4)
  'T','B','L','T','-','1','5','S',           // id: "TBLT-15S" (8)
};
constexpr size_t kInfo_Manufacturer_len = sizeof(kInfo_Manufacturer);

} // namespace synth
