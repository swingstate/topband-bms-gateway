#pragma once
#include <cstdint>
#include "safety_state.h"

// SMA Sunny Island BMS CAN protocol frame builders (architecture §4.5).
// Pure functions — no IDF dependencies, fully host-portable.
//
// Frame layout sources:
//   0x351, 0x355, 0x356 — byte-identical to V2.67 sendVictronCAN()
//     (legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350).
//   0x35A — V2.67 Victron alarm mapping plus SMA-specific charge/discharge
//     enable bits in byte 4 (V2.67 line 2345: g_can_protocol == 2 branch).
//   0x35E — "SMA     " manufacturer string (V2.67 line 2348).
//   0x35B — spec-derived from SMA Sunny Island BMS protocol documentation.
//     Not present in V2.67. Warning bitmap parallel to 0x35A alarm bitmap.
//
// Validated by test/host/test_can_sma.cpp against computed fixtures.

namespace can::sma {

// 0x351 — CVL / CCL / DCL limits (same encoding as Victron)
// cvl×10 LE bytes 0-1, ccl×10 LE bytes 2-3, dcl×10 LE bytes 4-5, bytes 6-7 zero
void build_0x351(const SafetyState& state, uint8_t out[8]);

// 0x355 — SOC / SOH / Total Capacity (same encoding as Victron)
// SOC-int LE bytes 0-1, SOH-int LE bytes 2-3, capacity×10 LE bytes 4-5, bytes 6-7 zero
void build_0x355(const SafetyState& state, uint8_t out[8]);

// 0x356 — Pack Voltage / Pack Current / Average Temperature (same encoding as Victron)
// voltage×100 LE bytes 0-1, current×10 signed LE bytes 2-3, temp×10 LE bytes 4-5, bytes 6-7 zero
void build_0x356(const SafetyState& state, uint8_t out[8]);

// 0x35A — Alarm flags (Victron alarm mapping + SMA charge/discharge enable)
// V2.67 line 2344: byte 4 alarm bits same as Victron.
// V2.67 line 2345: byte 4 bit1 set when CCL<0.1 (charge disabled); bit0 set when DCL<0.1.
void build_0x35A(const SafetyState& state, uint8_t out[8]);

// 0x35B — Warning bitmap (spec-derived, not present in V2.67)
// Softer-threshold parallel to 0x35A; byte 4 carries imbalance / temp warning bits.
void build_0x35B(const SafetyState& state, uint8_t out[8]);

// 0x35E — Manufacturer ID
// Always "SMA     " (8 ASCII bytes, trailing spaces) per V2.67 line 2348.
void build_0x35E(uint8_t out[8]);

// Enqueue all 6 SMA frames (0x351 → 0x355 → 0x356 → 0x35A → 0x35B → 0x35E) in order.
// Calls can::tx::enqueue() for each frame. Returns true iff all 6 enqueues succeed.
bool send_all_sma(const SafetyState& state);

}  // namespace can::sma
