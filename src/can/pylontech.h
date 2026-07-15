#pragma once
#include <cstdint>
#include "safety_state.h"

// Pylontech Low-Voltage BMS CAN protocol frame builders (architecture §4.5).
// Pure functions — no IDF dependencies, fully host-portable.
//
// Frame layout sources:
//   0x351, 0x355, 0x356, 0x35E — byte-identical to V2.67 sendVictronCAN()
//     (legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350) with Pylontech
//     manufacturer string "PYLON   " (V2.67 line 2347).
//   0x359, 0x35C — spec-derived from Pylontech LV BMS CAN Protocol v1.1
//     community documentation. Not present in V2.67. Flagged in each builder.
//
// Validated by test/host/test_can_pylontech.cpp against computed fixtures.

namespace can::pylontech {

// 0x351 — CVL / CCL / DCL / DVL limits
// cvl×10 LE bytes 0-1, ccl×10 LE bytes 2-3, dcl×10 LE bytes 4-5,
// dvl×10 LE bytes 6-7 (discharge voltage limit = configured pack low-voltage cutoff)
void build_0x351(const SafetyState& state, uint8_t out[8]);

// 0x355 — SOC / SOH / Total Capacity
// SOC-int LE bytes 0-1 (fused Combined SOC via can_tx_soc()), SOH-int LE bytes 2-3
// (BMS-only), capacity×10 LE bytes 4-5, bytes 6-7 zero
void build_0x355(const SafetyState& state, uint8_t out[8]);

// 0x356 — Pack Voltage / Pack Current / Average Temperature (same encoding as Victron)
// voltage×100 LE bytes 0-1, current×10 signed LE bytes 2-3, temp×10 LE bytes 4-5, bytes 6-7 zero
void build_0x356(const SafetyState& state, uint8_t out[8]);

// 0x359 — Alarm / warning / status (standard Pylontech LV layout, LSB0 bits,
// verified against OpenDTU-onBattery's Pylontech provider)
// Byte 0 alarms:  bit1 over-volt, bit2 under-volt, bit3 over-temp,
//                 bit4 under-temp, bit7 discharge over-current
// Byte 1 alarms:  bit0 charge over-current, bit3 BMS internal / system error
// Bytes 2-3:      warning-level mirror (unused — no warning thresholds), zero
// Byte 4:         battery module count = packs online (OpenDTU "Module Count")
// Bytes 5-7:      zero
void build_0x359(const SafetyState& state, uint8_t out[8]);

// 0x35C — Charge / discharge enable request (spec-derived, Pylontech LV BMS CAN Protocol v1.1)
// Byte 0 bit 7 = charge enable, bit 6 = discharge enable, bit 5 = request force charge (undervolt)
// Bytes 1-7: zero
// (V3.2: bit6/bit5 were previously swapped — see build_0x35C() in pylontech.cpp.)
void build_0x35C(const SafetyState& state, uint8_t out[8]);

// 0x35E — Manufacturer ID
// Always "PYLON   " (8 ASCII bytes, trailing spaces) per V2.67 line 2347.
void build_0x35E(uint8_t out[8]);

// Enqueue all 6 Pylontech frames (0x351 → 0x355 → 0x356 → 0x359 → 0x35C → 0x35E) in order.
// Calls can::tx::enqueue() for each frame. Returns true iff all 6 enqueues succeed.
bool send_all_pylontech(const SafetyState& state);

}  // namespace can::pylontech
