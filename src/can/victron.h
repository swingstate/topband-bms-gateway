#pragma once
#include <cstdint>
#include "safety_state.h"

// Victron CAN frame builders (architecture §4.5).
// Pure functions — no IDF dependencies, fully host-portable.
// Each builder zero-initialises `out` before filling it.
//
// Byte layout is byte-identical to V2.67's sendVictronCAN()
// (legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350).
// Validated by test/host/test_can_victron.cpp against Python-generated fixtures.

namespace can::victron {

// 0x351 — Charge Voltage Limit / Charge Current Limit / Discharge Current Limit
// cvl×10 LE bytes 0-1, ccl×10 LE bytes 2-3, dcl×10 LE bytes 4-5, bytes 6-7 zero
void build_0x351(const SafetyState& state, uint8_t out[8]);

// 0x355 — SOC / SOH / Total Capacity
// SOC-int LE bytes 0-1, SOH-int LE bytes 2-3, capacity×10 LE bytes 4-5, bytes 6-7 zero
void build_0x355(const SafetyState& state, uint8_t out[8]);

// 0x356 — Pack Voltage / Pack Current / Average Temperature
// voltage×100 LE bytes 0-1, current×10 signed LE bytes 2-3, temp×10 LE bytes 4-5, bytes 6-7 zero
void build_0x356(const SafetyState& state, uint8_t out[8]);

// 0x35A — Alarm flags (Victron protocol mapping)
// Only bytes 4 is populated; bytes 0-3 and 5-7 are zero.
void build_0x35A(const SafetyState& state, uint8_t out[8]);

// 0x35E — Manufacturer ID
// Always "TOPBAND " (8 ASCII bytes, trailing space) for Victron protocol.
void build_0x35E(uint8_t out[8]);

// Enqueue all 5 Victron frames (0x351 → 0x355 → 0x356 → 0x35A → 0x35E) in order.
// Calls can::tx::enqueue() for each frame. Returns true iff all 5 enqueues succeed.
bool send_all_victron(const SafetyState& state);

}  // namespace can::victron
