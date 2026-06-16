#pragma once
#include <cstdint>

// BUS-OFF detection and recovery state machine (architecture §3.6).
//
// States: HEALTHY → BUS_OFF_DETECTED → RECOVERING → HEALTHY
// Recovery uses bounded retry with exponential backoff (1 s, 2 s, 4 s, 8 s, cap 30 s).
// HIL-13 verifies: fault → state machine fires → recovery → frames resume, no reboot.

namespace can::busoff {

// Called by tx::enqueue() after each TWAI transmit attempt.
// Feeds consecutive-failure tracking into the state machine.
void on_tx_result(bool tx_succeeded);

// Called from ControlTask every 50 ms tick (same cadence as can_tx_if_due).
// Queries TWAI driver status; if BUS_OFF is detected, initiates recovery sequence.
void tick(uint32_t now_ms);

// Returns true when TWAI driver is in a healthy (RUNNING) state.
// Used by smoke reader and /api/diag to surface CAN health.
bool is_healthy();

#ifdef NATIVE_BUILD
// Test helpers — inject a simulated TWAI state for unit testing.
// state: 0 = RUNNING (healthy), 1 = BUS_OFF, 2 = RECOVERING, 3 = STOPPED
void set_mock_twai_state(int state);
void test_reset();
#endif

}  // namespace can::busoff
