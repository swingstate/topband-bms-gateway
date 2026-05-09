#pragma once
#include <cstdint>
#include "bms_snapshot.h"
#include "safety_state.h"
#include "storage/config.h"

// ── PrevSafetyState ───────────────────────────────────────────────────────────
// Edge-detection state carried between calls to runSafety(). In V2.67 these
// were static locals inside calculateVictronData(); V3.0 makes them explicit
// so the function is pure (same inputs → same output, no hidden state).
//
// Mirrors the V2.67 Python reference PrevState exactly:
//   was_pack_online[16], was_packs_online_any,
//   prev_alarm_flags, prev_factor_charge, prev_factor_discharge.
struct PrevSafetyState {
  bool    was_pack_online[16];
  bool    was_packs_online_any;    // any pack online last cycle
  uint8_t prev_alarm_flags;
  float   prev_factor_charge;
  float   prev_factor_discharge;
};

namespace safety {

// ── runSafety ─────────────────────────────────────────────────────────────────
// Pure safety-decision function. Architecture §4.4, §5.4 contract.
//
// Preconditions:
//   - snap.pack_count_configured <= 16
//   - cfg fields are validated (storage::validate() returned None)
//   - now_ms is injected by caller; never read inside this function via timers
//
// Postconditions:
//   - out fields are fully initialised (no uninitialised reads on caller side)
//   - out contains no I/O side effects; events[] holds state-transition entries
//   - snap and cfg are not modified
//
// Purity guarantees (tested by test_safety.cpp):
//   - No calls to millis(), esp_timer_*, xTaskGetTickCount(), time()
//   - No calls to ESP_LOG, printf, malloc, new, delete
//   - Deterministic: same (snap, cfg, prev, now_ms) → byte-identical out
void runSafety(const BmsSystemSnapshot& snap,
               const Config&            cfg,
               const PrevSafetyState&   prev,
               uint32_t                 now_ms,
               SafetyState&             out);

// ── update_prev_state ─────────────────────────────────────────────────────────
// Advances prev to reflect the just-computed SafetyState and current snapshot.
// Call immediately after runSafety() returns, before the next cycle.
void update_prev_state(const SafetyState&        current,
                       const BmsSystemSnapshot&  snap,
                       PrevSafetyState&          inout);

// ── make_default_prev ─────────────────────────────────────────────────────────
// Returns a PrevSafetyState suitable for the first cycle (no prior history).
PrevSafetyState make_default_prev();

}  // namespace safety
