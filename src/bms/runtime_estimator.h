#pragma once
#include <cstdint>

// Forward-declare to avoid pulling in large headers.
struct SafetyState;

// ── runtime_estimator — SOC + current → time remaining ───────────────────────
// Pure function (no state, no I/O). Stateless = immune to stale-snapshot bugs.

namespace bms::runtime_estimator {

enum class RuntimeStateEst : uint8_t {
  UntilEmpty = 0,   // discharging; estimate is time to empty
  UntilFull  = 1,   // charging;    estimate is time to full
  Idle       = 2,   // |current| < idle threshold; no meaningful estimate
};

// Compute runtime estimate from aggregated safety state.
// Returns estimated minutes; -1 if idle or data insufficient.
// state_out: set to the direction or idle reason.
int32_t estimate_min(const SafetyState& safety, RuntimeStateEst& state_out);

}  // namespace bms::runtime_estimator
