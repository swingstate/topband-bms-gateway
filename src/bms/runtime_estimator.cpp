#include "runtime_estimator.h"
#include "safety/runSafety.h"
#include <cmath>

// Minimum current threshold below which we call the system "idle".
static constexpr float IDLE_CURRENT_A = 0.5f;

namespace bms::runtime_estimator {

int32_t estimate_min(const SafetyState& safety, RuntimeStateEst& state_out) {
  float current = safety.pack_current_total;
  float soc     = safety.soc_avg;
  float cap_ah  = safety.capacity_total_ah;

  // Guard: need meaningful capacity and SOC.
  if (cap_ah <= 0.0f || soc < 0.0f || soc > 100.0f) {
    state_out = RuntimeStateEst::Idle;
    return -1;
  }

  // Sign convention (V2.67 / architecture): positive current = charging.
  if (current > IDLE_CURRENT_A) {
    // Charging — estimate time to full.
    float needed_ah = (100.0f - soc) * cap_ah / 100.0f;
    float hours     = needed_ah / current;
    state_out       = RuntimeStateEst::UntilFull;
    return (int32_t)(hours * 60.0f + 0.5f);
  }

  if (current < -IDLE_CURRENT_A) {
    // Discharging — estimate time to empty.
    float remain_ah = soc * cap_ah / 100.0f;
    float hours     = remain_ah / fabsf(current);
    state_out       = RuntimeStateEst::UntilEmpty;
    return (int32_t)(hours * 60.0f + 0.5f);
  }

  state_out = RuntimeStateEst::Idle;
  return -1;
}

}  // namespace bms::runtime_estimator
