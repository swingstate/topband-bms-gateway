#include "energy_integrator.h"
#include "storage/energy_store.h"

namespace bms::energy_integrator {

void integrate(float pack_power_w, float dt_s,
               uint32_t now_unix_s, int8_t tz_offset_h) {
  // dt sanity: ignore negative or implausibly large deltas.
  if (dt_s <= 0.0f || dt_s > 60.0f) return;

  // Power in W × dt in hours → kWh (signed: + = in, - = out).
  float delta_kwh = pack_power_w * (dt_s / 3600.0f) / 1000.0f;
  storage::energy_store::accumulate(delta_kwh);
  storage::energy_store::check_daily_rollover(now_unix_s, tz_offset_h);
}

}  // namespace bms::energy_integrator
