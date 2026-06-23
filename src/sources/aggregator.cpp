#include "aggregator.h"
#include <cmath>

namespace sources {

// BMS dead-zone threshold: at |current| below this level the BMS coulomb counter
// loses accuracy and may report 0. The shunt (if enabled) fills this gap.
// Named constant so the handover point is visible in one place.
static constexpr float SHUNT_LEAD_THRESHOLD_A = 0.5f;

void Aggregator::init(BmsSource* bms, ShuntSource* shunt, MpptSource* mppt) {
  m_bms   = bms;
  m_shunt = shunt;
  m_mppt  = mppt;
}

void Aggregator::set_shunt_mode(Config::ShuntCurrentMode mode) {
  m_shunt_mode = mode;
}

void Aggregator::update() {
  if (m_bms)   m_bms->update();
  if (m_shunt && m_shunt->enabled()) m_shunt->update();
  if (m_mppt  && m_mppt->enabled())  m_mppt->update();
}

SourceReading Aggregator::reading(Metric m) const {
  SourceReading bms_r   = m_bms   ? m_bms->reading(m)   : unavailable_reading();
  SourceReading shunt_r = (m_shunt && m_shunt->enabled()) ? m_shunt->reading(m)
                                                           : unavailable_reading();
  SourceReading mppt_r  = (m_mppt  && m_mppt->enabled())  ? m_mppt->reading(m)
                                                           : unavailable_reading();
  return select(m, bms_r, shunt_r, mppt_r, m_shunt_mode);
}

// ── Pure selection function ───────────────────────────────────────────────────
// This is the policy table.  Change selection rules here only; never scatter
// priority logic across call sites.
SourceReading Aggregator::select(Metric m,
                                 const SourceReading& bms_r,
                                 const SourceReading& shunt_r,
                                 const SourceReading& mppt_r,
                                 Config::ShuntCurrentMode mode) {
  switch (m) {
    case Metric::TOTAL_CURRENT: {
      switch (mode) {
        case Config::ShuntCurrentMode::ShuntLeads:
          // Shunt always leads; BMS is fallback when shunt unavailable.
          return shunt_r.is_usable() ? shunt_r : bms_r;
        case Config::ShuntCurrentMode::BmsLeads:
          // BMS always leads; shunt is display-only supplementary.
          return bms_r;
        default: // Auto
          // BMS leads. Below SHUNT_LEAD_THRESHOLD_A the BMS coulomb counter
          // is inaccurate (reports 0 or near-zero); shunt fills the gap.
          float bms_abs = bms_r.is_usable() ? fabsf(bms_r.value) : 0.0f;
          if (bms_abs < SHUNT_LEAD_THRESHOLD_A && shunt_r.is_usable()) return shunt_r;
          return bms_r;
      }
    }

    case Metric::TOTAL_VOLTAGE:
      // BMS leads; per-pack voltage is the primary control input.
      // Shunt voltage is secondary/reference only.
      return bms_r;

    case Metric::PV_POWER:
    case Metric::PV_CURRENT:
    case Metric::PV_VOLTAGE:
      // MPPT is the only source for PV data.
      return mppt_r;

    case Metric::SHUNT_SOC:
      // Shunt SOC — supplementary only, never overrides BMS SOC.
      // Returns Unavailable when shunt is not enabled.
      return shunt_r;

    default:
      return unavailable_reading();
  }
}

}  // namespace sources
