#include "aggregator.h"

namespace sources {

void Aggregator::init(BmsSource* bms, ShuntSource* shunt, MpptSource* mppt) {
  m_bms   = bms;
  m_shunt = shunt;
  m_mppt  = mppt;
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
  return select(m, bms_r, shunt_r, mppt_r);
}

// ── Pure selection function ───────────────────────────────────────────────────
// This is the policy table.  Change selection rules here only; never scatter
// priority logic across call sites.
SourceReading Aggregator::select(Metric m,
                                 const SourceReading& bms_r,
                                 const SourceReading& shunt_r,
                                 const SourceReading& mppt_r) {
  switch (m) {
    case Metric::TOTAL_CURRENT:
      // Shunt leads when it has ANY usable data (Valid or Stale).
      // BMS Stale (< 0.5 A) is specifically worse than shunt Stale — the shunt
      // is accurate at low currents while the BMS is not.
      if (shunt_r.is_usable()) return shunt_r;
      return bms_r;

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
