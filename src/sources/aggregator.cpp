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

void Aggregator::set_soc_mode(Config::SocMode mode) {
  m_soc_mode = mode;
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

// ── Bank-level SOC fusion (V3.2) ──────────────────────────────────────────────
// select_bank_soc() is the policy table for this; keep it here alongside
// select() rather than scattering the rule across call sites.
Aggregator::BankSocReading Aggregator::fuse_bank_soc(float bms_soc_avg) const {
  SourceReading shunt_r = (m_shunt && m_shunt->enabled())
                          ? m_shunt->reading(Metric::SHUNT_SOC)
                          : unavailable_reading("%");
  return select_bank_soc(bms_soc_avg, shunt_r, m_soc_mode);
}

Aggregator::BankSocReading Aggregator::select_bank_soc(float bms_soc_avg,
                                                        const SourceReading& shunt_r,
                                                        Config::SocMode mode) {
  switch (mode) {
    case Config::SocMode::RawBms:
      // Escape hatch: never use the shunt for the aggregate, even if enabled
      // and fresh.
      return { bms_soc_avg, false };

    case Config::SocMode::Hybrid:
      // Reserved for a future BMS-primary/shunt-correction model (Option D,
      // docs/research/v3.2-shunt-soc-fusion.md). Not implemented — falls
      // through to Calculated (Option C) until designed.
    case Config::SocMode::Calculated:
    default:
      // Option C: shunt is the always-on primary bank SOC once online and
      // fresh. No current-based branching, no threshold, no band — presence/
      // freshness of the shunt reading (status == Valid) is the only switch.
      // Stale/Unavailable shunt readings fall back to the BMS mean.
      if (shunt_r.status == ReadingStatus::Valid) return { shunt_r.value, true };
      return { bms_soc_avg, false };
  }
}

}  // namespace sources
