#include "aggregator.h"
#include <cmath>

namespace sources {

void Aggregator::init(BmsSource* bms, ShuntSource* shunt, MpptSource* mppt) {
  m_bms   = bms;
  m_shunt = shunt;
  m_mppt  = mppt;
}

void Aggregator::set_policy(Config::BatterySourcePolicy policy,
                            Config::MetricSource voltage_source,
                            Config::MetricSource current_source,
                            Config::MetricSource soc_source) {
  m_policy         = policy;
  m_voltage_source = voltage_source;
  m_current_source = current_source;
  m_soc_source     = soc_source;
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

// ── Pure selection function (raw, unfused passthrough) ───────────────────────
// This is the policy table.  Change selection rules here only; never scatter
// priority logic across call sites.
SourceReading Aggregator::select(Metric m,
                                 const SourceReading& bms_r,
                                 const SourceReading& shunt_r,
                                 const SourceReading& mppt_r) {
  switch (m) {
    case Metric::TOTAL_CURRENT:
    case Metric::TOTAL_VOLTAGE:
      // Raw passthrough — BMS leads unconditionally. The Battery Value
      // Sources fusion that actually drives the UI/MQTT lives in
      // fuse_bank_voltage()/fuse_bank_current(), not here.
      return bms_r;

    case Metric::PV_POWER:
    case Metric::PV_CURRENT:
    case Metric::PV_VOLTAGE:
      // MPPT is the only source for PV data.
      return mppt_r;

    case Metric::SHUNT_SOC:
      // Shunt SOC — raw passthrough only. Never overrides BMS SOC here;
      // see fuse_bank_soc() for the bank-level aggregate fusion policy.
      return shunt_r;

    default:
      return unavailable_reading();
  }
}

// ── Bank-level metric fusion (V3.2, Battery Value Sources) ───────────────────
// fuse_bank_*() are the public entry points; select_bank_value() is the shared
// policy table for all three bank-level metrics — keep it here alongside them
// rather than scattering the rule across call sites.
Aggregator::BankReading Aggregator::fuse_bank_voltage(float bms_voltage, bool bms_valid) const {
  SourceReading shunt_r = (m_shunt && m_shunt->enabled())
                          ? m_shunt->reading(Metric::TOTAL_VOLTAGE)
                          : unavailable_reading("V");
  return select_bank_value(bms_voltage, bms_valid, shunt_r, m_policy, m_voltage_source);
}

Aggregator::BankReading Aggregator::fuse_bank_current(float bms_current, bool bms_valid) const {
  SourceReading shunt_r = (m_shunt && m_shunt->enabled())
                          ? m_shunt->reading(Metric::TOTAL_CURRENT)
                          : unavailable_reading("A");
  return select_bank_value(bms_current, bms_valid, shunt_r, m_policy, m_current_source);
}

Aggregator::BankReading Aggregator::fuse_bank_soc(float bms_soc_avg, bool bms_valid) const {
  SourceReading shunt_r = (m_shunt && m_shunt->enabled())
                          ? m_shunt->reading(Metric::SHUNT_SOC)
                          : unavailable_reading("%");
  return select_bank_value(bms_soc_avg, bms_valid, shunt_r, m_policy, m_soc_source);
}

Aggregator::BankReading Aggregator::select_bank_value(float bms_value, bool bms_valid,
                                                       const SourceReading& shunt_r,
                                                       Config::BatterySourcePolicy policy,
                                                       Config::MetricSource manual_source) {
  const bool shunt_fresh = shunt_r.is_valid();  // status == Valid (not Stale/Unavailable)

  if (policy == Config::BatterySourcePolicy::Manual) {
    // Uses exactly the picked source, no cross-fallback — an explicit choice
    // that turns out unavailable reads as "no data" (valid=false), it does
    // not silently substitute the other source.
    if (manual_source == Config::MetricSource::Shunt)
      return { shunt_r.value, true, shunt_fresh };
    return { bms_value, false, bms_valid };
  }

  // Auto: shunt leads whenever fresh; BMS is the fallback (Option C rule,
  // applied uniformly to voltage/current/SOC — no per-metric picking).
  if (shunt_fresh) return { shunt_r.value, true, true };
  return { bms_value, false, bms_valid };
}

}  // namespace sources
