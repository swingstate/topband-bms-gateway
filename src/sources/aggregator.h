#pragma once
#include "source.h"
#include "bms_source.h"
#include "shunt_source.h"
#include "mppt_source.h"
#include "storage/config.h"

// ── Aggregator ─────────────────────────────────────────────────────────────────
//
// Decides, per metric, which source leads.
//
// Selection rules (encoded as pure functions — see select()/select_bank_value()):
//
//   TOTAL_CURRENT / TOTAL_VOLTAGE (raw, via reading()):
//     BMS leads unconditionally. Raw, unfused passthrough — NOT the display
//     value; see fuse_bank_voltage()/fuse_bank_current() below for the
//     Battery Value Sources fusion that actually drives the UI/MQTT.
//
//   PV_* metrics:
//     MpptSource only (no other source provides PV data).
//
//   SHUNT_SOC:
//     ShuntSource only — raw passthrough of the shunt's own SOC reading.
//     Never overrides BMS SOC directly via select()/reading(); see
//     fuse_bank_soc() below for the bank-level aggregate fusion policy.
//
// Phase A: Aggregator feeds the Diag spike panel (§3a).
// Phase B: Aggregator output wires into the CAN broadcast and MQTT publish paths.

namespace sources {

class Aggregator {
public:
  // Init: sources must be initialised before calling this.
  // Keeps non-owning pointers — lifetime managed by caller (boot.cpp singletons).
  void init(BmsSource* bms, ShuntSource* shunt, MpptSource* mppt);

  // Apply a new Battery Value Sources policy at runtime (e.g. after config save).
  // Thread-safe: called from httpd task, fuse_bank_*() from BMS task.
  void set_policy(Config::BatterySourcePolicy policy,
                  Config::MetricSource voltage_source,
                  Config::MetricSource current_source,
                  Config::MetricSource soc_source);

  // Refresh all enabled sources. Call once per BMS poller cycle.
  void update();

  // Return the raw (unfused) reading for metric m. See class comment.
  SourceReading reading(Metric m) const;

  // ── Bank-level metric fusion (V3.2, generalised for Battery Value Sources) ──
  // Result of fuse_bank_*(): the value to display/publish for a bank-level
  // metric, plus which source it came from this cycle (for UI source badges)
  // and whether either source actually had usable data right now.
  // NOT for safety/charge-taper or CAN TX — those must keep using the caller's
  // own BMS-only input, never this result. See runSafety.cpp and
  // can/victron.cpp / can/pylontech.cpp / can/sma.cpp.
  struct BankReading {
    float value;
    bool  from_shunt;  // true = value came from the shunt; false = BMS
    bool  valid;       // false = neither source has usable data right now;
                       // caller should surface "no data", not this value.
  };

  // bms_value: the caller's pure BMS figure for this metric (runSafety() output).
  // bms_valid: whether that BMS figure reflects real data right now (typically
  // packs_online > 0) — passed in because Aggregator has no access to per-pack data.
  BankReading fuse_bank_voltage(float bms_voltage, bool bms_valid) const;
  BankReading fuse_bank_current(float bms_current, bool bms_valid) const;
  BankReading fuse_bank_soc(float bms_soc_avg, bool bms_valid) const;

  // ── Pure selection functions ──────────────────────────────────────────────
  // Deterministic, no side effects, no I/O. Unit-testable in isolation.

  // Raw per-metric passthrough (see class comment). Returns the winning
  // reading given pre-fetched candidate readings.
  static SourceReading select(Metric m,
                              const SourceReading& bms_r,
                              const SourceReading& shunt_r,
                              const SourceReading& mppt_r);

  // Shared policy table behind fuse_bank_voltage/current/soc.
  //   Auto   — shunt leads whenever its reading is fresh (ReadingStatus::Valid);
  //            BMS is the fallback (used whenever bms_valid, even if 0).
  //   Manual — uses exactly manual_source, no cross-fallback: an explicit
  //            choice that turns out unavailable reads as "no data" (valid=false),
  //            it does not silently substitute the other source.
  static BankReading select_bank_value(float bms_value, bool bms_valid,
                                       const SourceReading& shunt_r,
                                       Config::BatterySourcePolicy policy,
                                       Config::MetricSource manual_source);

private:
  BmsSource*   m_bms   = nullptr;
  ShuntSource* m_shunt = nullptr;
  MpptSource*  m_mppt  = nullptr;
  // Atomic-ish: written from httpd task, read from BMS task.
  // Single-byte enums fit in one bus transaction on Xtensa — no spinlock needed.
  volatile Config::BatterySourcePolicy m_policy         = Config::BatterySourcePolicy::Auto;
  volatile Config::MetricSource        m_voltage_source = Config::MetricSource::Bms;
  volatile Config::MetricSource        m_current_source = Config::MetricSource::Bms;
  volatile Config::MetricSource        m_soc_source     = Config::MetricSource::Bms;
};

}  // namespace sources
