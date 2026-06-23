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
// Selection rules (encoded as a pure function — see select() below):
//
//   TOTAL_CURRENT:
//     If ShuntSource enabled AND its reading is Valid → shunt leads.
//     (This solves the <0.5 A BMS dead-zone that is the primary V3.1 motivation.)
//     Else if ShuntSource reading is Stale → shunt leads with Stale status.
//     Else → BMS aggregate (V3.0 behaviour).
//
//   TOTAL_VOLTAGE:
//     BMS leads (pack-level voltage is more accurate for charge control).
//     ShuntSource reading is secondary/reference only — see SHUNT_SOC for shunt V.
//
//   PV_* metrics:
//     MpptSource only (no other source provides PV data).
//
//   SHUNT_SOC:
//     ShuntSource only — exposed as a supplementary value, never overrides BMS SOC.
//     The owner verifies both on hardware before deciding to use shunt SOC for control.
//
// Phase A: Aggregator feeds the Diag spike panel (§3a).
// Phase B: Aggregator output wires into the CAN broadcast and MQTT publish paths.

namespace sources {

class Aggregator {
public:
  // Init: sources must be initialised before calling this.
  // Keeps non-owning pointers — lifetime managed by caller (boot.cpp singletons).
  void init(BmsSource* bms, ShuntSource* shunt, MpptSource* mppt);

  // Apply a new shunt current mode at runtime (e.g. after config save).
  // Thread-safe: called from httpd task, reading() from BMS task.
  void set_shunt_mode(Config::ShuntCurrentMode mode);

  // Refresh all enabled sources. Call once per BMS poller cycle.
  void update();

  // Return the winning reading for metric m, after applying selection rules.
  SourceReading reading(Metric m) const;

  // ── Pure selection function ───────────────────────────────────────────────
  // Deterministic, no side effects, no I/O. Unit-testable in isolation.
  // Returns the winning reading given pre-fetched candidate readings.
  // bms_r:   BmsSource reading for the metric.
  // shunt_r: ShuntSource reading (may be Unavailable if disabled).
  // mppt_r:  MpptSource reading (may be Unavailable if disabled).
  // mode:    shunt current mode (defaults to Auto for backward compat in tests).
  static SourceReading select(Metric m,
                              const SourceReading& bms_r,
                              const SourceReading& shunt_r,
                              const SourceReading& mppt_r,
                              Config::ShuntCurrentMode mode = Config::ShuntCurrentMode::Auto);

private:
  BmsSource*   m_bms   = nullptr;
  ShuntSource* m_shunt = nullptr;
  MpptSource*  m_mppt  = nullptr;
  // Atomic-ish: written from httpd task, read from BMS task.
  // Single-byte enum fits in one bus transaction on Xtensa — no spinlock needed.
  volatile Config::ShuntCurrentMode m_shunt_mode = Config::ShuntCurrentMode::Auto;
};

}  // namespace sources
