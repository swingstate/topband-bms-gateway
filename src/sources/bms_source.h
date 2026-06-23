#pragma once
#include "source.h"

// ── BmsSource ──────────────────────────────────────────────────────────────────
//
// Adapts the existing BMS aggregate path (bus::snapshot_bus) into the Source
// interface. Always enabled; the Aggregator uses it as the fallback for any
// metric not covered by an active BLE source.
//
// Key behaviour: TOTAL_CURRENT is marked Stale when |current| < 0.5 A.
// This is the known resolution limit of the TopBand BMS coulomb counter.
// The Aggregator uses this Stale flag to prefer the SmartShunt reading in that
// low-current regime, which is the primary motivation for V3.1.

namespace sources {

class BmsSource final : public ISource {
public:
  BmsSource() = default;

  const char* id()      const override { return "bms"; }
  bool        enabled() const override { return true; }

  // Reads from bus::snapshot_bus. Safe to call from any task.
  void update() override;

  // TOTAL_CURRENT: Stale when |current| < 0.5 A (known BMS limitation).
  // TOTAL_VOLTAGE: always Valid when at least one pack is online.
  // Other metrics: Unavailable (BMS does not provide PV or shunt data).
  SourceReading reading(Metric m) const override;

private:
  float    m_current_a   = 0.0f;
  float    m_voltage_v   = 0.0f;
  uint32_t m_ts_ms       = 0;
  bool     m_pack_online = false;

  static constexpr float BMS_CURRENT_RESOLUTION_A = 0.5f;
};

}  // namespace sources
