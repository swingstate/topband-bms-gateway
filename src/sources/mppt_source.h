#pragma once
#include "source.h"
#include "storage/config.h"
#include <cstdint>

// ── MpptSource ─────────────────────────────────────────────────────────────────
//
// Victron SmartSolar MPPT BLE source.
// Provides PV_POWER, PV_CURRENT, PV_VOLTAGE.
//
// Phase A: reads live BLE decode data written by ble_scanner task.
// Phase B: Aggregator exposes PV data for energy balance calculation.

namespace sources {

class MpptSource final : public ISource {
public:
  MpptSource() = default;

  const char* id()      const override { return "mppt"; }
  bool        enabled() const override { return m_enabled; }

  void init(const Config& cfg);

  void update() override;

  // Provides PV_POWER, PV_CURRENT, PV_VOLTAGE.
  SourceReading reading(Metric m) const override;

  // Called from NimBLE scan callback when a new MPPT advertisement is decoded.
  void set_decoded_values(float pv_power_w, float pv_voltage_v, float pv_current_a,
                          float batt_voltage_v, float batt_current_a,
                          uint32_t now_ms);

  uint32_t ms_since_last_seen(uint32_t now_ms) const;

  struct DiagSnap {
    bool     seen;
    float    pv_power_w;
    float    pv_voltage_v;
    float    pv_current_a;
    float    batt_voltage_v;
    float    batt_current_a;
    uint32_t last_seen_ms;
  };
  DiagSnap diag_snap() const;

private:
  bool     m_enabled       = false;
  float    m_pv_power_w    = 0.0f;
  float    m_pv_voltage_v  = 0.0f;
  float    m_pv_current_a  = 0.0f;
  float    m_batt_voltage_v = 0.0f;
  float    m_batt_current_a = 0.0f;
  uint32_t m_last_seen_ms  = 0;
  bool     m_ever_seen     = false;

  static constexpr uint32_t STALE_MS = 30000;

  mutable portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace sources
