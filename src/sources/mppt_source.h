#pragma once
#include "source.h"
#include "storage/config.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>

// ── MpptSource ─────────────────────────────────────────────────────────────────
//
// Victron SmartSolar MPPT BLE source (Solar Charger, record type 0x01).
// Provides PV_POWER, PV_CURRENT, PV_VOLTAGE.
//
// Solar Charger 0x01 record carries: battery_voltage, battery_charging_current,
// yield_today, solar_power.  pv_voltage and pv_current are NOT in the record;
// they are derived from (solar_power, batt_v, batt_i) via power-balance.
//
// Victron not-available sentinels: 0x7FFF for signed (int16) fields,
// 0xFFFF for unsigned (uint16) fields.  Fields at sentinel are stored with
// their validity flag clear; reading() returns Unavailable for those metrics.
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
  // Validity flags: false when the corresponding raw field held a Victron sentinel
  // (0x7FFF signed / 0xFFFF unsigned).  pv_derived_valid covers pv_voltage_v and
  // pv_current_a (both derived; false when any source field was a sentinel).
  void set_decoded_values(float pv_power_w, float pv_voltage_v, float pv_current_a,
                          float batt_voltage_v, float batt_current_a,
                          bool pv_power_valid, bool batt_v_valid, bool batt_i_valid,
                          bool pv_derived_valid,
                          uint32_t now_ms);

  uint32_t ms_since_last_seen(uint32_t now_ms) const;

  struct DiagSnap {
    bool     seen;
    float    pv_power_w;
    float    pv_voltage_v;
    float    pv_current_a;
    float    batt_voltage_v;
    float    batt_current_a;
    bool     pv_power_valid;    // false → solar_power field was Victron sentinel 0xFFFF
    bool     batt_v_valid;      // false → battery_voltage field was sentinel 0x7FFF
    bool     batt_i_valid;      // false → battery_current field was sentinel 0x7FFF
    bool     pv_derived_valid;  // false → pv_voltage_v / pv_current_a could not be derived
    uint32_t last_seen_ms;
  };
  DiagSnap diag_snap() const;

private:
  bool     m_enabled          = false;
  float    m_pv_power_w       = 0.0f;
  float    m_pv_voltage_v     = 0.0f;
  float    m_pv_current_a     = 0.0f;
  float    m_batt_voltage_v   = 0.0f;
  float    m_batt_current_a   = 0.0f;
  bool     m_pv_power_valid   = false;
  bool     m_batt_v_valid     = false;
  bool     m_batt_i_valid     = false;
  bool     m_pv_derived_valid = false;
  uint32_t m_last_seen_ms     = 0;
  bool     m_ever_seen        = false;

  static constexpr uint32_t STALE_MS = 30000;

  mutable portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace sources
