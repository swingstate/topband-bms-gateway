#pragma once
#include "source.h"
#include "storage/config.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>

// ── MpptSource ─────────────────────────────────────────────────────────────────
//
// Victron SmartSolar MPPT BLE source (Solar Charger, record type 0x01).
// Provides PV_POWER, PV_CURRENT, PV_VOLTAGE, YIELD_TODAY, CHARGE_STATE.
//
// Solar Charger 0x01 encrypted payload (new Product Advertisement format):
//   plain[0]:    charge_state  (uint8; 0=off, 3=bulk, 4=absorption, 5=float)
//   plain[1]:    charger_error (uint8)
//   plain[2-3]:  battery_voltage (int16 LE, /100 V; 0x7FFF=NA)
//   plain[4-5]:  battery_charging_current (int16 LE, /10 A; 0x7FFF=NA)
//   plain[6-7]:  yield_today (uint16 LE, *10 Wh; 0xFFFF=NA)
//   plain[8-9]:  solar_power / pv_power (uint16 LE, W; 0xFFFF=NA)       ← transmitted
//   plain[10-11]: pv_voltage / charger_voltage (uint16 LE, /100 V; 0xFFFF=NA) ← transmitted
//   plain[12-13]: pv_current / charger_current (int16 LE, /10 A; 0x7FFF=NA)   ← transmitted
//
// Reference: keshavdv/victron-ble solar_charger.py (charger_voltage, charger_current);
//            Fabian-Schmidt/esphome-victron_ble extended Solar Charger struct.
// All three of {pv_power, pv_voltage, pv_current} are transmitted directly from
// the MPPT's measured input-side sensors. None are derived/computed.
//
// Victron not-available sentinels: 0x7FFF for signed int16, 0xFFFF for unsigned uint16.
// Fields at sentinel are stored with their validity flag clear; reading() returns
// Unavailable for those metrics, never a phantom reading.
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
  // (0x7FFF signed / 0xFFFF unsigned).
  // pv_v_valid / pv_i_valid: true only when enc_len >= 12/14 AND no sentinel.
  // yield_valid: true when yield_today bytes were present and not sentinel.
  void set_decoded_values(uint8_t charge_state,
                          float pv_power_w, float pv_voltage_v, float pv_current_a,
                          float batt_voltage_v, float batt_current_a,
                          float yield_today_wh,
                          bool pv_power_valid, bool pv_v_valid, bool pv_i_valid,
                          bool batt_v_valid, bool batt_i_valid, bool yield_valid,
                          uint32_t now_ms);

  uint32_t ms_since_last_seen(uint32_t now_ms) const;

  struct DiagSnap {
    bool     seen;
    uint8_t  charge_state;      // 0=off, 3=bulk, 4=absorption, 5=float
    float    pv_power_w;
    float    pv_voltage_v;
    float    pv_current_a;
    float    batt_voltage_v;
    float    batt_current_a;
    float    yield_today_wh;
    bool     pv_power_valid;    // false → solar_power field was Victron sentinel 0xFFFF
    bool     pv_v_valid;        // false → pv_voltage field absent or sentinel 0xFFFF
    bool     pv_i_valid;        // false → pv_current field absent or sentinel 0x7FFF
    bool     batt_v_valid;      // false → battery_voltage field was sentinel 0x7FFF
    bool     batt_i_valid;      // false → battery_current field was sentinel 0x7FFF
    bool     yield_valid;       // false → yield_today field was sentinel 0xFFFF
    uint32_t last_seen_ms;
  };
  DiagSnap diag_snap() const;

private:
  bool     m_enabled         = false;
  uint8_t  m_charge_state    = 0;
  float    m_pv_power_w      = 0.0f;
  float    m_pv_voltage_v    = 0.0f;
  float    m_pv_current_a    = 0.0f;
  float    m_batt_voltage_v  = 0.0f;
  float    m_batt_current_a  = 0.0f;
  float    m_yield_today_wh  = 0.0f;
  bool     m_pv_power_valid  = false;
  bool     m_pv_v_valid      = false;
  bool     m_pv_i_valid      = false;
  bool     m_batt_v_valid    = false;
  bool     m_batt_i_valid    = false;
  bool     m_yield_valid     = false;
  uint32_t m_last_seen_ms    = 0;
  bool     m_ever_seen       = false;

  static constexpr uint32_t STALE_MS = 30000;

  mutable portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace sources
