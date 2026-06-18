#pragma once
#include "source.h"
#include "storage/config.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>

// ── ShuntSource ────────────────────────────────────────────────────────────────
//
// Victron SmartShunt BLE source.
// Provides TOTAL_CURRENT, TOTAL_VOLTAGE, SHUNT_SOC.
//
// Phase A: reads live BLE decode data written by ble_scanner task.
// Phase B: Aggregator wires shunt current into the hybrid path.
//
// Thread safety: update() and reading() are called from the Aggregator task.
// set_decoded_values() is called from the NimBLE scan callback (separate task).
// A portMUX spinlock protects the shared state.

namespace sources {

class ShuntSource final : public ISource {
public:
  ShuntSource() = default;

  const char* id()      const override { return "shunt"; }
  bool        enabled() const override { return m_enabled; }

  void init(const Config& cfg);

  // Called from Aggregator cycle — snapshots the latest BLE-decoded values.
  void update() override;

  // Provides TOTAL_CURRENT, TOTAL_VOLTAGE, SHUNT_SOC.
  // Returns Unavailable if BLE source is disabled or no advertisement received.
  SourceReading reading(Metric m) const override;

  // Called from NimBLE scan callback when a new SmartShunt advertisement is decoded.
  // MUST NOT log the key. Safe to call from any task.
  void set_decoded_values(float current_a, float voltage_v, float soc_pct,
                          uint32_t now_ms);

  // Diagnostic: milliseconds since last valid advertisement (0 = never seen).
  uint32_t ms_since_last_seen(uint32_t now_ms) const;

  // Diagnostic snapshot (for /api/diag BLE spike section).
  struct DiagSnap {
    bool    seen;
    float   current_a;
    float   voltage_v;
    float   soc_pct;
    uint32_t last_seen_ms;
  };
  DiagSnap diag_snap() const;

private:
  bool     m_enabled    = false;
  float    m_current_a  = 0.0f;
  float    m_voltage_v  = 0.0f;
  float    m_soc_pct    = 0.0f;
  uint32_t m_last_seen_ms = 0;
  bool     m_ever_seen  = false;

  // Stale threshold: if no advertisement within this window, reading is Stale.
  static constexpr uint32_t STALE_MS = 30000;

  mutable portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace sources
