#include "shunt_source.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"

namespace sources {

void ShuntSource::init(const Config& cfg) {
  m_enabled = cfg.ble_shunt_enabled;
}

void ShuntSource::update() {
  // Phase A: no aggregation needed here — values are pushed by ble_scanner
  // via set_decoded_values() and stored directly in member fields.
  // This function is a hook for Phase B stitching logic.
}

SourceReading ShuntSource::reading(Metric m) const {
  if (!m_enabled) return unavailable_reading("");

  float cur, vol, soc;
  uint32_t ts;
  bool seen;

  portENTER_CRITICAL(&m_mux);
  cur  = m_current_a;
  vol  = m_voltage_v;
  soc  = m_soc_pct;
  ts   = m_last_seen_ms;
  seen = m_ever_seen;
  portEXIT_CRITICAL(&m_mux);

  if (!seen) return unavailable_reading("");

  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  ReadingStatus st = ((now_ms - ts) < STALE_MS) ? ReadingStatus::Valid
                                                 : ReadingStatus::Stale;

  switch (m) {
    case Metric::TOTAL_CURRENT: return { cur, "A", ts, st };
    case Metric::TOTAL_VOLTAGE: return { vol, "V", ts, st };
    case Metric::SHUNT_SOC:     return { soc, "%", ts, st };
    default:                    return unavailable_reading("");
  }
}

void ShuntSource::set_decoded_values(float current_a, float voltage_v, float soc_pct,
                                     uint32_t now_ms) {
  portENTER_CRITICAL(&m_mux);
  m_current_a    = current_a;
  m_voltage_v    = voltage_v;
  m_soc_pct      = soc_pct;
  m_last_seen_ms = now_ms;
  m_ever_seen    = true;
  portEXIT_CRITICAL(&m_mux);
}

uint32_t ShuntSource::ms_since_last_seen(uint32_t now_ms) const {
  portENTER_CRITICAL(&m_mux);
  bool seen = m_ever_seen;
  uint32_t ts = m_last_seen_ms;
  portEXIT_CRITICAL(&m_mux);
  if (!seen) return 0;
  return now_ms - ts;
}

ShuntSource::DiagSnap ShuntSource::diag_snap() const {
  DiagSnap s{};
  portENTER_CRITICAL(&m_mux);
  s.seen        = m_ever_seen;
  s.current_a   = m_current_a;
  s.voltage_v   = m_voltage_v;
  s.soc_pct     = m_soc_pct;
  s.last_seen_ms = m_last_seen_ms;
  portEXIT_CRITICAL(&m_mux);
  return s;
}

}  // namespace sources
