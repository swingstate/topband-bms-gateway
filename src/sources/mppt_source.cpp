#include "mppt_source.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"

namespace sources {

void MpptSource::init(const Config& cfg) {
  m_enabled = cfg.ble_mppt_enabled;
}

void MpptSource::update() {}

SourceReading MpptSource::reading(Metric m) const {
  if (!m_enabled) return unavailable_reading("");

  float pp, pv, pc;
  bool pp_valid, pv_derived_valid;
  uint32_t ts;
  bool seen;

  portENTER_CRITICAL(&m_mux);
  pp              = m_pv_power_w;
  pv              = m_pv_voltage_v;
  pc              = m_pv_current_a;
  pp_valid        = m_pv_power_valid;
  pv_derived_valid = m_pv_derived_valid;
  ts              = m_last_seen_ms;
  seen            = m_ever_seen;
  portEXIT_CRITICAL(&m_mux);

  if (!seen) return unavailable_reading("");

  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  ReadingStatus st = ((now_ms - ts) < STALE_MS) ? ReadingStatus::Valid
                                                 : ReadingStatus::Stale;

  switch (m) {
    case Metric::PV_POWER:
      if (!pp_valid) return unavailable_reading("W");
      return { pp, "W", ts, st };
    case Metric::PV_VOLTAGE:
      if (!pv_derived_valid) return unavailable_reading("V");
      return { pv, "V", ts, st };
    case Metric::PV_CURRENT:
      if (!pv_derived_valid) return unavailable_reading("A");
      return { pc, "A", ts, st };
    default:
      return unavailable_reading("");
  }
}

void MpptSource::set_decoded_values(float pv_power_w, float pv_voltage_v, float pv_current_a,
                                    float batt_voltage_v, float batt_current_a,
                                    bool pv_power_valid, bool batt_v_valid, bool batt_i_valid,
                                    bool pv_derived_valid,
                                    uint32_t now_ms) {
  portENTER_CRITICAL(&m_mux);
  m_pv_power_w       = pv_power_w;
  m_pv_voltage_v     = pv_voltage_v;
  m_pv_current_a     = pv_current_a;
  m_batt_voltage_v   = batt_voltage_v;
  m_batt_current_a   = batt_current_a;
  m_pv_power_valid   = pv_power_valid;
  m_batt_v_valid     = batt_v_valid;
  m_batt_i_valid     = batt_i_valid;
  m_pv_derived_valid = pv_derived_valid;
  m_last_seen_ms     = now_ms;
  m_ever_seen        = true;
  portEXIT_CRITICAL(&m_mux);
}

uint32_t MpptSource::ms_since_last_seen(uint32_t now_ms) const {
  portENTER_CRITICAL(&m_mux);
  bool seen = m_ever_seen;
  uint32_t ts = m_last_seen_ms;
  portEXIT_CRITICAL(&m_mux);
  if (!seen) return 0;
  return now_ms - ts;
}

MpptSource::DiagSnap MpptSource::diag_snap() const {
  DiagSnap s{};
  portENTER_CRITICAL(&m_mux);
  s.seen             = m_ever_seen;
  s.pv_power_w       = m_pv_power_w;
  s.pv_voltage_v     = m_pv_voltage_v;
  s.pv_current_a     = m_pv_current_a;
  s.batt_voltage_v   = m_batt_voltage_v;
  s.batt_current_a   = m_batt_current_a;
  s.pv_power_valid   = m_pv_power_valid;
  s.batt_v_valid     = m_batt_v_valid;
  s.batt_i_valid     = m_batt_i_valid;
  s.pv_derived_valid = m_pv_derived_valid;
  s.last_seen_ms     = m_last_seen_ms;
  portEXIT_CRITICAL(&m_mux);
  return s;
}

}  // namespace sources
