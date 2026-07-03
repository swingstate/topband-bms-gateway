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

  uint8_t  cs;
  float    pp, pv, pc, yh;
  bool     pp_valid, pv_v_valid, pv_i_valid, yield_valid;
  uint32_t ts;
  bool     seen;

  portENTER_CRITICAL(&m_mux);
  cs         = m_charge_state;
  pp         = m_pv_power_w;
  pv         = m_pv_voltage_v;
  pc         = m_pv_current_a;
  yh         = m_yield_today_wh;
  pp_valid   = m_pv_power_valid;
  pv_v_valid = m_pv_v_valid;
  pv_i_valid = m_pv_i_valid;
  yield_valid = m_yield_valid;
  ts         = m_last_seen_ms;
  seen       = m_ever_seen;
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
      if (!pv_v_valid) return unavailable_reading("V");
      return { pv, "V", ts, st };
    case Metric::PV_CURRENT:
      if (!pv_i_valid) return unavailable_reading("A");
      return { pc, "A", ts, st };
    case Metric::YIELD_TODAY:
      if (!yield_valid) return unavailable_reading("Wh");
      return { yh, "Wh", ts, st };
    case Metric::CHARGE_STATE:
      // charge_state is always valid when the record was decoded (even 0=off is meaningful)
      return { (float)cs, "", ts, st };
    default:
      return unavailable_reading("");
  }
}

void MpptSource::set_decoded_values(uint8_t charge_state,
                                    float pv_power_w, float pv_voltage_v, float pv_current_a,
                                    float batt_voltage_v, float batt_current_a,
                                    float yield_today_wh,
                                    bool pv_power_valid, bool pv_v_valid, bool pv_i_valid,
                                    bool batt_v_valid, bool batt_i_valid, bool yield_valid,
                                    uint32_t now_ms) {
  portENTER_CRITICAL(&m_mux);
  m_charge_state   = charge_state;
  m_pv_power_w     = pv_power_w;
  m_pv_voltage_v   = pv_voltage_v;
  m_pv_current_a   = pv_current_a;
  m_batt_voltage_v = batt_voltage_v;
  m_batt_current_a = batt_current_a;
  m_yield_today_wh = yield_today_wh;
  m_pv_power_valid = pv_power_valid;
  m_pv_v_valid     = pv_v_valid;
  m_pv_i_valid     = pv_i_valid;
  m_batt_v_valid   = batt_v_valid;
  m_batt_i_valid   = batt_i_valid;
  m_yield_valid    = yield_valid;
  m_last_seen_ms   = now_ms;
  m_ever_seen      = true;
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
  s.seen           = m_ever_seen;
  s.charge_state   = m_charge_state;
  s.pv_power_w     = m_pv_power_w;
  s.pv_voltage_v   = m_pv_voltage_v;
  s.pv_current_a   = m_pv_current_a;
  s.batt_voltage_v = m_batt_voltage_v;
  s.batt_current_a = m_batt_current_a;
  s.yield_today_wh = m_yield_today_wh;
  s.pv_power_valid = m_pv_power_valid;
  s.pv_v_valid     = m_pv_v_valid;
  s.pv_i_valid     = m_pv_i_valid;
  s.batt_v_valid   = m_batt_v_valid;
  s.batt_i_valid   = m_batt_i_valid;
  s.yield_valid    = m_yield_valid;
  s.last_seen_ms   = m_last_seen_ms;
  portEXIT_CRITICAL(&m_mux);
  return s;
}

}  // namespace sources
