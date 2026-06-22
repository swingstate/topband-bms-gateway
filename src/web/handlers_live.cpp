#include "handlers_live.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "bms/runtime_estimator.h"
#include "can/tx.h"
#include "storage/energy_store.h"
#include "net/ntp.h"
#include "app/version.h"
#include "sources/registry.h"
#include "mqtt/publisher.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <ArduinoJson.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static const char* TAG = "web_live";

// Coexistence diagnostic: track /api/live handler wall-clock latency.
// Updated on every request; never locked (benign last-write-wins for diagnostics).
// Visible at /api/diag ble_spike.handler_last_ms and handler_max_ms.
// A single request taking >200 ms (one BLE scan window) while BLE is active
// confirms CPU starvation of the httpd task on Core 0.
static volatile uint32_t s_handler_last_ms = 0;
static volatile uint32_t s_handler_max_ms  = 0;

namespace web {

uint32_t live_handler_last_ms() { return s_handler_last_ms; }
uint32_t live_handler_max_ms()  { return s_handler_max_ms; }

esp_err_t handle_live(httpd_req_t* req) {
  const int64_t t_entry_us = esp_timer_get_time();

  BmsSystemSnapshot snap = {};
  bool has_snap = bus::snapshot_bus::read(snap);

  SafetyState safety = {};
  bool has_safety = bms::poller::read_safety_state(safety);

  bms::poller::PollerStats ps = {};
  bms::poller::get_stats(ps);

  can::tx::CanStats cs = {};
  can::tx::get_stats(cs);

  uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);

  JsonDocument doc;

  doc["uptime_s"]             = uptime_s;
  doc["bms_count_configured"] = snap.pack_count_configured;
  doc["bms_count_online"]     = has_snap ? (int)snap.pack_count_online : 0;

  JsonObject snapshot = doc["snapshot"].to<JsonObject>();
  if (has_snap) {
    snapshot["cycle_id"]    = snap.cycle_id;
    snapshot["produced_ms"] = snap.produced_ms;

    JsonArray packs = snapshot["packs"].to<JsonArray>();
    for (uint8_t i = 0; i < snap.pack_count_configured && i < 16; i++) {
      const BmsPackSnapshot& p = snap.pack[i];
      JsonObject pj = packs.add<JsonObject>();
      pj["bms_id"]       = p.bms_id;
      pj["online"]       = p.online;
      pj["pack_voltage"] = p.pack_voltage;
      pj["pack_current"] = p.pack_current;
      pj["soc"]          = p.soc;
      pj["soh"]          = p.soh;
      pj["cycles"]       = p.cycles;
      pj["temp_max_c"]   = p.temp_max_c;
      pj["temp_avg_c"]   = p.temp_avg_c;
      pj["cell_count"]   = p.cell_count;
      pj["rem_ah"]       = p.rem_ah;
      pj["full_ah"]      = p.full_ah;
      pj["cell_min_v"]   = p.cell_min_v;
      pj["cell_max_v"]   = p.cell_max_v;
      pj["cell_min_idx"] = p.cell_min_idx;
      pj["cell_max_idx"] = p.cell_max_idx;
      pj["cell_drift_v"] = p.cell_drift_v;
      pj["current_held"] = p.current_held;

      JsonArray cells = pj["cells"].to<JsonArray>();
      for (uint8_t c = 0; c < p.cell_count && c < 16; c++) {
        cells.add(p.cell_v[c]);
      }
    }
  }

  JsonObject saf = doc["safety"].to<JsonObject>();
  if (has_safety) {
    saf["cvl_volts"]          = safety.cvl_volts;
    saf["ccl_amps"]           = safety.ccl_amps;
    saf["dcl_amps"]           = safety.dcl_amps;
    saf["soc_avg"]            = safety.soc_avg;
    saf["soh_avg"]            = safety.soh_avg;
    saf["temp_avg"]           = safety.temp_avg;
    saf["pack_voltage_avg"]   = safety.pack_voltage_avg;
    saf["pack_current_total"] = safety.pack_current_total;
    saf["alarm_flags"]        = safety.alarm_flags;
    saf["sys_message"]        = safety.sys_message;
    saf["packs_online"]       = safety.packs_online;
    saf["packs_configured"]   = safety.packs_configured;
    saf["factor_charge"]      = safety.factor_charge;
    saf["factor_discharge"]   = safety.factor_discharge;
  }

  JsonObject stats = doc["stats"].to<JsonObject>();
  JsonObject sp = stats["poller"].to<JsonObject>();
  sp["cycles_completed"]     = ps.cycles_completed;
  sp["analog_polls_ok"]      = ps.analog_polls_ok;
  sp["analog_polls_timeout"] = ps.analog_polls_timeout;
  sp["cycle_avg_ms"]         = ps.cycle_avg_ms;
  sp["cycle_max_ms"]         = ps.cycle_max_ms;

  JsonObject sc = stats["can"].to<JsonObject>();
  sc["tx_ok"]        = (uint32_t)cs.tx_ok;
  sc["tx_fail"]      = (uint32_t)cs.tx_fail;
  sc["heartbeats"]   = cs.heartbeats;
  sc["express_sends"]= cs.express_sends;
  sc["bus_off_count"]= cs.bus_off_count;

  JsonObject sb = stats["bus"].to<JsonObject>();
  sb["publishes"] = bus::snapshot_bus::total_publishes();
  sb["reads"]     = bus::snapshot_bus::total_reads();
  sb["retries"]   = bus::snapshot_bus::total_read_retries();

  // ── Energy counters ───────────────────────────────────────────────────────
  // Read live from energy_store (H1 lesson: read live, not from stale snapshot).
  JsonObject energy = doc["energy"].to<JsonObject>();
  energy["today_in_kwh"]  = storage::energy_store::today_in_kwh();
  energy["today_out_kwh"] = storage::energy_store::today_out_kwh();
  energy["week_in_kwh"]   = storage::energy_store::week_in_kwh();
  energy["week_out_kwh"]  = storage::energy_store::week_out_kwh();
  energy["total_in_kwh"]  = storage::energy_store::total_in_kwh();
  energy["total_out_kwh"] = storage::energy_store::total_out_kwh();

  // ── Runtime estimate ──────────────────────────────────────────────────────
  {
    bms::runtime_estimator::RuntimeStateEst rt_state = bms::runtime_estimator::RuntimeStateEst::Idle;
    int32_t rt_min = -1;
    if (has_safety) {
      rt_min = bms::runtime_estimator::estimate_min(safety, rt_state);
    }
    doc["runtime_est_min"] = rt_min;
    doc["runtime_est_state"] = (rt_state == bms::runtime_estimator::RuntimeStateEst::UntilEmpty)
                               ? "until_empty"
                               : (rt_state == bms::runtime_estimator::RuntimeStateEst::UntilFull)
                                 ? "until_full"
                                 : "idle";
  }

  // ── NTP time (for settings page display) ─────────────────────────────────
  doc["now_ts_s"]   = net::ntp::now_unix_s();
  doc["ntp_synced"] = net::ntp::is_synced();

  // ── Sources (MPPT / Shunt / Solar Passthrough) ────────────────────────────
  {
    using sources::Metric;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

    sources::BmsSource*   bms   = sources::bms_source();
    sources::ShuntSource* shunt = sources::shunt_source();
    sources::MpptSource*  mppt  = sources::mppt_source();

    JsonObject src = doc["sources"].to<JsonObject>();

    // battery_current_src: mirrors aggregator TOTAL_CURRENT selection rule.
    // Shunt leads only when |BMS current| < 0.5 A and shunt has a valid reading.
    const char* cur_src = "bms";
    if (shunt && shunt->enabled()) {
      sources::SourceReading bms_r =
        bms ? bms->reading(Metric::TOTAL_CURRENT) : sources::unavailable_reading("A");
      sources::SourceReading shunt_r = shunt->reading(Metric::TOTAL_CURRENT);
      float bms_abs = bms_r.is_usable() ? fabsf(bms_r.value) : 0.0f;
      if (bms_abs < 0.5f && shunt_r.is_usable()) cur_src = "shunt";
    }
    src["battery_current_src"] = cur_src;

    // MPPT source details
    JsonObject jm = src["mppt"].to<JsonObject>();
    if (mppt) {
      auto ds = mppt->diag_snap();
      jm["enabled"]            = mppt->enabled();
      jm["seen"]               = ds.seen;
      jm["charge_state"]       = (int)ds.charge_state;
      jm["pv_power_w"]         = ds.pv_power_w;
      jm["pv_voltage_v"]       = ds.pv_voltage_v;
      jm["pv_current_a"]       = ds.pv_current_a;
      jm["batt_voltage_v"]     = ds.batt_voltage_v;
      jm["batt_current_a"]     = ds.batt_current_a;
      jm["yield_today_wh"]     = ds.yield_today_wh;
      jm["pv_power_valid"]     = ds.pv_power_valid;
      jm["pv_v_valid"]         = ds.pv_v_valid;
      jm["pv_i_valid"]         = ds.pv_i_valid;
      jm["batt_v_valid"]       = ds.batt_v_valid;
      jm["batt_i_valid"]       = ds.batt_i_valid;
      jm["yield_valid"]        = ds.yield_valid;
      jm["ms_since_last_seen"] = mppt->ms_since_last_seen(now_ms);
    } else {
      jm["enabled"] = false;
      jm["seen"]    = false;
    }

    // Shunt source details
    JsonObject js = src["shunt"].to<JsonObject>();
    if (shunt) {
      auto ds = shunt->diag_snap();
      js["enabled"]            = shunt->enabled();
      js["seen"]               = ds.seen;
      js["current_a"]          = ds.current_a;
      js["voltage_v"]          = ds.voltage_v;
      js["soc_pct"]            = ds.soc_pct;
      js["ms_since_last_seen"] = shunt->ms_since_last_seen(now_ms);
    } else {
      js["enabled"] = false;
      js["seen"]    = false;
    }

    // Solar Passthrough (OpenDTU MQTT, display-only)
    // received=true once the configured topic delivers its first message.
    // UI shows "unknown" until received; "active"/"inactive" after.
    JsonObject jp = src["solar_passthrough"].to<JsonObject>();
    bool pt_state = false;
    uint32_t pt_ts = 0;
    bool pt_received = mqtt::publisher::get_solar_passthrough(pt_state, pt_ts);
    jp["received"] = pt_received;
    jp["state"]    = pt_state;
    jp["ts_ms"]    = pt_ts;
  }

  // Estimate size then allocate. PSRAM preferred: this JSON can be 5-20 KB and
  // is allocated/freed on every /api/live poll, fragmenting internal heap.
  size_t est = measureJson(doc) + 1;
  char* buf = (char*)heap_caps_malloc(est, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf) buf = (char*)malloc(est);
  if (!buf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  size_t n = serializeJson(doc, buf, est);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t ret = httpd_resp_send(req, buf, (ssize_t)n);
  free(buf);

  {
    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t_entry_us) / 1000LL);
    s_handler_last_ms = elapsed_ms;
    if (elapsed_ms > s_handler_max_ms) s_handler_max_ms = elapsed_ms;
    if (elapsed_ms > 200) {
      // >200 ms = more than one full BLE scan window. Confirms CPU starvation
      // when BLE is active. DIAGNOSTIC ONLY — no corrective action taken here.
      ESP_LOGW(TAG, "/api/live latency %lu ms (BLE active starvation indicator)", (unsigned long)elapsed_ms);
    }
  }

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "client disconnected during /api/live send");
  }
  return ret;
}

// ── HStream helpers (same pattern as handlers_history / handlers_diag) ──────

namespace {

struct HS {
  httpd_req_t* req;
  char         buf[2048];
  size_t       pos;
  esp_err_t    err;
};

static void hs_flush(HS& s) {
  if (s.pos > 0 && s.err == ESP_OK)
    s.err = httpd_resp_send_chunk(s.req, s.buf, (ssize_t)s.pos);
  s.pos = 0;
}
static void hs_raw(HS& s, const char* d, size_t n) {
  while (n > 0 && s.err == ESP_OK) {
    size_t sp = sizeof(s.buf) - s.pos;
    size_t cp = n < sp ? n : sp;
    memcpy(s.buf + s.pos, d, cp);
    s.pos += cp; n -= cp; d += cp;
    if (s.pos == sizeof(s.buf)) hs_flush(s);
  }
}
static void hs_str(HS& s, const char* str) { hs_raw(s, str, strlen(str)); }
static void hs_u32(HS& s, uint32_t v) {
  char t[12]; snprintf(t, sizeof(t), "%u", (unsigned)v); hs_str(s, t);
}
static void hs_i32(HS& s, int32_t v) {
  char t[12]; snprintf(t, sizeof(t), "%d", (int)v); hs_str(s, t);
}
static void hs_f3(HS& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.3f", v); hs_str(s, t);
}
static void hs_f2(HS& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.2f", v); hs_str(s, t);
}
static void hs_f1(HS& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.1f", v); hs_str(s, t);
}
static void hs_bool(HS& s, bool v) { hs_str(s, v ? "true" : "false"); }
static void hs_jstr(HS& s, const char* v) {
  hs_str(s, "\"");
  for (const char* p = v; *p; p++) {
    if (*p == '"')       hs_str(s, "\\\"");
    else if (*p == '\\') hs_str(s, "\\\\");
    else if (*p == '\n') hs_str(s, "\\n");
    else                 hs_raw(s, p, 1);
  }
  hs_str(s, "\"");
}

// Temperature label for each sensor index.
// First 5 positions: T1..T5, then BAL, ENV, MOS.
static const char* temp_label(int idx) {
  static const char* const labels[] = { "T1","T2","T3","T4","T5","T6","T7","T8" };
  if (idx < 0 || idx >= 8) return "T?";
  return labels[idx];
}

}  // anonymous namespace

esp_err_t handle_bms_id(httpd_req_t* req) {
  // Extract pack id from URI: /api/bms/<id>
  const char* uri = req->uri;
  const char* last_slash = strrchr(uri, '/');
  if (!last_slash || *(last_slash + 1) == '\0') {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"Missing pack id\"}");
  }
  int pack_id = atoi(last_slash + 1);

  BmsSystemSnapshot snap = {};
  bool has_snap = bus::snapshot_bus::read(snap);

  if (!has_snap || pack_id < 0 || (uint8_t)pack_id >= snap.pack_count_configured) {
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"Pack not found\"}");
  }

  bms::poller::PollerStats ps = {};
  bms::poller::get_stats(ps);

  const BmsPackSnapshot& p = snap.pack[pack_id];
  uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  uint32_t age_ms = (p.last_seen_ms > 0 && now_ms >= p.last_seen_ms)
                    ? (now_ms - p.last_seen_ms) : 0;

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  HS s{req, {}, 0, ESP_OK};

  hs_str(s, "{");
  hs_str(s, "\"id\":"); hs_u32(s, (uint32_t)pack_id);
  hs_str(s, ",\"online\":"); hs_bool(s, p.online);
  hs_str(s, ",\"last_seen_age_ms\":"); hs_u32(s, age_ms);
  hs_str(s, ",\"soc\":"); hs_u32(s, p.soc);
  hs_str(s, ",\"soh\":"); hs_u32(s, p.soh);
  hs_str(s, ",\"cycles\":"); hs_u32(s, p.cycles);
  hs_str(s, ",\"pack_v\":"); hs_f2(s, p.pack_voltage);
  hs_str(s, ",\"current\":"); hs_f2(s, p.pack_current);
  hs_str(s, ",\"power\":"); hs_f1(s, p.pack_voltage * p.pack_current);
  hs_str(s, ",\"rem_ah\":"); hs_f1(s, p.rem_ah);
  hs_str(s, ",\"full_ah\":"); hs_f1(s, p.full_ah);
  hs_str(s, ",\"cell_count\":"); hs_u32(s, p.cell_count);
  hs_str(s, ",\"cell_min_v\":"); hs_f3(s, p.cell_min_v);
  hs_str(s, ",\"cell_max_v\":"); hs_f3(s, p.cell_max_v);
  hs_str(s, ",\"cell_min_idx\":"); hs_u32(s, p.cell_min_idx);
  hs_str(s, ",\"cell_max_idx\":"); hs_u32(s, p.cell_max_idx);

  // drift in mV integer
  int32_t drift_mv = (int32_t)(p.cell_drift_v * 1000.0f + 0.5f);
  hs_str(s, ",\"drift_mv\":"); hs_i32(s, drift_mv);

  // Cell voltages array
  hs_str(s, ",\"cells\":[");
  for (uint8_t c = 0; c < p.cell_count && c < 16; c++) {
    if (c > 0) hs_str(s, ",");
    hs_f3(s, p.cell_v[c]);
  }
  hs_str(s, "]");

  // Temperatures: regular sensors + MOS + ENV + BAL (if non-zero)
  hs_str(s, ",\"temps\":[");
  bool first_temp = true;
  for (uint8_t t = 0; t < p.temp_count && t < 8; t++) {
    if (!first_temp) hs_str(s, ",");
    first_temp = false;
    hs_str(s, "{\"lbl\":"); hs_jstr(s, temp_label(t));
    hs_str(s, ",\"val\":"); hs_f1(s, p.temp_c[t]);
    hs_str(s, "}");
  }
  // Supplemental sensors when non-zero
  if (p.mosfet_temp_c != 0.0f) {
    if (!first_temp) hs_str(s, ",");
    first_temp = false;
    hs_str(s, "{\"lbl\":\"MOS\",\"val\":"); hs_f1(s, p.mosfet_temp_c); hs_str(s, "}");
  }
  if (p.environment_temp_c != 0.0f) {
    if (!first_temp) hs_str(s, ",");
    first_temp = false;
    hs_str(s, "{\"lbl\":\"ENV\",\"val\":"); hs_f1(s, p.environment_temp_c); hs_str(s, "}");
  }
  if (p.balancer_temp_c != 0.0f) {
    if (!first_temp) hs_str(s, ",");
    hs_str(s, "{\"lbl\":\"BAL\",\"val\":"); hs_f1(s, p.balancer_temp_c); hs_str(s, "}");
  }
  hs_str(s, "]");

  // Alarm bits as hex string
  char alarm_hex[20];
  snprintf(alarm_hex, sizeof(alarm_hex), "0x%016llX",
           (unsigned long long)p.alarm_bits);
  hs_str(s, ",\"alarm_bits\":"); hs_jstr(s, alarm_hex);

  // Sysparam section
  uint32_t sp_age_s = (p.sysparam_valid && p.last_sysparam_ms > 0 && now_ms >= p.last_sysparam_ms)
                      ? (now_ms - p.last_sysparam_ms) / 1000u : 0u;
  hs_str(s, ",\"sysparam\":{");
  hs_str(s, "\"valid\":"); hs_bool(s, p.sysparam_valid);
  hs_str(s, ",\"age_s\":"); hs_u32(s, sp_age_s);
  if (p.sysparam_valid) {
    hs_str(s, ",\"cell_high_v\":"); hs_f3(s, p.sys_cell_high_v);
    hs_str(s, ",\"cell_low_v\":"); hs_f3(s, p.sys_cell_low_v);
    hs_str(s, ",\"module_high_v\":"); hs_f2(s, p.sys_module_high_v);
    hs_str(s, ",\"module_low_v\":"); hs_f2(s, p.sys_module_low_v);
    hs_str(s, ",\"module_under_v\":"); hs_f2(s, p.sys_module_under_v);
    hs_str(s, ",\"charge_t_min\":"); hs_f1(s, p.sys_charge_low_t);
    hs_str(s, ",\"charge_t_max\":"); hs_f1(s, p.sys_charge_high_t);
    hs_str(s, ",\"discharge_t_min\":"); hs_f1(s, p.sys_discharge_low_t);
    hs_str(s, ",\"discharge_t_max\":"); hs_f1(s, p.sys_discharge_high_t);
    hs_str(s, ",\"charge_max_a\":"); hs_f1(s, p.sys_charge_max_a);
    hs_str(s, ",\"discharge_max_a\":"); hs_f1(s, p.sys_discharge_max_a);
  }
  hs_str(s, "}");

  // Per-pack RS485 stats
  const auto& ps_pack = ps.pack[pack_id];
  uint32_t success_pct = (ps_pack.polls > 0)
    ? (ps_pack.ok * 100u / ps_pack.polls)
    : 0;
  hs_str(s, ",\"rs485\":{");
  hs_str(s, "\"polls\":"); hs_u32(s, ps_pack.polls);
  hs_str(s, ",\"ok\":"); hs_u32(s, ps_pack.ok);
  hs_str(s, ",\"timeouts\":"); hs_u32(s, ps_pack.timeouts);
  hs_str(s, ",\"errors\":"); hs_u32(s, ps_pack.errors);
  hs_str(s, ",\"success_pct\":"); hs_u32(s, success_pct);
  hs_str(s, "}");

  hs_str(s, "}");
  hs_flush(s);
  if (s.err == ESP_OK) s.err = httpd_resp_send_chunk(s.req, nullptr, 0);
  return s.err;
}

}  // namespace web
