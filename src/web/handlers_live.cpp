#include "handlers_live.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "app/version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <ArduinoJson.h>
#include <cstdio>
#include <cstring>
#include <string>

static const char* TAG = "web_live";

namespace web {

esp_err_t handle_live(httpd_req_t* req) {
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

  // Estimate size then allocate.
  size_t est = measureJson(doc) + 1;
  char* buf = (char*)malloc(est);
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

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "client disconnected during /api/live send");
  }
  return ret;
}

esp_err_t handle_bms_id(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_status(req, "501 Not Implemented");
  return httpd_resp_sendstr(req,
    "{\"error\":\"Per-pack detail coming in Phase H\",\"code\":501}");
}

}  // namespace web
