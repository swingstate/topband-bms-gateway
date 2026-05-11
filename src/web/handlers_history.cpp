#include "handlers_history.h"
#include "storage/history_store.h"
#include "net/ntp.h"
#include "bus/types.h"
#include "esp_log.h"
#include <ArduinoJson.h>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstdlib>

static const char* TAG = "web_hist";

// ── Query-param helpers ───────────────────────────────────────────────────────
static bool query_param(httpd_req_t* req, const char* key, char* out, size_t len) {
  char* uri_buf = (char*)malloc(512);
  if (!uri_buf) return false;
  strncpy(uri_buf, req->uri, 511);
  uri_buf[511] = '\0';
  esp_err_t r = httpd_query_key_value(uri_buf, key, out, len);
  free(uri_buf);
  return r == ESP_OK;
}

// Check whether 'target' appears in comma-separated 'list' (or list == "all").
static bool series_requested(const char* list, const char* target) {
  if (strcmp(list, "all") == 0) return true;
  // Simple linear scan for comma-delimited token.
  const char* p = list;
  size_t tlen = strlen(target);
  while (*p) {
    const char* comma = strchr(p, ',');
    size_t seg = comma ? (size_t)(comma - p) : strlen(p);
    if (seg == tlen && strncmp(p, target, seg) == 0) return true;
    p += seg;
    if (*p == ',') p++;
  }
  return false;
}

// ── Fine ring → JSON series ───────────────────────────────────────────────────
// Module-level static buffers for fine/coarse reads (H1 lesson: no large arrays on stack).
static HistoryFinePoint   s_fine_buf[HISTORY_FINE_CAPACITY]   = {};
static HistoryCoarsePoint s_coarse_buf[HISTORY_COARSE_CAPACITY] = {};

static void add_fine_series(JsonDocument& doc, const char* series_name) {
  size_t count = storage::history_store::read_fine(s_fine_buf, HISTORY_FINE_CAPACITY);
  uint32_t base   = storage::history_store::fine_epoch_base();
  uint32_t res    = HISTORY_FINE_RESOLUTION_S;

  JsonObject ser = doc["series"].add<JsonObject>();
  ser["name"]        = series_name;
  ser["tier"]        = "fine";
  ser["resolution_s"]= res;
  ser["window_s"]    = HISTORY_FINE_CAPACITY * res;
  ser["t0_epoch"]    = base;

  JsonArray pts = ser["points"].to<JsonArray>();
  for (size_t i = 0; i < count; i++) {
    const HistoryFinePoint& fp = s_fine_buf[i];
    float val = 0.0f;
    if (strcmp(series_name, "power") == 0)       val = (float)fp.power_w;
    else if (strcmp(series_name, "voltage") == 0) val = (float)fp.voltage_x100 / 100.0f;
    else if (strcmp(series_name, "soc") == 0)     val = (float)fp.soc_x10 / 10.0f;
    else if (strcmp(series_name, "temp") == 0) val = (float)fp.temp_x10 / 10.0f;
    else {
      pts.add(nullptr);
      continue;
    }
    if (fp.flags == 0) pts.add(nullptr);  // placeholder, NTP not synced
    else               pts.add(val);
  }
}

static void add_coarse_series(JsonDocument& doc, const char* series_name) {
  size_t count = storage::history_store::read_coarse(s_coarse_buf, HISTORY_COARSE_CAPACITY);
  uint32_t base = storage::history_store::coarse_epoch_base();
  uint32_t res  = HISTORY_COARSE_RESOLUTION_S;

  JsonObject ser = doc["series"].add<JsonObject>();
  ser["name"]        = series_name;
  ser["tier"]        = "coarse";
  ser["resolution_s"]= res;
  ser["window_s"]    = (uint32_t)HISTORY_COARSE_CAPACITY * res;
  ser["t0_epoch"]    = base;

  JsonArray pts  = ser["points"].to<JsonArray>();
  JsonArray pmin = ser["min"].to<JsonArray>();
  JsonArray pmax = ser["max"].to<JsonArray>();

  for (size_t i = 0; i < count; i++) {
    const HistoryCoarsePoint& cp = s_coarse_buf[i];
    if (cp.t_epoch == 0) {
      pts.add(nullptr); pmin.add(nullptr); pmax.add(nullptr);
      continue;
    }
    if (strcmp(series_name, "power") == 0) {
      pts.add((float)cp.power_avg);
      pmin.add((float)cp.power_min);
      pmax.add((float)cp.power_max);
    } else if (strcmp(series_name, "voltage") == 0) {
      pts.add((float)cp.volt_avg / 100.0f);
      pmin.add((float)cp.volt_min / 100.0f);
      pmax.add((float)cp.volt_max / 100.0f);
    } else if (strcmp(series_name, "soc") == 0) {
      // SOC stored as avg only (no min/max in coarse ring to stay within 24-byte budget).
      pts.add((float)cp.soc_avg / 10.0f);
      pmin.add(nullptr);
      pmax.add(nullptr);
    } else if (strcmp(series_name, "temp") == 0) {
      pts.add((float)cp.temp_avg / 10.0f);
      pmin.add((float)cp.temp_min / 10.0f);
      pmax.add((float)cp.temp_max / 10.0f);
    } else {
      pts.add(nullptr); pmin.add(nullptr); pmax.add(nullptr);
    }
  }
}

namespace web {

esp_err_t handle_history(httpd_req_t* req) {
  char series_buf[64] = "power";
  char tier_buf[16]   = "coarse";
  query_param(req, "series", series_buf, sizeof(series_buf));
  query_param(req, "tier",   tier_buf,   sizeof(tier_buf));

  bool want_fine   = (strcmp(tier_buf, "fine") == 0 || strcmp(tier_buf, "both") == 0);
  bool want_coarse = (strcmp(tier_buf, "coarse") == 0 || strcmp(tier_buf, "both") == 0);

  // Parse requested series list (comma-separated or "all").
  static const char* ALL_SERIES[] = { "power", "voltage", "soc", "temp" };

  // ArduinoJson document. History payload can be large for coarse: 2016 × (avg+min+max)
  // Use PSRAM-backed allocation via ArduinoJson's JsonDocument (heap-allocated).
  // Estimate: 2016 points × 4 series × 3 arrays × ~8 chars/value ≈ 200 KB worst-case.
  // Use streaming chunked response to avoid one huge buffer.
  // For H2 we use the simpler single-buffer approach with a 64 KB doc,
  // since we serve at most 48h ≈ 576 coarse points typically.
  JsonDocument doc;
  doc["now_ts_s"] = net::ntp::now_unix_s();
  doc["series"].to<JsonArray>();  // initialise as array

  for (const char* sname : ALL_SERIES) {
    if (!series_requested(series_buf, sname)) continue;
    if (want_fine)   add_fine_series(doc, sname);
    if (want_coarse) add_coarse_series(doc, sname);
  }

  size_t est = measureJson(doc) + 1;
  char* buf = (char*)malloc(est);
  if (!buf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  size_t n = serializeJson(doc, buf, est);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  esp_err_t ret = httpd_resp_send(req, buf, (ssize_t)n);
  free(buf);

  if (ret != ESP_OK)
    ESP_LOGD(TAG, "client disconnected during /api/history");
  return ret;
}

esp_err_t handle_history_export(httpd_req_t* req) {
  // Read both rings.
  size_t fine_n   = storage::history_store::read_fine(s_fine_buf, HISTORY_FINE_CAPACITY);
  size_t coarse_n = storage::history_store::read_coarse(s_coarse_buf, HISTORY_COARSE_CAPACITY);
  uint32_t fine_base   = storage::history_store::fine_epoch_base();
  uint32_t coarse_base = storage::history_store::coarse_epoch_base();

  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=\"history.csv\"");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  // Header row.
  const char* hdr = "timestamp_local,tier,power_w,voltage_v,soc_pct,temp_c\r\n";
  if (httpd_resp_send_chunk(req, hdr, strlen(hdr)) != ESP_OK) {
    return httpd_resp_send_chunk(req, nullptr, 0);
  }

  char row[128];
  // Coarse rows (older data first).
  for (size_t i = 0; i < coarse_n; i++) {
    const HistoryCoarsePoint& cp = s_coarse_buf[i];
    uint32_t ts = (coarse_base != 0) ? coarse_base + (uint32_t)i * HISTORY_COARSE_RESOLUTION_S
                                      : cp.t_epoch;
    if (ts == 0) continue;
    time_t t = (time_t)ts;
    struct tm tm_local = {};
    localtime_r(&t, &tm_local);
    char ts_str[32];
    strftime(ts_str, sizeof(ts_str), "%Y-%m-%dT%H:%M:%S", &tm_local);
    snprintf(row, sizeof(row), "%s,coarse,%d,%.2f,%.1f,%.1f\r\n",
             ts_str,
             (int)cp.power_avg,
             (float)cp.volt_avg / 100.0f,
             (float)cp.soc_avg  / 10.0f,
             (float)cp.temp_avg / 10.0f);
    if (httpd_resp_send_chunk(req, row, strlen(row)) != ESP_OK)
      return httpd_resp_send_chunk(req, nullptr, 0);
  }

  // Fine rows (recent high-resolution data).
  for (size_t i = 0; i < fine_n; i++) {
    const HistoryFinePoint& fp = s_fine_buf[i];
    uint32_t ts = (fine_base != 0)
                  ? fine_base + (uint32_t)fp.t_offset_s
                  : 0;
    if (ts == 0) continue;
    time_t t = (time_t)ts;
    struct tm tm_local = {};
    localtime_r(&t, &tm_local);
    char ts_str[32];
    strftime(ts_str, sizeof(ts_str), "%Y-%m-%dT%H:%M:%S", &tm_local);
    snprintf(row, sizeof(row), "%s,fine,%d,%.2f,%.1f,%.1f\r\n",
             ts_str,
             (int)fp.power_w,
             (float)fp.voltage_x100 / 100.0f,
             (float)fp.soc_x10      / 10.0f,
             (float)fp.temp_x10     / 10.0f);
    if (httpd_resp_send_chunk(req, row, strlen(row)) != ESP_OK)
      return httpd_resp_send_chunk(req, nullptr, 0);
  }

  return httpd_resp_send_chunk(req, nullptr, 0);  // end of chunked response
}

}  // namespace web
