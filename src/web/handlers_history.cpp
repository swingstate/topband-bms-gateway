#include "handlers_history.h"
#include "storage/history_store.h"
#include "net/ntp.h"
#include "bus/types.h"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <cmath>

static const char* TAG = "web_hist";

// ── Query-param helpers ───────────────────────────────────────────────────────
static bool query_param(httpd_req_t* req, const char* key, char* out, size_t len) {
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen == 0) return false;
  qlen++;  // null terminator
  char* qbuf = (char*)malloc(qlen);
  if (!qbuf) return false;
  esp_err_t r = httpd_req_get_url_query_str(req, qbuf, qlen);
  if (r == ESP_OK) {
    r = httpd_query_key_value(qbuf, key, out, len);
  }
  free(qbuf);
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

// ── Read buffers (BSS — no heap) ─────────────────────────────────────────────
static HistoryFinePoint   s_fine_buf[HISTORY_FINE_CAPACITY]   = {};
static HistoryCoarsePoint s_coarse_buf[HISTORY_COARSE_CAPACITY] = {};

// ── Zero-heap streaming JSON writer ──────────────────────────────────────────
// Writes JSON directly to the HTTP chunked response using a 2 KB stack buffer.
// No heap allocation at all — critical for avoiding OOM under concurrent loads.

struct HStream {
  httpd_req_t* req;
  char         buf[2048];
  size_t       pos;
  esp_err_t    err;
};

static void hs_flush(HStream& s) {
  if (s.pos > 0 && s.err == ESP_OK)
    s.err = httpd_resp_send_chunk(s.req, s.buf, (ssize_t)s.pos);
  s.pos = 0;
}

static void hs_raw(HStream& s, const char* data, size_t n) {
  while (n > 0 && s.err == ESP_OK) {
    size_t space = sizeof(s.buf) - s.pos;
    size_t copy  = n < space ? n : space;
    memcpy(s.buf + s.pos, data, copy);
    s.pos += copy; n -= copy; data += copy;
    if (s.pos == sizeof(s.buf)) hs_flush(s);
  }
}
static void hs_str(HStream& s, const char* str) { hs_raw(s, str, strlen(str)); }
static void hs_uint(HStream& s, uint32_t v) {
  char t[12]; snprintf(t, sizeof(t), "%u", (unsigned)v); hs_str(s, t);
}
static void hs_int(HStream& s, int v) {
  char t[12]; snprintf(t, sizeof(t), "%d", v); hs_str(s, t);
}
static void hs_f1(HStream& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.1f", v); hs_str(s, t);
}
static void hs_f2(HStream& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.2f", v); hs_str(s, t);
}

static void hs_fine_series(HStream& s, const char* name, bool first) {
  size_t   count = storage::history_store::read_fine(s_fine_buf, HISTORY_FINE_CAPACITY);
  uint32_t res   = HISTORY_FINE_RESOLUTION_S;

  // Anchor t0_epoch to current NTP time rather than stored epoch_base.
  // epoch_base can be permanently stuck at a wrong value if time() was off
  // when the first sample was written. Deriving from now_unix_s() keeps the
  // x-axis correct regardless of how epoch_base was set. Falls back to
  // epoch_base only when NTP has not yet synced. See docs/diag-chart-tz-offset.md.
  uint32_t now_s = net::ntp::now_unix_s();
  uint32_t base  = (now_s != 0 && count > 0)
                   ? now_s - (uint32_t)(count - 1) * res
                   : storage::history_store::fine_epoch_base();

  if (!first) hs_str(s, ",");
  hs_str(s, "{\"name\":\""); hs_str(s, name); hs_str(s, "\"");
  hs_str(s, ",\"tier\":\"fine\",\"resolution_s\":"); hs_uint(s, res);
  hs_str(s, ",\"window_s\":"); hs_uint(s, HISTORY_FINE_CAPACITY * res);
  hs_str(s, ",\"t0_epoch\":"); hs_uint(s, base);
  hs_str(s, ",\"points\":[");

  for (size_t i = 0; i < count; i++) {
    if (i > 0) hs_str(s, ",");
    const HistoryFinePoint& fp = s_fine_buf[i];
    if (fp.flags == 0) { hs_str(s, "null"); continue; }
    if      (!strcmp(name, "power"))   hs_int(s, (int)fp.power_w);
    else if (!strcmp(name, "voltage")) hs_f2(s, (float)fp.voltage_x100 / 100.0f);
    else if (!strcmp(name, "soc"))     hs_f1(s, (float)fp.soc_x10 / 10.0f);
    else if (!strcmp(name, "temp"))    hs_f1(s, (float)fp.temp_x10 / 10.0f);
    else if (!strcmp(name, "drift"))   hs_uint(s, storage::history_store::read_fine_drift_at(i));
    else                               hs_str(s, "null");
  }
  hs_str(s, "]}");
}

static void hs_coarse_series(HStream& s, const char* name, bool first) {
  size_t   count = storage::history_store::read_coarse(s_coarse_buf, HISTORY_COARSE_CAPACITY);
  uint32_t res   = HISTORY_COARSE_RESOLUTION_S;

  // Same NTP-anchored t0_epoch fix as hs_fine_series. See docs/diag-chart-tz-offset.md.
  uint32_t now_s = net::ntp::now_unix_s();
  uint32_t base  = (now_s != 0 && count > 0)
                   ? now_s - (uint32_t)(count - 1) * res
                   : storage::history_store::coarse_epoch_base();

  if (!first) hs_str(s, ",");
  hs_str(s, "{\"name\":\""); hs_str(s, name); hs_str(s, "\"");
  hs_str(s, ",\"tier\":\"coarse\",\"resolution_s\":"); hs_uint(s, res);
  hs_str(s, ",\"window_s\":"); hs_uint(s, (uint32_t)HISTORY_COARSE_CAPACITY * res);
  hs_str(s, ",\"t0_epoch\":"); hs_uint(s, base);
  hs_str(s, ",\"points\":[");

  for (size_t i = 0; i < count; i++) {
    if (i > 0) hs_str(s, ",");
    const HistoryCoarsePoint& cp = s_coarse_buf[i];
    if (cp.t_epoch == 0) { hs_str(s, "null"); continue; }
    if      (!strcmp(name, "power"))   hs_int(s, (int)cp.power_avg);
    else if (!strcmp(name, "voltage")) hs_f2(s, (float)cp.volt_avg / 100.0f);
    else if (!strcmp(name, "soc"))     hs_f1(s, (float)cp.soc_avg  / 10.0f);
    else if (!strcmp(name, "temp"))    hs_f1(s, (float)cp.temp_avg / 10.0f);
    else if (!strcmp(name, "drift"))   hs_uint(s, storage::history_store::read_coarse_drift_at(i));
    else                               hs_str(s, "null");
  }
  hs_str(s, "]}");
}

namespace web {

esp_err_t handle_history(httpd_req_t* req) {
  char series_buf[64] = "power";
  char tier_buf[16]   = "fine";
  query_param(req, "series", series_buf, sizeof(series_buf));
  query_param(req, "tier",   tier_buf,   sizeof(tier_buf));

  bool want_fine   = (strcmp(tier_buf, "fine")   == 0 || strcmp(tier_buf, "both") == 0);
  bool want_coarse = (strcmp(tier_buf, "coarse") == 0 || strcmp(tier_buf, "both") == 0);
  static const char* ALL_SERIES[] = { "power", "voltage", "soc", "temp", "drift" };

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  // Stream JSON directly — zero heap allocation.
  HStream s = { req, {}, 0, ESP_OK };

  hs_str(s, "{\"now_ts_s\":"); hs_uint(s, net::ntp::now_unix_s());
  hs_str(s, ",\"series\":[");

  bool first = true;
  for (const char* sname : ALL_SERIES) {
    if (!series_requested(series_buf, sname)) continue;
    if (want_fine)   { hs_fine_series(s, sname, first);   first = false; }
    if (want_coarse) { hs_coarse_series(s, sname, first); first = false; }
  }

  hs_str(s, "]}");
  hs_flush(s);
  if (s.err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  else ESP_LOGD(TAG, "client disconnected during /api/history");
  return s.err;
}

esp_err_t handle_history_export(httpd_req_t* req) {
  // Read both rings.
  size_t fine_n   = storage::history_store::read_fine(s_fine_buf, HISTORY_FINE_CAPACITY);
  size_t coarse_n = storage::history_store::read_coarse(s_coarse_buf, HISTORY_COARSE_CAPACITY);

  // Anchor bases to NTP time so the CSV is self-correcting. Same reasoning
  // as hs_fine_series / hs_coarse_series. See docs/diag-chart-tz-offset.md.
  uint32_t now_s = net::ntp::now_unix_s();
  uint32_t fine_base = (now_s != 0 && fine_n > 0)
                       ? now_s - (uint32_t)(fine_n - 1) * HISTORY_FINE_RESOLUTION_S
                       : storage::history_store::fine_epoch_base();
  uint32_t coarse_base = (now_s != 0 && coarse_n > 0)
                         ? now_s - (uint32_t)(coarse_n - 1) * HISTORY_COARSE_RESOLUTION_S
                         : storage::history_store::coarse_epoch_base();

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
  // Use index-based ts (fine_base + i * res) for consistency with the chart's t0_epoch logic.
  for (size_t i = 0; i < fine_n; i++) {
    const HistoryFinePoint& fp = s_fine_buf[i];
    if (fp.flags == 0) continue;
    uint32_t ts = (fine_base != 0) ? fine_base + (uint32_t)i * HISTORY_FINE_RESOLUTION_S : 0;
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
