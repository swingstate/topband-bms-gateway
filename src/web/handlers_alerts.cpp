#include "handlers_alerts.h"
#include "auth.h"
#include "storage/alerts_store.h"
#include "diag/alerts.h"
#include "bus/queues.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "web_alerts";

// ── Query-param helper (shared pattern) ──────────────────────────────────────

static bool query_param(httpd_req_t* req, const char* key, char* out, size_t len) {
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen == 0) return false;
  qlen++;
  char* qbuf = (char*)malloc(qlen);
  if (!qbuf) return false;
  esp_err_t r = httpd_req_get_url_query_str(req, qbuf, qlen);
  if (r == ESP_OK) r = httpd_query_key_value(qbuf, key, out, len);
  free(qbuf);
  return r == ESP_OK;
}

// ── Streaming JSON helpers ────────────────────────────────────────────────────

struct AStream {
  httpd_req_t* req;
  char         buf[2048];
  size_t       pos;
  esp_err_t    err;
};

static void as_flush(AStream& s) {
  if (s.pos > 0 && s.err == ESP_OK)
    s.err = httpd_resp_send_chunk(s.req, s.buf, (ssize_t)s.pos);
  s.pos = 0;
}
static void as_raw(AStream& s, const char* data, size_t n) {
  while (n > 0 && s.err == ESP_OK) {
    size_t space = sizeof(s.buf) - s.pos;
    size_t copy  = n < space ? n : space;
    memcpy(s.buf + s.pos, data, copy);
    s.pos += copy; n -= copy; data += copy;
    if (s.pos == sizeof(s.buf)) as_flush(s);
  }
}
static void as_str(AStream& s, const char* str) { as_raw(s, str, strlen(str)); }
static void as_uint(AStream& s, uint64_t v) {
  char t[24]; snprintf(t, sizeof(t), "%llu", (unsigned long long)v); as_str(s, t);
}
static void as_json_str(AStream& s, const char* v) {
  as_str(s, "\"");
  for (const char* p = v; *p; p++) {
    if (*p == '"')       as_str(s, "\\\"");
    else if (*p == '\\') as_str(s, "\\\\");
    else if (*p == '\n') as_str(s, "\\n");
    else if (*p == '\r') as_str(s, "\\r");
    else if (*p == '\t') as_str(s, "\\t");
    else                 as_raw(s, p, 1);
  }
  as_str(s, "\"");
}

static const char* severity_name(uint8_t sev) {
  switch (sev) {
    case 0: return "INFO";
    case 1: return "WARN";
    case 2: return "ERROR";
    case 3: return "CRITICAL";
    default: return "UNKNOWN";
  }
}

// Derive source name from flags low byte.
static const char* source_from_flags(uint16_t flags) {
  uint8_t src_id = (uint8_t)(flags & 0xFFu);
  return diag::alerts::source_name(static_cast<diag::alerts::Source>(src_id));
}

// ── GET /api/alerts ───────────────────────────────────────────────────────────

namespace web {

esp_err_t handle_alerts_get(httpd_req_t* req) {
  char tmp[16] = {};
  uint32_t limit = 50;
  uint32_t skip  = 0;
  uint8_t  min_sev = 0;

  if (query_param(req, "limit", tmp, sizeof(tmp))) limit = (uint32_t)atoi(tmp);
  if (query_param(req, "skip",  tmp, sizeof(tmp))) skip  = (uint32_t)atoi(tmp);
  if (query_param(req, "min_severity", tmp, sizeof(tmp))) min_sev = (uint8_t)atoi(tmp);

  if (limit == 0 || limit > 200) limit = 50;

  // Heap-allocate the read buffer — AlertEntry is 132 B × 200 = 26 KB.
  AlertEntry* buf = static_cast<AlertEntry*>(malloc(limit * sizeof(AlertEntry)));
  if (!buf) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
  }

  size_t count = storage::alerts_store::read(buf, limit, (size_t)skip);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  AStream s = { req, {}, 0, ESP_OK };

  as_str(s, "{\"total\":");
  as_uint(s, storage::alerts_store::stored_count());
  as_str(s, ",\"alerts\":[");

  bool first = true;
  for (size_t i = 0; i < count; i++) {
    const AlertEntry& a = buf[i];
    if (a.severity < min_sev) continue;
    if (!first) as_str(s, ",");
    first = false;

    as_str(s, "{\"ts_epoch\":");
    as_uint(s, a.ts_epoch);
    as_str(s, ",\"uptime_s\":");
    as_uint(s, a.uptime_s);
    as_str(s, ",\"severity\":");
    as_json_str(s, severity_name(a.severity));
    as_str(s, ",\"severity_n\":");
    as_uint(s, a.severity);
    as_str(s, ",\"source\":");
    as_json_str(s, source_from_flags(a.flags));
    as_str(s, ",\"message\":");
    as_json_str(s, a.message);
    as_str(s, "}");
  }

  free(buf);

  as_str(s, "]}");
  as_flush(s);
  if (s.err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  else ESP_LOGD(TAG, "client disconnected during /api/alerts");
  return s.err;
}

// ── DELETE /api/alerts ────────────────────────────────────────────────────────

esp_err_t handle_alerts_delete(httpd_req_t* req) {
  storage::alerts_store::clear_all();
  ESP_LOGI(TAG, "All alerts cleared");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

}  // namespace web
