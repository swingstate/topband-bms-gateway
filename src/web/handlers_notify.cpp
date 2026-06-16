#include "handlers_notify.h"
#include "app/boot.h"
#include "notify/notify.h"
#include "storage/config.h"
#include "esp_http_server.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>
#include <cstdint>

static size_t read_body_notify(httpd_req_t* req, char* buf, size_t max_len) {
  size_t remaining = req->content_len;
  size_t total = 0;
  if (remaining == 0 || remaining > max_len) return 0;
  while (remaining > 0) {
    int n = httpd_req_recv(req, buf + total, remaining);
    if (n <= 0) return total;
    total     += (size_t)n;
    remaining -= (size_t)n;
  }
  buf[total] = '\0';
  return total;
}

static esp_err_t send_err_notify(httpd_req_t* req, int code, const char* msg) {
  char body[200];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
  httpd_resp_set_type(req, "application/json");
  if (code == 400)      httpd_resp_set_status(req, "400 Bad Request");
  else if (code == 409) httpd_resp_set_status(req, "409 Conflict");
  else                  httpd_resp_set_status(req, "500 Internal Server Error");
  return httpd_resp_sendstr(req, body);
}

namespace web {

// POST /api/notify/telegram/test
// Body JSON keys: notify_telegram_enabled, notify_telegram_token, notify_telegram_chat_id.
// Token field blank → use the saved token (leave-blank-to-keep pattern).
esp_err_t handle_notify_telegram_test_post(httpd_req_t* req) {
  // 1024 B: the test POST carries only 3 notify fields (~150 B typical), but
  // keep margin so an oversized body is rejected cleanly rather than causing
  // httpd to RST the connection with an unconsumed socket buffer.
  char body[1024] = {};
  size_t n = read_body_notify(req, body, sizeof(body) - 1);
  if (n == 0) return send_err_notify(req, 400, "Empty body");

  JsonDocument doc;
  if (deserializeJson(doc, body, n) != DeserializationError::Ok)
    return send_err_notify(req, 400, "JSON parse error");

  // Build a Config from form values.  Start from DEFAULT_CONFIG so unrecognised
  // fields stay at safe values; overlay only the Telegram-relevant fields.
  Config form_cfg = DEFAULT_CONFIG;
  form_cfg.notify_telegram_enabled = doc["notify_telegram_enabled"] | false;

  const char* token = doc["notify_telegram_token"] | "";
  snprintf(form_cfg.notify_telegram_token, sizeof(form_cfg.notify_telegram_token),
           "%s", token);

  const char* chat_id = doc["notify_telegram_chat_id"] | "";
  snprintf(form_cfg.notify_telegram_chat_id, sizeof(form_cfg.notify_telegram_chat_id),
           "%s", chat_id);

  // notify::test() merges form_cfg + saved_cfg internally (blank token → use saved).
  const Config& saved_cfg = app::get_config();
  bool accepted = notify::test("telegram", form_cfg, saved_cfg);
  if (!accepted)
    return send_err_notify(req, 409, "Test already running");

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true,\"status\":\"running\"}");
}

// GET /api/notify/telegram/test
esp_err_t handle_notify_telegram_test_get(httpd_req_t* req) {
  notify::TestResult r = notify::test_result();

  const char* st = "idle";
  switch (r.status) {
    case notify::TestStatus::Running: st = "running"; break;
    case notify::TestStatus::Ok:      st = "ok";      break;
    case notify::TestStatus::Failed:  st = "failed";  break;
    default: break;
  }

  // Escape message for inline JSON.
  char safe_msg[160];
  size_t si = 0;
  for (size_t i = 0; r.message[i] && si < sizeof(safe_msg) - 2; ++i) {
    char c = r.message[i];
    if (c == '"' || c == '\\') safe_msg[si++] = '\\';
    safe_msg[si++] = c;
  }
  safe_msg[si] = '\0';

  char resp[256];
  snprintf(resp, sizeof(resp),
           "{\"status\":\"%s\",\"message\":\"%s\"}", st, safe_msg);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, resp);
}

// GET /api/notify/status
esp_err_t handle_notify_status_get(httpd_req_t* req) {
  notify::TelegramStatus s = notify::telegram_status();

  char resp[200];
  snprintf(resp, sizeof(resp),
           "{\"token_stored\":%s,\"chat_id_stored\":%s,"
           "\"verified\":%s,\"last_ok_ts\":%lu}",
           s.token_stored   ? "true" : "false",
           s.chat_id_stored ? "true" : "false",
           s.verified       ? "true" : "false",
           (unsigned long)s.last_ok_ts);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, resp);
}

// GET /api/notify/alert-types
// Alert-type descriptor table (mirrors notify.cpp k_event_desc + SafetyState::SafetyEvent).
// This endpoint is the single authoritative source for the UI's alert-toggle list.
// When new SafetyEvent values are added to safety_state.h and their entry is added here,
// the UI picks them up automatically — no JS edit required.
esp_err_t handle_notify_alert_types_get(httpd_req_t* req) {
  struct AlertTypeDesc {
    uint8_t     id;
    const char* name;
    const char* group;  // for UI grouping: "voltage", "temperature", "cell", "system"
  };

  static const AlertTypeDesc k_types[] = {
    {  1, "Pack went offline",             "system"      },
    {  2, "Pack came online",              "system"      },
    {  3, "Over-voltage started",          "voltage"     },
    {  4, "Over-voltage cleared",          "voltage"     },
    {  5, "Cell over-voltage started",     "voltage"     },
    {  6, "Cell over-voltage cleared",     "voltage"     },
    {  7, "Under-voltage started",         "voltage"     },
    {  8, "Under-voltage cleared",         "voltage"     },
    {  9, "Charge stopped: temperature",   "temperature" },
    { 10, "Charge resumed: temperature",   "temperature" },
    { 11, "Discharge stopped: temperature","temperature" },
    { 12, "Discharge resumed: temperature","temperature" },
    { 13, "Cell imbalance detected",       "cell"        },
    { 14, "Cell imbalance cleared",        "cell"        },
    { 15, "BMS reported alarm",            "system"      },
    { 16, "No packs online",               "system"      },
    { 17, "Packs online recovered",        "system"      },
  };
  static constexpr size_t K_COUNT = sizeof(k_types) / sizeof(k_types[0]);

  // Build JSON array. Each element: {"id":N,"name":"...","group":"..."}
  // Max size: K_COUNT * ~80 chars + brackets = ~1400 chars; 1600 is safe.
  char buf[1600];
  size_t pos = 0;
  buf[pos++] = '[';
  for (size_t i = 0; i < K_COUNT; ++i) {
    char entry[96];
    int n = snprintf(entry, sizeof(entry),
                     "%s{\"id\":%u,\"name\":\"%s\",\"group\":\"%s\"}",
                     (i > 0) ? "," : "",
                     (unsigned)k_types[i].id,
                     k_types[i].name,
                     k_types[i].group);
    if (n > 0 && (pos + (size_t)n) < sizeof(buf) - 2) {
      memcpy(buf + pos, entry, (size_t)n);
      pos += (size_t)n;
    }
  }
  buf[pos++] = ']';
  buf[pos]   = '\0';

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
  return httpd_resp_sendstr(req, buf);
}

}  // namespace web
