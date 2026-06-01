#include "handlers_notify.h"
#include "app/boot.h"
#include "notify/notify.h"
#include "storage/config.h"
#include "esp_http_server.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

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
  char body[512] = {};
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

}  // namespace web
