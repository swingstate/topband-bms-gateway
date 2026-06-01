#include "server.h"
#include "auth.h"
#include "handlers_live.h"
#include "handlers_config.h"
#include "handlers_actions.h"
#include "handlers_history.h"
#include "handlers_diag.h"
#include "handlers_alerts.h"
#include "handlers_ota.h"
#include "handlers_notify.h"
#include "handlers_static.h"
#include "app/self_test.h"
#include "esp_log.h"
#include "esp_http_server.h"

static const char* TAG = "httpd";
static httpd_handle_t g_server = nullptr;

// Register a URI handler.
static void reg(httpd_handle_t srv,
                const char* uri, httpd_method_t method,
                esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t h = {};
  h.uri     = uri;
  h.method  = method;
  h.handler = handler;
  httpd_register_uri_handler(srv, &h);
}

// ── Auth-check wrapper ────────────────────────────────────────────────────────
// Wraps any handler with a session-cookie + CSRF check.
// Returns 401 if the check fails.

struct AuthCtx {
  esp_err_t (*real)(httpd_req_t*);
};

static esp_err_t auth_dispatch(httpd_req_t* req) {
  if (!web::auth::check(req)) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"unauthorized\"}");
  }
  auto* ctx = static_cast<AuthCtx*>(req->user_ctx);
  return ctx->real(req);
}

// Statically allocated auth context slots (one per protected route).
// We need as many as there are auth-required routes.
static AuthCtx g_auth_ctx[30];
static int     g_auth_ctx_count = 0;

static void reg_auth(httpd_handle_t srv,
                     const char* uri, httpd_method_t method,
                     esp_err_t (*handler)(httpd_req_t*)) {
  if (g_auth_ctx_count >= 30) {
    ESP_LOGE(TAG, "reg_auth: out of context slots");
    return;
  }
  g_auth_ctx[g_auth_ctx_count].real = handler;

  httpd_uri_t h = {};
  h.uri      = uri;
  h.method   = method;
  h.handler  = auth_dispatch;
  h.user_ctx = &g_auth_ctx[g_auth_ctx_count];
  g_auth_ctx_count++;

  httpd_register_uri_handler(srv, &h);
}

namespace web {

bool start_httpd(const Config& /*cfg*/) {
  g_auth_ctx_count = 0;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.task_priority      = 4;
  config.stack_size         = 12 * 1024;
  config.max_open_sockets   = 4;
  config.max_uri_handlers   = 32;
  config.recv_wait_timeout  = 5;
  config.send_wait_timeout  = 5;
  config.lru_purge_enable   = true;
  config.keep_alive_enable  = false;
  config.uri_match_fn       = httpd_uri_match_wildcard;

  esp_err_t ret = httpd_start(&g_server, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
    return false;
  }

  // ── Public endpoints (no auth) ────────────────────────────────────────────
  reg(g_server, "/api/health",         HTTP_GET,  handle_health);
  reg(g_server, "/api/auth/login",     HTTP_POST, web::auth::handler_login);
  reg(g_server, "/api/auth/logout",    HTTP_POST, web::auth::handler_logout);
  reg(g_server, "/api/auth/set_password", HTTP_POST, web::auth::handler_set_password);

  // ── Auth-protected API endpoints ──────────────────────────────────────────
  reg_auth(g_server, "/api/live",         HTTP_GET,  handle_live);
  reg_auth(g_server, "/api/bms/*",        HTTP_GET,  handle_bms_id);
  reg_auth(g_server, "/api/config",       HTTP_GET,  handle_config_get);
  reg_auth(g_server, "/api/config",       HTTP_POST, handle_config_post);
  reg_auth(g_server, "/api/wifi",              HTTP_POST, handle_wifi_post);
  reg_auth(g_server, "/api/wifi/status",       HTTP_GET,  handle_wifi_status_get);
  reg_auth(g_server, "/api/wifi/scan",         HTTP_GET,  handle_wifi_scan_get);
  reg_auth(g_server, "/api/wifi/configure",    HTTP_POST, handle_wifi_configure_post);
  reg_auth(g_server, "/api/restart",      HTTP_POST, handle_restart);
  reg_auth(g_server, "/api/backup",                    HTTP_GET,  handle_backup);
  reg_auth(g_server, "/api/factory_reset",             HTTP_POST, handle_factory_reset);
  reg_auth(g_server, "/api/svc/ha/discovery/send",    HTTP_POST, handle_ha_discovery_send);
  reg_auth(g_server, "/api/svc/ha/discovery/clear",   HTTP_POST, handle_ha_discovery_clear);

  // ── History endpoints (Phase H2) ─────────────────────────────────────────
  reg_auth(g_server, "/api/history",            HTTP_GET, handle_history);
  reg_auth(g_server, "/api/history/export.csv", HTTP_GET, handle_history_export);

  // ── Diag and alerts endpoints (Phase H3a) ─────────────────────────────────
  reg_auth(g_server, "/api/diag",              HTTP_GET, handle_diag);
  reg_auth(g_server, "/api/diag/coredump.bin", HTTP_GET, handle_diag_coredump);
  reg_auth(g_server, "/api/alerts",            HTTP_GET, handle_alerts_get);
  reg_auth(g_server, "/api/alerts",  HTTP_DELETE, handle_alerts_delete);

  // ── OTA endpoints (Phase I) ─────────────────────────────────────���─────────
  reg_auth(g_server, "/api/ota/upload", HTTP_POST, web::handlers_ota::handler_ota_upload);
  reg_auth(g_server, "/api/ota/status", HTTP_GET,  web::handlers_ota::handler_ota_status);

  // ── Config import / MQTT test (iter/config-io-mqtt-test) ──────────────────
  reg_auth(g_server, "/api/restore",   HTTP_POST, handle_restore);
  reg_auth(g_server, "/api/mqtt/test", HTTP_POST, handle_mqtt_test_post);
  reg_auth(g_server, "/api/mqtt/test", HTTP_GET,  handle_mqtt_test_get);

  // ── Notification test (iter/notifications-telegram) ───────────────────────
  reg_auth(g_server, "/api/notify/telegram/test", HTTP_POST,
           web::handle_notify_telegram_test_post);
  reg_auth(g_server, "/api/notify/telegram/test", HTTP_GET,
           web::handle_notify_telegram_test_get);

  // Static files — catch-all last (handles login.html, setup.html, etc.)
  reg(g_server, "/*", HTTP_GET, handle_static);

  app::self_test::mark_passed(app::self_test::HTTP_SERVER_UP);

  ESP_LOGI(TAG, "HTTP server started on port 80 (auth enabled)");
  return true;
}

void stop_httpd() {
  if (g_server) {
    httpd_stop(g_server);
    g_server = nullptr;
    ESP_LOGI(TAG, "HTTP server stopped");
  }
}

}  // namespace web
