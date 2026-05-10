#include "server.h"
#include "handlers_live.h"
#include "handlers_config.h"
#include "handlers_actions.h"
#include "handlers_static.h"
#include "esp_log.h"
#include "esp_http_server.h"

static const char* TAG = "httpd";
static httpd_handle_t g_server = nullptr;

// Register a single URI handler. Zero-initialize the struct so all optional
// fields (user_ctx, is_websocket, etc.) are always set to safe defaults.
static void reg(httpd_handle_t srv,
                const char* uri, httpd_method_t method,
                esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t h = {};
  h.uri     = uri;
  h.method  = method;
  h.handler = handler;
  httpd_register_uri_handler(srv, &h);
}

namespace web {

bool start_httpd(const Config& /*cfg*/) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.task_priority      = 4;
  config.stack_size         = 12 * 1024;
  config.max_open_sockets   = 4;
  config.max_uri_handlers   = 32;
  config.recv_wait_timeout  = 5;
  config.send_wait_timeout  = 5;
  config.lru_purge_enable   = true;
  config.keep_alive_enable  = false;
  // Wildcard URI matching so the static handler catches unmatched paths.
  config.uri_match_fn       = httpd_uri_match_wildcard;

  esp_err_t ret = httpd_start(&g_server, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
    return false;
  }

  // ── Register routes (specific before wildcard) ────────────────────────────
  reg(g_server, "/api/health",  HTTP_GET,  handle_health);
  reg(g_server, "/api/live",    HTTP_GET,  handle_live);
  reg(g_server, "/api/bms/*",   HTTP_GET,  handle_bms_id);
  reg(g_server, "/api/config",  HTTP_GET,  handle_config_get);
  reg(g_server, "/api/config",  HTTP_POST, handle_config_post);
  reg(g_server, "/api/wifi",    HTTP_POST, handle_wifi_post);
  reg(g_server, "/api/restart", HTTP_POST, handle_restart);
  reg(g_server, "/api/backup",  HTTP_GET,  handle_backup);
  reg(g_server, "/wifi-setup",  HTTP_GET,  handle_wifi_setup_page);
  reg(g_server, "/*",           HTTP_GET,  handle_static);  // catch-all last

  ESP_LOGI(TAG, "HTTP server started on port 80");
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
