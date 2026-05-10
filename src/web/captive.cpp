#include "captive.h"
#include "net/wifi.h"
#include "storage/lfs_store.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

static const char* TAG = "captive";

static httpd_handle_t g_captive = nullptr;

// ── Minimal inline fallback served if LittleFS is unavailable ─────────────────
static constexpr const char* FALLBACK_HTML =
    "<!DOCTYPE html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>TopBand Setup</title></head><body>"
    "<h2>TopBand BMS Gateway — WiFi Setup</h2>"
    "<p>LittleFS not available. Please flash the firmware again.</p>"
    "</body></html>";

// ── LittleFS file helper ──────────────────────────────────────────────────────

static esp_err_t serve_lfs_file(httpd_req_t* req, const char* lfs_path,
                                const char* mime) {
  if (!storage::lfs::exists(lfs_path)) {
    httpd_resp_set_status(req, "404 Not Found");
    return httpd_resp_sendstr(req, "{\"error\":\"Not found\"}");
  }
  FILE* f = fopen(lfs_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"open failed\"}");
  }
  httpd_resp_set_type(req, mime);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  static constexpr size_t CHUNK = 2048;
  char* buf = (char*)malloc(CHUNK);
  if (!buf) {
    fclose(f);
    return httpd_resp_sendstr(req, "OOM");
  }

  esp_err_t ret = ESP_OK;
  size_t n;
  while ((n = fread(buf, 1, CHUNK, f)) > 0) {
    ret = httpd_resp_send_chunk(req, buf, (ssize_t)n);
    if (ret != ESP_OK) break;
  }
  fclose(f);
  free(buf);
  if (ret == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  return ret;
}

// ── MIME helper ───────────────────────────────────────────────────────────────

static const char* mime_for(const char* path) {
  const char* ext = strrchr(path, '.');
  if (!ext) return "application/octet-stream";
  if (strcmp(ext, ".html") == 0) return "text/html; charset=utf-8";
  if (strcmp(ext, ".css")  == 0) return "text/css";
  if (strcmp(ext, ".js")   == 0) return "application/javascript";
  if (strcmp(ext, ".svg")  == 0) return "image/svg+xml";
  if (strcmp(ext, ".ico")  == 0) return "image/x-icon";
  return "application/octet-stream";
}

// ── 302 redirect helper ───────────────────────────────────────────────────────

static esp_err_t redirect_to_setup(httpd_req_t* req) {
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/setup.html");
  return httpd_resp_sendstr(req, "");
}

// ── Handlers ──────────────────────────────────────────────────────────────────

// GET / or known OS captive-detect probes → 302 to /setup.html
static esp_err_t h_root(httpd_req_t* req) {
  return redirect_to_setup(req);
}

// GET /setup.html → serve from LittleFS
static esp_err_t h_setup_html(httpd_req_t* req) {
  const char* path = "/lfs/ui/setup.html";
  if (!storage::lfs::exists(path)) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_sendstr(req, FALLBACK_HTML);
  }
  return serve_lfs_file(req, path, "text/html; charset=utf-8");
}

// GET /style.css, /app.js, /favicon.svg → serve from LittleFS
static esp_err_t h_static_asset(httpd_req_t* req) {
  char path[600];
  snprintf(path, sizeof(path), "/lfs/ui%s", req->uri);
  return serve_lfs_file(req, path, mime_for(path));
}

// GET /api/wifi/scan
static esp_err_t h_wifi_scan(httpd_req_t* req) {
  auto results = net::wifi::scan(5000);

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto& r : results) {
    JsonObject o = arr.add<JsonObject>();
    o["ssid"]   = r.ssid;
    o["rssi"]   = r.rssi;
    o["secure"] = r.secure;
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
  return ret;
}

// POST /api/wifi/configure — body: {"ssid":"...","password":"..."}
static esp_err_t h_wifi_configure(httpd_req_t* req) {
  char body[512] = {};
  size_t remaining = req->content_len;
  if (remaining == 0 || remaining >= sizeof(body)) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"bad body\"}");
  }
  size_t total = 0;
  while (remaining > 0) {
    int n = httpd_req_recv(req, body + total, remaining);
    if (n <= 0) break;
    total     += (size_t)n;
    remaining -= (size_t)n;
  }
  body[total] = '\0';

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"JSON parse error\"}");
  }

  const char* ssid = doc["ssid"]     | "";
  const char* pass = doc["password"] | "";
  if (ssid[0] == '\0') {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"ssid required\"}");
  }

  // Persist credentials and kick off async connection.
  // The connect_task will call esp_restart() on success.
  net::wifi::save_creds(ssid, pass);
  net::wifi::start_connection_async(ssid, pass, 30000);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req,
      "{\"ok\":true,\"status\":\"connecting\"}");
}

// GET /api/wifi/status — polled by browser during connect attempt
static esp_err_t h_wifi_status(httpd_req_t* req) {
  net::wifi::Mode state = net::wifi::get_state();

  const char* state_str = "unknown";
  char ip_buf[24] = {};

  switch (state) {
    case net::wifi::Mode::ApActive:     state_str = "ap_active";    break;
    case net::wifi::Mode::StaConnecting:state_str = "connecting";   break;
    case net::wifi::Mode::StaConnected:
      state_str = "connected";
      net::wifi::get_ip(ip_buf, sizeof(ip_buf));
      break;
    case net::wifi::Mode::StaFailed:    state_str = "failed";       break;
    default:                            state_str = "off";          break;
  }

  char resp[128];
  snprintf(resp, sizeof(resp),
           "{\"state\":\"%s\",\"ip\":\"%s\"}", state_str, ip_buf);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, resp);
}

// Catch-all: redirect everything else to /setup.html
static esp_err_t h_catchall(httpd_req_t* req) {
  return redirect_to_setup(req);
}

// ── Server lifecycle ──────────────────────────────────────────────────────────

static void reg(httpd_handle_t srv, const char* uri, httpd_method_t method,
                esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t h = {};
  h.uri     = uri;
  h.method  = method;
  h.handler = handler;
  httpd_register_uri_handler(srv, &h);
}

namespace web::captive {

bool start() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.task_priority    = 4;
  cfg.stack_size       = 10 * 1024;
  cfg.max_open_sockets = 4;
  cfg.max_uri_handlers = 16;
  cfg.recv_wait_timeout  = 10;
  cfg.send_wait_timeout  = 10;
  cfg.lru_purge_enable   = true;
  cfg.uri_match_fn       = httpd_uri_match_wildcard;

  if (httpd_start(&g_captive, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed");
    return false;
  }

  // Specific routes first.
  reg(g_captive, "/setup.html",           HTTP_GET,  h_setup_html);
  reg(g_captive, "/style.css",            HTTP_GET,  h_static_asset);
  reg(g_captive, "/app.js",               HTTP_GET,  h_static_asset);
  reg(g_captive, "/favicon.svg",          HTTP_GET,  h_static_asset);

  reg(g_captive, "/api/wifi/scan",        HTTP_GET,  h_wifi_scan);
  reg(g_captive, "/api/wifi/configure",   HTTP_POST, h_wifi_configure);
  reg(g_captive, "/api/wifi/status",      HTTP_GET,  h_wifi_status);

  // OS captive-portal probe paths — all redirect to setup page.
  // Returning 302 (instead of the "success" body the OS expects) signals that
  // a captive portal is present and the OS opens its built-in browser.
  reg(g_captive, "/generate_204",              HTTP_GET, h_root); // Android
  reg(g_captive, "/connecttest.txt",           HTTP_GET, h_root); // Windows
  reg(g_captive, "/hotspot-detect.html",       HTTP_GET, h_root); // Apple
  reg(g_captive, "/library/test/success.html", HTTP_GET, h_root); // Apple
  reg(g_captive, "/ncsi.txt",                  HTTP_GET, h_root); // Windows NCSI

  // Root redirect + wildcard catch-all.
  reg(g_captive, "/",  HTTP_GET, h_root);
  reg(g_captive, "/*", HTTP_GET, h_catchall);

  ESP_LOGI(TAG, "Captive portal HTTP server started on port 80");
  return true;
}

void stop() {
  if (g_captive) {
    httpd_stop(g_captive);
    g_captive = nullptr;
    ESP_LOGI(TAG, "Captive portal server stopped");
  }
}

}  // namespace web::captive
