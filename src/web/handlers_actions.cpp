#include "handlers_actions.h"
#include "config_json.h"
#include "app/boot.h"
#include "app/version.h"
#include "storage/config.h"
#include "storage/nvs_store.h"
#include "storage/ui_provisioner.h"
#include "mqtt/publisher.h"
#include "net/ntp.h"
#include "net/wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <ctime>
#include <cstdio>

static const char* TAG = "web_act";

static const char* mqtt_state_str(mqtt::publisher::State s) {
  switch (s) {
    case mqtt::publisher::State::Disabled:     return "disabled";
    case mqtt::publisher::State::Disconnected: return "disconnected";
    case mqtt::publisher::State::Connecting:   return "connecting";
    case mqtt::publisher::State::Connected:    return "connected";
    case mqtt::publisher::State::Failed:       return "failed";
    default:                                   return "unknown";
  }
}

namespace web {

esp_err_t handle_health(httpd_req_t* req) {
  uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
  const Config& cfg = app::get_config();

  JsonDocument doc;
  doc["ok"]           = true;
  doc["uptime_s"]     = uptime_s;
  doc["auth_enabled"] = cfg.auth_enabled;
  doc["version"]      = FW_VERSION_FULL;
  doc["ui_version"]   = storage::ui_provisioner::UI_VERSION;
  doc["build"]        = BUILD_DATE " " BUILD_TIME;

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["enabled"]       = cfg.mqtt_enabled;
  mqtt["state"]         = mqtt_state_str(mqtt::publisher::get_state());
  mqtt["publish_ok"]    = mqtt::publisher::get_publish_ok();
  mqtt["publish_fail"]  = mqtt::publisher::get_publish_fail();
  mqtt["publish_drops"] = mqtt::publisher::get_publish_drops();

  // NTP status — used by Settings → Time section.
  doc["now_ts_s"]   = net::ntp::now_unix_s();
  doc["ntp_synced"] = net::ntp::is_synced();

  doc["free_heap_b"]  = esp_get_free_heap_size();
  doc["free_psram_b"] = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  char body[1024];
  size_t n = serializeJson(doc, body, sizeof(body));
  if (n == 0) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, body, (ssize_t)n);
}

// FreeRTOS task that waits 2 s then calls esp_restart().
static void restart_task(void* arg) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  ESP_LOGI(TAG, "Executing soft restart (scheduled by POST /api/restart)");
  esp_restart();
}

esp_err_t handle_restart(httpd_req_t* req) {
  ESP_LOGI(TAG, "Soft restart scheduled via /api/restart");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Restarting in 2s\"}");

  // Detached task so the response is sent before restart.
  xTaskCreate(restart_task, "restart", 2048, NULL, 1, NULL);
  return ESP_OK;
}

esp_err_t handle_backup(httpd_req_t* req) {
  const Config& cfg = app::get_config();

  // ISO-8601 timestamp for the metadata wrapper. Use epoch 0 if time not set.
  time_t now = time(nullptr);
  char ts_buf[32] = "1970-01-01T00:00:00Z";
  if (now > 1000000) {
    struct tm tm_info = {};
    gmtime_r(&now, &tm_info);
    strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", &tm_info);
  }

  // Build the wrapped backup document.
  JsonDocument doc;
  doc["_format"]   = "topband-bms-config";
  doc["_version"]  = "v3.0";
  doc["_exported"] = ts_buf;

  JsonObject cfg_obj = doc["config"].to<JsonObject>();
  JsonDocument cfg_doc;
  config_to_json(cfg, cfg_doc);
  // Copy cfg_doc fields into cfg_obj.
  for (auto kv : cfg_doc.as<JsonObjectConst>()) {
    cfg_obj[kv.key()] = kv.value();
  }

  size_t est = measureJsonPretty(doc) + 1;
  char* buf = (char*)malloc(est);
  if (!buf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  size_t n = serializeJsonPretty(doc, buf, est);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=\"topband-config.json\"");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  esp_err_t ret = httpd_resp_send(req, buf, (ssize_t)n);
  free(buf);
  return ret;
}

esp_err_t handle_factory_reset(httpd_req_t* req) {
  // Require explicit confirmation to prevent accidental calls.
  char body[128] = {};
  size_t remaining = req->content_len;
  if (remaining > 0 && remaining < sizeof(body)) {
    size_t total = 0;
    while (remaining > 0) {
      int n = httpd_req_recv(req, body + total, remaining);
      if (n <= 0) break;
      total     += (size_t)n;
      remaining -= (size_t)n;
    }
    body[total] = '\0';
  }

  JsonDocument doc;
  bool confirmed = false;
  if (deserializeJson(doc, body) == DeserializationError::Ok) {
    confirmed = doc["confirm"] | false;
  }
  if (!confirmed) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"confirm:true required\"}");
  }

  ESP_LOGW(TAG, "Factory reset requested via API — clearing WiFi + auth");

  // Clear auth and WiFi in Config blob.
  Config cleared = app::get_config();
  cleared.auth_enabled = false;
  cleared.auth_hash[0] = '\0';
  cleared.wifi_ssid[0] = '\0';
  storage::saveConfig(cleared);

  // Clear esp_wifi NVS credentials.
  wifi_config_t empty = {};
  esp_wifi_set_config(WIFI_IF_STA, &empty);

  // Respond before restarting.
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Factory reset — rebooting\"}");

  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;
}

esp_err_t handle_ha_discovery_send(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");
  if (mqtt::publisher::get_state() != mqtt::publisher::State::Connected) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "{\"error\":\"MQTT not connected\"}");
  }
  mqtt::publisher::trigger_ha_discovery();
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t handle_ha_discovery_clear(httpd_req_t* req) {
  // Reset the NVS version marker so cleanup_stale() runs on next MQTT connect.
  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open("gateway", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    nvs_erase_key(nvs, "ha_disco_v");
    nvs_commit(nvs);
    nvs_close(nvs);
  }
  // Also trigger full discovery publish now if connected.
  if (mqtt::publisher::get_state() == mqtt::publisher::State::Connected) {
    mqtt::publisher::trigger_ha_discovery();
  }
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// ── WiFi status / scan / configure (main STA server) ─────────────────────────

esp_err_t handle_wifi_status_get(httpd_req_t* req) {
  net::wifi::IpInfo ip = net::wifi::get_ip_info();
  std::string ssid     = net::wifi::get_ssid();
  int8_t rssi          = net::wifi::get_rssi();
  uint32_t conn_s      = net::wifi::connected_for_s();

  char hostname[32] = {};
  net::wifi::get_hostname(hostname, sizeof(hostname));

  char mdns[64] = {};
  snprintf(mdns, sizeof(mdns), "%s.local", hostname);

  JsonDocument doc;
  doc["connected"]       = net::wifi::is_connected();
  doc["ssid"]            = ssid;
  doc["rssi"]            = rssi;
  doc["ip"]              = ip.ip;
  doc["gateway"]         = ip.gw;
  doc["netmask"]         = ip.netmask;
  doc["dns"]             = ip.dns;
  doc["mdns_hostname"]   = mdns;
  doc["connected_for_s"] = conn_s;

  char body[512];
  size_t n = serializeJson(doc, body, sizeof(body));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, body, (ssize_t)n);
}

esp_err_t handle_wifi_scan_get(httpd_req_t* req) {
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

esp_err_t handle_wifi_configure_post(httpd_req_t* req) {
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

  // Persist credentials and kick off async connect. On success the gateway
  // reboots and comes up on the new network. On failure it falls back to AP.
  net::wifi::save_creds(ssid, pass);
  net::wifi::start_connection_async(ssid, pass, 30000);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req,
      "{\"ok\":true,\"status\":\"connecting\"}");
}

}  // namespace web
