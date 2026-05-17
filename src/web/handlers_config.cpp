#include "handlers_config.h"
#include "config_json.h"
#include "app/boot.h"
#include "storage/config.h"
#include "storage/nvs_store.h"
#include "net/wifi.h"
#include "mqtt/publisher.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <cstring>

static const char* TAG = "web_cfg";

// Max request body for config JSON: 4 KB is plenty.
static constexpr size_t CFG_BODY_MAX = 4096;

namespace web {

esp_err_t handle_config_get(httpd_req_t* req) {
  const Config& cfg = app::get_config();

  JsonDocument doc;
  config_to_json(cfg, doc);

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

// Helper: read the entire request body (up to max_len). Returns bytes read.
static size_t read_body(httpd_req_t* req, char* buf, size_t max_len) {
  size_t remaining = req->content_len;
  size_t total = 0;
  if (remaining == 0 || remaining > max_len) return 0;
  while (remaining > 0) {
    int n = httpd_req_recv(req, buf + total, remaining);
    if (n <= 0) return total;
    total     += n;
    remaining -= n;
  }
  buf[total] = '\0';
  return total;
}

static esp_err_t send_json_error(httpd_req_t* req, int status,
                                 const char* message) {
  char body[256];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_status(req, status == 400 ? "400 Bad Request" : "500 Internal Server Error");
  return httpd_resp_sendstr(req, body);
}

esp_err_t handle_config_post(httpd_req_t* req) {
  char* body = (char*)malloc(CFG_BODY_MAX + 1);
  if (!body) return send_json_error(req, 500, "OOM");

  size_t n = read_body(req, body, CFG_BODY_MAX);
  if (n == 0) {
    free(body);
    return send_json_error(req, 400, "Empty or oversized body");
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body, n);
  free(body);

  if (err) {
    ESP_LOGW(TAG, "config POST: JSON parse error: %s", err.c_str());
    return send_json_error(req, 400, "JSON parse error");
  }

  // Start from current config and overlay incoming values.
  Config new_cfg = app::get_config();
  json_to_config(doc, new_cfg);

  // Validate.
  char field_err[64] = {};
  ValidationError verr = storage::validate(new_cfg, field_err, sizeof(field_err));
  if (verr != ValidationError::None) {
    char msg[128];
    snprintf(msg, sizeof(msg), "Validation failed: field=%s", field_err);
    ESP_LOGW(TAG, "config POST: %s", msg);
    return send_json_error(req, 400, msg);
  }

  // Reject enabling auth when no password hash is stored.
  if (new_cfg.auth_enabled && new_cfg.auth_hash[0] == '\0') {
    ESP_LOGW(TAG, "config POST: auth_enabled=true but auth_hash is empty");
    return send_json_error(req, 400, "Cannot enable auth without password");
  }

  // Snapshot current MQTT settings for change detection before overwriting.
  const Config& old_cfg = app::get_config();
  bool mqtt_changed = (old_cfg.mqtt_enabled    != new_cfg.mqtt_enabled   ||
                       old_cfg.mqtt_port        != new_cfg.mqtt_port       ||
                       strcmp(old_cfg.mqtt_host,       new_cfg.mqtt_host)       != 0 ||
                       strcmp(old_cfg.mqtt_user,       new_cfg.mqtt_user)       != 0 ||
                       strcmp(old_cfg.mqtt_base_topic, new_cfg.mqtt_base_topic) != 0 ||
                       (new_cfg.mqtt_pass_obf[0] != '\0'));  // non-empty = password updated

  // Persist to NVS and update runtime config.
  if (!app::update_and_save_config(new_cfg)) {
    return send_json_error(req, 500, "NVS save failed");
  }

  ESP_LOGI(TAG, "Config saved via HTTP POST /api/config");

  // Reconnect MQTT when connection-relevant settings changed.
  if (mqtt_changed) {
    ESP_LOGI(TAG, "MQTT settings changed — reconfiguring publisher");
    mqtt::publisher::reconfigure(new_cfg);
  }

  // Return the saved config.
  JsonDocument resp;
  config_to_json(new_cfg, resp);

  size_t est = measureJson(resp) + 1;
  char* rbuf = (char*)malloc(est);
  if (!rbuf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  size_t rn = serializeJson(resp, rbuf, est);

  httpd_resp_set_type(req, "application/json");
  esp_err_t ret = httpd_resp_send(req, rbuf, (ssize_t)rn);
  free(rbuf);
  return ret;
}

// POST /api/wifi — {ssid, pass} → persist via esp_wifi_set_config → restart.
esp_err_t handle_wifi_post(httpd_req_t* req) {
  char* body = (char*)malloc(512);
  if (!body) return send_json_error(req, 500, "OOM");

  size_t n = read_body(req, body, 511);
  if (n == 0) {
    free(body);
    return send_json_error(req, 400, "Empty body");
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body, n);
  free(body);
  if (err) return send_json_error(req, 400, "JSON parse error");

  const char* ssid = doc["ssid"] | "";
  const char* pass = doc["pass"] | "";
  if (ssid[0] == '\0') return send_json_error(req, 400, "ssid required");

  if (!net::wifi::save_creds(ssid, pass)) {
    return send_json_error(req, 500, "Failed to save credentials");
  }

  // Also update config.wifi_ssid and persist config.
  Config new_cfg = app::get_config();
  snprintf(new_cfg.wifi_ssid, sizeof(new_cfg.wifi_ssid), "%s", ssid);
  app::update_and_save_config(new_cfg);

  // Respond before rebooting.
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Credentials saved — rebooting\"}");

  // Schedule restart after 2 s so the response reaches the client.
  vTaskDelay(pdMS_TO_TICKS(2000));
  esp_restart();
  return ESP_OK;
}

}  // namespace web
