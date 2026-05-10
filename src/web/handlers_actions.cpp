#include "handlers_actions.h"
#include "config_json.h"
#include "app/boot.h"
#include "app/version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
#include <ctime>
#include <cstdio>

static const char* TAG = "web_act";

namespace web {

esp_err_t handle_health(httpd_req_t* req) {
  uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);

  char body[256];
  snprintf(body, sizeof(body),
           "{\"ok\":true,\"uptime_s\":%lu,\"version\":\"%s\",\"build\":\"%s %s\"}",
           (unsigned long)uptime_s, FW_VERSION, BUILD_DATE, BUILD_TIME);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, body);
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

}  // namespace web
