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
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <ArduinoJson.h>
#include <ctime>
#include <cstdio>
#include <cstring>

static const char* TAG = "web_act";

// ── Shared helpers ────────────────────────────────────────────────────────────

static size_t read_body_act(httpd_req_t* req, char* buf, size_t max_len) {
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

static esp_err_t send_err_act(httpd_req_t* req, int code, const char* msg) {
  char body[256];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
  httpd_resp_set_type(req, "application/json");
  if (code == 400)      httpd_resp_set_status(req, "400 Bad Request");
  else if (code == 409) httpd_resp_set_status(req, "409 Conflict");
  else if (code == 422) httpd_resp_set_status(req, "422 Unprocessable Entity");
  else                  httpd_resp_set_status(req, "500 Internal Server Error");
  return httpd_resp_sendstr(req, body);
}

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

  JsonObject wifi_o = doc["wifi"].to<JsonObject>();
  wifi_o["connected"] = net::wifi::is_connected();
  wifi_o["rssi"]      = (int)net::wifi::get_rssi();

  // NTP status — used by Settings → Time section.
  doc["now_ts_s"]   = net::ntp::now_unix_s();
  doc["ntp_synced"] = net::ntp::is_synced();

  doc["free_heap_b"]  = esp_get_free_heap_size();
  doc["free_psram_b"] = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  char body[1152];
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

// ── POST /api/restore ─────────────────────────────────────────────────────────
// Body: {"backup": {...full backup JSON...}, "include_hardware": bool}
// Validates the backup, merges onto current config per scope, saves, reboots.
// Max body 8 KB (backup ~4 KB + envelope).
static constexpr size_t RESTORE_BODY_MAX = 8192;

esp_err_t handle_restore(httpd_req_t* req) {
  char* body = (char*)malloc(RESTORE_BODY_MAX + 1);
  if (!body) return send_err_act(req, 500, "OOM");

  size_t n = read_body_act(req, body, RESTORE_BODY_MAX);
  if (n == 0) {
    free(body);
    return send_err_act(req, 400, "Empty or oversized body");
  }

  JsonDocument req_doc;
  DeserializationError derr = deserializeJson(req_doc, body, n);
  free(body);
  if (derr) return send_err_act(req, 400, "Invalid JSON — malformed backup file");

  bool include_hardware = req_doc["include_hardware"] | false;

  // The backup object must be present.
  JsonVariantConst backup_v = req_doc["backup"];
  if (!backup_v.is<JsonObjectConst>())
    return send_err_act(req, 400, "Missing 'backup' field");
  JsonObjectConst backup = backup_v.as<JsonObjectConst>();

  // Validate envelope.
  const char* fmt = backup["_format"] | "";
  if (strcmp(fmt, "topband-bms-config") != 0)
    return send_err_act(req, 400, "Not a TopBand BMS config backup (_format mismatch)");

  JsonVariantConst cfg_v = backup["config"];
  if (!cfg_v.is<JsonObjectConst>())
    return send_err_act(req, 400, "Backup has no 'config' section");
  JsonObjectConst cfg_obj = cfg_v.as<JsonObjectConst>();

  // Schema-version compatibility check.
  int backup_schema = cfg_obj["schema_version"] | -1;
  if (backup_schema < 0)
    return send_err_act(req, 400, "Backup missing schema_version");
  if (backup_schema > (int)CURRENT_SCHEMA_VERSION) {
    char msg[160];
    snprintf(msg, sizeof(msg),
      "Backup schema v%d is newer than this firmware (v%d) — update device firmware first",
      backup_schema, (int)CURRENT_SCHEMA_VERSION);
    return send_err_act(req, 400, msg);
  }

  // Defaults-then-overlay: start from current live config, overlay backup fields.
  // Fields missing from an older backup stay at their current (valid default) values.
  Config new_cfg = app::get_config();

  // Copy backup config into a JsonDocument so json_to_config can accept it.
  JsonDocument cfg_doc;
  for (auto kv : cfg_obj) cfg_doc[kv.key()] = kv.value();

  json_to_config(cfg_doc, new_cfg);

  // Scope filter: if settings-only, restore hardware fields from the live config.
  if (!include_hardware) {
    const Config& live = app::get_config();
    new_cfg.board_preset  = live.board_preset;
    new_cfg.pins          = live.pins;
    new_cfg.rs485_enabled = live.rs485_enabled;
  }

  // Passwords are never in backups — force-clear regardless of what the backup says.
  new_cfg.mqtt_pass_obf[0] = '\0';
  new_cfg.auth_hash[0]     = '\0';

  // Full value-range validation (same path as /api/config POST).
  char field_err[64] = {};
  ValidationError verr = storage::validate(new_cfg, field_err, sizeof(field_err));
  if (verr != ValidationError::None) {
    char msg[128];
    snprintf(msg, sizeof(msg), "Validation failed: %s", field_err);
    return send_err_act(req, 422, msg);
  }

  if (!app::update_and_save_config(new_cfg)) {
    return send_err_act(req, 500, "NVS save failed");
  }

  ESP_LOGI(TAG, "Config restored via /api/restore (include_hardware=%d, schema=%d)",
           (int)include_hardware, backup_schema);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req,
    "{\"ok\":true,\"message\":\"Import successful — rebooting to apply\",\"reboot_in_s\":3}");

  xTaskCreate([](void*) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  }, "restore_rst", 2048, nullptr, 1, nullptr);

  return ESP_OK;
}

// ── MQTT connection test ──────────────────────────────────────────────────────
// Runs a throwaway esp_mqtt client against the unsaved form values.
// The live publisher is never touched.

enum class MqttTestStatus : uint8_t { Idle, Running, Ok, Failed };

struct MqttTestResult {
  MqttTestStatus status = MqttTestStatus::Idle;
  char stage[16]   = {};    // "tcp", "auth", "publish", "done"
  char message[128] = {};
};

static portMUX_TYPE s_test_mux     = portMUX_INITIALIZER_UNLOCKED;
static MqttTestResult s_test_result = {};

struct MqttTestCtx {
  char host[64];
  uint16_t port;
  char user[32];
  char pass[64];
  char base_topic[64];
  SemaphoreHandle_t done_sem;
  volatile bool connected = false;
  char err_stage[16]  = {};
  char err_msg[80]    = {};
};

static void set_test_result(MqttTestStatus st, const char* stage, const char* msg) {
  portENTER_CRITICAL(&s_test_mux);
  s_test_result.status = st;
  snprintf(s_test_result.stage,   sizeof(s_test_result.stage),   "%s", stage);
  snprintf(s_test_result.message, sizeof(s_test_result.message), "%s", msg);
  portEXIT_CRITICAL(&s_test_mux);
}

static void mqtt_test_event_cb(void* handler_args, esp_event_base_t /*base*/,
                               int32_t event_id, void* event_data) {
  MqttTestCtx* ctx = static_cast<MqttTestCtx*>(handler_args);
  auto* ev = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
    case MQTT_EVENT_CONNECTED:
      ctx->connected = true;
      xSemaphoreGive(ctx->done_sem);
      break;

    case MQTT_EVENT_ERROR: {
      ctx->connected = false;
      if (ev->error_handle &&
          ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        snprintf(ctx->err_stage, sizeof(ctx->err_stage), "auth");
        snprintf(ctx->err_msg,   sizeof(ctx->err_msg),
                 "Authentication failed — check user/password (broker returned code %d)",
                 (int)ev->error_handle->connect_return_code);
      } else {
        snprintf(ctx->err_stage, sizeof(ctx->err_stage), "tcp");
        snprintf(ctx->err_msg,   sizeof(ctx->err_msg),
                 "TCP connect failed — host/port unreachable");
      }
      xSemaphoreGive(ctx->done_sem);
      break;
    }

    default:
      break;
  }
}

static void mqtt_test_task(void* arg) {
  MqttTestCtx* ctx = static_cast<MqttTestCtx*>(arg);

  esp_mqtt_client_config_t mcfg = {};
  mcfg.broker.address.hostname   = ctx->host;
  mcfg.broker.address.port       = ctx->port;
  mcfg.broker.address.transport  = MQTT_TRANSPORT_OVER_TCP;
  mcfg.credentials.client_id     = "topband_conntest";
  mcfg.credentials.username      = ctx->user[0] ? ctx->user : nullptr;
  mcfg.credentials.authentication.password = ctx->pass[0] ? ctx->pass : nullptr;
  mcfg.network.timeout_ms        = 5000;
  mcfg.network.reconnect_timeout_ms = 10000;  // irrelevant — we stop before retry
  mcfg.session.keepalive          = 5;

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mcfg);
  if (!client) {
    set_test_result(MqttTestStatus::Failed, "tcp", "Failed to create test MQTT client");
    vSemaphoreDelete(ctx->done_sem);
    free(ctx);
    vTaskDelete(nullptr);
    return;
  }

  esp_mqtt_client_register_event(client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                  mqtt_test_event_cb, ctx);
  esp_mqtt_client_start(client);

  // Wait up to 7 s for TCP connect + MQTT handshake (5 s network timeout + headroom).
  bool got = (xSemaphoreTake(ctx->done_sem, pdMS_TO_TICKS(7000)) == pdTRUE);

  if (!got) {
    set_test_result(MqttTestStatus::Failed, "tcp",
                    "Timeout — broker did not respond within 7 s (check host/port)");
  } else if (!ctx->connected) {
    set_test_result(MqttTestStatus::Failed, ctx->err_stage, ctx->err_msg);
  } else {
    // TCP + auth succeeded; publish a test message (QoS 0, non-retained).
    char test_topic[128];
    snprintf(test_topic, sizeof(test_topic), "%s/_conntest", ctx->base_topic);
    int mid = esp_mqtt_client_publish(client, test_topic, "conntest", 8, 0, 0);
    if (mid >= 0) {
      set_test_result(MqttTestStatus::Ok, "done",
                      "Connection OK — broker reachable, authenticated, test message published");
    } else {
      set_test_result(MqttTestStatus::Failed, "publish",
                      "Connected and authenticated but publish failed");
    }
  }

  esp_mqtt_client_stop(client);
  esp_mqtt_client_destroy(client);
  vSemaphoreDelete(ctx->done_sem);
  free(ctx);
  vTaskDelete(nullptr);
}

// POST /api/mqtt/test — kick off a throwaway connection test against the body values.
// Returns 200 immediately; client polls GET /api/mqtt/test for the result.
esp_err_t handle_mqtt_test_post(httpd_req_t* req) {
  // Reject if a test is already in flight.
  portENTER_CRITICAL(&s_test_mux);
  bool busy = (s_test_result.status == MqttTestStatus::Running);
  portEXIT_CRITICAL(&s_test_mux);
  if (busy) return send_err_act(req, 409, "Test already running");

  char body[512] = {};
  size_t n = read_body_act(req, body, sizeof(body) - 1);
  if (n == 0) return send_err_act(req, 400, "Empty body");

  JsonDocument doc;
  if (deserializeJson(doc, body, n) != DeserializationError::Ok)
    return send_err_act(req, 400, "JSON parse error");

  auto* ctx = static_cast<MqttTestCtx*>(malloc(sizeof(MqttTestCtx)));
  if (!ctx) return send_err_act(req, 500, "OOM");
  memset(ctx, 0, sizeof(MqttTestCtx));

  snprintf(ctx->host,       sizeof(ctx->host),       "%s", (const char*)(doc["host"]       | ""));
  ctx->port = (uint16_t)(doc["port"] | 1883);
  snprintf(ctx->user,       sizeof(ctx->user),       "%s", (const char*)(doc["user"]       | ""));
  snprintf(ctx->base_topic, sizeof(ctx->base_topic), "%s", (const char*)(doc["base_topic"] | "topband-bms"));

  // Password: blank in the form = reuse saved credential.
  const char* form_pass = doc["pass"] | "";
  if (form_pass[0] != '\0') {
    snprintf(ctx->pass, sizeof(ctx->pass), "%s", form_pass);
  } else {
    snprintf(ctx->pass, sizeof(ctx->pass), "%s", app::get_config().mqtt_pass_obf);
  }

  if (ctx->host[0] == '\0') {
    free(ctx);
    return send_err_act(req, 400, "host required");
  }

  ctx->done_sem = xSemaphoreCreateBinary();
  if (!ctx->done_sem) {
    free(ctx);
    return send_err_act(req, 500, "semaphore alloc failed");
  }

  // Mark running before the task starts to prevent a race on the status read.
  set_test_result(MqttTestStatus::Running, "tcp", "Connecting…");

  BaseType_t ok = xTaskCreate(mqtt_test_task, "mqtt_test", 6144, ctx, 2, nullptr);
  if (ok != pdPASS) {
    set_test_result(MqttTestStatus::Idle, "", "");
    vSemaphoreDelete(ctx->done_sem);
    free(ctx);
    return send_err_act(req, 500, "Task create failed");
  }

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true,\"status\":\"running\"}");
}

// GET /api/mqtt/test — return current test result.
esp_err_t handle_mqtt_test_get(httpd_req_t* req) {
  portENTER_CRITICAL(&s_test_mux);
  MqttTestResult snap = s_test_result;
  portEXIT_CRITICAL(&s_test_mux);

  const char* st_str = "idle";
  switch (snap.status) {
    case MqttTestStatus::Running: st_str = "running"; break;
    case MqttTestStatus::Ok:      st_str = "ok";      break;
    case MqttTestStatus::Failed:  st_str = "failed";  break;
    default: break;
  }

  char body[256];
  // Escape message for JSON (replace " with \")
  char safe_msg[128];
  size_t si = 0;
  for (size_t i = 0; snap.message[i] && si < sizeof(safe_msg) - 2; i++) {
    if (snap.message[i] == '"' || snap.message[i] == '\\')
      safe_msg[si++] = '\\';
    safe_msg[si++] = snap.message[i];
  }
  safe_msg[si] = '\0';

  snprintf(body, sizeof(body),
           "{\"status\":\"%s\",\"stage\":\"%s\",\"message\":\"%s\"}",
           st_str, snap.stage, safe_msg);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, body);
}

}  // namespace web
