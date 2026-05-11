#include "mqtt/publisher.h"
#include "mqtt/topics.h"
#include "mqtt/ha_discovery.h"
#include "bus/queues.h"
#include "app/boot.h"
#include "app/version.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "mqtt_pub";

// ── Module-private state ──────────────────────────────────────────────────────

static esp_mqtt_client_handle_t s_client       = nullptr;
static TaskHandle_t             s_task_handle  = nullptr;
static portMUX_TYPE             s_mux          = portMUX_INITIALIZER_UNLOCKED;

static mqtt::publisher::State s_state          = mqtt::publisher::State::Disabled;
static bool                   s_just_connected = false;

static uint64_t s_publish_ok    = 0;
static uint64_t s_publish_fail  = 0;
static uint64_t s_publish_drops = 0;
static uint32_t s_publish_max_ms = 0;

// Config copy for use inside MqttTask (updated under s_mux on reconfigure).
static Config s_cfg{};

// Effective base topic = cfg.mqtt_base_topic + "-" + last4hexMAC.
// Built once at start(), stable until stop().
static char s_effective_base[80] = {};

// device_uid for HA discovery: "topband_bms_xxxxxxxxxxxx" (full MAC, 12 hex chars).
static char s_device_uid[32] = {};

// ── Helpers ───────────────────────────────────────────────────────────────────

static void compute_mac_identifiers(const Config& cfg) {
  uint8_t mac[6] = {};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) {
    // Fallback: use all zeros (still unique per cfg.mqtt_base_topic in practice)
    ESP_LOGW(TAG, "Failed to read WiFi MAC — using zeros for device UID");
  }
  // Effective base: configured prefix + "-" + last 2 bytes of MAC (4 hex chars)
  snprintf(s_effective_base, sizeof(s_effective_base),
           "%s-%02x%02x", cfg.mqtt_base_topic, mac[4], mac[5]);

  // Full-MAC unique ID for HA entity/device identifiers
  snprintf(s_device_uid, sizeof(s_device_uid),
           "topband_bms_%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void publish_status_online() {
  char topic[128];
  if (!mqtt::topics::build(s_effective_base, mqtt::topics::STATUS, topic, sizeof(topic))) return;

  int msg_id = esp_mqtt_client_publish(s_client, topic, "online", 6, 0, 1 /*retain*/);
  if (msg_id < 0) {
    ESP_LOGW(TAG, "publish status=online failed");
  } else {
    ESP_LOGI(TAG, "MQTT connected — published %s online", topic);
  }
}

// Called from MqttTask context when draining q_mqtt_publish.
static void publish_request(const MqttPublishRequest& req) {
  char topic[128];
  size_t tlen = 0;

  switch (req.topic) {
    case MqttPublishRequest::Topic::Data:
      tlen = mqtt::topics::build(s_effective_base, mqtt::topics::DATA, topic, sizeof(topic));
      break;
    case MqttPublishRequest::Topic::Status:
      tlen = mqtt::topics::build(s_effective_base, mqtt::topics::STATUS, topic, sizeof(topic));
      break;
    case MqttPublishRequest::Topic::Alarm:
      tlen = mqtt::topics::build(s_effective_base, mqtt::topics::ALARM, topic, sizeof(topic));
      break;
    case MqttPublishRequest::Topic::Diag:
      tlen = mqtt::topics::build(s_effective_base, mqtt::topics::DIAG, topic, sizeof(topic));
      break;
    case MqttPublishRequest::Topic::Cells:
      tlen = mqtt::topics::build_cells(s_effective_base, req.pack_id, topic, sizeof(topic));
      break;
    default:
      return;
  }

  if (!tlen) {
    ESP_LOGW(TAG, "publish_request: topic build failed (base too long?)");
    portENTER_CRITICAL(&s_mux);
    s_publish_fail++;
    portEXIT_CRITICAL(&s_mux);
    return;
  }

  uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
  int msg_id = esp_mqtt_client_publish(s_client, topic, req.payload, req.payload_len,
                                        0 /*qos*/, req.retained ? 1 : 0);
  uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;

  portENTER_CRITICAL(&s_mux);
  if (msg_id >= 0) {
    s_publish_ok++;
    if (elapsed > s_publish_max_ms) s_publish_max_ms = elapsed;
  } else {
    s_publish_fail++;
  }
  portEXIT_CRITICAL(&s_mux);
}

// ── MQTT event handler (runs in esp_mqtt internal task) ───────────────────────

static void mqtt_event_handler(void* /*arg*/, esp_event_base_t /*base*/,
                                int32_t event_id, void* event_data) {
  esp_mqtt_event_handle_t ev = static_cast<esp_mqtt_event_handle_t>(event_data);
  (void)ev;

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
    case MQTT_EVENT_CONNECTED:
      portENTER_CRITICAL(&s_mux);
      s_state = mqtt::publisher::State::Connected;
      s_just_connected = true;
      portEXIT_CRITICAL(&s_mux);
      break;

    case MQTT_EVENT_DISCONNECTED:
      portENTER_CRITICAL(&s_mux);
      s_state = mqtt::publisher::State::Disconnected;
      portEXIT_CRITICAL(&s_mux);
      ESP_LOGW(TAG, "MQTT disconnected — auto-reconnect pending");
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
      portENTER_CRITICAL(&s_mux);
      if (s_state != mqtt::publisher::State::Connected) {
        s_state = mqtt::publisher::State::Disconnected;
      }
      portEXIT_CRITICAL(&s_mux);
      break;

    default:
      break;
  }
}

// ── MqttTask body ─────────────────────────────────────────────────────────────

static void mqtt_task_entry(void* /*arg*/) {
  ESP_LOGI(TAG, "MqttTask started (Core 1) — effective_base=%s uid=%s",
           s_effective_base, s_device_uid);

  for (;;) {
    // Check connect event (set by event handler, consumed here)
    bool just_conn = false;
    portENTER_CRITICAL(&s_mux);
    just_conn = s_just_connected;
    s_just_connected = false;
    bool connected = (s_state == mqtt::publisher::State::Connected);
    portEXIT_CRITICAL(&s_mux);

    if (just_conn) {
      publish_status_online();

      // Read ha_discovery_enabled from the live config so that toggling it in
      // the UI takes effect on the next connect without a full publisher restart.
      // s_cfg.ha_discovery_enabled is only set at start() time and goes stale
      // if the user enables discovery while MQTT is already connected.
      bool ha_en = app::get_config().ha_discovery_enabled;

      if (ha_en) {
        // Snapshot s_cfg once under lock for the discovery calls.
        Config cfg_snap;
        portENTER_CRITICAL(&s_mux);
        cfg_snap = s_cfg;
        portEXIT_CRITICAL(&s_mux);
        mqtt::ha_discovery::cleanup_stale(s_client, cfg_snap, s_device_uid, s_effective_base);
        mqtt::ha_discovery::publish_all(s_client, cfg_snap, s_device_uid, s_effective_base);
      }
    }

    // Drain publish queue (only when connected)
    if (connected) {
      MqttPublishRequest req;
      while (xQueueReceive(q_mqtt_publish, &req, 0) == pdTRUE) {
        publish_request(req);
      }
    } else {
      // Discard stale items while disconnected to prevent queue backup.
      // The HousekeepingTask will re-publish on next cadence after reconnect.
      MqttPublishRequest req;
      uint8_t max_drain = 4;
      while (max_drain-- && xQueueReceive(q_mqtt_publish, &req, 0) == pdTRUE) {
        portENTER_CRITICAL(&s_mux);
        s_publish_drops++;
        portEXIT_CRITICAL(&s_mux);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace mqtt::publisher {

bool start(const Config& cfg) {
  if (!cfg.mqtt_enabled) {
    ESP_LOGI(TAG, "MQTT disabled in config — not starting");
    return false;
  }
  if (cfg.mqtt_host[0] == '\0') {
    ESP_LOGW(TAG, "MQTT enabled but host is empty — not starting");
    return false;
  }

  // Snapshot config for use in MqttTask
  portENTER_CRITICAL(&s_mux);
  s_cfg = cfg;
  portEXIT_CRITICAL(&s_mux);

  compute_mac_identifiers(cfg);

  // Build LWT topic
  char lwt_topic[128];
  if (!mqtt::topics::build(s_effective_base, mqtt::topics::STATUS, lwt_topic, sizeof(lwt_topic))) {
    ESP_LOGE(TAG, "MQTT: LWT topic too long");
    return false;
  }

  // Client ID: "topband-bms-" + last 4 hex MAC chars
  char client_id[32];
  snprintf(client_id, sizeof(client_id), "topband-bms-%.4s", s_device_uid + 22);

  esp_mqtt_client_config_t mcfg = {};
  mcfg.broker.address.hostname    = cfg.mqtt_host;
  mcfg.broker.address.transport   = MQTT_TRANSPORT_OVER_TCP;
  mcfg.broker.address.port        = cfg.mqtt_port;
  mcfg.credentials.client_id      = client_id;
  mcfg.session.keepalive           = 30;
  mcfg.session.last_will.topic     = lwt_topic;
  mcfg.session.last_will.msg       = "offline";
  mcfg.session.last_will.msg_len   = 7;
  mcfg.session.last_will.qos       = 0;
  mcfg.session.last_will.retain    = 1;
  // Auto-reconnect every 5 s — adequate for a local broker
  mcfg.network.reconnect_timeout_ms = 5000;

  if (cfg.mqtt_user[0] != '\0') {
    mcfg.credentials.username = cfg.mqtt_user;
  }
  if (cfg.mqtt_pass_obf[0] != '\0') {
    mcfg.credentials.authentication.password = cfg.mqtt_pass_obf;
  }

  s_client = esp_mqtt_client_init(&mcfg);
  if (!s_client) {
    ESP_LOGE(TAG, "esp_mqtt_client_init failed");
    return false;
  }

  esp_mqtt_client_register_event(s_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                   mqtt_event_handler, nullptr);

  // Spawn MqttTask on Core 1, priority 3 (architecture §3.2)
  BaseType_t r = xTaskCreatePinnedToCore(
      mqtt_task_entry, "mqtt", 6144, nullptr, 3, &s_task_handle, /*core*/ 1);
  if (r != pdPASS) {
    ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
    esp_mqtt_client_destroy(s_client);
    s_client = nullptr;
    return false;
  }

  portENTER_CRITICAL(&s_mux);
  s_state = State::Connecting;
  portEXIT_CRITICAL(&s_mux);

  esp_err_t err = esp_mqtt_client_start(s_client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_mqtt_client_start: %s", esp_err_to_name(err));
    portENTER_CRITICAL(&s_mux);
    s_state = State::Failed;
    portEXIT_CRITICAL(&s_mux);
    return false;
  }

  ESP_LOGI(TAG, "MQTT publisher started — broker=%s:%u base=%s",
           cfg.mqtt_host, cfg.mqtt_port, s_effective_base);
  return true;
}

void stop() {
  // Kill task first so it stops draining the queue
  if (s_task_handle) {
    vTaskDelete(s_task_handle);
    s_task_handle = nullptr;
  }

  // Drain remaining queue items (avoid stale data on next start)
  if (q_mqtt_publish) {
    MqttPublishRequest req;
    while (xQueueReceive(q_mqtt_publish, &req, 0) == pdTRUE) {}
  }

  if (s_client) {
    esp_mqtt_client_destroy(s_client);
    s_client = nullptr;
  }

  portENTER_CRITICAL(&s_mux);
  s_state         = State::Disabled;
  s_just_connected = false;
  portEXIT_CRITICAL(&s_mux);

  ESP_LOGI(TAG, "MQTT publisher stopped");
}

void reconfigure(const Config& cfg) {
  ESP_LOGI(TAG, "MQTT reconfigure: host=%s port=%u enabled=%d",
           cfg.mqtt_host, cfg.mqtt_port, (int)cfg.mqtt_enabled);
  stop();
  if (cfg.mqtt_enabled) {
    start(cfg);
  }
}

State get_state() {
  portENTER_CRITICAL(&s_mux);
  State st = s_state;
  portEXIT_CRITICAL(&s_mux);
  return st;
}

uint64_t get_publish_ok() {
  portENTER_CRITICAL(&s_mux);
  uint64_t v = s_publish_ok;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

uint64_t get_publish_fail() {
  portENTER_CRITICAL(&s_mux);
  uint64_t v = s_publish_fail;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

uint64_t get_publish_drops() {
  portENTER_CRITICAL(&s_mux);
  uint64_t v = s_publish_drops;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

uint32_t get_publish_max_ms() {
  portENTER_CRITICAL(&s_mux);
  uint32_t v = s_publish_max_ms;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

void get_effective_base(char* out, size_t out_size) {
  if (!out || out_size == 0) return;
  snprintf(out, out_size, "%s", s_effective_base);
}

void trigger_ha_discovery() {
  portENTER_CRITICAL(&s_mux);
  if (s_state == State::Connected) {
    s_just_connected = true;
    // Force discovery enabled for this one cycle; the live config is authoritative
    // for subsequent connects. Admin explicitly requested this, so bypass the flag.
    s_cfg.ha_discovery_enabled = true;
  }
  portEXIT_CRITICAL(&s_mux);
}

}  // namespace mqtt::publisher
