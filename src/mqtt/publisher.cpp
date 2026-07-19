#include "mqtt/publisher.h"
#include "mqtt/topics.h"
#include "mqtt/ha_discovery.h"
#include "mqtt/config_backup.h"
#include "bus/queues.h"
#include "app/boot.h"
#include "app/version.h"
#include "net/ntp.h"
#include "diag/alerts.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ArduinoJson.h>
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

// HA discovery is sent only once per boot. Cleared by trigger_ha_discovery()
// so the user can force a re-send from the UI without rebooting.
static bool s_ha_discovery_done = false;

// Cooperative-shutdown flag. Set by stop() so MqttTask exits its loop cleanly
// before stop() acquires s_mux. Prevents the spinlock-orphan scenario where
// vTaskDelete() during a portENTER_CRITICAL in publish_request() would leave
// s_mux permanently locked. Per docs/diag-mqtt-crash-review.md Finding 7.
static volatile bool s_stop_requested = false;

// ── Solar Passthrough (display-only, subscribed to configured OpenDTU topic) ──
// Written from the esp_mqtt internal task (MQTT_EVENT_DATA).
// Read from httpd task (handlers_live). Guarded by s_mux.
static char     s_passthrough_topic[64]  = {};  // copy of cfg at connect time
static bool     s_passthrough_state      = false;
static bool     s_passthrough_received   = false;  // true after first message
static uint32_t s_passthrough_ts_ms      = 0;

// ── Config backup (retained, on-save + daily) ──────────────────────────────
// Published to {effective_base}/system/config_backup. We also self-subscribe
// to that same topic so the last retained payload is available in RAM for
// "Restore from MQTT backup" — the device is the only writer, but after an
// NVS wipe the broker's retained copy is the sole surviving source of truth.
static char     s_config_backup_topic[128] = {};   // built once in start()
static bool     s_backup_publish_requested = false;  // set by request_config_backup_publish(), guarded by s_mux
static uint32_t s_last_backup_publish_epoch = 0;           // 0 = never published this session

// PSRAM cache of the last-received retained backup payload. Written only from
// the esp_mqtt internal task (MQTT_EVENT_DATA); read from httpd task via
// get_config_backup(). s_backup_assembling is touched only from the esp_mqtt
// task across successive MQTT_EVENT_DATA fragments, so it needs no lock.
static constexpr size_t CONFIG_BACKUP_CACHE_CAP = 3072;
static char*    s_backup_cache      = nullptr;  // heap_caps_malloc'd in start()
static size_t   s_backup_cache_len  = 0;
static bool     s_backup_received   = false;
static bool     s_backup_assembling = false;

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

// Builds and retained-publishes the config-backup payload. Called only from
// MqttTask context (has s_client) when connected. WiFi/MQTT-broker fields are
// excluded by mqtt::config_backup::build_json — never published here.
static void publish_config_backup_now() {
  if (s_config_backup_topic[0] == '\0') return;

  JsonDocument doc;
  mqtt::config_backup::build_json(app::get_config(), doc);

  size_t est = measureJson(doc) + 1;
  char* buf = (char*)heap_caps_malloc(est, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf) {
    ESP_LOGW(TAG, "config-backup publish: OOM (%u bytes)", (unsigned)est);
    return;
  }
  size_t n = serializeJson(doc, buf, est);

  int msg_id = esp_mqtt_client_publish(s_client, s_config_backup_topic, buf, (int)n,
                                        0 /*qos*/, 1 /*retain*/);
  if (msg_id < 0) {
    ESP_LOGW(TAG, "config-backup publish failed (topic=%s)", s_config_backup_topic);
  } else {
    ESP_LOGI(TAG, "config-backup published — %s (%u bytes, retained)",
             s_config_backup_topic, (unsigned)n);
  }
  free(buf);

  uint32_t now = net::ntp::now_unix_s();
  if (now > 0) s_last_backup_publish_epoch = now;
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
    case MqttPublishRequest::Topic::IndividualValue:
      tlen = mqtt::topics::build(s_effective_base, req.topic_suffix, topic, sizeof(topic));
      break;
    case MqttPublishRequest::Topic::Discovery:
      // topic_suffix holds the full homeassistant/... path built by ha_discovery.
      tlen = strlcpy(topic, req.topic_suffix, sizeof(topic));
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
    case MQTT_EVENT_CONNECTED: {
      portENTER_CRITICAL(&s_mux);
      s_state = mqtt::publisher::State::Connected;
      s_just_connected = true;
      // Snapshot topic under lock so the subscribe call below is lock-free.
      char pt[sizeof(s_passthrough_topic)];
      memcpy(pt, s_passthrough_topic, sizeof(pt));
      portEXIT_CRITICAL(&s_mux);
      diag::alerts::emit(diag::alerts::Severity::Info, "mqtt",
                         "connected to %s", s_cfg.mqtt_host);
      if (pt[0] != '\0') {
        int rc = esp_mqtt_client_subscribe(s_client, pt, 0);
        if (rc < 0) {
          ESP_LOGW(TAG, "solar-passthrough subscribe failed (topic=%s)", pt);
        } else {
          ESP_LOGI(TAG, "subscribed to solar-passthrough topic: %s", pt);
        }
      }
      if (s_config_backup_topic[0] != '\0') {
        int rc = esp_mqtt_client_subscribe(s_client, s_config_backup_topic, 0);
        if (rc < 0) {
          ESP_LOGW(TAG, "config-backup subscribe failed (topic=%s)", s_config_backup_topic);
        } else {
          ESP_LOGI(TAG, "subscribed to own config-backup topic: %s", s_config_backup_topic);
        }
      }
      break;
    }

    case MQTT_EVENT_DISCONNECTED:
      portENTER_CRITICAL(&s_mux);
      s_state = mqtt::publisher::State::Disconnected;
      portEXIT_CRITICAL(&s_mux);
      ESP_LOGW(TAG, "MQTT disconnected — auto-reconnect pending");
      diag::alerts::emit(diag::alerts::Severity::Warn, "mqtt", "disconnected");
      break;

    case MQTT_EVENT_DATA: {
      // Handle incoming messages: solar-passthrough (1 byte) and our own
      // config-backup topic (can span multiple events — see below).
      if (!ev) break;

      if (ev->topic_len > 0) {
        // Start of a new message — the topic is only present on the first
        // fragment, so this is where we identify which topic it belongs to.
        char topic_buf[128];
        size_t tlen = (ev->topic_len < sizeof(topic_buf) - 1) ? (size_t)ev->topic_len
                                                               : sizeof(topic_buf) - 1;
        memcpy(topic_buf, ev->topic, tlen);
        topic_buf[tlen] = '\0';

        portENTER_CRITICAL(&s_mux);
        bool is_pt = (s_passthrough_topic[0] != '\0' &&
                      strncmp(topic_buf, s_passthrough_topic, sizeof(s_passthrough_topic) - 1) == 0);
        portEXIT_CRITICAL(&s_mux);

        if (is_pt && ev->data_len > 0) {
          // Payload: "1"/"true"/"True" = active; anything else = inactive.
          char c = ev->data[0];
          bool active = (c == '1' || c == 't' || c == 'T');
          uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
          portENTER_CRITICAL(&s_mux);
          s_passthrough_state    = active;
          s_passthrough_received = true;
          s_passthrough_ts_ms    = now_ms;
          portEXIT_CRITICAL(&s_mux);
        }

        bool is_backup = (s_config_backup_topic[0] != '\0' &&
                          strncmp(topic_buf, s_config_backup_topic, sizeof(s_config_backup_topic) - 1) == 0);
        s_backup_assembling = (is_backup && s_backup_cache != nullptr &&
                               ev->total_data_len > 0 &&
                               (size_t)ev->total_data_len <= CONFIG_BACKUP_CACHE_CAP);
        if (is_backup && !s_backup_assembling) {
          ESP_LOGW(TAG, "config-backup payload too large for cache (%d > %u) — ignoring",
                   ev->total_data_len, (unsigned)CONFIG_BACKUP_CACHE_CAP);
        }
      }

      // Reassemble the config-backup payload, which may arrive across
      // several MQTT_EVENT_DATA events when it exceeds the client's internal
      // buffer (current_data_offset/total_data_len track the split).
      if (s_backup_assembling && ev->data_len > 0 && s_backup_cache) {
        size_t off = (size_t)ev->current_data_offset;
        size_t n   = (size_t)ev->data_len;
        if (off + n <= CONFIG_BACKUP_CACHE_CAP) {
          memcpy(s_backup_cache + off, ev->data, n);
          size_t total = (size_t)ev->total_data_len;
          if (off + n >= total) {
            portENTER_CRITICAL(&s_mux);
            s_backup_cache_len = total;
            s_backup_received  = true;
            portEXIT_CRITICAL(&s_mux);
            s_backup_assembling = false;
            ESP_LOGI(TAG, "cached retained config-backup from broker (%u bytes)", (unsigned)total);
          }
        } else {
          ESP_LOGW(TAG, "config-backup payload overflow — discarding");
          s_backup_assembling = false;
        }
      }
      break;
    }

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
    // Check stop flag before any work this tick.
    if (s_stop_requested) {
      s_task_handle = nullptr;
      vTaskDelete(nullptr);  // self-delete; never returns
    }

    // Check connect event (set by event handler, consumed here).
    // Snapshot ha_en and ha_done under the same critical section as
    // s_just_connected — ensures SMP memory-barrier consistency.
    // Per docs/diag-mqtt-crash-review.md Finding 6.
    bool just_conn = false;
    bool ha_en     = false;
    bool ha_done   = false;
    bool connected = false;
    portENTER_CRITICAL(&s_mux);
    just_conn = s_just_connected;
    s_just_connected = false;
    connected = (s_state == mqtt::publisher::State::Connected);
    ha_en     = s_cfg.ha_discovery_enabled;
    ha_done   = s_ha_discovery_done;
    portEXIT_CRITICAL(&s_mux);

    if (just_conn) {
      publish_status_online();

      if (ha_en && !ha_done) {
        // Snapshot s_cfg once under lock for the discovery calls.
        Config cfg_snap;
        portENTER_CRITICAL(&s_mux);
        cfg_snap = s_cfg;
        portEXIT_CRITICAL(&s_mux);
        // Enqueue stale-topic tombstones first (NVS check done at boot).
        mqtt::ha_discovery::publish_cleanup_if_needed(s_device_uid);
        // Enqueue all discovery payloads — MqttTask drains at 50 ms cadence.
        mqtt::ha_discovery::publish_all(cfg_snap, s_device_uid, s_effective_base);
        s_ha_discovery_done = true;
      }
    }

    // Config-backup: edge-triggered on-save publish + once-daily redundant
    // refresh. Only when connected — no aggressive retry while disconnected,
    // the daily timer (or next save) catches up naturally.
    if (connected) {
      portENTER_CRITICAL(&s_mux);
      bool requested = s_backup_publish_requested;
      s_backup_publish_requested = false;
      portEXIT_CRITICAL(&s_mux);

      uint32_t now = net::ntp::now_unix_s();
      bool daily_due = (now > 0) &&
          (s_last_backup_publish_epoch == 0 || (now - s_last_backup_publish_epoch) >= 86400u);

      if (requested || daily_due) {
        publish_config_backup_now();
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

  // Snapshot config for use in MqttTask; also copy passthrough topic so the
  // MQTT_EVENT_CONNECTED handler can subscribe without holding a long lock.
  portENTER_CRITICAL(&s_mux);
  s_cfg = cfg;
  snprintf(s_passthrough_topic, sizeof(s_passthrough_topic), "%s",
           cfg.mqtt_solar_passthrough_topic);
  portEXIT_CRITICAL(&s_mux);

  compute_mac_identifiers(cfg);

  // Build LWT topic
  char lwt_topic[128];
  if (!mqtt::topics::build(s_effective_base, mqtt::topics::STATUS, lwt_topic, sizeof(lwt_topic))) {
    ESP_LOGE(TAG, "MQTT: LWT topic too long");
    return false;
  }

  // Build config-backup topic and (re)allocate its PSRAM read-back cache.
  if (!mqtt::topics::build(s_effective_base, mqtt::topics::SYSTEM_CONFIG_BACKUP,
                            s_config_backup_topic, sizeof(s_config_backup_topic))) {
    s_config_backup_topic[0] = '\0';
    ESP_LOGW(TAG, "config-backup topic too long — feature disabled this session");
  }
  if (!s_backup_cache) {
    s_backup_cache = (char*)heap_caps_malloc(CONFIG_BACKUP_CACHE_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_backup_cache) {
      ESP_LOGW(TAG, "config-backup cache alloc failed — restore-from-MQTT unavailable this session");
    }
  }
  s_backup_received     = false;
  s_backup_assembling   = false;
  s_last_backup_publish_epoch = 0;

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
  // Internal MQTT task must run below httpd (pri 4) to avoid starving HTTP
  // handlers. Default is CONFIG_MQTT_TASK_PRIORITY = 5, which preempts httpd.
  mcfg.task.priority   = 3;
  mcfg.task.stack_size = 6144;

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

  // Spawn MqttTask on Core 1, priority 3 (architecture §3.2).
  // 8192 B: 884 B Config snapshot + HA-discovery helpers (640-700 B char[] locals under
  // 16-pack × 15-cell load) left only 256 B HWM at 6144 — one exception frame from overflow.
  BaseType_t r = xTaskCreatePinnedToCore(
      mqtt_task_entry, "mqtt", 8192, nullptr, 3, &s_task_handle, /*core*/ 1);
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
  // Cooperative shutdown: set flag, wait for task to self-exit (max 200 ms).
  // Avoids vTaskDelete-while-in-critical-section spinlock orphan (Finding 7).
  if (s_task_handle) {
    s_stop_requested = true;
    for (int i = 0; i < 20 && s_task_handle != nullptr; ++i) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_task_handle != nullptr) {
      // Timeout safeguard — task did not exit cleanly; force-delete as last resort.
      ESP_LOGW("mqtt_pub", "stop(): task did not exit cooperatively, force-deleting");
      vTaskDelete(s_task_handle);
      s_task_handle = nullptr;
    }
    s_stop_requested = false;
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
    // Force discovery enabled and clear the once-per-boot guard so the
    // admin-requested re-send actually fires. The live config is authoritative
    // for automatic reconnects.
    s_cfg.ha_discovery_enabled = true;
    s_ha_discovery_done = false;
  }
  portEXIT_CRITICAL(&s_mux);
}

bool get_solar_passthrough(bool& out_state, uint32_t& out_ts_ms) {
  portENTER_CRITICAL(&s_mux);
  bool configured = (s_passthrough_topic[0] != '\0');
  bool received   = s_passthrough_received;
  bool state      = s_passthrough_state;
  uint32_t ts     = s_passthrough_ts_ms;
  portEXIT_CRITICAL(&s_mux);

  if (!configured || !received) return false;
  out_state  = state;
  out_ts_ms  = ts;
  return true;
}

void request_config_backup_publish() {
  portENTER_CRITICAL(&s_mux);
  s_backup_publish_requested = true;
  portEXIT_CRITICAL(&s_mux);
}

bool get_config_backup(char* out, size_t out_size, size_t* out_len) {
  if (!out || out_size == 0) return false;

  portENTER_CRITICAL(&s_mux);
  bool received = s_backup_received;
  size_t len    = s_backup_cache_len;
  portEXIT_CRITICAL(&s_mux);

  if (!received || len == 0 || !s_backup_cache || len >= out_size) return false;

  // Copied outside the critical section: s_backup_cache is only overwritten
  // by a new retained delivery on the esp_mqtt task, which is rare (only
  // after our own publishes or a reconnect) — an occasional stale read here
  // is harmless for this non-safety-critical, explicit-user-action feature.
  memcpy(out, s_backup_cache, len);
  out[len] = '\0';
  if (out_len) *out_len = len;
  return true;
}

}  // namespace mqtt::publisher
