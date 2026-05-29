#include "mqtt/ha_discovery.h"
#include "mqtt/topics.h"
#include "bus/queues.h"
#include "app/version.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>
#include <cctype>

static const char* TAG = "ha_disc";

// Reusable JSON document for all publish_*_entity() helpers.
// All callers run sequentially from publish_all() — no concurrent access.
// Pool grows to peak entity size on first use then stays; no per-entity DRAM
// alloc/free. Per docs/diag-mqtt-crash-review.md Finding 1.
static JsonDocument s_disc_doc;

// ── Entity table ──────────────────────────────────────────────────────────────

struct EntityDef {
  const char* key;            // unique_id suffix and HA entity key
  const char* topic_suffix;   // MQTT topic suffix for this value (e.g. "/soc")
  const char* name;           // human-readable HA entity name
  const char* device_class;   // HA device class (nullptr = no class)
  const char* unit;           // unit_of_measurement (nullptr = none)
  const char* state_class;    // "measurement" | "total_increasing" | nullptr
  const char* icon;           // optional icon override (nullptr = default)
};

// Each entity uses its own individual plain-text topic (no value_template needed).
static const EntityDef SYSTEM_ENTITIES[] = {
  { "soc",              "/soc",              "TopBand BMS — SOC",                    "battery",     "%",   "measurement",       nullptr },
  { "soh",              "/soh",              "TopBand BMS — SOH",                    nullptr,       "%",   "measurement",       "mdi:battery-heart" },
  { "voltage",          "/voltage",          "TopBand BMS — Pack Voltage",           "voltage",     "V",   "measurement",       nullptr },
  { "current",          "/current",          "TopBand BMS — Total Current",          "current",     "A",   "measurement",       nullptr },
  { "power",            "/power",            "TopBand BMS — Pack Power",             "power",       "W",   "measurement",       nullptr },
  { "temperature",      "/temperature",      "TopBand BMS — Temperature",            "temperature", "°C",  "measurement",       nullptr },
  { "cell_v_min",       "/cell_v_min",       "TopBand BMS — Cell Voltage Min",       "voltage",     "V",   "measurement",       nullptr },
  { "cell_v_max",       "/cell_v_max",       "TopBand BMS — Cell Voltage Max",       "voltage",     "V",   "measurement",       nullptr },
  { "cell_drift",       "/cell_drift",       "TopBand BMS — Cell Drift",             "voltage",     "V",   "measurement",       nullptr },
  { "cvl",              "/cvl",              "TopBand BMS — Charge Voltage Limit",   "voltage",     "V",   "measurement",       nullptr },
  { "ccl",              "/ccl",              "TopBand BMS — Charge Current Limit",   "current",     "A",   "measurement",       nullptr },
  { "dcl",              "/dcl",              "TopBand BMS — Discharge Current Limit","current",     "A",   "measurement",       nullptr },
  { "alarm_flags",      "/alarm_flags",      "TopBand BMS — Alarm Flags",            nullptr,       nullptr, nullptr,           "mdi:alarm-light" },
  { "bms_online",       "/bms_online",       "TopBand BMS — Packs Online",           nullptr,       nullptr, "measurement",     "mdi:battery-charging" },
  { "bms_configured",   "/bms_configured",   "TopBand BMS — Packs Configured",       nullptr,       nullptr, "measurement",     "mdi:battery-charging" },
  { "sys_message",      "/sys_message",      "TopBand BMS — System Message",         nullptr,       nullptr, nullptr,           "mdi:message-alert" },
  { "energy_today_in",  "/energy_today_in",  "TopBand BMS — Energy In Today",        "energy",      "kWh", "total_increasing",  nullptr },
  { "energy_today_out", "/energy_today_out", "TopBand BMS — Energy Out Today",       "energy",      "kWh", "total_increasing",  nullptr },
  { "runtime_est_min",  "/runtime_est_min",  "TopBand BMS — Runtime Estimate",       nullptr,       "min", "measurement",       "mdi:timer-outline" },
  { "runtime_est_state","/runtime_est_state","TopBand BMS — Runtime State",          nullptr,       nullptr, nullptr,           "mdi:timer-outline" },
};
static constexpr size_t N_SYSTEM = sizeof(SYSTEM_ENTITIES) / sizeof(SYSTEM_ENTITIES[0]);

// Per-pack entity suffixes appended as "pack_{n}_{key}"
// Published at PerCell level (state_topic = JSON cells topic, needs value_template).
struct PackEntityDef {
  const char* key;
  const char* name_fmt;      // sprintf format: %s = gateway tail (4-char), %u = pack number (1-based)
  const char* device_class;
  const char* unit;
};

static const PackEntityDef PACK_ENTITIES[] = {
  { "voltage",   "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Voltage", "voltage",  "V"  },
  { "current",   "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Current", "current",  "A"  },
  { "soc",       "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 SOC",     "battery",  "%"  },
  { "alarm_bits","TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Alarms",  nullptr,    nullptr },
};
static constexpr size_t N_PACK = sizeof(PACK_ENTITIES) / sizeof(PACK_ENTITIES[0]);

// Per-pack plain-text entities: published at PerPack level.
// state_topic = {base}/pack{N}/{topic_suffix}  (plain text, no value_template).
// Each pack gets its own HA sub-device parented to the gateway device.
struct PlainPackEntityDef {
  const char* topic_suffix;   // MQTT topic suffix, e.g. "soc"
  const char* uid_key;        // entity unique_id part appended after "p{N}_"
  const char* name_fmt;       // sprintf format: %s = gateway tail (4-char), %u = pack number
  const char* device_class;
  const char* unit;
  const char* state_class;
  bool        is_binary;      // true → binary_sensor component
};

static const PlainPackEntityDef PLAIN_PACK_ENTITIES[] = {
  { "soc",         "soc",      "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 SOC",         "battery",     "%",   "measurement", false },
  { "voltage",     "voltage",  "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Voltage",     "voltage",     "V",   "measurement", false },
  { "current",     "current",  "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Current",     "current",     "A",   "measurement", false },
  { "power",       "power",    "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Power",       "power",       "W",   "measurement", false },
  { "temperature", "temp",     "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Temperature", "temperature", "\xc2\xb0" "C", "measurement", false },
  { "cell_v_min",  "cvmin",    "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Cell V Min",  "voltage",     "V",   "measurement", false },
  { "cell_v_max",  "cvmax",    "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Cell V Max",  "voltage",     "V",   "measurement", false },
  { "cell_drift",  "cdrift",   "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Cell Drift",  "voltage",     "V",   "measurement", false },
  { "soh",         "soh",      "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 SOH",         nullptr,       "%",   "measurement", false },
  { "cycles",      "cycles",   "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Cycles",      nullptr,       nullptr, nullptr,     false },
  { "online",      "online",   "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Online",      "connectivity",nullptr, nullptr,     true  },
};
static constexpr size_t N_PLAIN_PACK = sizeof(PLAIN_PACK_ENTITIES) / sizeof(PLAIN_PACK_ENTITIES[0]);

// ── Internal state ────────────────────────────────────────────────────────────

// Set by check_at_boot() when firmware version changed; cleared by
// publish_cleanup_if_needed() after tombstones are enqueued.
static bool s_cleanup_needed = false;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Enqueue one discovery message (full disc_topic path, retained).
// payload == nullptr → empty retained payload (tombstone for stale entities).
static void enqueue_disc(const char* disc_topic,
                          const char* payload, size_t payload_len) {
  if (!q_mqtt_publish) return;
  MqttPublishRequest req = {};
  req.topic    = MqttPublishRequest::Topic::Discovery;
  req.pack_id  = 0xFF;
  req.retained = true;
  snprintf(req.topic_suffix, sizeof(req.topic_suffix), "%s", disc_topic);
  if (payload && payload_len > 0) {
    size_t copy = payload_len < sizeof(req.payload) - 1 ? payload_len : sizeof(req.payload) - 1;
    memcpy(req.payload, payload, copy);
    req.payload[copy] = '\0';
    req.payload_len = (uint16_t)copy;
  } else {
    req.payload[0] = '\0';
    req.payload_len = 0;
  }
  // Drop oldest on full queue — same policy as post_mqtt in housekeeping.
  if (xQueueSend(q_mqtt_publish, &req, 0) != pdTRUE) {
    static MqttPublishRequest s_dropped;
    xQueueReceive(q_mqtt_publish, &s_dropped, 0);
    xQueueSend(q_mqtt_publish, &req, 0);
  }
}

static void build_device_block(JsonDocument& doc, const char* device_uid) {
  JsonObject dev = doc["device"].to<JsonObject>();
  char ident[48];
  snprintf(ident, sizeof(ident), "topband_bms_%s", device_uid + 12 /*skip "topband_bms_"*/);
  dev["identifiers"].to<JsonArray>().add(ident);

  // device_uid = "topband_bms_xxxxxxxxxxxx" (12-char hex MAC).
  // Append the last 4 hex chars (last 2 MAC bytes) uppercased so multiple
  // gateways on the same broker are distinguishable in the HA device list.
  char tail[5];
  snprintf(tail, sizeof(tail), "%s", device_uid + 20);
  for (char* p = tail; *p; p++) *p = (char)toupper((unsigned char)*p);
  char dev_name[40];
  snprintf(dev_name, sizeof(dev_name), "TopBand BMS Gateway %s", tail);

  dev["name"]         = dev_name;
  dev["manufacturer"] = "TopBand";
  dev["model"]        = "BMS Gateway";
  dev["sw_version"]   = FW_VERSION_FULL;
}

static void publish_system_entity(const EntityDef& ent,
                                   const char* device_uid,
                                   const char* effective_base) {
  char disc_topic[160];
  char uid_key[64];
  snprintf(uid_key, sizeof(uid_key), "%s_%s", device_uid, ent.key);
  if (!mqtt::topics::build_ha_discovery(device_uid, ent.key, disc_topic, sizeof(disc_topic))) {
    ESP_LOGW(TAG, "Discovery topic too long for entity %s", ent.key);
    return;
  }

  // Each entity has its own plain-text state topic — no value_template needed.
  char avail_topic[128];
  char state_topic[128];
  mqtt::topics::build(effective_base, mqtt::topics::STATUS,    avail_topic, sizeof(avail_topic));
  mqtt::topics::build(effective_base, ent.topic_suffix, state_topic, sizeof(state_topic));

  s_disc_doc.clear();
  s_disc_doc["name"]               = ent.name;
  s_disc_doc["state_topic"]        = state_topic;
  s_disc_doc["unique_id"]          = uid_key;
  s_disc_doc["availability_topic"] = avail_topic;
  if (ent.device_class) s_disc_doc["device_class"]       = ent.device_class;
  if (ent.unit)         s_disc_doc["unit_of_measurement"] = ent.unit;
  if (ent.state_class)  s_disc_doc["state_class"]         = ent.state_class;
  if (ent.icon)         s_disc_doc["icon"]                = ent.icon;
  build_device_block(s_disc_doc, device_uid);

  char buf[640];
  size_t n = serializeJson(s_disc_doc, buf, sizeof(buf));
  if (n == 0) {
    ESP_LOGW(TAG, "Discovery payload overflow for %s", ent.key);
    return;
  }
  enqueue_disc(disc_topic, buf, n);
}

static void publish_pack_entity(uint8_t pack_idx,           // 0-based
                                 const PackEntityDef& ent,
                                 const char* device_uid,
                                 const char* effective_base) {
  // entity key = "pack_{n}_{key}" (n is 1-based for human readability)
  char entity_key[32];
  snprintf(entity_key, sizeof(entity_key), "pack_%u_%s", (unsigned)(pack_idx + 1), ent.key);

  char disc_topic[160];
  if (!mqtt::topics::build_ha_discovery(device_uid, entity_key, disc_topic, sizeof(disc_topic))) {
    return;
  }

  char avail_topic[128];
  char state_topic[128];
  mqtt::topics::build(effective_base, mqtt::topics::STATUS, avail_topic, sizeof(avail_topic));
  mqtt::topics::build_cells(effective_base, pack_idx, state_topic, sizeof(state_topic));

  char tail[5];
  snprintf(tail, sizeof(tail), "%s", device_uid + 20);
  for (char* p = tail; *p; p++) *p = (char)toupper((unsigned char)*p);
  char name[64];
  snprintf(name, sizeof(name), ent.name_fmt, tail, (unsigned)(pack_idx + 1));

  char uid_key[64];
  snprintf(uid_key, sizeof(uid_key), "%s_%s", device_uid, entity_key);

  char vt[48];
  snprintf(vt, sizeof(vt), "{{ value_json.%s }}", ent.key);

  s_disc_doc.clear();
  s_disc_doc["name"]               = name;
  s_disc_doc["state_topic"]        = state_topic;
  s_disc_doc["value_template"]     = vt;
  s_disc_doc["unique_id"]          = uid_key;
  s_disc_doc["availability_topic"] = avail_topic;
  if (ent.device_class) s_disc_doc["device_class"]       = ent.device_class;
  if (ent.unit)         s_disc_doc["unit_of_measurement"] = ent.unit;
  build_device_block(s_disc_doc, device_uid);

  char buf[640];
  size_t n = serializeJson(s_disc_doc, buf, sizeof(buf));
  if (n == 0) return;
  enqueue_disc(disc_topic, buf, n);
}

static void publish_plain_pack_entity(uint8_t pack_idx,
                                       const PlainPackEntityDef& ent,
                                       const char* device_uid,
                                       const char* effective_base) {
  const uint8_t pack_n = pack_idx + 1;   // 1-based

  // entity unique_id: "{device_uid}_p{N}_{uid_key}"
  char uid_key[48];
  snprintf(uid_key, sizeof(uid_key), "%s_p%u_%s", device_uid, (unsigned)pack_n, ent.uid_key);

  // Discovery topic: sensor or binary_sensor component
  char disc_topic[180];
  snprintf(disc_topic, sizeof(disc_topic),
           ent.is_binary ? "homeassistant/binary_sensor/%s/config"
                         : "homeassistant/sensor/%s/config",
           uid_key);

  char avail_topic[128];
  char state_topic[128];
  mqtt::topics::build(effective_base, mqtt::topics::STATUS, avail_topic, sizeof(avail_topic));
  // state_topic = {effective_base}/pack{N}/{topic_suffix}
  snprintf(state_topic, sizeof(state_topic), "%s/pack%u/%s",
           effective_base, (unsigned)pack_n, ent.topic_suffix);

  char tail[5];
  snprintf(tail, sizeof(tail), "%s", device_uid + 20);
  for (char* p = tail; *p; p++) *p = (char)toupper((unsigned char)*p);
  char name[64];
  snprintf(name, sizeof(name), ent.name_fmt, tail, (unsigned)pack_n);

  // Sub-device block: each pack is its own device, parented to the gateway.
  // gateway identifier = "topband_bms_XXXX" (device_uid itself)
  char pack_dev_id[52];
  snprintf(pack_dev_id, sizeof(pack_dev_id), "%s_p%u", device_uid, (unsigned)pack_n);
  char pack_dev_name[40];
  snprintf(pack_dev_name, sizeof(pack_dev_name), "TopBand BMS Gateway %s Pack %u",
           tail, (unsigned)pack_n);

  s_disc_doc.clear();
  s_disc_doc["name"]               = name;
  s_disc_doc["state_topic"]        = state_topic;
  s_disc_doc["unique_id"]          = uid_key;
  s_disc_doc["availability_topic"] = avail_topic;
  if (ent.device_class) s_disc_doc["device_class"]       = ent.device_class;
  if (ent.unit)         s_disc_doc["unit_of_measurement"] = ent.unit;
  if (ent.state_class)  s_disc_doc["state_class"]         = ent.state_class;
  if (ent.is_binary) {
    // binary_sensor needs explicit payload mapping for "true"/"false" strings
    s_disc_doc["payload_on"]  = "true";
    s_disc_doc["payload_off"] = "false";
  }

  JsonObject dev = s_disc_doc["device"].to<JsonObject>();
  dev["identifiers"].to<JsonArray>().add(pack_dev_id);
  dev["name"]         = pack_dev_name;
  dev["manufacturer"] = "TopBand";
  // sw_version omitted: gateway cannot read BMS firmware version over RS485;
  // showing the gateway FW on the pack device would be misleading.
  dev["via_device"]   = device_uid;

  char buf[700];
  size_t n = serializeJson(s_disc_doc, buf, sizeof(buf));
  if (n == 0) {
    ESP_LOGW(TAG, "Plain-pack discovery overflow for %s", uid_key);
    return;
  }
  enqueue_disc(disc_topic, buf, n);
}

static void publish_cell_entity(uint8_t pack_idx,
                                 uint8_t cell_idx,       // 0-based
                                 const char* device_uid,
                                 const char* effective_base) {
  const uint8_t pack_n = pack_idx + 1;
  const uint8_t cell_n = cell_idx + 1;  // 1-based, zero-padded in topic

  char uid_key[64];
  snprintf(uid_key, sizeof(uid_key), "%s_p%u_cell_v_%02u",
           device_uid, (unsigned)pack_n, (unsigned)cell_n);

  char disc_topic[180];
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/sensor/%s/config", uid_key);

  char avail_topic[128];
  char state_topic[128];
  mqtt::topics::build(effective_base, mqtt::topics::STATUS, avail_topic, sizeof(avail_topic));
  snprintf(state_topic, sizeof(state_topic), "%s/pack%u/cell_v_%02u",
           effective_base, (unsigned)pack_n, (unsigned)cell_n);

  char tail[5];
  snprintf(tail, sizeof(tail), "%s", device_uid + 20);
  for (char* p = tail; *p; p++) *p = (char)toupper((unsigned char)*p);
  char name[64];
  snprintf(name, sizeof(name), "TopBand BMS Gateway %s Pack %u \xe2\x80\x94 Cell V %02u",
           tail, (unsigned)pack_n, (unsigned)cell_n);

  char pack_dev_id[52];
  snprintf(pack_dev_id, sizeof(pack_dev_id), "%s_p%u", device_uid, (unsigned)pack_n);
  char pack_dev_name[40];
  snprintf(pack_dev_name, sizeof(pack_dev_name), "TopBand BMS Gateway %s Pack %u",
           tail, (unsigned)pack_n);

  s_disc_doc.clear();
  s_disc_doc["name"]                = name;
  s_disc_doc["state_topic"]         = state_topic;
  s_disc_doc["unique_id"]           = uid_key;
  s_disc_doc["availability_topic"]  = avail_topic;
  s_disc_doc["device_class"]        = "voltage";
  s_disc_doc["unit_of_measurement"] = "V";
  s_disc_doc["state_class"]         = "measurement";

  JsonObject dev2 = s_disc_doc["device"].to<JsonObject>();
  dev2["identifiers"].to<JsonArray>().add(pack_dev_id);
  dev2["name"]         = pack_dev_name;
  dev2["manufacturer"] = "TopBand";
  dev2["via_device"]   = device_uid;

  char buf[700];
  size_t n = serializeJson(s_disc_doc, buf, sizeof(buf));
  if (n == 0) {
    ESP_LOGW(TAG, "Cell entity discovery overflow for %s", uid_key);
    return;
  }
  enqueue_disc(disc_topic, buf, n);
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace mqtt::ha_discovery {

void publish_all(const Config& cfg,
                 const char* device_uid, const char* effective_base) {
  const bool have_per_pack = (cfg.mqtt_level >= Config::MqttLevel::PerPack);
  const bool have_per_cell = (cfg.mqtt_level >= Config::MqttLevel::PerCell);

  ESP_LOGI(TAG, "Queuing HA discovery (%zu system, per_pack=%d, per_cell=%d)",
           N_SYSTEM, (int)have_per_pack, (int)have_per_cell);

  // Each entity is enqueued into q_mqtt_publish; MqttTask drains at 50 ms cadence.
  // Total time ~4.65 s for 93 entities — fully non-blocking from callers perspective.
  for (size_t i = 0; i < N_SYSTEM; ++i) {
    publish_system_entity(SYSTEM_ENTITIES[i], device_uid, effective_base);
  }

  if (have_per_pack) {
    for (uint8_t p = 0; p < cfg.bms_count && p < 16; ++p) {
      for (size_t i = 0; i < N_PLAIN_PACK; ++i) {
        publish_plain_pack_entity(p, PLAIN_PACK_ENTITIES[i], device_uid, effective_base);
      }
    }
  }

  if (have_per_cell) {
    for (uint8_t p = 0; p < cfg.bms_count && p < 16; ++p) {
      for (size_t i = 0; i < N_PACK; ++i) {
        publish_pack_entity(p, PACK_ENTITIES[i], device_uid, effective_base);
      }
    }
    // Individual cell voltage sensors: all 15 slots per pack.
    for (uint8_t p = 0; p < cfg.bms_count && p < 16; ++p) {
      for (uint8_t ci = 0; ci < 15; ++ci) {
        publish_cell_entity(p, ci, device_uid, effective_base);
      }
    }
  }

  ESP_LOGI(TAG, "HA discovery enqueued (%zu+ items)", N_SYSTEM);
}

void check_at_boot() {
  // Read NVS version marker. If firmware changed: write new marker (flash commit
  // here at boot, not on MQTT connect) and arm s_cleanup_needed for the next
  // connect. Per docs/diag-mqtt-crash-review.md Finding 5.
  nvs_handle_t nvs = 0;
  if (nvs_open("gateway", NVS_READWRITE, &nvs) != ESP_OK) return;

  char stored[32] = {};
  size_t stored_len = sizeof(stored);
  nvs_get_str(nvs, "ha_disco_v", stored, &stored_len);  // ok to fail (key missing)

  if (strcmp(stored, FW_VERSION) != 0) {
    s_cleanup_needed = true;
    nvs_set_str(nvs, "ha_disco_v", FW_VERSION);
    nvs_commit(nvs);  // blocking flash write runs once at boot, off MQTT task
    ESP_LOGI(TAG, "HA cleanup armed: firmware '%s' -> '%s'", stored, FW_VERSION);
  }
  nvs_close(nvs);
}

void publish_cleanup_if_needed(const char* device_uid) {
  if (!s_cleanup_needed) return;
  s_cleanup_needed = false;

  static const char* STALE_KEYS[] = {
    "diag_loop_max_ms",
    "diag_handler_max_ms",
    "soc_avg", "soh_avg", "pack_voltage_avg", "pack_current_total",
    "pack_power_w", "temp_avg", "temp_max", "cell_v_drift",
    "cvl_v", "ccl_a", "dcl_a", "bms_count_online",
  };
  for (const char* key : STALE_KEYS) {
    char disc_topic[160];
    if (mqtt::topics::build_ha_discovery(device_uid, key, disc_topic, sizeof(disc_topic))) {
      enqueue_disc(disc_topic, nullptr, 0);
    }
  }
  ESP_LOGI(TAG, "HA cleanup tombstones enqueued for %s", FW_VERSION);
}

}  // namespace mqtt::ha_discovery
