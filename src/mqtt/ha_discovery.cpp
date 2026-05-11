#include "mqtt/ha_discovery.h"
#include "mqtt/topics.h"
#include "app/version.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "ha_disc";

// ── Entity table ──────────────────────────────────────────────────────────────

struct EntityDef {
  const char* key;            // unique_id suffix and value_template key
  const char* name;           // human-readable HA entity name
  const char* device_class;   // HA device class (nullptr = no class)
  const char* unit;           // unit_of_measurement (nullptr = none)
  const char* state_class;    // "measurement" | "total_increasing" | nullptr
  const char* icon;           // optional icon override (nullptr = default)
};

static const EntityDef SYSTEM_ENTITIES[] = {
  { "soc_avg",           "TopBand BMS — SOC Average",            "battery",     "%",  "measurement", nullptr },
  { "soh_avg",           "TopBand BMS — SOH Average",            nullptr,       "%",  "measurement", "mdi:battery-heart" },
  { "pack_voltage_avg",  "TopBand BMS — Pack Voltage",           "voltage",     "V",  "measurement", nullptr },
  { "pack_current_total","TopBand BMS — Total Current",          "current",     "A",  "measurement", nullptr },
  { "pack_power_w",      "TopBand BMS — Pack Power",             "power",       "W",  "measurement", nullptr },
  { "temp_avg",          "TopBand BMS — Temp Average",           "temperature", "°C", "measurement", nullptr },
  { "temp_max",          "TopBand BMS — Temp Max",               "temperature", "°C", "measurement", nullptr },
  { "cell_v_min",        "TopBand BMS — Cell Voltage Min",       "voltage",     "V",  "measurement", nullptr },
  { "cell_v_max",        "TopBand BMS — Cell Voltage Max",       "voltage",     "V",  "measurement", nullptr },
  { "cell_v_drift",      "TopBand BMS — Cell Voltage Drift",     "voltage",     "V",  "measurement", nullptr },
  { "cvl_v",             "TopBand BMS — Charge Voltage Limit",   "voltage",     "V",  "measurement", nullptr },
  { "ccl_a",             "TopBand BMS — Charge Current Limit",   "current",     "A",  "measurement", nullptr },
  { "dcl_a",             "TopBand BMS — Discharge Current Limit","current",     "A",  "measurement", nullptr },
  { "alarm_flags",       "TopBand BMS — Alarm Flags",            nullptr,       nullptr, nullptr,    "mdi:alarm-light" },
  { "bms_count_online",  "TopBand BMS — Packs Online",           nullptr,       nullptr, "measurement", "mdi:battery-charging" },
  { "sys_message",       "TopBand BMS — System Message",         nullptr,       nullptr, nullptr,    "mdi:message-alert" },
};
static constexpr size_t N_SYSTEM = sizeof(SYSTEM_ENTITIES) / sizeof(SYSTEM_ENTITIES[0]);

// Per-pack entity suffixes appended as "pack_{n}_{key}"
struct PackEntityDef {
  const char* key;
  const char* name_fmt;      // sprintf format, receives pack number (1-based)
  const char* device_class;
  const char* unit;
};

static const PackEntityDef PACK_ENTITIES[] = {
  { "voltage",   "TopBand BMS — Pack %u Voltage", "voltage",  "V"  },
  { "current",   "TopBand BMS — Pack %u Current", "current",  "A"  },
  { "soc",       "TopBand BMS — Pack %u SOC",     "battery",  "%"  },
  { "alarm_bits","TopBand BMS — Pack %u Alarms",  nullptr,    nullptr },
};
static constexpr size_t N_PACK = sizeof(PACK_ENTITIES) / sizeof(PACK_ENTITIES[0]);

// ── Helpers ───────────────────────────────────────────────────────────────────

static void publish_one(esp_mqtt_client_handle_t client,
                         const char* disc_topic,
                         const char* payload,          // nullptr → empty (cleanup)
                         int payload_len) {
  int msg_id = esp_mqtt_client_publish(client, disc_topic,
                                        payload, payload_len,
                                        0 /*qos*/, 1 /*retain*/);
  if (msg_id < 0) {
    ESP_LOGW(TAG, "Discovery publish failed: %s", disc_topic);
  }
}

static void build_device_block(JsonDocument& doc, const char* device_uid) {
  JsonObject dev = doc["device"].to<JsonObject>();
  char ident[48];
  snprintf(ident, sizeof(ident), "topband_bms_%s", device_uid + 12 /*skip "topband_bms_"*/);
  dev["identifiers"].to<JsonArray>().add(ident);
  dev["name"]         = "TopBand BMS Gateway";
  dev["manufacturer"] = "TopBand";
  dev["model"]        = "ESP32-S3";
  dev["sw_version"]   = FW_VERSION;
}

static void publish_system_entity(esp_mqtt_client_handle_t client,
                                   const EntityDef& ent,
                                   const char* device_uid,
                                   const char* effective_base) {
  char disc_topic[160];
  // unique_id composed with device_uid
  char uid_key[64];
  snprintf(uid_key, sizeof(uid_key), "%s_%s", device_uid, ent.key);
  if (!mqtt::topics::build_ha_discovery(device_uid, ent.key, disc_topic, sizeof(disc_topic))) {
    ESP_LOGW(TAG, "Discovery topic too long for entity %s", ent.key);
    return;
  }

  // Build availability + state topic paths
  char avail_topic[128];
  char state_topic[128];
  mqtt::topics::build(effective_base, mqtt::topics::STATUS, avail_topic, sizeof(avail_topic));
  mqtt::topics::build(effective_base, mqtt::topics::DATA,   state_topic, sizeof(state_topic));

  char vt[64];
  snprintf(vt, sizeof(vt), "{{ value_json.%s }}", ent.key);

  JsonDocument doc;
  doc["name"]                = ent.name;
  doc["state_topic"]         = state_topic;
  doc["value_template"]      = vt;
  doc["unique_id"]           = uid_key;
  doc["availability_topic"]  = avail_topic;
  if (ent.device_class) doc["device_class"]       = ent.device_class;
  if (ent.unit)         doc["unit_of_measurement"] = ent.unit;
  if (ent.state_class)  doc["state_class"]         = ent.state_class;
  if (ent.icon)         doc["icon"]                = ent.icon;
  build_device_block(doc, device_uid);

  char buf[640];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) {
    ESP_LOGW(TAG, "Discovery payload overflow for %s", ent.key);
    return;
  }
  publish_one(client, disc_topic, buf, (int)n);
}

static void publish_pack_entity(esp_mqtt_client_handle_t client,
                                 uint8_t pack_idx,           // 0-based
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

  char name[64];
  snprintf(name, sizeof(name), ent.name_fmt, (unsigned)(pack_idx + 1));

  char uid_key[64];
  snprintf(uid_key, sizeof(uid_key), "%s_%s", device_uid, entity_key);

  char vt[48];
  snprintf(vt, sizeof(vt), "{{ value_json.%s }}", ent.key);

  JsonDocument doc;
  doc["name"]               = name;
  doc["state_topic"]        = state_topic;
  doc["value_template"]     = vt;
  doc["unique_id"]          = uid_key;
  doc["availability_topic"] = avail_topic;
  if (ent.device_class) doc["device_class"]       = ent.device_class;
  if (ent.unit)         doc["unit_of_measurement"] = ent.unit;
  build_device_block(doc, device_uid);

  char buf[640];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return;
  publish_one(client, disc_topic, buf, (int)n);
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace mqtt::ha_discovery {

void publish_all(esp_mqtt_client_handle_t client, const Config& cfg,
                 const char* device_uid, const char* effective_base) {
  ESP_LOGI(TAG, "Publishing HA discovery (%zu system + %u×%zu pack entities)",
           N_SYSTEM, (unsigned)cfg.bms_count, N_PACK);

  for (size_t i = 0; i < N_SYSTEM; ++i) {
    publish_system_entity(client, SYSTEM_ENTITIES[i], device_uid, effective_base);
    // Brief yield between publishes to avoid overwhelming the MQTT stack
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Per-pack entities (use per-pack cells topic as state_topic)
  for (uint8_t p = 0; p < cfg.bms_count && p < 16; ++p) {
    for (size_t i = 0; i < N_PACK; ++i) {
      publish_pack_entity(client, p, PACK_ENTITIES[i], device_uid, effective_base);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  ESP_LOGI(TAG, "HA discovery complete");
}

void cleanup_stale(esp_mqtt_client_handle_t client, const Config& cfg,
                   const char* device_uid, const char* effective_base) {
  (void)cfg; (void)effective_base;

  // Gate: only run once per firmware version build.
  // NVS key "ha_disco_v" (≤15 chars) stores the last FW_VERSION we ran cleanup for.
  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open("gateway", NVS_READWRITE, &nvs);
  if (err != ESP_OK) return;

  char stored[32] = {};
  size_t stored_len = sizeof(stored);
  nvs_get_str(nvs, "ha_disco_v", stored, &stored_len);  // ok to fail (key missing)

  if (strcmp(stored, FW_VERSION) == 0) {
    // Already ran for this version
    nvs_close(nvs);
    return;
  }

  ESP_LOGI(TAG, "HA discovery cleanup: firmware changed from '%s' to '%s'",
           stored, FW_VERSION);

  // V2.67-era topic names that no longer exist in V3.0.
  // Publishing empty retained payload removes the stale entity from HA.
  static const char* STALE_KEYS[] = {
    "diag_loop_max_ms",
    "diag_handler_max_ms",
  };
  for (const char* key : STALE_KEYS) {
    char disc_topic[160];
    if (mqtt::topics::build_ha_discovery(device_uid, key, disc_topic, sizeof(disc_topic))) {
      publish_one(client, disc_topic, nullptr, 0);
    }
  }

  // Write new version marker
  nvs_set_str(nvs, "ha_disco_v", FW_VERSION);
  nvs_commit(nvs);
  nvs_close(nvs);

  ESP_LOGI(TAG, "HA discovery cleanup done — marker written for %s", FW_VERSION);
}

}  // namespace mqtt::ha_discovery
