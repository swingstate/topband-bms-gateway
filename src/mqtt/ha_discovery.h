#pragma once
#include "storage/config.h"
#include "mqtt_client.h"

namespace mqtt::ha_discovery {

// Publish all HA discovery payloads for this firmware version.
// Called on every MQTT connect (payloads are retained → idempotent).
// effective_base = cfg.mqtt_base_topic + "-" + last4hexMAC (from publisher).
void publish_all(esp_mqtt_client_handle_t client, const Config& cfg,
                 const char* device_uid, const char* effective_base);

// Publish empty retained payload for any discovery topics this firmware no longer
// uses. Gated by NVS flag so it runs at most once per firmware version.
void cleanup_stale(esp_mqtt_client_handle_t client, const Config& cfg,
                   const char* device_uid, const char* effective_base);

}  // namespace mqtt::ha_discovery
