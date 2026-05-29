#pragma once
#include "storage/config.h"

namespace mqtt::ha_discovery {

// Enqueue all HA discovery payloads into q_mqtt_publish (retained).
// Non-blocking: each entity becomes one queue item drained by MqttTask at its
// normal 50 ms cadence. Total discovery time ~4.65 s for 93 entities.
// Replaces direct esp_mqtt_client_publish() loop to eliminate the 1-1.5 s
// blocking window that starved queue drain on every connect/reconnect.
// Per docs/diag-mqtt-crash-review.md Finding 2.
void publish_all(const Config& cfg,
                 const char* device_uid, const char* effective_base);

// Call once at boot (after NVS init, before MQTT starts). Reads the NVS
// "ha_disco_v" key; if the firmware version changed, writes the new marker
// (NVS commit here at boot, not on MQTT connect) and sets an internal flag so
// publish_cleanup_if_needed() knows to send stale-topic tombstones.
// Per docs/diag-mqtt-crash-review.md Finding 5.
void check_at_boot();

// Enqueue empty retained payloads for stale discovery topics (tombstones).
// Non-blocking. Only posts if check_at_boot() found a version change.
// Call from the MqttTask connect path after publish_all().
void publish_cleanup_if_needed(const char* device_uid);

}  // namespace mqtt::ha_discovery
