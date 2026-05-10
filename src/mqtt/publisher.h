#pragma once
#include <cstdint>
#include "storage/config.h"

namespace mqtt::publisher {

enum class State : uint8_t {
  Disabled     = 0,
  Disconnected = 1,
  Connecting   = 2,
  Connected    = 3,
  Failed       = 4,
};

// Spawn MqttTask and start the esp_mqtt client.
// Called from boot.cpp after WiFi STA connect when cfg.mqtt_enabled is true.
// Returns false if MQTT is disabled or task creation fails.
bool start(const Config& cfg);

// Stop the MQTT client and kill MqttTask.
// Safe to call when already stopped.
void stop();

// Apply a new config without device reboot.
// Internally: stop → start with new credentials/host.
void reconfigure(const Config& cfg);

// Thread-safe state/counter accessors.
State    get_state();
uint64_t get_publish_ok();
uint64_t get_publish_fail();
uint64_t get_publish_drops();
uint32_t get_publish_max_ms();

// Returns the effective MQTT base topic (cfg.mqtt_base_topic + "-" + last4hexMAC).
// Valid after start() has been called; returns empty string before that.
void get_effective_base(char* out, size_t out_size);

// Force HA discovery publish on the next MqttTask tick (no-op if disconnected).
void trigger_ha_discovery();

}  // namespace mqtt::publisher
