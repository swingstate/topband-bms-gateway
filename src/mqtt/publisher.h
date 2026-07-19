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

// Solar Passthrough state received from the configured OpenDTU MQTT topic.
// out_state: true = passthrough active, false = inactive.
// out_ts_ms: esp_timer millisecond timestamp of last received message (0 = never received).
// Returns false when no topic is configured or no message has been received yet.
bool get_solar_passthrough(bool& out_state, uint32_t& out_ts_ms);

// Requests a retained publish of the config-backup payload on MqttTask's next
// tick. No-op if disconnected — the daily timer or next successful save will
// retry. Call this from app::update_and_save_config() so every save is
// reflected (edge-triggered, exactly once per real save).
void request_config_backup_publish();

// Copies the last-received retained config-backup payload — received via our
// own self-subscription to {effective_base}/system/config_backup — into out.
// Returns false if no backup has been received this session, or it doesn't
// fit in out_size. out is NUL-terminated on success; out_len (if non-null)
// receives the payload length excluding the terminator.
bool get_config_backup(char* out, size_t out_size, size_t* out_len);

}  // namespace mqtt::publisher
