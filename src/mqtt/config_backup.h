#pragma once
#include "storage/config.h"
#include <ArduinoJson.h>

// ── MQTT config-backup payload builder ─────────────────────────────────────
// Host-portable (ArduinoJson + storage/config.h only, no IDF deps) so it can
// be exercised directly from test/host.

namespace mqtt::config_backup {

// Builds the JSON payload published (retained) to the config-backup MQTT
// topic. Reuses web::config_to_json() for the full field set, then removes
// WiFi SSID and MQTT broker host/port/username/password — those are needed
// just to reach this backup in the first place, so including them would be
// circular. Fields are omitted entirely, not blanked, so a consumer can tell
// "deliberately excluded" apart from "saved as empty".
void build_json(const Config& cfg, JsonDocument& doc);

}  // namespace mqtt::config_backup
