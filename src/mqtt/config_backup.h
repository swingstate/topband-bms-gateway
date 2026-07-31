#pragma once
#include "storage/config.h"
#include <ArduinoJson.h>

// ── MQTT config-backup payload builder ─────────────────────────────────────
// Host-portable (ArduinoJson + storage/config.h only, no IDF deps) so it can
// be exercised directly from test/host.

namespace mqtt::config_backup {

// Builds the JSON payload published (retained) to the config-backup MQTT
// topic. Reuses web::config_to_json() for the full field set, then removes
// WiFi SSID, MQTT broker host/port/username/password, and the local web-UI
// auth username — the WiFi/MQTT fields are needed just to reach this backup
// in the first place, so including them would be circular; the auth username
// is withheld for the same reason mqtt_user is (it's half of a login
// credential pair, even though it isn't secret by itself). Fields are
// omitted entirely, not blanked, so a consumer can tell "deliberately
// excluded" apart from "saved as empty".
void build_json(const Config& cfg, JsonDocument& doc);

}  // namespace mqtt::config_backup
