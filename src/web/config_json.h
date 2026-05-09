#pragma once
#include "storage/config.h"
#include <ArduinoJson.h>

// ── Config ↔ JSON helpers (shared by config and backup handlers) ─────────────
// Used by GET /api/config, POST /api/config, GET /api/backup.

namespace web {

// Serialize cfg into doc. Sensitive fields (mqtt_pass_obf, auth_hash) are
// replaced with empty strings so they are never returned to the browser.
void config_to_json(const Config& cfg, JsonDocument& doc);

// Deserialize doc into cfg. Only fields present in doc are updated;
// the rest of cfg retains its current value. Returns false on type errors.
bool json_to_config(const JsonDocument& doc, Config& cfg);

}  // namespace web
