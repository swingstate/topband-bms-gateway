#pragma once
#include <cstddef>
#include <cstdint>

namespace mqtt::topics {

constexpr const char* DATA   = "/data";
constexpr const char* STATUS = "/status";
constexpr const char* ALARM  = "/alarm";
constexpr const char* DIAG   = "/diag";

// Build {base}{suffix} into out. Returns bytes written (0 on overflow).
size_t build(const char* base, const char* suffix, char* out, size_t out_size);

// Build {base}/cells/bms{n} into out.
size_t build_cells(const char* base, uint8_t bms_id, char* out, size_t out_size);

// Build homeassistant/sensor/{device_uid}_{entity_key}/config into out.
size_t build_ha_discovery(const char* device_uid, const char* entity_key,
                           char* out, size_t out_size);

}  // namespace mqtt::topics
