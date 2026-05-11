#include "mqtt/topics.h"
#include <cstdio>

namespace mqtt::topics {

size_t build(const char* base, const char* suffix, char* out, size_t out_size) {
  int n = snprintf(out, out_size, "%s%s", base, suffix);
  return (n > 0 && (size_t)n < out_size) ? (size_t)n : 0;
}

size_t build_cells(const char* base, uint8_t bms_id, char* out, size_t out_size) {
  int n = snprintf(out, out_size, "%s/cells/bms%u", base, (unsigned)bms_id);
  return (n > 0 && (size_t)n < out_size) ? (size_t)n : 0;
}

size_t build_ha_discovery(const char* device_uid, const char* entity_key,
                           char* out, size_t out_size) {
  int n = snprintf(out, out_size, "homeassistant/sensor/%s_%s/config",
                   device_uid, entity_key);
  return (n > 0 && (size_t)n < out_size) ? (size_t)n : 0;
}

}  // namespace mqtt::topics
