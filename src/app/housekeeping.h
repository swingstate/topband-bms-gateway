#pragma once
#include "storage/config.h"

namespace app::housekeeping {

// Spawn HousekeepingTask on Core 1 at priority 1 (architecture §3.2).
// Runs forever. Posts pre-serialized MQTT payloads to q_mqtt_publish
// at the cadences configured in §4.7.
// Returns false on FreeRTOS task creation failure.
// Idempotent: calling start() a second time is a no-op and returns true.
bool start(const Config& cfg);

}  // namespace app::housekeeping
