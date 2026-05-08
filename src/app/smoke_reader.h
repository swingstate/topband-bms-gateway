#pragma once

// ── Smoke reader (Phase C only) ─────────────────────────────────────────────
// Temporary Core 1 task that reads the snapshot bus and logs a one-line
// summary every 100 ms. Proves the lock-free cross-core read path works.
// Enabled by SMOKE_READER_ENABLED build flag (defined in platformio.ini for
// Phase C). Will be removed / replaced by HTTP/MQTT/History consumers in
// later phases.

namespace app {
#if SMOKE_READER_ENABLED
void start_smoke_reader();
#endif
}  // namespace app
