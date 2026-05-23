#pragma once
#include <cstdint>

// ── Post-OTA self-test / rollback guard ──────────────────────────────────────
// Architecture §8.5, §4.1.
//
// On first boot after esp_ota_set_boot_partition(), the running partition is in
// state ESP_OTA_IMG_PENDING_VERIFY. init() detects this and arms a 5-minute
// watchdog. If all four health checks pass AND uptime ≥ 30 s, it calls
// esp_ota_mark_app_valid_cancel_rollback() — firmware is permanently kept.
// If the deadline expires without all checks, esp_restart() is called. The
// bootloader sees the unvalidated partition and switches back automatically.

namespace app::self_test {

// One bit per health criterion. Callers use the enum value directly.
enum Check : uint8_t {
  WIFI_CONNECTED     = 1 << 0,
  HTTP_SERVER_UP     = 1 << 1,
  SNAPSHOT_PUBLISHED = 1 << 2,
  CONTROLTASK_ALIVE  = 1 << 3,
};

// Call very early in boot.cpp — before WiFi, before tasks.
// Reads the running partition state. No-op if partition is already valid
// (normal boot) or if OTA partition data is unavailable.
void init();

// Called by the task that just satisfied criterion c.
// Thread-safe. No-op when not in a validation window.
void mark_passed(Check c);

// True while the 5-minute validation window is running.
bool in_validation_window();

// Snapshot for /api/ota/status — always safe to call.
struct Status {
  bool     in_progress;
  uint32_t elapsed_s;
  uint32_t deadline_s;       // 300 s
  uint8_t  checks_passed_mask;
};
Status get_status();

}  // namespace app::self_test
