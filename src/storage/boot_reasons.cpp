#include "boot_reasons.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>

static const char* TAG = "boot_reasons";

static constexpr const char* NVS_NS   = "boot_rsns";
static constexpr const char* KEY_RING = "ring";
static constexpr size_t      RING_SIZE = 5;
// Uptime threshold: a SW reset is "quick" (deliberate rapid-press) only when
// the device hadn't been running long enough to be in normal operation.
static constexpr uint32_t RAPID_THRESHOLD_MS  = 5000;
// A SW reset that follows a long-running boot is a normal operation restart
// (OTA apply, settings save). Clear the ring rather than accumulate.
static constexpr uint32_t NORMAL_BOOT_MS = 30000;

// In-RAM copy of the NVS ring. 0 = empty (never written).
static uint32_t g_ring[RING_SIZE] = {};
static bool g_loaded = false;

static void load_ring() {
  if (g_loaded) return;
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
    size_t len = sizeof(g_ring);
    nvs_get_blob(h, KEY_RING, g_ring, &len);
    nvs_close(h);
  }
  g_loaded = true;
}

static void save_ring() {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
    return;
  }
  nvs_set_blob(h, KEY_RING, g_ring, sizeof(g_ring));
  nvs_commit(h);
  nvs_close(h);
}

namespace storage::boot_reasons {

void record_this_boot() {
  esp_reset_reason_t reason = esp_reset_reason();

  // Only ESP_RST_SW (deliberate software restart via esp_restart()) counts.
  // All other reset reasons excluded: crashes/WDT never wipe config, and
  // POWERON/BROWNOUT are indistinguishable on ESP32-S3.
  if (reason != ESP_RST_SW) {
    ESP_LOGD(TAG,
             "record_this_boot: reset_reason=%d — excluded (only ESP_RST_SW counts)",
             (int)reason);
    return;
  }

  // Record this early-boot uptime (always ~100 ms — esp_timer resets on every
  // hardware reset so we cannot observe the PREVIOUS boot's runtime here).
  // mark_healthy() is the mechanism that clears the ring after a stable run.
  uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
  ESP_LOGI(TAG, "record_this_boot: uptime=%u ms (SW reset — counting)", (unsigned)uptime_ms);
  load_ring();
  memmove(g_ring, g_ring + 1, sizeof(uint32_t) * (RING_SIZE - 1));
  g_ring[RING_SIZE - 1] = uptime_ms;
  save_ring();
}

void mark_healthy() {
  // Called by housekeeping once the device has been running stably for ≥ 30 s.
  // Clears the rapid-reset ring so that legitimate SW restarts (OTA apply, settings
  // save, self-test timeout) do not accumulate toward the 5x-wipe threshold across
  // separate boot cycles.
  //
  // Background: record_this_boot() always sees early-boot uptime (~100 ms) because
  // esp_timer resets on every hardware reset — there is no software-visible way to
  // observe how long the PREVIOUS boot ran.  The "clear on long-running boot" logic
  // must therefore run DURING the current boot, not at the next boot's record call.
  load_ring();
  bool had_entries = false;
  for (size_t i = 0; i < RING_SIZE; i++) {
    if (g_ring[i] != 0) { had_entries = true; break; }
  }
  if (had_entries) {
    memset(g_ring, 0, sizeof(g_ring));
    g_loaded = true;
    save_ring();
    ESP_LOGI(TAG, "mark_healthy: stable ≥30 s — rapid-reset ring cleared");
  }
}

bool is_5x_reset_detected() {
  load_ring();
  for (size_t i = 0; i < RING_SIZE; i++) {
    if (g_ring[i] == 0 || g_ring[i] >= RAPID_THRESHOLD_MS) {
      return false;
    }
  }
  ESP_LOGW(TAG, "5x rapid reset detected (all 5 uptimes < %u ms)",
           (unsigned)RAPID_THRESHOLD_MS);
  return true;
}

void clear() {
  memset(g_ring, 0, sizeof(g_ring));
  g_loaded = true;
  save_ring();
  ESP_LOGI(TAG, "boot_reasons ring cleared");
}

}  // namespace storage::boot_reasons
