#include "solar_day_ring.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm>
#include <cstdio>

static const char* TAG = "solar_ring";

// Persistence format magic + version — bump version if SolarDayPoint layout changes.
static constexpr uint32_t PERSIST_MAGIC   = 0x534F4C44u;  // "SOLD"
static constexpr uint32_t PERSIST_VERSION = 1u;
static constexpr const char* PERSIST_PATH     = "/lfs/solar_day.bin";
static constexpr const char* PERSIST_PATH_TMP = "/lfs/solar_day.bin.tmp";

// PSRAM BSS: 288 × 8 B = 2304 B — zero net internal DRAM.
// CPU-only access from HistoryTask (writer) and HttpTask (reader); never DMA.
// EXT_RAM_BSS_ATTR requires CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y
// (already set in sdkconfig.defaults for the existing history ring buffers).
static EXT_RAM_BSS_ATTR SolarDayPoint s_ring[SOLAR_DAY_CAPACITY];

// Tiny state: lives in DRAM BSS (12 B — negligible).
static uint32_t s_head    = 0;  // next-write slot (0..SOLAR_DAY_CAPACITY-1)
static uint32_t s_count   = 0;  // valid entries currently stored
static uint32_t s_day_utc = 0;  // epoch / 86400 of stored day (0 = unset)

namespace app::solar_day_ring {

static void reset_ring(uint32_t day_utc) {
  memset(s_ring, 0, sizeof(s_ring));
  s_head    = 0;
  s_count   = 0;
  s_day_utc = day_utc;
}

void append(uint32_t t_epoch, float pv_power_w, bool valid) {
  if (t_epoch == 0) return;  // NTP not synced yet

  const uint32_t day = t_epoch / 86400u;
  if (day != s_day_utc) {
    reset_ring(day);  // new UTC day — start fresh for today
  }

  SolarDayPoint pt = {};
  pt.t_epoch    = t_epoch;
  pt.pv_power_w = valid
      ? (uint16_t)std::min<uint32_t>((uint32_t)pv_power_w, 0xFFFEu)
      : 0xFFFFu;  // sentinel: MPPT not valid this slot

  s_ring[s_head] = pt;
  s_head = (s_head + 1u) % SOLAR_DAY_CAPACITY;
  if (s_count < SOLAR_DAY_CAPACITY) s_count++;
}

uint32_t t0_epoch() {
  if (s_count == 0) return 0;
  // Ring not full: oldest at index 0. Full: oldest at current head.
  uint32_t oldest = (s_count < SOLAR_DAY_CAPACITY) ? 0u : s_head;
  return s_ring[oldest].t_epoch;
}

uint32_t midnight_epoch() {
  return (s_day_utc != 0) ? s_day_utc * 86400u : 0u;
}

uint32_t count() { return s_count; }

size_t read(SolarDayPoint* out, size_t max_count) {
  if (s_count == 0 || max_count == 0) return 0;
  const uint32_t n     = std::min<uint32_t>(s_count, (uint32_t)max_count);
  const uint32_t start = (s_count < SOLAR_DAY_CAPACITY) ? 0u : s_head;
  for (uint32_t i = 0; i < n; i++) {
    out[i] = s_ring[(start + i) % SOLAR_DAY_CAPACITY];
  }
  return (size_t)n;
}

// Persist ring state to LittleFS. Writes header then the full ring array
// directly — no large stack allocation (ring lives in PSRAM BSS).
// Atomic: write to .tmp then rename, matching history_store::flush() pattern.
void save() {
  FILE* f = fopen(PERSIST_PATH_TMP, "wb");
  if (!f) {
    ESP_LOGW(TAG, "save: fopen(%s) failed", PERSIST_PATH_TMP);
    return;
  }
  uint32_t hdr[5] = { PERSIST_MAGIC, PERSIST_VERSION, s_day_utc, s_head, s_count };
  bool ok = (fwrite(hdr, 1, sizeof(hdr), f) == sizeof(hdr));
  ok      = ok && (fwrite(s_ring, 1, sizeof(s_ring), f) == sizeof(s_ring));
  fclose(f);
  if (!ok) {
    ESP_LOGW(TAG, "save: short write — discarding");
    remove(PERSIST_PATH_TMP);
    return;
  }
  remove(PERSIST_PATH);
  if (rename(PERSIST_PATH_TMP, PERSIST_PATH) != 0) {
    ESP_LOGW(TAG, "save: rename failed");
    remove(PERSIST_PATH_TMP);
  }
}

// Restore ring state from LittleFS. Safe to call before the main loop starts.
// Validates magic/version and count bounds; falls through to fresh-start on any
// corruption. Day-rollover is handled naturally by the next append() call.
void load() {
  FILE* f = fopen(PERSIST_PATH, "rb");
  if (!f) return;  // no file on first boot — expected

  uint32_t hdr[5] = {};
  bool ok = (fread(hdr, 1, sizeof(hdr), f) == sizeof(hdr));
  if (!ok || hdr[0] != PERSIST_MAGIC || hdr[1] != PERSIST_VERSION) {
    fclose(f);
    ESP_LOGW(TAG, "load: stale or corrupt header, starting fresh");
    return;
  }

  const uint32_t day_utc = hdr[2];
  const uint32_t head    = hdr[3];
  const uint32_t count   = hdr[4];

  if (count > SOLAR_DAY_CAPACITY || (count > 0 && head >= SOLAR_DAY_CAPACITY)) {
    fclose(f);
    ESP_LOGW(TAG, "load: corrupt counts (%u/%u), starting fresh", (unsigned)count, (unsigned)head);
    return;
  }

  // Read ring data directly into PSRAM BSS — no intermediate stack buffer.
  ok = (fread(s_ring, 1, sizeof(s_ring), f) == sizeof(s_ring));
  fclose(f);
  if (!ok) {
    memset(s_ring, 0, sizeof(s_ring));
    ESP_LOGW(TAG, "load: short ring read, starting fresh");
    return;
  }

  s_day_utc = day_utc;
  s_head    = head;
  s_count   = count;
  ESP_LOGI(TAG, "load: restored %u pts for day %u (head=%u)",
           (unsigned)s_count, (unsigned)s_day_utc, (unsigned)s_head);
}

}  // namespace app::solar_day_ring
