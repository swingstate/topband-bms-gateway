#include "energy_store.h"
#include "lfs_store.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <ctime>

static const char* TAG = "energy_store";

// ── On-disk format (architecture §6.7) ────────────────────────────────────────
// 264-byte file, atomic write via tmp+rename.
static const uint32_t ENERGY_SCHEMA_V1 = 1;

// The struct layout matches architecture §6.7 with explicit byte packing.
// schema_version(4) + day_epoch(4) + today floats(8) + days[7]×2×4(56) +
// months[12]×2×4(96) + total[2]×4(8) + crc32(4) = 180 bytes.
// We add padding to reach an architecturally-clean 264 bytes.
struct EnergyFile {
  uint32_t schema_version;      // must be ENERGY_SCHEMA_V1
  uint32_t day_epoch;           // midnight unix timestamp of "today" (local)
  float    in_today_kwh;
  float    out_today_kwh;
  float    in_days_kwh[7];      // index 0 = yesterday, rolling 7-day history
  float    out_days_kwh[7];
  float    in_months_kwh[12];   // index 0 = this month-to-date (excl. today)
  float    out_months_kwh[12];
  float    total_in_kwh;        // lifetime accumulation
  float    total_out_kwh;
  uint8_t  _pad[84];            // reserved, zeroed — brings total to 264 bytes
  uint32_t crc32;               // CRC-32 over all preceding bytes
};
static_assert(sizeof(EnergyFile) == 264, "EnergyFile must be 264 bytes");

static const char* ENERGY_PATH = "/lfs/energy/counters.bin";
static const char* ENERGY_TMP  = "/lfs/energy/counters.tmp";

// ── In-RAM state ──────────────────────────────────────────────────────────────
static EnergyFile s_ef = {};
static bool s_dirty = false;
static bool s_initialized = false;
static uint32_t s_last_day_epoch = 0;  // local-midnight epoch of "today"

// ── CRC-32/IEEE ───────────────────────────────────────────────────────────────
static uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  while (len--) {
    crc ^= *data++;
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  }
  return ~crc;
}

// ── Local midnight epoch helper ────────────────────────────────────────────────
// Returns the unix timestamp of 00:00:00 local time for the given unix ts.
// tz_offset_h: hours from UTC.
static uint32_t local_midnight(uint32_t unix_ts, int8_t tz_offset_h) {
  int32_t local_s = (int32_t)unix_ts + (int32_t)tz_offset_h * 3600;
  int32_t day_local = local_s / 86400;
  return (uint32_t)(day_local * 86400 - (int32_t)tz_offset_h * 3600);
}

namespace storage::energy_store {

bool init() {
  storage::lfs::mkdir_p("/lfs/energy");

  // Try loading existing file.
  FILE* f = fopen(ENERGY_PATH, "rb");
  bool loaded = false;
  if (f) {
    EnergyFile tmp = {};
    if (fread(&tmp, sizeof(tmp), 1, f) == 1) {
      uint32_t expected = crc32_ieee(reinterpret_cast<uint8_t*>(&tmp),
                                      sizeof(tmp) - sizeof(uint32_t));
      if (tmp.schema_version == ENERGY_SCHEMA_V1 && tmp.crc32 == expected) {
        s_ef = tmp;
        loaded = true;
        ESP_LOGI(TAG, "loaded: today_in=%.2f today_out=%.2f total_in=%.1f total_out=%.1f",
                 s_ef.in_today_kwh, s_ef.out_today_kwh,
                 s_ef.total_in_kwh, s_ef.total_out_kwh);
      } else {
        ESP_LOGW(TAG, "CRC mismatch or bad schema — starting fresh");
      }
    }
    fclose(f);
  }

  if (!loaded) {
    memset(&s_ef, 0, sizeof(s_ef));
    s_ef.schema_version = ENERGY_SCHEMA_V1;
    s_dirty = true;
  }
  s_last_day_epoch = s_ef.day_epoch;
  s_initialized = true;
  return true;
}

void accumulate(float delta_kwh) {
  if (!s_initialized) return;
  if (delta_kwh > 0.0f) {
    s_ef.in_today_kwh  += delta_kwh;
    s_ef.total_in_kwh  += delta_kwh;
  } else if (delta_kwh < 0.0f) {
    s_ef.out_today_kwh -= delta_kwh;   // negate: out_today is always positive
    s_ef.total_out_kwh -= delta_kwh;
  }
  s_dirty = true;
}

void check_daily_rollover(uint32_t now_unix_s, int8_t tz_offset_h) {
  if (!s_initialized || now_unix_s == 0) return;

  uint32_t today_midnight = local_midnight(now_unix_s, tz_offset_h);

  if (s_last_day_epoch == 0) {
    // First call with valid time — just anchor today.
    s_last_day_epoch  = today_midnight;
    s_ef.day_epoch    = today_midnight;
    s_dirty = true;
    return;
  }

  if (today_midnight <= s_last_day_epoch) return;  // Same day.

  ESP_LOGI(TAG, "daily rollover — yesterday=%.2f in / %.2f out kWh",
           s_ef.in_today_kwh, s_ef.out_today_kwh);

  // Rotate daily arrays: shift right, insert yesterday at index 0.
  for (int i = 6; i > 0; i--) {
    s_ef.in_days_kwh[i]  = s_ef.in_days_kwh[i - 1];
    s_ef.out_days_kwh[i] = s_ef.out_days_kwh[i - 1];
  }
  s_ef.in_days_kwh[0]  = s_ef.in_today_kwh;
  s_ef.out_days_kwh[0] = s_ef.out_today_kwh;

  // Check for month rollover (local month changed).
  // Simple check: if today_midnight day-of-month == 1.
  {
    time_t ts = (time_t)today_midnight;
    struct tm t = {};
    gmtime_r(&ts, &t);
    if (t.tm_mday == 1) {
      ESP_LOGI(TAG, "monthly rollover");
      for (int i = 11; i > 0; i--) {
        s_ef.in_months_kwh[i]  = s_ef.in_months_kwh[i - 1];
        s_ef.out_months_kwh[i] = s_ef.out_months_kwh[i - 1];
      }
      // Month-to-date = sum of days this month. After rollover, index 0 is last month.
      float m_in = 0, m_out = 0;
      for (int i = 0; i < 7; i++) { m_in += s_ef.in_days_kwh[i]; m_out += s_ef.out_days_kwh[i]; }
      s_ef.in_months_kwh[0]  = m_in;
      s_ef.out_months_kwh[0] = m_out;
    }
  }

  // Reset today counters.
  s_ef.in_today_kwh  = 0.0f;
  s_ef.out_today_kwh = 0.0f;
  s_ef.day_epoch     = today_midnight;
  s_last_day_epoch   = today_midnight;
  s_dirty = true;
}

float today_in_kwh()  { return s_ef.in_today_kwh; }
float today_out_kwh() { return s_ef.out_today_kwh; }
float total_in_kwh()  { return s_ef.total_in_kwh; }
float total_out_kwh() { return s_ef.total_out_kwh; }

float week_in_kwh() {
  float sum = s_ef.in_today_kwh;
  for (int i = 0; i < 7; i++) sum += s_ef.in_days_kwh[i];
  return sum;
}
float week_out_kwh() {
  float sum = s_ef.out_today_kwh;
  for (int i = 0; i < 7; i++) sum += s_ef.out_days_kwh[i];
  return sum;
}
float month_in_kwh() {
  return s_ef.in_months_kwh[0] + s_ef.in_today_kwh;
}
float month_out_kwh() {
  return s_ef.out_months_kwh[0] + s_ef.out_today_kwh;
}

bool persist() {
  if (!s_initialized || !s_dirty) return true;

  EnergyFile tmp = s_ef;
  tmp.crc32 = crc32_ieee(reinterpret_cast<uint8_t*>(&tmp),
                          sizeof(tmp) - sizeof(uint32_t));

  FILE* f = fopen(ENERGY_TMP, "wb");
  if (!f) {
    ESP_LOGE(TAG, "open tmp failed");
    return false;
  }
  bool ok = (fwrite(&tmp, sizeof(tmp), 1, f) == 1);
  fflush(f);
  fclose(f);
  if (!ok) { ESP_LOGE(TAG, "write failed"); return false; }

  remove(ENERGY_PATH);
  if (rename(ENERGY_TMP, ENERGY_PATH) != 0) {
    ESP_LOGE(TAG, "rename failed");
    return false;
  }

  s_dirty = false;
  ESP_LOGD(TAG, "persisted: today_in=%.2f out=%.2f total_in=%.1f",
           s_ef.in_today_kwh, s_ef.out_today_kwh, s_ef.total_in_kwh);
  return true;
}

}  // namespace storage::energy_store
