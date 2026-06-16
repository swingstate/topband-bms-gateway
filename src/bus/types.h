#pragma once
#include <cstdint>

// ── On-disk history point types (architecture §5.7) ────────────────────────────
// These are the compact binary layouts written to LittleFS ring files.
// Scaling is explicit so floats are not needed in the file format.

struct HistoryFinePoint {    // 14 bytes on disk
  uint16_t t_offset_s;      // seconds from epoch_base stored in ring header
  int16_t  power_w;         // W (signed; + = charging)
  int16_t  voltage_x100;    // V × 100 (e.g. 5204 = 52.04 V)
  int16_t  soc_x10;         // % × 10 (e.g. 785 = 78.5 %)
  int16_t  temp_x10;        // °C × 10 (e.g. 224 = 22.4 °C)
  uint16_t flags;            // 0x0001 = NTP-valid timestamp; 0 = placeholder
  uint16_t drift_mv;         // cell voltage spread (max − min) in mV; persisted since format v2
};
static_assert(sizeof(HistoryFinePoint) == 14, "HistoryFinePoint must be 14 bytes");

struct HistoryCoarsePoint {  // 28 bytes on disk: 4 + 10×2 + 2 + 2(pad) = 28
  uint32_t t_epoch;          // absolute unix timestamp; 0 = placeholder/unsynced
  int16_t  power_avg;        // W avg over 5-min bucket
  int16_t  power_min;
  int16_t  power_max;
  int16_t  volt_avg;         // V × 100
  int16_t  volt_min;
  int16_t  volt_max;
  int16_t  soc_avg;          // % × 10 (avg only; soc is stable within 5 min)
  int16_t  temp_avg;         // °C × 10
  int16_t  temp_min;
  int16_t  temp_max;
  uint16_t drift_mv;         // avg cell voltage spread over 5-min bucket, in mV
  uint16_t _pad;             // explicit tail pad to 28 B (4-byte struct alignment)
};
static_assert(sizeof(HistoryCoarsePoint) == 28, "HistoryCoarsePoint must be 28 bytes");

// ── Ring file header (shared by fine and coarse files) ────────────────────────
struct HistoryRingHeader {   // 28 bytes
  uint32_t magic;            // fine: HISTORY_FINE_MAGIC, coarse: HISTORY_COARSE_MAGIC
  uint32_t format_version;   // must equal HISTORY_FORMAT_VERSION; old-format files are discarded
  uint32_t epoch_base;       // unix time of the oldest valid entry's 0-offset anchor
  uint32_t head;             // index of next-write slot (0..capacity-1)
  uint32_t count;            // number of valid entries (0..capacity)
  uint32_t resolution_s;     // seconds between samples
  uint32_t capacity;         // max entries in this ring
};
static_assert(sizeof(HistoryRingHeader) == 28, "HistoryRingHeader must be 28 bytes");

// Format v1 (implicit, pre-drift): magic 'HFRF'/'HCRF', no format_version field.
// Format v2 (current): magic 'HFRG'/'HCRG', format_version == 2, drift_mv in structs.
// Old-format files fail the magic check and are discarded once on first boot after upgrade.
constexpr uint32_t HISTORY_FORMAT_VERSION = 2u;
constexpr uint32_t HISTORY_FINE_MAGIC     = 0x48465247u;  // 'HFRG'
constexpr uint32_t HISTORY_COARSE_MAGIC   = 0x48435247u;  // 'HCRG'

// Fine ring: 10-second samples, 2-hour window.
constexpr uint32_t HISTORY_FINE_RESOLUTION_S = 10;
constexpr uint32_t HISTORY_FINE_CAPACITY      = 720;   // 720 × 10s = 2 h

// Coarse ring: 5-minute samples, 7-day window.
constexpr uint32_t HISTORY_COARSE_RESOLUTION_S = 300;
constexpr uint32_t HISTORY_COARSE_CAPACITY      = 2016; // 2016 × 5min = 7 d
