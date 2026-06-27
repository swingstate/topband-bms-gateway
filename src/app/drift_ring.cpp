#include "drift_ring.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm>

static const char* TAG = "drift_ring";

namespace app::drift_ring {

// ── Internal types ─────────────────────────────────────────────────────────────

struct CellBand {
  uint16_t mn;  // mV, 0 = no data
  uint16_t mx;  // mV, 0 = no data
};

// One UTC day of per-pack, per-cell accumulation.
// sizeof(DayBucket) = 4 + 16×16×4 + 16×2 + 16×1 = 4 + 1024 + 32 + 16 = 1076 B
struct DayBucket {
  uint32_t day_utc;                              // epoch / 86400; 0 = empty
  CellBand cell[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS]; // running min/max per pack/cell
  uint16_t spread[DRIFT_MAX_PACKS];              // max pack spread seen this day (mV)
  uint8_t  nc[DRIFT_MAX_PACKS];                 // cell count seen per pack (0 = not seen)
};

// ── PSRAM storage — EXT_RAM_BSS_ATTR, zero internal-DRAM cost ────────────────
// 5 completed-day buckets + 1 today accumulator + all-time bands = ~7.5 KB PSRAM.
// CPU-only access (no DMA, no ISR). Both HistoryTask (writer, Core 1) and
// HttpTask (reader, Core 0) access without a lock; see header for rationale.

static EXT_RAM_BSS_ATTR DayBucket s_days[DRIFT_HISTORY_DAYS]; // completed days ring
static EXT_RAM_BSS_ATTR DayBucket s_today;                    // current day accumulator
static EXT_RAM_BSS_ATTR CellBand  s_alltime[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS]; // ever

// Tiny metadata (DRAM BSS, negligible — 2 bytes).
static uint8_t s_day_head  = 0;  // oldest valid slot index in s_days
static uint8_t s_day_count = 0;  // how many slots in s_days are valid (0..DRIFT_HISTORY_DAYS)

// ── commit_today ──────────────────────────────────────────────────────────────
// Push s_today into the circular s_days ring then zero-initialise s_today.
static void commit_today() {
  if (s_today.day_utc == 0) return;  // nothing accumulated

  if (s_day_count < DRIFT_HISTORY_DAYS) {
    // Ring has room: append at (head + count).
    s_days[(s_day_head + s_day_count) % DRIFT_HISTORY_DAYS] = s_today;
    s_day_count++;
  } else {
    // Ring full: overwrite oldest slot, then advance head.
    s_days[s_day_head] = s_today;
    s_day_head = (s_day_head + 1u) % DRIFT_HISTORY_DAYS;
  }
  memset(&s_today, 0, sizeof(s_today));
}

// ── update ────────────────────────────────────────────────────────────────────
void update(const BmsSystemSnapshot& snap, uint32_t t_epoch) {
  if (t_epoch == 0) return;

  const uint32_t day = t_epoch / 86400u;

  // Day rollover: commit yesterday's bucket and start fresh.
  if (s_today.day_utc != 0 && day != s_today.day_utc) {
    ESP_LOGI(TAG, "day rollover: committing day %u, ring size %u -> %u",
             (unsigned)s_today.day_utc,
             (unsigned)s_day_count,
             (unsigned)(s_day_count < DRIFT_HISTORY_DAYS ? s_day_count + 1 : DRIFT_HISTORY_DAYS));
    commit_today();
  }
  s_today.day_utc = day;

  // Update per-pack, per-cell running min/max for today.
  for (uint8_t pi = 0; pi < snap.pack_count_configured && pi < DRIFT_MAX_PACKS; pi++) {
    const BmsPackSnapshot& p = snap.pack[pi];
    if (!p.online) continue;

    const uint8_t nc = (p.cell_count < DRIFT_MAX_CELLS)
                       ? p.cell_count
                       : (uint8_t)DRIFT_MAX_CELLS;
    if (nc == 0) continue;

    s_today.nc[pi] = nc;

    uint16_t pack_min = 0xFFFFu;
    uint16_t pack_max = 0u;

    for (uint8_t ci = 0; ci < nc; ci++) {
      // Convert V → mV with range guard.
      const uint16_t mv = (uint16_t)(p.cell_v[ci] * 1000.0f + 0.5f);
      if (mv < 2000u || mv > 5000u) continue;

      CellBand& tb = s_today.cell[pi][ci];
      if (tb.mn == 0u || mv < tb.mn) tb.mn = mv;
      if (mv > tb.mx)                tb.mx = mv;

      CellBand& ab = s_alltime[pi][ci];
      if (ab.mn == 0u || mv < ab.mn) ab.mn = mv;
      if (mv > ab.mx)                ab.mx = mv;

      if (mv < pack_min) pack_min = mv;
      if (mv > pack_max) pack_max = mv;
    }

    if (pack_max > pack_min) {
      const uint16_t spread = pack_max - pack_min;
      if (spread > s_today.spread[pi]) s_today.spread[pi] = spread;
    }
  }
}

// ── read_pack ─────────────────────────────────────────────────────────────────
bool read_pack(uint8_t pack_idx, PackDrift& out) {
  if (pack_idx >= DRIFT_MAX_PACKS) { out = {}; return false; }

  // Determine cell count: prefer today's value, then scan historical buckets.
  uint8_t nc = s_today.nc[pack_idx];
  if (nc == 0) {
    for (uint8_t d = 0; d < s_day_count; d++) {
      const uint8_t slot = (s_day_head + d) % DRIFT_HISTORY_DAYS;
      if (s_days[slot].nc[pack_idx] > 0) {
        nc = s_days[slot].nc[pack_idx];
        break;
      }
    }
  }
  if (nc == 0) { out = {}; return false; }

  out = {};
  out.cell_count  = nc;
  out.has_history = (s_today.day_utc != 0 || s_day_count > 0);

  // Per-cell 5-day band: union of s_today and all completed days in the ring.
  for (uint8_t ci = 0; ci < nc; ci++) {
    uint16_t d5min = 0;
    uint16_t d5max = 0;

    // Inline lambda-equivalent helper to absorb one CellBand.
    auto absorb = [&](const CellBand& b) {
      if (b.mn == 0u) return;
      d5min = (d5min == 0u) ? b.mn : std::min(d5min, b.mn);
      d5max = std::max(d5max, b.mx);
    };

    absorb(s_today.cell[pack_idx][ci]);
    for (uint8_t d = 0; d < s_day_count; d++) {
      absorb(s_days[(s_day_head + d) % DRIFT_HISTORY_DAYS].cell[pack_idx][ci]);
    }

    out.cells[ci].d5min  = d5min;
    out.cells[ci].d5max  = d5max;
    out.cells[ci].ev_min = s_alltime[pack_idx][ci].mn;
    out.cells[ci].ev_max = s_alltime[pack_idx][ci].mx;
  }

  // Trend: completed days oldest-first, then today's running accumulator.
  uint8_t n = 0;
  for (uint8_t d = 0; d < s_day_count; d++) {
    const uint8_t slot = (s_day_head + d) % DRIFT_HISTORY_DAYS;
    if (s_days[slot].day_utc != 0u) {
      out.trend_spread[n++] = s_days[slot].spread[pack_idx];
    }
  }
  if (s_today.day_utc != 0u) {
    out.trend_spread[n++] = s_today.spread[pack_idx];
  }
  out.n_days = n;

  return true;
}

// ── complete_days ─────────────────────────────────────────────────────────────
uint8_t complete_days() { return s_day_count; }

}  // namespace app::drift_ring
