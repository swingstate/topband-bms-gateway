#include "drift_ring.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

static const char* TAG = "drift_ring";

// Persistence format — bump PERSIST_VERSION if DayBucket layout changes.
// The header additionally stores sizeof(DayBucket) as a layout guard.
static constexpr uint32_t PERSIST_MAGIC   = 0x44524654u;  // "DRFT"
static constexpr uint32_t PERSIST_VERSION = 1u;
static constexpr const char* PERSIST_PATH     = "/lfs/drift_days.bin";
static constexpr const char* PERSIST_PATH_TMP = "/lfs/drift_days.bin.tmp";

namespace app::drift_ring {

// ── Internal types ─────────────────────────────────────────────────────────────

struct CellBand {
  uint16_t mn;  // mV, 0 = no data
  uint16_t mx;  // mV, 0 = no data
};

// One UTC day of per-pack, per-cell accumulation.
//
// sizeof(DayBucket):
//   4 (day_utc)
//   + 3 × (16 × 16 × 4)  = 3 × 1024 = 3072 B  (cell, cell_high, cell_low)
//   + 2 × (16 × 2)        = 64 B                (spread_high, spread_low)
//   + 16                   = 16 B                (nc)
//   = 3156 B
struct DayBucket {
  uint32_t day_utc;                                        // epoch/86400; 0 = empty

  // Union of high+low regions (mid-SoC excluded).
  CellBand cell[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS];

  // Top-of-charge region (SoC >= SOC_HIGH_THRESHOLD).
  CellBand cell_high[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS];

  // Bottom-of-discharge region (SoC <= SOC_LOW_THRESHOLD).
  CellBand cell_low[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS];

  // Max pack spread within each region this day (mV).
  uint16_t spread_high[DRIFT_MAX_PACKS];
  uint16_t spread_low[DRIFT_MAX_PACKS];

  uint8_t  nc[DRIFT_MAX_PACKS];  // cell count seen per pack (0 = not seen)
};

// ── PSRAM storage — EXT_RAM_BSS_ATTR, zero internal-DRAM cost ────────────────
// 5 completed-day buckets + 1 today accumulator + all-time bands ≈ 22.8 KB PSRAM.

static EXT_RAM_BSS_ATTR DayBucket s_days[DRIFT_HISTORY_DAYS]; // completed days ring
static EXT_RAM_BSS_ATTR DayBucket s_today;                    // current day accumulator
static EXT_RAM_BSS_ATTR CellBand  s_alltime[DRIFT_MAX_PACKS][DRIFT_MAX_CELLS]; // unconditional

// Tiny metadata (DRAM BSS, negligible — 2 bytes).
static uint8_t s_day_head  = 0;  // oldest valid slot index in s_days
static uint8_t s_day_count = 0;  // how many slots in s_days are valid (0..DRIFT_HISTORY_DAYS)

// ── commit_today ──────────────────────────────────────────────────────────────
static void commit_today() {
  if (s_today.day_utc == 0) return;

  if (s_day_count < DRIFT_HISTORY_DAYS) {
    s_days[(s_day_head + s_day_count) % DRIFT_HISTORY_DAYS] = s_today;
    s_day_count++;
  } else {
    s_days[s_day_head] = s_today;
    s_day_head = (s_day_head + 1u) % DRIFT_HISTORY_DAYS;
  }
  memset(&s_today, 0, sizeof(s_today));
}

// ── prune_stale_days ──────────────────────────────────────────────────────────
// Drop completed days that fell out of the DRIFT_HISTORY_DAYS window. Only
// relevant after load(): the device may have been off for longer than the
// window, and stale buckets must not masquerade as recent band data.
// The ring is ordered oldest-first, so pruning always advances the head.
static void prune_stale_days(uint32_t today) {
  while (s_day_count > 0) {
    const DayBucket& oldest = s_days[s_day_head];
    const bool too_old = (today > oldest.day_utc) &&
                         (today - oldest.day_utc > DRIFT_HISTORY_DAYS);
    const bool corrupt = (oldest.day_utc == 0) || (oldest.day_utc >= today);
    if (!too_old && !corrupt) break;
    ESP_LOGI(TAG, "prune: dropping day %u (today=%u)",
             (unsigned)oldest.day_utc, (unsigned)today);
    s_day_head = (s_day_head + 1u) % DRIFT_HISTORY_DAYS;
    s_day_count--;
  }
}

// ── update ────────────────────────────────────────────────────────────────────
void update(const BmsSystemSnapshot& snap, uint32_t t_epoch) {
  if (t_epoch == 0) return;

  const uint32_t day = t_epoch / 86400u;

  if (s_today.day_utc != 0 && day != s_today.day_utc) {
    ESP_LOGI(TAG, "day rollover: committing day %u, ring size %u -> %u",
             (unsigned)s_today.day_utc,
             (unsigned)s_day_count,
             (unsigned)(s_day_count < DRIFT_HISTORY_DAYS ? s_day_count + 1 : DRIFT_HISTORY_DAYS));
    commit_today();
    save();  // once per UTC day; HistoryTask context, file I/O is fine here
  }
  // Cheap when nothing is stale (single compare); matters after load().
  prune_stale_days(day);
  s_today.day_utc = day;

  for (uint8_t pi = 0; pi < snap.pack_count_configured && pi < DRIFT_MAX_PACKS; pi++) {
    const BmsPackSnapshot& p = snap.pack[pi];
    if (!p.online) continue;

    const uint8_t nc = (p.cell_count < DRIFT_MAX_CELLS)
                       ? p.cell_count
                       : (uint8_t)DRIFT_MAX_CELLS;
    if (nc == 0) continue;

    s_today.nc[pi] = nc;

    // Determine SoC region for this pack.
    const bool is_high    = (p.soc >= SOC_HIGH_THRESHOLD);
    const bool is_low     = (p.soc <= SOC_LOW_THRESHOLD);
    const bool in_region  = is_high || is_low;

    uint16_t high_min = 0xFFFFu, high_max = 0u;
    uint16_t low_min  = 0xFFFFu, low_max  = 0u;

    for (uint8_t ci = 0; ci < nc; ci++) {
      const uint16_t mv = (uint16_t)(p.cell_v[ci] * 1000.0f + 0.5f);
      if (mv < 2000u || mv > 5000u) continue;

      // Union band: only at extremes (not flat mid-SoC).
      if (in_region) {
        CellBand& tb = s_today.cell[pi][ci];
        if (tb.mn == 0u || mv < tb.mn) tb.mn = mv;
        if (mv > tb.mx)                tb.mx = mv;
      }

      // Top-of-charge band.
      if (is_high) {
        CellBand& th = s_today.cell_high[pi][ci];
        if (th.mn == 0u || mv < th.mn) th.mn = mv;
        if (mv > th.mx)                th.mx = mv;
        if (mv < high_min) high_min = mv;
        if (mv > high_max) high_max = mv;
      }

      // Bottom-of-discharge band.
      if (is_low) {
        CellBand& tl = s_today.cell_low[pi][ci];
        if (tl.mn == 0u || mv < tl.mn) tl.mn = mv;
        if (mv > tl.mx)                tl.mx = mv;
        if (mv < low_min) low_min = mv;
        if (mv > low_max) low_max = mv;
      }

      // All-time bands: unconditional (full voltage history, any SoC).
      CellBand& ab = s_alltime[pi][ci];
      if (ab.mn == 0u || mv < ab.mn) ab.mn = mv;
      if (mv > ab.mx)                ab.mx = mv;
    }

    // Track per-region pack spread.
    if (is_high && high_max > high_min) {
      const uint16_t sp = high_max - high_min;
      if (sp > s_today.spread_high[pi]) s_today.spread_high[pi] = sp;
    }
    if (is_low && low_max > low_min) {
      const uint16_t sp = low_max - low_min;
      if (sp > s_today.spread_low[pi]) s_today.spread_low[pi] = sp;
    }
  }
}

// ── read_pack ─────────────────────────────────────────────────────────────────
bool read_pack(uint8_t pack_idx, PackDrift& out) {
  if (pack_idx >= DRIFT_MAX_PACKS) { out = {}; return false; }

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

  // Helper: absorb one CellBand into a running min/max pair.
  auto absorb = [](const CellBand& b, uint16_t& mn, uint16_t& mx) {
    if (b.mn == 0u) return;
    mn = (mn == 0u) ? b.mn : std::min(mn, b.mn);
    mx = std::max(mx, b.mx);
  };

  // Per-cell 5-day bands: union of both extreme regions + per-region.
  for (uint8_t ci = 0; ci < nc; ci++) {
    uint16_t d5mn = 0, d5mx = 0;
    uint16_t tocmn = 0, tocmx = 0;
    uint16_t bodmn = 0, bodmx = 0;

    absorb(s_today.cell[pack_idx][ci],      d5mn,  d5mx);
    absorb(s_today.cell_high[pack_idx][ci], tocmn, tocmx);
    absorb(s_today.cell_low[pack_idx][ci],  bodmn, bodmx);

    for (uint8_t d = 0; d < s_day_count; d++) {
      const auto& bkt = s_days[(s_day_head + d) % DRIFT_HISTORY_DAYS];
      absorb(bkt.cell[pack_idx][ci],      d5mn,  d5mx);
      absorb(bkt.cell_high[pack_idx][ci], tocmn, tocmx);
      absorb(bkt.cell_low[pack_idx][ci],  bodmn, bodmx);
    }

    out.cells[ci].d5min   = d5mn;
    out.cells[ci].d5max   = d5mx;
    out.cells[ci].toc_min = tocmn;
    out.cells[ci].toc_max = tocmx;
    out.cells[ci].bod_min = bodmn;
    out.cells[ci].bod_max = bodmx;
    out.cells[ci].ev_min  = s_alltime[pack_idx][ci].mn;
    out.cells[ci].ev_max  = s_alltime[pack_idx][ci].mx;
  }

  // Region-gated pack spreads (max over entire 5-day window).
  auto absorb_sp = [](uint16_t v, uint16_t& mx, bool& has) {
    if (v == 0) return;
    has = true;
    if (v > mx) mx = v;
  };

  absorb_sp(s_today.spread_high[pack_idx], out.toc_spread, out.has_toc);
  absorb_sp(s_today.spread_low[pack_idx],  out.bod_spread, out.has_bod);
  for (uint8_t d = 0; d < s_day_count; d++) {
    const auto& bkt = s_days[(s_day_head + d) % DRIFT_HISTORY_DAYS];
    absorb_sp(bkt.spread_high[pack_idx], out.toc_spread, out.has_toc);
    absorb_sp(bkt.spread_low[pack_idx],  out.bod_spread, out.has_bod);
  }

  // First full: cell with highest toc_max (hits ceiling first, limits charging).
  // First empty: cell with lowest bod_min (empties first, limits discharge).
  uint16_t best_toc = 0u, worst_bod = 0xFFFFu;
  for (uint8_t ci = 0; ci < nc; ci++) {
    const auto& c = out.cells[ci];
    if (c.toc_max > 0 && c.toc_max > best_toc) {
      best_toc = c.toc_max;
      out.first_full_idx = ci;
    }
    if (c.bod_min > 0 && c.bod_min < worst_bod) {
      worst_bod = c.bod_min;
      out.first_empty_idx = ci;
    }
  }
  out.first_full_mv  = (best_toc > 0u) ? best_toc : 0u;
  out.first_empty_mv = (out.has_bod && worst_bod < 0xFFFFu) ? worst_bod : 0u;

  // Trend: ToC spread per day, oldest-first, then today.
  // Drift rate is computed from the slope of this series.
  uint8_t n = 0;
  for (uint8_t d = 0; d < s_day_count; d++) {
    const uint8_t slot = (s_day_head + d) % DRIFT_HISTORY_DAYS;
    if (s_days[slot].day_utc != 0u) {
      out.trend_spread[n++] = s_days[slot].spread_high[pack_idx];
    }
  }
  if (s_today.day_utc != 0u) {
    out.trend_spread[n++] = s_today.spread_high[pack_idx];
  }
  out.n_days = n;

  return true;
}

// ── complete_days ─────────────────────────────────────────────────────────────
uint8_t complete_days() { return s_day_count; }

// ── save ──────────────────────────────────────────────────────────────────────
// Persist completed days + all-time bands (atomic tmp+rename, same pattern as
// solar_day_ring::save). Today's accumulator is intentionally not persisted.
void save() {
  FILE* f = fopen(PERSIST_PATH_TMP, "wb");
  if (!f) {
    ESP_LOGW(TAG, "save: fopen(%s) failed", PERSIST_PATH_TMP);
    return;
  }
  uint32_t hdr[5] = { PERSIST_MAGIC, PERSIST_VERSION,
                      (uint32_t)sizeof(DayBucket), s_day_head, s_day_count };
  bool ok = (fwrite(hdr, 1, sizeof(hdr), f) == sizeof(hdr));
  ok      = ok && (fwrite(s_days, 1, sizeof(s_days), f) == sizeof(s_days));
  ok      = ok && (fwrite(s_alltime, 1, sizeof(s_alltime), f) == sizeof(s_alltime));
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
    return;
  }
  ESP_LOGI(TAG, "save: %u completed days persisted", (unsigned)s_day_count);
}

// ── load ──────────────────────────────────────────────────────────────────────
// Restore completed days + all-time bands. Validates magic/version/layout and
// count bounds; falls through to a fresh start on any mismatch. Stale days are
// pruned by the first update() call once NTP time is available (load() runs at
// boot, before time sync, so "today" is unknown here).
void load() {
  FILE* f = fopen(PERSIST_PATH, "rb");
  if (!f) return;  // no file on first boot — expected

  uint32_t hdr[5] = {};
  bool ok = (fread(hdr, 1, sizeof(hdr), f) == sizeof(hdr));
  if (!ok || hdr[0] != PERSIST_MAGIC || hdr[1] != PERSIST_VERSION ||
      hdr[2] != (uint32_t)sizeof(DayBucket)) {
    fclose(f);
    ESP_LOGW(TAG, "load: stale or corrupt header, starting fresh");
    return;
  }

  const uint32_t head  = hdr[3];
  const uint32_t count = hdr[4];
  if (count > DRIFT_HISTORY_DAYS || head >= DRIFT_HISTORY_DAYS) {
    fclose(f);
    ESP_LOGW(TAG, "load: corrupt counts (%u/%u), starting fresh",
             (unsigned)count, (unsigned)head);
    return;
  }

  // Read directly into PSRAM BSS — no intermediate buffer.
  ok = (fread(s_days, 1, sizeof(s_days), f) == sizeof(s_days));
  ok = ok && (fread(s_alltime, 1, sizeof(s_alltime), f) == sizeof(s_alltime));
  fclose(f);
  if (!ok) {
    memset(s_days, 0, sizeof(s_days));
    memset(s_alltime, 0, sizeof(s_alltime));
    ESP_LOGW(TAG, "load: short read, starting fresh");
    return;
  }

  s_day_head  = (uint8_t)head;
  s_day_count = (uint8_t)count;
  ESP_LOGI(TAG, "load: restored %u completed days (head=%u)",
           (unsigned)s_day_count, (unsigned)s_day_head);
}

}  // namespace app::drift_ring
