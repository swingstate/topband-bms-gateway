#include "history_store.h"
#include "lfs_store.h"
#include "bus/types.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "hist_store";

// ── In-RAM ring state ─────────────────────────────────────────────────────────
// Both rings are held in BSS (static). Declared here as module statics to avoid
// large stack frames (H1 lesson: never declare >1 KB structs as task locals).

static HistoryRingHeader s_fine_hdr   = {};
static HistoryRingHeader s_coarse_hdr = {};

static HistoryFinePoint   s_fine_buf[HISTORY_FINE_CAPACITY]   = {};
static HistoryCoarsePoint s_coarse_buf[HISTORY_COARSE_CAPACITY] = {};

static bool s_fine_dirty   = false;
static bool s_coarse_dirty = false;
static bool s_initialized  = false;

// ── File paths ────────────────────────────────────────────────────────────────
static const char* FINE_PATH   = "/lfs/history/fine.bin";
static const char* COARSE_PATH = "/lfs/history/coarse.bin";
static const char* FINE_TMP    = "/lfs/history/fine.tmp";
static const char* COARSE_TMP  = "/lfs/history/coarse.tmp";

namespace storage::history_store {

// ── Internal helpers ──────────────────────────────────────────────────────────

static bool load_ring_fine() {
  FILE* f = fopen(FINE_PATH, "rb");
  if (!f) return false;

  HistoryRingHeader hdr = {};
  if (fread(&hdr, sizeof(hdr), 1, f) != 1 || hdr.magic != HISTORY_FINE_MAGIC) {
    fclose(f);
    return false;
  }
  // Sanity check capacity stored in header.
  if (hdr.capacity != HISTORY_FINE_CAPACITY) {
    ESP_LOGW(TAG, "fine ring capacity mismatch (file=%u expected=%u) — resetting",
             (unsigned)hdr.capacity, (unsigned)HISTORY_FINE_CAPACITY);
    fclose(f);
    return false;
  }
  size_t data_bytes = HISTORY_FINE_CAPACITY * sizeof(HistoryFinePoint);
  if (fread(s_fine_buf, 1, data_bytes, f) != data_bytes) {
    fclose(f);
    return false;
  }
  fclose(f);
  s_fine_hdr = hdr;
  return true;
}

static bool load_ring_coarse() {
  FILE* f = fopen(COARSE_PATH, "rb");
  if (!f) return false;

  HistoryRingHeader hdr = {};
  if (fread(&hdr, sizeof(hdr), 1, f) != 1 || hdr.magic != HISTORY_COARSE_MAGIC) {
    fclose(f);
    return false;
  }
  if (hdr.capacity != HISTORY_COARSE_CAPACITY) {
    ESP_LOGW(TAG, "coarse ring capacity mismatch — resetting");
    fclose(f);
    return false;
  }
  size_t data_bytes = HISTORY_COARSE_CAPACITY * sizeof(HistoryCoarsePoint);
  if (fread(s_coarse_buf, 1, data_bytes, f) != data_bytes) {
    fclose(f);
    return false;
  }
  fclose(f);
  s_coarse_hdr = hdr;
  return true;
}

static bool write_ring_file(const char* path, const char* tmp_path,
                             const HistoryRingHeader& hdr,
                             const void* data, size_t data_bytes) {
  FILE* f = fopen(tmp_path, "wb");
  if (!f) {
    ESP_LOGE(TAG, "open tmp failed: %s", tmp_path);
    return false;
  }
  bool ok = (fwrite(&hdr, sizeof(hdr), 1, f) == 1) &&
            (fwrite(data, 1, data_bytes, f) == data_bytes);
  fflush(f);
  fclose(f);
  if (!ok) {
    ESP_LOGE(TAG, "write failed for %s", tmp_path);
    return false;
  }
  // Atomic rename: remove destination first (LittleFS doesn't always overwrite).
  remove(path);
  if (rename(tmp_path, path) != 0) {
    ESP_LOGE(TAG, "rename %s → %s failed", tmp_path, path);
    return false;
  }
  return true;
}

static void init_fine_header() {
  s_fine_hdr.magic       = HISTORY_FINE_MAGIC;
  s_fine_hdr.epoch_base  = 0;   // updated on first NTP-synced sample
  s_fine_hdr.head        = 0;
  s_fine_hdr.count       = 0;
  s_fine_hdr.resolution_s = HISTORY_FINE_RESOLUTION_S;
  s_fine_hdr.capacity    = HISTORY_FINE_CAPACITY;
  memset(s_fine_buf, 0, sizeof(s_fine_buf));
}

static void init_coarse_header() {
  s_coarse_hdr.magic       = HISTORY_COARSE_MAGIC;
  s_coarse_hdr.epoch_base  = 0;
  s_coarse_hdr.head        = 0;
  s_coarse_hdr.count       = 0;
  s_coarse_hdr.resolution_s = HISTORY_COARSE_RESOLUTION_S;
  s_coarse_hdr.capacity    = HISTORY_COARSE_CAPACITY;
  memset(s_coarse_buf, 0, sizeof(s_coarse_buf));
}

// ── Public API ────────────────────────────────────────────────────────────────

bool init() {
  // Ensure directory exists.
  if (!storage::lfs::mkdir_p("/lfs/history")) {
    ESP_LOGE(TAG, "cannot create /lfs/history — LittleFS not ready?");
    return false;
  }

  // Try to load persisted fine ring.
  if (!load_ring_fine()) {
    ESP_LOGI(TAG, "fine ring not found or invalid — starting fresh");
    init_fine_header();
    s_fine_dirty = true;
  } else {
    ESP_LOGI(TAG, "fine ring loaded: count=%u head=%u epoch_base=%u",
             (unsigned)s_fine_hdr.count, (unsigned)s_fine_hdr.head,
             (unsigned)s_fine_hdr.epoch_base);
  }

  // Try to load persisted coarse ring.
  if (!load_ring_coarse()) {
    ESP_LOGI(TAG, "coarse ring not found or invalid — starting fresh");
    init_coarse_header();
    s_coarse_dirty = true;
  } else {
    ESP_LOGI(TAG, "coarse ring loaded: count=%u head=%u epoch_base=%u",
             (unsigned)s_coarse_hdr.count, (unsigned)s_coarse_hdr.head,
             (unsigned)s_coarse_hdr.epoch_base);
  }

  s_initialized = true;
  return true;
}

bool append_fine(const HistoryFinePoint& pt) {
  if (!s_initialized) return false;

  // epoch_base is maintained by history_task. If it's still 0 and this
  // is a valid-time sample, history_task will have set it before calling here.

  // Write into ring at head slot.
  s_fine_buf[s_fine_hdr.head] = pt;

  s_fine_hdr.head = (s_fine_hdr.head + 1) % HISTORY_FINE_CAPACITY;
  if (s_fine_hdr.count < HISTORY_FINE_CAPACITY) {
    s_fine_hdr.count++;
  } else {
    // Ring wrapped: advance epoch_base by one resolution to track oldest entry.
    s_fine_hdr.epoch_base += s_fine_hdr.resolution_s;
  }

  s_fine_dirty = true;
  return true;
}

bool append_coarse(const HistoryCoarsePoint& pt) {
  if (!s_initialized) return false;

  if (s_coarse_hdr.epoch_base == 0 && pt.t_epoch != 0) {
    // First NTP-valid coarse entry — anchor epoch_base to this point's time.
    // epoch_base = t_epoch of oldest entry; on wrap we advance by resolution.
    s_coarse_hdr.epoch_base = pt.t_epoch;
  }

  s_coarse_buf[s_coarse_hdr.head] = pt;

  s_coarse_hdr.head = (s_coarse_hdr.head + 1) % HISTORY_COARSE_CAPACITY;
  if (s_coarse_hdr.count < HISTORY_COARSE_CAPACITY) {
    s_coarse_hdr.count++;
  } else {
    s_coarse_hdr.epoch_base += s_coarse_hdr.resolution_s;
  }

  s_coarse_dirty = true;
  return true;
}

void flush() {
  if (!s_initialized) return;

  if (s_fine_dirty) {
    bool ok = write_ring_file(FINE_PATH, FINE_TMP, s_fine_hdr,
                               s_fine_buf, sizeof(s_fine_buf));
    if (ok) {
      s_fine_dirty = false;
      ESP_LOGD(TAG, "fine ring persisted: count=%u", (unsigned)s_fine_hdr.count);
    }
  }
  if (s_coarse_dirty) {
    bool ok = write_ring_file(COARSE_PATH, COARSE_TMP, s_coarse_hdr,
                               s_coarse_buf, sizeof(s_coarse_buf));
    if (ok) {
      s_coarse_dirty = false;
      ESP_LOGD(TAG, "coarse ring persisted: count=%u", (unsigned)s_coarse_hdr.count);
    }
  }
}

size_t read_fine(HistoryFinePoint* out, size_t max_count) {
  if (!s_initialized || s_fine_hdr.count == 0) return 0;
  size_t n = (s_fine_hdr.count < (uint32_t)max_count) ? s_fine_hdr.count : (uint32_t)max_count;

  // Oldest entry is at: (head - count + capacity) % capacity
  uint32_t oldest = (s_fine_hdr.head + HISTORY_FINE_CAPACITY - s_fine_hdr.count)
                    % HISTORY_FINE_CAPACITY;
  for (size_t i = 0; i < n; i++) {
    out[i] = s_fine_buf[(oldest + i) % HISTORY_FINE_CAPACITY];
  }
  return n;
}

size_t read_coarse(HistoryCoarsePoint* out, size_t max_count) {
  if (!s_initialized || s_coarse_hdr.count == 0) return 0;
  size_t n = (s_coarse_hdr.count < (uint32_t)max_count)
             ? s_coarse_hdr.count : (uint32_t)max_count;

  uint32_t oldest = (s_coarse_hdr.head + HISTORY_COARSE_CAPACITY - s_coarse_hdr.count)
                    % HISTORY_COARSE_CAPACITY;
  for (size_t i = 0; i < n; i++) {
    out[i] = s_coarse_buf[(oldest + i) % HISTORY_COARSE_CAPACITY];
  }
  return n;
}

void set_fine_epoch_base(uint32_t epoch_s) {
  if (s_fine_hdr.epoch_base == 0) {
    s_fine_hdr.epoch_base = epoch_s;
  }
}

uint32_t fine_epoch_base()   { return s_fine_hdr.epoch_base; }
uint32_t coarse_epoch_base() { return s_coarse_hdr.epoch_base; }
uint32_t fine_count()        { return s_fine_hdr.count; }
uint32_t coarse_count()      { return s_coarse_hdr.count; }

}  // namespace storage::history_store
