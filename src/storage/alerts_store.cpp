#include "storage/alerts_store.h"
#include "storage/lfs_store.h"
#include "bus/queues.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

#ifndef NATIVE_BUILD
#include "esp_log.h"
#include "esp_heap_caps.h"
static const char* TAG = "alerts_store";
#else
#define ESP_LOGI(tag, ...) (void)0
#define ESP_LOGW(tag, ...) (void)0
#define ESP_LOGE(tag, ...) (void)0
#define ESP_LOGD(tag, ...) (void)0
#endif

// ── File layout ───────────────────────────────────────────────────────────────
//
// Byte 0..3   magic (0xAB1E2345)
// Byte 4      version (1)
// Byte 5..7   pad
// Byte 8..11  head_index (uint32_t) — next write slot
// Byte 12..15 count (uint32_t) — entries stored (0..RING_CAPACITY)
// Byte 16..N  200 × AlertEntry (132 bytes each)
// Final 4 B   CRC32 of all preceding bytes (omitted for now; validated by magic)

static constexpr uint32_t MAGIC   = 0xAB1E2345u;
static constexpr uint8_t  VERSION = 1;
static constexpr size_t   HEADER_SIZE = 16;
static constexpr size_t   BODY_SIZE   =
    storage::alerts_store::RING_CAPACITY * sizeof(AlertEntry);
static constexpr size_t   FILE_SIZE   = HEADER_SIZE + BODY_SIZE;

#ifndef NATIVE_BUILD
static constexpr const char* RING_PATH = "/lfs/alerts/ring.bin";
static constexpr const char* RING_DIR  = "/lfs/alerts";
#else
// Host tests run against the real filesystem (no ESP VFS mount), so use a
// relative path confined to the test binary's CWD instead of the real "/lfs"
// root.
static constexpr const char* RING_PATH = "./alerts_store_test_ring.bin";
static constexpr const char* RING_DIR  = ".";
#endif

// ── In-RAM state ──────────────────────────────────────────────────────────────
// s_ring and s_io_buf are PSRAM-allocated in init() to avoid filling DRAM BSS
// with ~53 KB of static data.

static AlertEntry* s_ring   = nullptr;  // PSRAM-allocated ring
static uint8_t*    s_io_buf = nullptr;  // PSRAM-allocated I/O scratch (FILE_SIZE)
static uint32_t    s_head   = 0;        // next write index
static uint32_t    s_count  = 0;        // entries stored (0..RING_CAPACITY)
static bool        s_dirty  = false;
static bool        s_inited = false;

static bool alloc_psram_buffers() {
#ifndef NATIVE_BUILD
  s_ring = static_cast<AlertEntry*>(
      heap_caps_malloc(BODY_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!s_ring) s_ring = static_cast<AlertEntry*>(malloc(BODY_SIZE));

  s_io_buf = static_cast<uint8_t*>(
      heap_caps_malloc(FILE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!s_io_buf) s_io_buf = static_cast<uint8_t*>(malloc(FILE_SIZE));
#else
  s_ring   = static_cast<AlertEntry*>(malloc(BODY_SIZE));
  s_io_buf = static_cast<uint8_t*>(malloc(FILE_SIZE));
#endif
  return s_ring != nullptr && s_io_buf != nullptr;
}

// ── Serialise / deserialise ───────────────────────────────────────────────────

static bool write_to_disk() {
  if (!s_ring || !s_io_buf) return false;

  uint32_t magic = MAGIC;
  uint32_t hd    = s_head;
  uint32_t cnt   = s_count;

  memcpy(s_io_buf + 0, &magic, 4);
  s_io_buf[4] = VERSION;
  s_io_buf[5] = 0; s_io_buf[6] = 0; s_io_buf[7] = 0;
  memcpy(s_io_buf + 8,  &hd,  4);
  memcpy(s_io_buf + 12, &cnt, 4);
  memcpy(s_io_buf + HEADER_SIZE, s_ring, BODY_SIZE);

  return storage::lfs::write_file_atomic(RING_PATH, s_io_buf, FILE_SIZE);
}

static bool read_from_disk() {
  if (!s_ring || !s_io_buf) return false;

  // The ring file is a fixed-size binary blob, not text — read_file() always
  // reserves one byte for a null terminator, so requesting FILE_SIZE bytes
  // through it can only ever return FILE_SIZE - 1 and this check would always
  // fail. Use the exact-size binary reader instead.
  if (!storage::lfs::read_file_exact(RING_PATH, s_io_buf, FILE_SIZE)) return false;

  uint32_t magic = 0;
  memcpy(&magic, s_io_buf + 0, 4);
  if (magic != MAGIC || s_io_buf[4] != VERSION) return false;

  uint32_t hd  = 0;
  uint32_t cnt = 0;
  memcpy(&hd,  s_io_buf + 8,  4);
  memcpy(&cnt, s_io_buf + 12, 4);

  if (hd >= storage::alerts_store::RING_CAPACITY) return false;
  if (cnt > storage::alerts_store::RING_CAPACITY) return false;

  s_head  = hd;
  s_count = cnt;
  memcpy(s_ring, s_io_buf + HEADER_SIZE, BODY_SIZE);
  return true;
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace storage::alerts_store {

bool init() {
  if (s_inited) return true;

  if (!alloc_psram_buffers()) {
    ESP_LOGE(TAG, "alerts_store: PSRAM alloc failed — alerts disabled");
    return false;
  }

  lfs::mkdir_p(RING_DIR);

  if (!read_from_disk()) {
    ESP_LOGW(TAG, "alerts ring not found or corrupt — starting fresh");
    memset(s_ring, 0, BODY_SIZE);
    s_head  = 0;
    s_count = 0;
    s_dirty = true;
    write_to_disk();
    s_dirty = false;
  } else {
    ESP_LOGI(TAG, "alerts ring loaded — %lu entries", (unsigned long)s_count);
  }

  s_inited = true;
  return true;
}

bool append(const AlertEntry& a) {
  if (!s_ring) return false;
  s_ring[s_head] = a;
  s_head = (s_head + 1) % RING_CAPACITY;
  if (s_count < RING_CAPACITY) s_count++;
  s_dirty = true;
  return true;
}

size_t read(AlertEntry* out, size_t max_count, size_t skip_from_newest) {
  if (!out || max_count == 0 || s_count == 0 || !s_ring) return 0;

  // Oldest entry sits at (head - count + cap) % cap.
  // We iterate newest→oldest for the skip/limit API.
  size_t available = s_count;
  if (skip_from_newest >= available) return 0;

  size_t to_read = available - skip_from_newest;
  if (to_read > max_count) to_read = max_count;

  // newest index = (s_head - 1 + cap) % cap
  // Walk backwards: newest - skip, newest - skip - 1, ...
  for (size_t i = 0; i < to_read; i++) {
    // Index of (skip_from_newest + i)-th from newest:
    size_t offset = skip_from_newest + i;
    size_t idx = (s_head + RING_CAPACITY - 1 - offset) % RING_CAPACITY;
    out[i] = s_ring[idx];
  }
  return to_read;
}

void clear_all() {
  if (!s_ring) return;
  memset(s_ring, 0, BODY_SIZE);
  s_head  = 0;
  s_count = 0;
  s_dirty = true;
  write_to_disk();
  s_dirty = false;
}

bool persist() {
  if (!s_dirty) return true;
  bool ok = write_to_disk();
  if (ok) {
    s_dirty = false;
    ESP_LOGD(TAG, "alerts ring persisted (%lu entries)", (unsigned long)s_count);
  } else {
    ESP_LOGE(TAG, "alerts ring persist failed");
  }
  return ok;
}

size_t stored_count() {
  return s_count;
}

#ifdef NATIVE_BUILD
// Test-only: drop in-RAM state and clear the inited flag so a subsequent
// init() call re-reads whatever is currently on disk, simulating a power
// cycle without re-running the whole process.
void test_simulate_reboot() {
  s_inited = false;
  s_head   = 0;
  s_count  = 0;
  s_dirty  = false;
  if (s_ring) memset(s_ring, 0, BODY_SIZE);
}
#endif

}  // namespace storage::alerts_store
