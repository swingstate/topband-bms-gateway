#include "bus/snapshot_bus.h"
#include <atomic>
#include <cstring>

#ifndef NATIVE_BUILD
#include "esp_heap_caps.h"
#include "esp_log.h"
static const char* TAG = "snapshot_bus";
#endif

// ── Module-private state ────────────────────────────────────────────────────

// Two snapshot slots. Allocated in init(); never freed (device lifetime).
static BmsSystemSnapshot* slots_[2] = {nullptr, nullptr};

// Index of the slot currently visible to readers (0 or 1).
static std::atomic<uint8_t> active_slot_{0};

// Per-slot seqlock counters — kept SEPARATE from BmsSystemSnapshot::seq.
// Using reinterpret_cast<atomic*>(&slot->seq) is technically UB under strict
// aliasing and at -O3 the compiler may serve the memcpy-cached value instead
// of doing a fresh load. Real std::atomic<> objects avoid that entirely.
// After a successful read the confirmed seq value is written into out.seq
// so callers can inspect it (e.g. the seq-semantics unit test).
static std::atomic<uint32_t> slot_seq_[2] = {0, 0};

// Global sequence counter. Always even at rest; used to derive the next odd
// "write in progress" value for begin_publish.
static std::atomic<uint32_t> seq_counter_{0};

// Diagnostic counters.
static std::atomic<uint64_t> total_publishes_{0};
static std::atomic<uint64_t> total_reads_{0};
static std::atomic<uint64_t> total_retries_{0};

#ifndef NDEBUG
// Debug guard: asserts single-producer discipline.
static std::atomic<bool> in_publish_{false};
#endif

// ── Public API ───────────────────────────────────────────────────────────────

namespace bus::snapshot_bus {

bool init() {
#ifdef NATIVE_BUILD
  slots_[0] = static_cast<BmsSystemSnapshot*>(malloc(sizeof(BmsSystemSnapshot)));
  slots_[1] = static_cast<BmsSystemSnapshot*>(malloc(sizeof(BmsSystemSnapshot)));
#else
  uint32_t psram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  slots_[0] = static_cast<BmsSystemSnapshot*>(
      heap_caps_malloc(sizeof(BmsSystemSnapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  slots_[1] = static_cast<BmsSystemSnapshot*>(
      heap_caps_malloc(sizeof(BmsSystemSnapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  uint32_t psram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "Snapshot bus: PSRAM before=%lu after=%lu allocated=%lu B (2×%zu B)",
           (unsigned long)psram_before, (unsigned long)psram_after,
           (unsigned long)(psram_before - psram_after),
           sizeof(BmsSystemSnapshot));
#endif

  if (!slots_[0] || !slots_[1]) {
#ifndef NATIVE_BUILD
    ESP_LOGE(TAG, "PSRAM allocation for snapshot bus failed — cannot continue");
#endif
    if (slots_[0]) { free(slots_[0]); slots_[0] = nullptr; }
    if (slots_[1]) { free(slots_[1]); slots_[1] = nullptr; }
    return false;
  }

  memset(slots_[0], 0, sizeof(BmsSystemSnapshot));
  memset(slots_[1], 0, sizeof(BmsSystemSnapshot));

  active_slot_.store(0, std::memory_order_relaxed);
  slot_seq_[0].store(0, std::memory_order_relaxed);
  slot_seq_[1].store(0, std::memory_order_relaxed);
  seq_counter_.store(0, std::memory_order_relaxed);
  total_publishes_.store(0, std::memory_order_relaxed);
  total_reads_.store(0, std::memory_order_relaxed);
  total_retries_.store(0, std::memory_order_relaxed);
#ifndef NDEBUG
  in_publish_.store(false, std::memory_order_relaxed);
#endif
  return true;
}

BmsSystemSnapshot* begin_publish() {
#ifndef NDEBUG
  bool expected = false;
  bool claimed  = in_publish_.compare_exchange_strong(expected, true,
                    std::memory_order_acquire, std::memory_order_relaxed);
  (void)claimed;
#endif

  uint8_t active   = active_slot_.load(std::memory_order_relaxed);
  uint8_t inactive = static_cast<uint8_t>(1 - active);

  // Mark the inactive slot "write in progress" with an odd seq value.
  uint32_t new_seq = seq_counter_.fetch_add(1, std::memory_order_relaxed) + 1;
  slot_seq_[inactive].store(new_seq, std::memory_order_relaxed);

  // seq_cst fence: the odd seq store above must become visible BEFORE the
  // caller's subsequent data stores (StoreStore ordering). An acquire fence
  // does NOT provide that — it orders prior loads against later accesses,
  // not a prior relaxed store against later plain stores (review F9). The
  // exposure window was tiny (swap-based double buffer + seq re-check), but
  // this is the one primitive every consumer trusts, so use the full fence.
  std::atomic_thread_fence(std::memory_order_seq_cst);

  return slots_[inactive];
}

void publish() {
  uint8_t active   = active_slot_.load(std::memory_order_relaxed);
  uint8_t inactive = static_cast<uint8_t>(1 - active);

  // Release fence: all writes to the inactive slot are visible before
  // active_slot is swapped and before seq is advanced to even.
  std::atomic_thread_fence(std::memory_order_release);

  // Swap: the just-filled inactive slot becomes active.
  active_slot_.store(inactive, std::memory_order_release);

  // Advance seq to even — marks the now-active slot as stable.
  slot_seq_[inactive].fetch_add(1, std::memory_order_release);

  // Keep seq_counter_ even (ready for next begin_publish).
  seq_counter_.fetch_add(1, std::memory_order_relaxed);

  total_publishes_.fetch_add(1, std::memory_order_relaxed);

#ifndef NDEBUG
  in_publish_.store(false, std::memory_order_release);
#endif
}

bool read(BmsSystemSnapshot& out) {
  if (total_publishes_.load(std::memory_order_relaxed) == 0) {
    return false;
  }

  constexpr int MAX_RETRIES = 10;
  for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
    uint8_t idx = active_slot_.load(std::memory_order_acquire);

    // seq_before is a real atomic load — no aliasing with the memcpy below.
    uint32_t seq_before = slot_seq_[idx].load(std::memory_order_acquire);
    if (seq_before & 1u) {
      total_retries_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    memcpy(&out, slots_[idx], sizeof(BmsSystemSnapshot));

    // Full acquire fence: pins all memcpy loads before the seq_after read.
    // On ARM64 this generates DMB ISHLD, preventing the processor from
    // reordering the data reads to after the sequence check below.
    std::atomic_thread_fence(std::memory_order_acquire);

    uint32_t seq_after = slot_seq_[idx].load(std::memory_order_acquire);
    if (seq_before != seq_after) {
      total_retries_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    // Expose the confirmed seqlock value through the snapshot so callers
    // (and unit tests) can inspect it.
    out.seq = seq_before;
    total_reads_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }
  return false;
}

uint64_t total_publishes()    { return total_publishes_.load(std::memory_order_relaxed); }
uint64_t total_reads()        { return total_reads_.load(std::memory_order_relaxed); }
uint64_t total_read_retries() { return total_retries_.load(std::memory_order_relaxed); }

}  // namespace bus::snapshot_bus
