#include "diag/alerts.h"
#include "bus/queues.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>

#ifndef NATIVE_BUILD
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"
#include "net/ntp.h"
#else
#include <ctime>
#endif

// ── Source name table ─────────────────────────────────────────────────────────

static const char* const SOURCE_NAMES[] = {
  "unknown", "poller", "safety", "can", "mqtt", "wifi", "boot", "ota",
};
static constexpr size_t SOURCE_COUNT =
    sizeof(SOURCE_NAMES) / sizeof(SOURCE_NAMES[0]);

namespace diag::alerts {

const char* source_name(Source s) {
  size_t idx = static_cast<size_t>(s);
  return (idx < SOURCE_COUNT) ? SOURCE_NAMES[idx] : SOURCE_NAMES[0];
}

Source source_from_name(const char* name) {
  if (!name) return Source::Unknown;
  for (size_t i = 0; i < SOURCE_COUNT; i++) {
    if (strcmp(name, SOURCE_NAMES[i]) == 0) return static_cast<Source>(i);
  }
  return Source::Unknown;
}

}  // namespace diag::alerts

// ── Throttle table ────────────────────────────────────────────────────────────

static constexpr uint32_t THROTTLE_WINDOW_MS = 60000u;
static constexpr size_t   THROTTLE_SLOTS     = 16;

struct ThrottleEntry {
  bool     active;          // false = slot is free
  uint8_t  source_id;
  uint32_t msg_hash;
  uint32_t last_emit_ms;   // monotonic ms
  uint32_t suppressed;     // count suppressed since last_emit_ms
};

static ThrottleEntry s_throttle[THROTTLE_SLOTS];

#ifndef NATIVE_BUILD
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

static uint32_t djb2(const char* s) {
  uint32_t h = 5381u;
  while (*s) h = h * 33u ^ (uint8_t)*s++;
  return h;
}

// Returns current monotonic milliseconds.
static uint32_t mono_ms() {
#ifndef NATIVE_BUILD
  return (uint32_t)(esp_timer_get_time() / 1000LL);
#else
  return 0u;  // host tests disable time-based throttle
#endif
}

#ifndef NATIVE_BUILD
// Returns category byte for AlertEntry from source enum.
static uint8_t category_from_source(diag::alerts::Source s) {
  switch (s) {
    case diag::alerts::Source::Poller:
    case diag::alerts::Source::Safety:
      return 0;  // safety
    case diag::alerts::Source::Wifi:
    case diag::alerts::Source::Mqtt:
      return 1;  // net
    case diag::alerts::Source::Can:
      return 2;  // hw
    case diag::alerts::Source::Ota:
      return 3;  // ota
    case diag::alerts::Source::Boot:
      return 4;  // user
    default:
      return 4;
  }
}
#endif  // !NATIVE_BUILD

// ── NATIVE_BUILD in-memory store ──────────────────────────────────────────────

#ifdef NATIVE_BUILD
static constexpr size_t TEST_RING_CAP = 64;
static diag::alerts::TestAlert s_test_ring[TEST_RING_CAP];
static size_t s_test_count = 0;
#endif

// ── emit() ────────────────────────────────────────────────────────────────────

namespace diag::alerts {

void emit(Severity sev, const char* source, const char* fmt, ...) {
  Source src = source_from_name(source);
  uint8_t src_id = static_cast<uint8_t>(src);

  // Format the message.
  char msg[120];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);

  uint32_t hash = djb2(msg) ^ ((uint32_t)src_id << 24u);
  uint32_t now  = mono_ms();

  uint32_t suppressed_count = 0;

#ifndef NATIVE_BUILD
  portENTER_CRITICAL(&s_mux);
#endif

  // Look for matching throttle slot.
  int found_slot = -1;
  int empty_slot = -1;
  for (int i = 0; i < (int)THROTTLE_SLOTS; i++) {
    if (!s_throttle[i].active) {
      if (empty_slot < 0) empty_slot = i;
    } else if (s_throttle[i].source_id == src_id &&
               s_throttle[i].msg_hash  == hash) {
      found_slot = i;
      break;
    }
  }

  if (found_slot >= 0) {
    uint32_t age = now - s_throttle[found_slot].last_emit_ms;
    if (age < THROTTLE_WINDOW_MS) {
      // Within window: suppress.
      s_throttle[found_slot].suppressed++;
#ifndef NATIVE_BUILD
      portEXIT_CRITICAL(&s_mux);
#endif
      return;
    }
    // Throttle window expired: collect suppressed count and re-emit.
    suppressed_count = s_throttle[found_slot].suppressed;
    s_throttle[found_slot].last_emit_ms = now;
    s_throttle[found_slot].suppressed   = 0;
  } else {
    // New entry.
    int slot = (empty_slot >= 0) ? empty_slot : 0;  // evict slot 0 if full
    s_throttle[slot] = { true, src_id, hash, now, 0 };
  }

#ifndef NATIVE_BUILD
  portEXIT_CRITICAL(&s_mux);
#endif

  // Append suppression note to message if needed.
  if (suppressed_count > 0) {
    size_t used = strlen(msg);
    snprintf(msg + used, sizeof(msg) - used, " (%lu suppressed)",
             (unsigned long)suppressed_count);
  }

#ifdef NATIVE_BUILD
  // Host build: store in test ring.
  if (s_test_count < TEST_RING_CAP) {
    TestAlert& a = s_test_ring[s_test_count++];
    a.ts_epoch   = (uint32_t)time(nullptr);
    a.severity   = static_cast<uint8_t>(sev);
    a.source_id  = src_id;
    strncpy(a.message, msg, sizeof(a.message) - 1);
    a.message[sizeof(a.message) - 1] = '\0';
  }
#else
  // Target build: post AlertEntry to q_alert.
  AlertEntry entry{};
  entry.ts_epoch = net::ntp::now_unix_s();
  entry.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
  entry.severity = static_cast<uint8_t>(sev);
  entry.category = category_from_source(src);
  entry.flags    = static_cast<uint16_t>(src_id);
  strncpy(entry.message, msg, sizeof(entry.message) - 1);
  entry.message[sizeof(entry.message) - 1] = '\0';

  if (!q_alert) return;

  // Non-blocking: drop oldest on full.
  if (xQueueSend(q_alert, &entry, 0) != pdTRUE) {
    AlertEntry dropped{};
    xQueueReceive(q_alert, &dropped, 0);
    xQueueSend(q_alert, &entry, 0);
  }
#endif
}

#ifdef NATIVE_BUILD
size_t test_get_all(TestAlert* out, size_t max_count) {
  size_t n = s_test_count < max_count ? s_test_count : max_count;
  for (size_t i = 0; i < n; i++) out[i] = s_test_ring[i];
  return n;
}

void test_reset() {
  memset(s_test_ring, 0, sizeof(s_test_ring));
  s_test_count = 0;
  memset(s_throttle, 0, sizeof(s_throttle));
}
#endif

}  // namespace diag::alerts
