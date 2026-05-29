#include "diag/log_ring.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#ifndef NATIVE_BUILD
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#endif

// ── Ring storage ──────────────────────────────────────────────────────────────

static constexpr size_t MAX_LINE = 120;

struct RingLine {
  char text[MAX_LINE];
};

static RingLine* s_buf   = nullptr;  // PSRAM-allocated ring
static size_t    s_cap   = 0;
static size_t    s_head  = 0;  // next-write index (wraps at s_cap)
static size_t    s_count = 0;  // valid entries (saturates at s_cap)

#ifndef NATIVE_BUILD
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
// Original vprintf so we can chain to UART output.
static int (*s_orig_vprintf)(const char*, va_list) = nullptr;
#endif

// ── vprintf hook (target only) ────────────────────────────────────────────────

#ifndef NATIVE_BUILD
static int log_hook(const char* fmt, va_list args) {
  // MAX_LINE + 4: append() truncates at MAX_LINE-1 = 119 chars anyway.
  // Original 256-byte buffer wasted 132 B on every calling task's stack.
  // Per docs/diag-mqtt-crash-review.md Finding 8.
  char line[MAX_LINE + 4];
  va_list args_copy;
  va_copy(args_copy, args);
  vsnprintf(line, sizeof(line), fmt, args_copy);
  va_end(args_copy);

  diag::log_ring::append(line);

  if (s_orig_vprintf) return s_orig_vprintf(fmt, args);
  return 0;
}
#endif

// ── Public API ────────────────────────────────────────────────────────────────

namespace diag::log_ring {

void init(size_t capacity_lines) {
  if (s_buf) return;  // idempotent
  s_cap = capacity_lines;

#ifndef NATIVE_BUILD
  s_buf = static_cast<RingLine*>(
      heap_caps_malloc(s_cap * sizeof(RingLine), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!s_buf) {
    s_buf = static_cast<RingLine*>(malloc(s_cap * sizeof(RingLine)));
  }
#else
  s_buf = static_cast<RingLine*>(malloc(s_cap * sizeof(RingLine)));
#endif

  if (s_buf) {
    memset(s_buf, 0, s_cap * sizeof(RingLine));
  }

#ifndef NATIVE_BUILD
  s_orig_vprintf = esp_log_set_vprintf(log_hook);
#endif
}

void append(const char* line) {
  if (!s_buf || !line) return;

#ifndef NATIVE_BUILD
  portENTER_CRITICAL(&s_mux);
#endif

  size_t idx = s_head;
  strncpy(s_buf[idx].text, line, MAX_LINE - 1);
  s_buf[idx].text[MAX_LINE - 1] = '\0';

  // Strip trailing CR/LF for clean JSON embedding.
  size_t len = strlen(s_buf[idx].text);
  while (len > 0 &&
         (s_buf[idx].text[len - 1] == '\n' || s_buf[idx].text[len - 1] == '\r')) {
    s_buf[idx].text[--len] = '\0';
  }

  s_head = (s_head + 1) % s_cap;
  if (s_count < s_cap) s_count++;

#ifndef NATIVE_BUILD
  portEXIT_CRITICAL(&s_mux);
#endif
}

size_t count() {
  return s_count;
}

size_t snapshot(char* out, size_t out_size) {
  if (!s_buf || !out || out_size == 0) return 0;

  // Snapshot ring metadata under lock; read data without lock (stale is OK).
#ifndef NATIVE_BUILD
  portENTER_CRITICAL(&s_mux);
#endif
  size_t cnt  = s_count;
  size_t head = s_head;
#ifndef NATIVE_BUILD
  portEXIT_CRITICAL(&s_mux);
#endif

  // Oldest entry sits at (head - count + cap) % cap.
  size_t start = (head + s_cap - cnt) % s_cap;

  size_t written = 0;
  for (size_t i = 0; i < cnt; i++) {
    size_t      idx  = (start + i) % s_cap;
    const char* text = s_buf[idx].text;
    size_t      len  = strlen(text);
    if (len == 0) continue;

    // Append line + newline, guarding against out_size overflow.
    size_t avail = out_size - written - 2;  // room for text + '\n' + '\0'
    if (avail == 0) break;
    size_t copy = len < avail ? len : avail;
    memcpy(out + written, text, copy);
    written += copy;
    out[written++] = '\n';
  }
  out[written] = '\0';
  return written;
}

#ifdef NATIVE_BUILD
void reset_for_test(size_t capacity_lines) {
  free(s_buf);
  s_buf   = nullptr;
  s_cap   = 0;
  s_head  = 0;
  s_count = 0;
  init(capacity_lines);
}
#endif

}  // namespace diag::log_ring
