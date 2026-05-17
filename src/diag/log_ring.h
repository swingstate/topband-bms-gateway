#pragma once
#include <cstddef>

// In-RAM ring buffer of recent ESP-IDF log lines.
// Capacity 200 lines × 120 bytes ≈ 24 KB, PSRAM-allocated.
// A vprintf hook feeds every log line into this ring; /api/diag reads it out.

namespace diag::log_ring {

// Allocate ring storage and install the ESP-IDF vprintf hook.
// Call once during boot, before any logging begins.
void init(size_t capacity_lines = 200);

// Append a pre-formatted log line. Called from the vprintf hook.
// Thread-safe via a short portMUX spinlock (< 1 µs hold).
void append(const char* line);

// Copy ring contents oldest→newest into out as newline-delimited text.
// Returns total bytes written (excluding null terminator).
size_t snapshot(char* out, size_t out_size);

// Number of lines currently stored.
size_t count();

#ifdef NATIVE_BUILD
// Re-initialize ring from scratch (frees and reallocates buffer).
// For use in host tests only — allows isolated per-test-case ring state.
void reset_for_test(size_t capacity_lines = 10);
#endif

}  // namespace diag::log_ring
