#pragma once
#include <cstddef>
#include <cstdint>

// ── LittleFS file store ──────────────────────────────────────────────────────
// Thin wrappers around esp_littlefs. All paths are absolute LittleFS paths
// (e.g. "/lfs/ui/index.html"). Blocking but bounded: file operations complete
// within tens of milliseconds on typical flash. Do NOT call from ControlTask.

namespace storage::lfs {

// Mount LittleFS on the "littlefs" partition. Formats on mount failure.
// Returns false only if both mount and format fail (degraded mode).
// Must be called once during boot before any other lfs:: function.
bool init();

// Returns true if the file exists on LittleFS.
bool exists(const char* path);

// Read a text file into buf (null-terminates). Returns number of bytes read,
// or 0 on failure. buf_size must include room for the null terminator, so at
// most buf_size - 1 bytes of file content are ever returned. Not suitable for
// reading a fixed-size binary blob back to its exact length — use
// read_file_exact() for that.
size_t read_file(const char* path, char* buf, size_t buf_size);

// Read exactly `len` bytes of a fixed-size binary file into buf. No null
// terminator is reserved or written. Returns true only if the file exists
// and yields exactly `len` bytes; false if missing, short, or unreadable.
bool read_file_exact(const char* path, uint8_t* buf, size_t len);

// Atomic write: writes to <path>.tmp, then renames to <path>.
// Prevents partial-write corruption from power loss.
// Returns false on any failure.
bool write_file_atomic(const char* path, const uint8_t* data, size_t len);

// Create directory (and any missing parents). Returns true if already exists.
bool mkdir_p(const char* path);

// Remove a file. Returns true if removed, false if not found or error.
bool remove_file(const char* path);

// Partition capacity in bytes. Returns 0 before init() is called.
size_t total_bytes();
size_t free_bytes();

}  // namespace storage::lfs
