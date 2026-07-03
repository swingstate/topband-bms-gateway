#pragma once
#include <cstdint>

// ── Coredump probe — one-time, cached, stale-dump aware ─────────────────────
//
// esp_core_dump_get_summary() reads and parses the dump from flash. A dump
// written by an older BIN-format firmware passes esp_core_dump_image_check()
// under the ELF config but the summary parser then walks garbage "ELF"
// headers with thousands of small flash reads: measured ~33 s per call on
// hardware (preview.3: every /api/diag request paid this, starving the whole
// single-threaded HTTP server).
//
// probe() runs the check + summary parse exactly once (boot calls it before
// the HTTP server starts; later callers get the cached result — the dump
// cannot change during uptime, a new panic implies a reboot). Unusable dumps
// (foreign format / failed parse) are erased so they cost nothing on any
// later boot and stop raising a false "previous panic" alert.

namespace diag::coredump {

struct ProbeResult {
  bool     present;      // a usable coredump exists in flash
  bool     has_summary;  // summary fields below are filled (ELF format only)
  char     task[32];     // crashing task name
  char     sha256[68];   // app ELF SHA256 (hex string as stored in the dump)
  char     pc_hex[16];   // exception PC as "0x%08x"
  uint32_t version;      // core dump format version
};

// First call probes flash (may take seconds once if a stale dump exists,
// which it then erases); subsequent calls return the cached result.
const ProbeResult& probe();

}  // namespace diag::coredump
