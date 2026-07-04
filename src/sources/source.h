#pragma once
#include <cstdint>

// ── Source abstraction (V3.1) ─────────────────────────────────────────────────
//
// Design principle: source selection PER METRIC, not per device.
// The Aggregator decides, for each metric, which enabled Source provides the
// most trustworthy reading at any given moment.
//
// SCOPE BOUNDARY: per-pack metrics (cell voltages, per-pack SOC, temperature)
// remain on the existing BMS path and are explicitly OUT of scope here.
// This abstraction covers system-level aggregate metrics only.
//
// To add a new source (e.g. VE.Direct MPPT over serial):
//   1. Add src/sources/vedirect_source.{h,cpp} implementing ISource.
//   2. Add its Metric accessors and Config fields.
//   3. Register in Aggregator.
//   No changes to this header needed for new sources.

namespace sources {

enum class Metric : uint8_t {
  TOTAL_CURRENT  = 0,   // combined bank current (A); positive = charging
  TOTAL_VOLTAGE  = 1,   // bank terminal voltage (V)
  PV_POWER       = 2,   // PV array power (W)
  PV_CURRENT     = 3,   // PV array current (A); measured directly from MPPT input side
  PV_VOLTAGE     = 4,   // PV array voltage (V); measured directly from MPPT input side
  SHUNT_SOC      = 5,   // shunt-counted SOC (%). Never overrides per-pack BMS SOC (the shunt
                         // cannot see individual packs). MAY become the bank-level aggregate
                         // SOC via Aggregator::fuse_bank_soc() — see aggregator.h/.cpp (V3.2,
                         // Option C in docs/research/v3.2-shunt-soc-fusion.md).
  YIELD_TODAY    = 6,   // PV yield today (Wh); from MPPT Solar Charger record bytes 6-7
  CHARGE_STATE   = 7,   // MPPT charger state (0=off,3=bulk,4=absorption,5=float); uint8 cast to float
};

enum class ReadingStatus : uint8_t {
  Valid       = 0,  // fresh, trustworthy value
  Stale       = 1,  // present but low-confidence (e.g. BMS <0.5A resolution limit)
  Unavailable = 2,  // source not enabled or no data received yet
};

struct SourceReading {
  float         value;
  const char*   unit;         // e.g. "A", "V", "W", "%" — never nullptr
  uint32_t      timestamp_ms; // esp_timer_get_time()/1000 when taken; 0 = not set
  ReadingStatus status;

  bool is_valid()    const { return status == ReadingStatus::Valid; }
  bool is_usable()   const { return status != ReadingStatus::Unavailable; }
};

// Canonical sentinel returned by sources that do not provide a given metric.
inline SourceReading unavailable_reading(const char* unit = "") {
  return { 0.0f, unit, 0, ReadingStatus::Unavailable };
}

// ── ISource ────────────────────────────────────────────────────────────────────
class ISource {
public:
  virtual ~ISource() = default;

  // Short lowercase identifier, e.g. "bms", "shunt", "mppt".
  virtual const char* id() const = 0;

  // True if this source is configured and enabled.
  virtual bool enabled() const = 0;

  // Refresh internal state from the underlying data path. Called once per
  // aggregator cycle (typically every BMS poller cycle, ~1 s).
  virtual void update() = 0;

  // Return the current reading for metric m.
  // Sources that do not provide a metric MUST return unavailable_reading().
  // A source that provides a metric but has no fresh data returns Stale, never 0.
  virtual SourceReading reading(Metric m) const = 0;
};

}  // namespace sources
