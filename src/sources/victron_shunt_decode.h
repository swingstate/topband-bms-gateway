#pragma once
#include <cstdint>
#include <cstddef>

// ── Victron new-format (Product Advertisement) BATTERY_MONITOR bit-packed decode ──
//
// The SmartShunt Instant Readout payload (record_type 0x02, new/2022+ firmware,
// md[2] == 0x10) is NOT byte-aligned. It is a sequence of bit-packed fields, LSB
// first, read as one continuous little-endian bitstream:
//
//   bits   0-15  remaining_mins   (u16,           sentinel 0xFFFF = n/a)
//   bits  16-31  voltage          (i16, /100 V,   sentinel 0x7FFF = n/a)
//   bits  32-47  alarm_reason     (u16,           not decoded — unused)
//   bits  48-63  aux_input        (u16,           meaning depends on aux_mode; unused)
//   bits  64-65  aux_mode         (u2,            not decoded — unused)
//   bits  66-87  current          (i22, /1000 A,  sentinel 0x3FFFFF = n/a)
//   bits  88-107 consumed_ah      (u20, /10 Ah negated, sentinel 0xFFFFF = n/a)
//   bits 108-117 soc              (u10, /10 %,    sentinel 0x3FF = shunt not yet
//                                                  synchronized/calibrated)
//
// Verified against keshavdv/victron-ble battery_monitor.py (authoritative
// open-source reference for this protocol). Needs >= 15 decrypted bytes (bit
// 117 falls in byte 14). This is distinct from the byte-aligned legacy
// (pre-2022, old-format) SmartShunt layout, which decode_shunt() in
// ble_scanner.cpp still handles separately.

namespace sources {

struct ShuntDecodedNewFmt {
  float voltage_v     = 0.0f;
  bool  voltage_valid = false;
  float current_a     = 0.0f;
  bool  current_valid = false;
  float soc_pct       = 0.0f;
  bool  soc_valid      = false;  // false = shunt not yet synchronized (raw sentinel 0x3FF)
  // Shunt's own hardware Coulomb counter: Ah drawn since last full-charge sync.
  // Negative when discharged (VictronConnect "Consumed Amp Hours" convention).
  // Read-only diagnostic reference only — not yet wired into any fused source.
  float consumed_ah       = 0.0f;
  bool  consumed_ah_valid = false;  // false = raw sentinel 0xFFFFF (not available)
};

// Parses the bit-packed new-format BATTERY_MONITOR payload.
// Returns false if plain_len < 15 (insufficient data for all fields).
bool decode_shunt_new_fmt(const uint8_t* plain, size_t plain_len, ShuntDecodedNewFmt& out);

}  // namespace sources
