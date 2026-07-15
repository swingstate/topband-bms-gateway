#include "victron_shunt_decode.h"

namespace sources {

namespace {

// LSB-first bit reader over a byte buffer, matching Victron's Extra
// Manufacturer Data bit-packing (keshavdv/victron-ble BitReader): the first
// bit read is bit 0 of byte 0, and each subsequent bit read becomes the next
// higher-order bit of the accumulated value.
uint32_t read_bits_u(const uint8_t* data, size_t data_len, size_t& bit_pos, int nbits) {
  uint32_t value = 0;
  for (int i = 0; i < nbits; i++) {
    size_t byte_idx = bit_pos >> 3;
    int    bit_idx  = (int)(bit_pos & 7u);
    uint32_t bit = (byte_idx < data_len) ? ((uint32_t)(data[byte_idx] >> bit_idx) & 1u) : 0u;
    value |= (bit << i);
    bit_pos++;
  }
  return value;
}

int32_t sign_extend(uint32_t value, int nbits) {
  uint32_t sign_bit = 1u << (nbits - 1);
  return (int32_t)((value ^ sign_bit) - sign_bit);
}

}  // namespace

bool decode_shunt_new_fmt(const uint8_t* plain, size_t plain_len, ShuntDecodedNewFmt& out) {
  out = ShuntDecodedNewFmt{};
  if (plain_len < 15) return false;

  size_t bit_pos = 0;
  read_bits_u(plain, plain_len, bit_pos, 16);            // remaining_mins — unused
  uint32_t v_raw   = read_bits_u(plain, plain_len, bit_pos, 16);
  read_bits_u(plain, plain_len, bit_pos, 16);             // alarm_reason — unused
  read_bits_u(plain, plain_len, bit_pos, 16);             // aux_input — unused
  read_bits_u(plain, plain_len, bit_pos, 2);              // aux_mode — unused
  uint32_t i_raw   = read_bits_u(plain, plain_len, bit_pos, 22);
  uint32_t cah_raw = read_bits_u(plain, plain_len, bit_pos, 20);
  uint32_t soc_raw = read_bits_u(plain, plain_len, bit_pos, 10);

  out.voltage_valid = (v_raw != 0x7FFFu);
  if (out.voltage_valid) out.voltage_v = (float)sign_extend(v_raw, 16) / 100.0f;

  out.current_valid = (i_raw != 0x3FFFFFu);
  if (out.current_valid) out.current_a = (float)sign_extend(i_raw, 22) / 1000.0f;

  out.soc_valid = (soc_raw != 0x3FFu);
  if (out.soc_valid) out.soc_pct = (float)soc_raw / 10.0f;

  // Consumed Ah: shunt's own Coulomb counter, 0.1 Ah units, negated so that a
  // discharged battery reads negative (VictronConnect convention). Matches
  // keshavdv/victron-ble battery_monitor.py. Sentinel 0xFFFFF = not available.
  out.consumed_ah_valid = (cah_raw != 0xFFFFFu);
  if (out.consumed_ah_valid) out.consumed_ah = -(float)cah_raw / 10.0f;

  return true;
}

}  // namespace sources
