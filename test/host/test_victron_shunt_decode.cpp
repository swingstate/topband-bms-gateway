#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstring>

#include "sources/victron_shunt_decode.h"

using namespace sources;
using Catch::Matchers::WithinAbs;

namespace {

// LSB-first bit writer — inverse of the production read_bits_u(), used only
// here to build synthetic test vectors matching the real Victron bit-packing.
void write_bits_u(uint8_t* data, size_t& bit_pos, uint32_t value, int nbits) {
  for (int i = 0; i < nbits; i++) {
    size_t byte_idx = bit_pos >> 3;
    int    bit_idx  = (int)(bit_pos & 7u);
    if ((value >> i) & 1u) data[byte_idx] |= (uint8_t)(1u << bit_idx);
    bit_pos++;
  }
}

// Builds a 15-byte new-format BATTERY_MONITOR plaintext with the given
// (already-scaled-to-raw) field values.
void build_payload(uint8_t out[15], uint32_t remaining_mins, uint32_t voltage_raw,
                    uint32_t alarm, uint32_t aux, uint32_t aux_mode,
                    uint32_t current_raw, uint32_t consumed_ah, uint32_t soc_raw) {
  memset(out, 0, 15);
  size_t bp = 0;
  write_bits_u(out, bp, remaining_mins, 16);
  write_bits_u(out, bp, voltage_raw,    16);
  write_bits_u(out, bp, alarm,          16);
  write_bits_u(out, bp, aux,            16);
  write_bits_u(out, bp, aux_mode,        2);
  write_bits_u(out, bp, current_raw,    22);
  write_bits_u(out, bp, consumed_ah,    20);
  write_bits_u(out, bp, soc_raw,        10);
}

}  // namespace

TEST_CASE("victron_shunt_decode: correct field extraction (real-world-like values)",
          "[victron_shunt_decode]") {
  uint8_t payload[15];
  // voltage 50.09 V -> raw 5009; current 1.5 A -> raw 1500; soc 65.0% -> raw 650.
  build_payload(payload, /*remaining_mins=*/2295, /*voltage_raw=*/5009, /*alarm=*/0,
                /*aux=*/0, /*aux_mode=*/0, /*current_raw=*/1500, /*consumed_ah=*/0,
                /*soc_raw=*/650);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));

  REQUIRE(out.voltage_valid);
  REQUIRE_THAT(out.voltage_v, WithinAbs(50.09f, 0.001f));
  REQUIRE(out.current_valid);
  REQUIRE_THAT(out.current_a, WithinAbs(1.5f, 0.001f));
  REQUIRE(out.soc_valid);
  REQUIRE_THAT(out.soc_pct, WithinAbs(65.0f, 0.001f));
}

TEST_CASE("victron_shunt_decode: reproduces the field-report bug when misread byte-aligned",
          "[victron_shunt_decode]") {
  // This is the owner's actual field report: hero tile 22.95 V, true bank
  // voltage ~50 V. remaining_mins=2295 raw, interpreted /100 by the OLD
  // (buggy) byte-aligned code as "voltage", produces exactly 22.95 — proving
  // the old code was reading remaining_mins, not voltage. The corrected
  // decoder must read the true voltage field (bytes 2-3) instead.
  uint8_t payload[15];
  build_payload(payload, /*remaining_mins=*/2295, /*voltage_raw=*/5009, /*alarm=*/0,
                /*aux=*/0, /*aux_mode=*/0, /*current_raw=*/0, /*consumed_ah=*/0,
                /*soc_raw=*/0x3FF /* unsynced */);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));

  REQUIRE(out.voltage_valid);
  REQUIRE_THAT(out.voltage_v, WithinAbs(50.09f, 0.001f));
  // remaining_mins/100 would be 22.95 -- confirm the decoder does NOT produce that.
  REQUIRE_THAT(out.voltage_v, !WithinAbs(22.95f, 0.001f));
}

TEST_CASE("victron_shunt_decode: sentinel voltage (0x7FFF) is not available",
          "[victron_shunt_decode]") {
  uint8_t payload[15];
  build_payload(payload, 0, 0x7FFF, 0, 0, 0, 0, 0, 0);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE_FALSE(out.voltage_valid);
}

TEST_CASE("victron_shunt_decode: sentinel current (0x3FFFFF) is not available",
          "[victron_shunt_decode]") {
  uint8_t payload[15];
  build_payload(payload, 0, 0, 0, 0, 0, 0x3FFFFF, 0, 0);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE_FALSE(out.current_valid);
}

TEST_CASE("victron_shunt_decode: sentinel SOC (0x3FF) means shunt not yet synchronized",
          "[victron_shunt_decode]") {
  uint8_t payload[15];
  build_payload(payload, 0, 0, 0, 0, 0, 0, 0, 0x3FF);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE_FALSE(out.soc_valid);
}

TEST_CASE("victron_shunt_decode: consumed_ah decodes negated in 0.1 Ah units",
          "[victron_shunt_decode]") {
  // 123.4 Ah drawn -> raw 1234; decoder negates -> -123.4 Ah (VictronConnect
  // "Consumed Amp Hours" convention: discharged reads negative).
  uint8_t payload[15];
  build_payload(payload, 0, 5009, 0, 0, 0, 1500, /*consumed_ah=*/1234, 650);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE(out.consumed_ah_valid);
  REQUIRE_THAT(out.consumed_ah, WithinAbs(-123.4f, 0.001f));
}

TEST_CASE("victron_shunt_decode: consumed_ah sentinel (0xFFFFF) is not available",
          "[victron_shunt_decode]") {
  uint8_t payload[15];
  build_payload(payload, 0, 5009, 0, 0, 0, 1500, /*consumed_ah=*/0xFFFFF, 650);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE_FALSE(out.consumed_ah_valid);
  // A downstream field (SOC) must still decode correctly, proving the 20-bit
  // consumed_ah field occupies exactly its documented bit span.
  REQUIRE(out.soc_valid);
  REQUIRE_THAT(out.soc_pct, WithinAbs(65.0f, 0.001f));
}

TEST_CASE("victron_shunt_decode: negative current sign-extends correctly",
          "[victron_shunt_decode]") {
  // -2.5 A discharge -> raw -2500 as 22-bit two's complement.
  uint8_t payload[15];
  uint32_t raw22 = (uint32_t)(-2500) & 0x3FFFFFu;
  build_payload(payload, 0, 0, 0, 0, 0, raw22, 0, 0);

  ShuntDecodedNewFmt out;
  REQUIRE(decode_shunt_new_fmt(payload, sizeof(payload), out));
  REQUIRE(out.current_valid);
  REQUIRE_THAT(out.current_a, WithinAbs(-2.5f, 0.001f));
}

TEST_CASE("victron_shunt_decode: rejects payload shorter than 15 bytes",
          "[victron_shunt_decode]") {
  uint8_t payload[14] = {};
  ShuntDecodedNewFmt out;
  REQUIRE_FALSE(decode_shunt_new_fmt(payload, sizeof(payload), out));
}
