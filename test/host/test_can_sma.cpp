// test/host/test_can_sma.cpp
//
// Catch2 host tests for can/sma.cpp.
//
// Fixtures cover 4 representative operating states. Expected frame bytes are
// computed from V2.67 sendVictronCAN() with g_can_protocol==2 (for 0x351/0x355/
// 0x356/0x35A/0x35E) and spec-derived SMA Sunny Island BMS protocol (for 0x35B).
//
// Test sections:
//   [sma][frames]   Per-frame byte assertions for 4 safety state fixtures
//   [sma]           Property tests: manufacturer ID, signed current, alarm bits

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <cstdint>

#include "can/sma.h"
#include "safety_state.h"

// ─────────────────────────────────────────────────────────────────────────────
// Fixture helpers
// ─────────────────────────────────────────────────────────────────────────────

static SafetyState make_state(float cvl, float ccl, float dcl,
                               float soc, float soh, float cap_ah,
                               float volt, float curr, float temp,
                               uint8_t alarm_flags) {
  SafetyState s{};
  s.cvl_volts          = cvl;
  s.ccl_amps           = ccl;
  s.dcl_amps           = dcl;
  s.soc_avg            = soc;
  s.soh_avg            = soh;
  s.capacity_total_ah  = cap_ah;
  s.pack_voltage_avg   = volt;
  s.pack_current_total = curr;
  s.temp_avg           = temp;
  s.alarm_flags        = alarm_flags;
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 1: idle, 4 packs, no alarms
// cvl=52.5  ccl=100.0  dcl=100.0  soc=72  soh=98  cap=400
// volt=52.1  curr=0.0  temp=25.0  alarm=0x00
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SMA 0x351: idle 4 packs", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x351(s, out);

  // Same encoding as Victron: cvl=52.5→525=0x020D, ccl→1000=0x03E8, dcl→1000=0x03E8
  REQUIRE(out[0] == 0x0D);  REQUIRE(out[1] == 0x02);
  REQUIRE(out[2] == 0xE8);  REQUIRE(out[3] == 0x03);
  REQUIRE(out[4] == 0xE8);  REQUIRE(out[5] == 0x03);
  REQUIRE(out[6] == 0x00);  REQUIRE(out[7] == 0x00);
}

TEST_CASE("SMA 0x355: idle 4 packs", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x355(s, out);

  // soc=72=0x48, soh=98=0x62, cap=4000=0x0FA0
  REQUIRE(out[0] == 0x48);  REQUIRE(out[1] == 0x00);
  REQUIRE(out[2] == 0x62);  REQUIRE(out[3] == 0x00);
  REQUIRE(out[4] == 0xA0);  REQUIRE(out[5] == 0x0F);
}

TEST_CASE("SMA 0x356: idle", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x356(s, out);

  // volt=52.1*100=5210=0x145A, curr=0, temp=25*10=250=0xFA
  REQUIRE(out[0] == 0x5A);  REQUIRE(out[1] == 0x14);
  REQUIRE(out[2] == 0x00);  REQUIRE(out[3] == 0x00);
  REQUIRE(out[4] == 0xFA);  REQUIRE(out[5] == 0x00);
}

TEST_CASE("SMA 0x35A: idle, no alarms, CCL+DCL normal → enable bits clear", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x35A(s, out);

  // No alarm_flags, ccl > 0.1 → bit1 clear; dcl > 0.1 → bit0 clear
  for (int i = 0; i < 8; ++i) REQUIRE(out[i] == 0x00);
}

TEST_CASE("SMA 0x35B: idle, no warning bits", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x35B(s, out);

  for (int i = 0; i < 8; ++i) REQUIRE(out[i] == 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 2: charging at high SOC, CCL throttled
// cvl=53.2  ccl=20.0  dcl=100.0  soc=95  soh=99  cap=400
// volt=53.1  curr=20.0  temp=30.0  alarm=0x00
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SMA 0x351: charging high SOC throttled CCL", "[sma][frames]") {
  auto s = make_state(53.2f, 20.0f, 100.0f, 95.0f, 99.0f, 400.0f,
                      53.1f, 20.0f, 30.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x351(s, out);

  // cvl=532=0x0214, ccl=200=0x00C8, dcl=1000=0x03E8
  REQUIRE(out[0] == 0x14);  REQUIRE(out[1] == 0x02);
  REQUIRE(out[2] == 0xC8);  REQUIRE(out[3] == 0x00);
  REQUIRE(out[4] == 0xE8);  REQUIRE(out[5] == 0x03);
}

TEST_CASE("SMA 0x35A: no alarm flags, ccl/dcl normal → all zeros", "[sma][frames]") {
  auto s = make_state(53.2f, 20.0f, 100.0f, 95.0f, 99.0f, 400.0f,
                      53.1f, 20.0f, 30.0f, 0x00);
  uint8_t out[8];
  can::sma::build_0x35A(s, out);

  for (int i = 0; i < 8; ++i) REQUIRE(out[i] == 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 3: discharging at low SOC with undervolt
// cvl=52.5  ccl=100.0  dcl=80.0  soc=15  soh=97  cap=400
// volt=49.8  curr=-60.0  temp=28.0  alarm=0x10 (undervolt)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SMA 0x356: discharging negative current", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::sma::build_0x356(s, out);

  auto enc = static_cast<int16_t>(out[2] | (static_cast<uint16_t>(out[3]) << 8));
  REQUIRE(enc == -600);
}

TEST_CASE("SMA 0x35A: undervolt alarm_flags 0x10 sets byte4 bit4", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::sma::build_0x35A(s, out);

  // alarm_flags 0x10 (undervolt) → byte4 bit4
  REQUIRE((out[4] & 0x10) != 0);
  // ccl > 0.1 → bit1 (charge disabled) clear
  REQUIRE((out[4] & 0x02) == 0);
  // dcl > 0.1 → bit0 (discharge disabled) clear
  REQUIRE((out[4] & 0x01) == 0);
}

TEST_CASE("SMA 0x35B: undervolt sets advisory bit in warning frame", "[sma][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::sma::build_0x35B(s, out);

  // alarm_flags 0x10 (undervolt) → byte4 bit4 warning advisory
  REQUIRE((out[4] & 0x10) != 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 4: alarm — overvolt + temp stop, CCL/DCL zeroed
// cvl=52.5  ccl=0.0  dcl=0.0  soc=100  soh=98  cap=400
// volt=54.2  curr=0.0  temp=45.0  alarm=0x02|0x08
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SMA 0x35A: overvolt+temp, ccl=0+dcl=0 set charge+discharge-disable bits", "[sma][frames]") {
  auto s = make_state(52.5f, 0.0f, 0.0f, 100.0f, 98.0f, 400.0f,
                      54.2f, 0.0f, 45.0f, 0x02 | 0x08);
  uint8_t out[8];
  can::sma::build_0x35A(s, out);

  // alarm_flags 0x02 → byte4 bit6 (overvolt alarm)
  REQUIRE((out[4] & 0x40) != 0);
  // alarm_flags 0x08 → byte4 bit5 (temp stop)
  REQUIRE((out[4] & 0x20) != 0);
  // ccl=0 < 0.1 → byte4 bit1 (charge disabled) — V2.67 line 2345
  REQUIRE((out[4] & 0x02) != 0);
  // dcl=0 < 0.1 → byte4 bit0 (discharge disabled) — V2.67 line 2345
  REQUIRE((out[4] & 0x01) != 0);
}

TEST_CASE("SMA 0x35B: temp stop sets warning bit", "[sma][frames]") {
  auto s = make_state(52.5f, 0.0f, 0.0f, 100.0f, 98.0f, 400.0f,
                      54.2f, 0.0f, 45.0f, 0x02 | 0x08);
  uint8_t out[8];
  can::sma::build_0x35B(s, out);

  // alarm_flags 0x08 (temp stop) → byte4 bit5 (temp warning)
  REQUIRE((out[4] & 0x20) != 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Property tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SMA 0x35E manufacturer ID is always 'SMA     '", "[sma]") {
  uint8_t out[8];
  can::sma::build_0x35E(out);
  REQUIRE(memcmp(out, "SMA     ", 8) == 0);
}

TEST_CASE("SMA 0x356 negative current encoded as signed 16-bit LE", "[sma]") {
  SafetyState s{};
  s.pack_current_total = -15.5f;
  uint8_t out[8];
  can::sma::build_0x356(s, out);
  auto encoded = static_cast<int16_t>(out[2] | (static_cast<uint16_t>(out[3]) << 8));
  REQUIRE(encoded == -155);
  REQUIRE(out[2] == 0x65);
  REQUIRE(out[3] == 0xFF);
}

TEST_CASE("SMA 0x35A bits 0x01, 0x04 and 0x40 all set byte4 bit7 (high-priority alarm)", "[sma]") {
  SafetyState s{};
  s.ccl_amps = 100.0f;
  s.dcl_amps = 100.0f;
  uint8_t out[8];

  SECTION("bit 0x01 only (charge over-current)") {
    s.alarm_flags = 0x01;
    can::sma::build_0x35A(s, out);
    REQUIRE((out[4] & 0x80) != 0);
  }
  SECTION("bit 0x04 only (discharge over-current, V3.3)") {
    s.alarm_flags = 0x04;
    can::sma::build_0x35A(s, out);
    REQUIRE((out[4] & 0x80) != 0);
  }
  SECTION("bit 0x40 only") {
    s.alarm_flags = 0x40;
    can::sma::build_0x35A(s, out);
    REQUIRE((out[4] & 0x80) != 0);
  }
}

TEST_CASE("SMA 0x35A: CCL zeroed sets charge-disable bit, DCL normal leaves bit0 clear", "[sma]") {
  SafetyState s{};
  s.ccl_amps    = 0.0f;   // below 0.1 threshold
  s.dcl_amps    = 100.0f;
  s.alarm_flags = 0x00;
  uint8_t out[8];
  can::sma::build_0x35A(s, out);

  REQUIRE((out[4] & 0x02) != 0);  // charge disabled
  REQUIRE((out[4] & 0x01) == 0);  // discharge enabled
}

TEST_CASE("SMA 0x35B: imbalance alarm_flags 0x20 sets warning bit", "[sma]") {
  SafetyState s{};
  s.alarm_flags = 0x20;
  uint8_t out[8];
  can::sma::build_0x35B(s, out);

  REQUIRE((out[4] & 0x40) != 0);  // byte4 bit6 = cell drift warning
}

TEST_CASE("SMA 0x351 bytes 6-7 always zero", "[sma]") {
  SafetyState s{};
  s.cvl_volts = 53.5f;  s.ccl_amps = 120.0f;  s.dcl_amps = 120.0f;
  uint8_t out[8];
  can::sma::build_0x351(s, out);
  REQUIRE(out[6] == 0x00);
  REQUIRE(out[7] == 0x00);
}

TEST_CASE("SMA 0x355 SOC follows fused Combined SOC when valid", "[sma]") {
  SafetyState s{};
  s.soc_avg = 72.0f;
  s.soc_display = 88.0f;
  s.soc_display_valid = true;
  s.soh_avg = 98.0f;
  uint8_t out[8];
  can::sma::build_0x355(s, out);
  REQUIRE(out[0] == 88);  // fused SOC, not soc_avg=72
  REQUIRE(out[2] == 98);  // SOH stays BMS-only
}
