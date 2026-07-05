// test/host/test_can_pylontech.cpp
//
// Catch2 host tests for can/pylontech.cpp.
//
// Fixtures cover 4 representative operating states. Expected frame bytes are
// computed manually from the V2.67 sendVictronCAN() encoding (for 0x351/0x355/
// 0x356/0x35E) and the spec-derived Pylontech LV BMS CAN Protocol v1.1 mapping
// (for 0x359 and 0x35C). Each test case asserts byte-for-byte equality, making
// frame format regressions visible immediately.
//
// Test sections:
//   [pylontech][frames]   Per-frame byte assertions for 4 safety state fixtures
//   [pylontech]           Property tests: manufacturer ID, signed current, enable flags

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <cstdint>

#include "can/pylontech.h"
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

TEST_CASE("Pylontech 0x351: idle 4 packs", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x351(s, out);

  // cvl=52.5→525=0x020D, ccl=100.0→1000=0x03E8, dcl=100.0→1000=0x03E8
  REQUIRE(out[0] == 0x0D);  REQUIRE(out[1] == 0x02);  // cv=525
  REQUIRE(out[2] == 0xE8);  REQUIRE(out[3] == 0x03);  // ccl=1000
  REQUIRE(out[4] == 0xE8);  REQUIRE(out[5] == 0x03);  // dcl=1000
  REQUIRE(out[6] == 0x00);  REQUIRE(out[7] == 0x00);
}

TEST_CASE("Pylontech 0x355: idle 4 packs", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x355(s, out);

  // soc=72→0x0048, soh=98→0x0062, cap=400*10=4000=0x0FA0
  REQUIRE(out[0] == 0x48);  REQUIRE(out[1] == 0x00);  // soc=72
  REQUIRE(out[2] == 0x62);  REQUIRE(out[3] == 0x00);  // soh=98
  REQUIRE(out[4] == 0xA0);  REQUIRE(out[5] == 0x0F);  // cap=4000
  REQUIRE(out[6] == 0x00);  REQUIRE(out[7] == 0x00);
}

TEST_CASE("Pylontech 0x356: idle 4 packs", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x356(s, out);

  // volt=52.1*100=5210=0x145A, curr=0*10=0, temp=25*10=250=0x00FA
  REQUIRE(out[0] == 0x5A);  REQUIRE(out[1] == 0x14);  // v=5210
  REQUIRE(out[2] == 0x00);  REQUIRE(out[3] == 0x00);  // i=0
  REQUIRE(out[4] == 0xFA);  REQUIRE(out[5] == 0x00);  // t=250
}

TEST_CASE("Pylontech 0x359: idle no alarms", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);

  REQUIRE(out[0] == 0x00);  // no protection bits
  REQUIRE(out[1] == 0x00);  // no warning bits
  REQUIRE(out[2] == 0x00);  // no fault bits
  for (int i = 3; i < 8; ++i) REQUIRE(out[i] == 0x00);
}

TEST_CASE("Pylontech 0x35C: idle, both enabled", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 100.0f, 72.0f, 98.0f, 400.0f,
                      52.1f, 0.0f, 25.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x35C(s, out);

  // ccl=100 > 0.1 → charge enable (bit7); dcl=100 > 0.1 → discharge enable (bit6).
  // No undervolt alarm → force-charge (bit5) clear.
  REQUIRE(out[0] == (0x80 | 0x40));
  for (int i = 1; i < 8; ++i) REQUIRE(out[i] == 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 2: charging at high SOC (CCL throttled)
// cvl=53.2  ccl=20.0  dcl=100.0  soc=95  soh=99  cap=400
// volt=53.1  curr=20.0  temp=30.0  alarm=0x00
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Pylontech 0x351: charging high SOC (throttled CCL)", "[pylontech][frames]") {
  auto s = make_state(53.2f, 20.0f, 100.0f, 95.0f, 99.0f, 400.0f,
                      53.1f, 20.0f, 30.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x351(s, out);

  // cvl=53.2→532=0x0214, ccl=20*10=200=0x00C8, dcl=100*10=1000=0x03E8
  REQUIRE(out[0] == 0x14);  REQUIRE(out[1] == 0x02);  // cv=532
  REQUIRE(out[2] == 0xC8);  REQUIRE(out[3] == 0x00);  // ccl=200
  REQUIRE(out[4] == 0xE8);  REQUIRE(out[5] == 0x03);  // dcl=1000
}

TEST_CASE("Pylontech 0x356: charging 20A positive current", "[pylontech][frames]") {
  auto s = make_state(53.2f, 20.0f, 100.0f, 95.0f, 99.0f, 400.0f,
                      53.1f, 20.0f, 30.0f, 0x00);
  uint8_t out[8];
  can::pylontech::build_0x356(s, out);

  // volt=53.1*100=5310=0x14BE, curr=20*10=200=0x00C8, temp=30*10=300=0x012C
  REQUIRE(out[0] == 0xBE);  REQUIRE(out[1] == 0x14);  // v=5310
  REQUIRE(out[2] == 0xC8);  REQUIRE(out[3] == 0x00);  // i=200 (positive=charging)
  REQUIRE(out[4] == 0x2C);  REQUIRE(out[5] == 0x01);  // t=300
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 3: discharging at low SOC
// cvl=52.5  ccl=100.0  dcl=80.0  soc=15  soh=97  cap=400
// volt=49.8  curr=-60.0  temp=28.0  alarm=0x10 (undervolt)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Pylontech 0x356: discharging negative current", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::pylontech::build_0x356(s, out);

  // curr=-60.0*10=-600 → 0xFDA8 signed LE
  auto enc = static_cast<int16_t>(out[2] | (static_cast<uint16_t>(out[3]) << 8));
  REQUIRE(enc == -600);
}

TEST_CASE("Pylontech 0x359: undervolt alarm sets UVP bit", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);

  // alarm_flags 0x10 → byte0 bit1 (cell UVP)
  REQUIRE((out[0] & 0x02) != 0);
  // alarm_flags 0x10 → byte0 bit6 force-charge; no charge OCP (ccl=100 > 0.1)
  REQUIRE((out[0] & 0x10) == 0);
}

TEST_CASE("Pylontech 0x35C: undervolt, force-charge bit set", "[pylontech][frames]") {
  auto s = make_state(52.5f, 100.0f, 80.0f, 15.0f, 97.0f, 400.0f,
                      49.8f, -60.0f, 28.0f, 0x10);
  uint8_t out[8];
  can::pylontech::build_0x35C(s, out);

  // charge enable (bit7) + discharge enable (bit6, dcl=80 > 0.1) + force charge (bit5, undervolt)
  REQUIRE((out[0] & 0x80) != 0);  // charge enable
  REQUIRE((out[0] & 0x40) != 0);  // discharge enable (dcl=80 > 0.1)
  REQUIRE((out[0] & 0x20) != 0);  // force charge (undervolt)
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture 4: alarm condition — overvolt + temp stop, CCL/DCL zeroed
// cvl=52.5  ccl=0.0  dcl=0.0  soc=100  soh=98  cap=400
// volt=54.2  curr=0.0  temp=45.0  alarm=0x02|0x08 (overvolt+temp stop)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Pylontech 0x359: overvolt + temp stop sets protection bits", "[pylontech][frames]") {
  auto s = make_state(52.5f, 0.0f, 0.0f, 100.0f, 98.0f, 400.0f,
                      54.2f, 0.0f, 45.0f, 0x02 | 0x08);
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);

  // 0x02 overvolt → byte0 bit0 (cell OVP)
  REQUIRE((out[0] & 0x01) != 0);
  // 0x08 temp stop → byte0 bit2 (charge OTP) and bit3 (discharge OTP)
  REQUIRE((out[0] & 0x04) != 0);
  REQUIRE((out[0] & 0x08) != 0);
  // ccl=0 < 0.1 → byte0 bit4 (charge OCP proxy)
  REQUIRE((out[0] & 0x10) != 0);
  // dcl=0 < 0.1 → byte0 bit5 (discharge OCP proxy)
  REQUIRE((out[0] & 0x20) != 0);
}

TEST_CASE("Pylontech 0x35C: CCL+DCL zeroed means no enable bits", "[pylontech][frames]") {
  auto s = make_state(52.5f, 0.0f, 0.0f, 100.0f, 98.0f, 400.0f,
                      54.2f, 0.0f, 45.0f, 0x02 | 0x08);
  uint8_t out[8];
  can::pylontech::build_0x35C(s, out);

  // ccl < 0.1 → bit7 (charge enable) not set
  REQUIRE((out[0] & 0x80) == 0);
  // dcl < 0.1 → bit6 (discharge enable) not set
  REQUIRE((out[0] & 0x40) == 0);
  // no undervolt alarm → force charge (bit5) not set
  REQUIRE((out[0] & 0x20) == 0);
  REQUIRE(out[0] == 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
// Property tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Pylontech 0x35E manufacturer ID is always 'PYLON   '", "[pylontech]") {
  uint8_t out[8];
  can::pylontech::build_0x35E(out);
  REQUIRE(memcmp(out, "PYLON   ", 8) == 0);
}

TEST_CASE("Pylontech 0x356 negative current encoded as signed 16-bit LE", "[pylontech]") {
  SafetyState s{};
  s.pack_current_total = -15.5f;
  uint8_t out[8];
  can::pylontech::build_0x356(s, out);
  // -15.5 * 10 = -155 → 0xFF65 signed 16-bit LE
  auto encoded = static_cast<int16_t>(out[2] | (static_cast<uint16_t>(out[3]) << 8));
  REQUIRE(encoded == -155);
  REQUIRE(out[2] == 0x65);
  REQUIRE(out[3] == 0xFF);
}

TEST_CASE("Pylontech 0x359 imbalance alarm maps to warning byte, not protection byte", "[pylontech]") {
  SafetyState s{};
  s.alarm_flags  = 0x20;  // imbalance only
  s.ccl_amps     = 100.0f;
  s.dcl_amps     = 100.0f;
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);

  REQUIRE(out[0] == 0x00);         // no protection bits
  REQUIRE((out[1] & 0x40) != 0);  // byte1 bit6 = cell drift warning
  REQUIRE(out[2] == 0x00);
}

TEST_CASE("Pylontech 0x359 BMS-reported alarm maps to byte2 and byte0 AFE bit", "[pylontech]") {
  SafetyState s{};
  s.alarm_flags = 0x40;  // BMS-reported critical alarm
  s.ccl_amps    = 100.0f;
  s.dcl_amps    = 100.0f;
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);

  REQUIRE((out[0] & 0x80) != 0);  // byte0 bit7 = AFE/critical
  REQUIRE((out[2] & 0x01) != 0);  // byte2 bit0 = pack-level fault
}

TEST_CASE("Pylontech 0x359 byte4 carries battery module count = packs online", "[pylontech]") {
  // OpenDTU-onBattery reads data[4] directly as "Module Count" ("Batteriemodule").
  // A healthy 3-pack system must report 3, not 0 and not packs*cells.
  SafetyState s{};
  s.packs_online     = 3;
  s.packs_configured = 3;
  s.ccl_amps         = 100.0f;
  s.dcl_amps         = 100.0f;
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);
  REQUIRE(out[4] == 3);
  // Module count is independent of the alarm/protection bytes.
  REQUIRE(out[0] == 0x00);
  REQUIRE(out[1] == 0x00);
}

TEST_CASE("Pylontech 0x359 module count tracks online, not configured", "[pylontech]") {
  // One of three configured packs offline → count reflects the two communicating.
  SafetyState s{};
  s.packs_online     = 2;
  s.packs_configured = 3;
  s.ccl_amps         = 100.0f;
  s.dcl_amps         = 100.0f;
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);
  REQUIRE(out[4] == 2);
}

TEST_CASE("Pylontech 0x359 module count is zero when no packs online", "[pylontech]") {
  SafetyState s{};
  s.packs_online = 0;
  s.alarm_flags  = 0x80;  // no packs online
  uint8_t out[8];
  can::pylontech::build_0x359(s, out);
  REQUIRE(out[4] == 0);
  REQUIRE((out[2] & 0x80) != 0);  // byte2 bit7 = system fault still set
}

TEST_CASE("Pylontech 0x351 bytes 6-7 carry discharge voltage limit (DVL)", "[pylontech]") {
  SafetyState s{};
  s.cvl_volts = 53.5f;  s.ccl_amps = 120.0f;  s.dcl_amps = 120.0f;
  s.dvl_volts = 47.5f;  // configured pack low-voltage cutoff
  uint8_t out[8];
  can::pylontech::build_0x351(s, out);
  // dvl=47.5*10=475=0x01DB, LE
  REQUIRE(out[6] == 0xDB);
  REQUIRE(out[7] == 0x01);
}

TEST_CASE("Pylontech 0x351 DVL is zero when unset (regression guard)", "[pylontech]") {
  // A default-constructed state has dvl_volts=0 → bytes 6-7 zero. runSafety()
  // always populates dvl_volts from cfg.safe_pack_volt, so 0 only appears here.
  SafetyState s{};
  s.cvl_volts = 53.5f;
  uint8_t out[8];
  can::pylontech::build_0x351(s, out);
  REQUIRE(out[6] == 0x00);
  REQUIRE(out[7] == 0x00);
}

TEST_CASE("Pylontech 0x355 SOC follows fused Combined SOC when valid", "[pylontech]") {
  // soc_display_valid → CAN uses soc_display (e.g. shunt-led), not soc_avg.
  SafetyState s{};
  s.soc_avg = 72.0f;
  s.soc_display = 88.0f;
  s.soc_display_valid = true;
  s.soh_avg = 98.0f;
  uint8_t out[8];
  can::pylontech::build_0x355(s, out);
  REQUIRE(out[0] == 88);  // fused SOC, not soc_avg=72
  REQUIRE(out[1] == 0x00);
  REQUIRE(out[2] == 98);  // SOH stays BMS-only
}

TEST_CASE("Pylontech 0x355 SOC falls back to BMS soc_avg when fused invalid", "[pylontech]") {
  SafetyState s{};
  s.soc_avg = 72.0f;
  s.soc_display = 88.0f;       // stale/garbage
  s.soc_display_valid = false; // no valid fused value → fall back to BMS
  uint8_t out[8];
  can::pylontech::build_0x355(s, out);
  REQUIRE(out[0] == 72);  // BMS soc_avg fallback
}
