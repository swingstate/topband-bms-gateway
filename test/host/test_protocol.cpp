// Host-side Catch2 tests for bms/protocol — parse_response_header and all
// interpret_* functions.  The make_response / make_response_raw helpers build
// valid ASCII wire frames from binary INFO byte arrays so every test is fully
// self-contained.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "bms/protocol.h"
#include "fixtures/synthetic_frames.cpp"

using namespace bms::protocol;
using Catch::Approx;

// ── Frame building helpers ────────────────────────────────────────────────────

static const char kHexChars[] = "0123456789ABCDEF";

static void push_hex_byte(std::vector<uint8_t>& v, uint8_t b) {
  v.push_back((uint8_t)kHexChars[b >> 4]);
  v.push_back((uint8_t)kHexChars[b & 0xF]);
}

static void push_hex_u16(std::vector<uint8_t>& v, uint16_t w) {
  push_hex_byte(v, (uint8_t)(w >> 8));
  push_hex_byte(v, (uint8_t)(w & 0xFF));
}

static uint16_t calc_lenid_field(uint16_t lenid) {
  lenid &= 0x0FFF;
  uint16_t s    = (uint16_t)(((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF));
  uint16_t lchk = (uint16_t)((~(s & 0xF) + 1) & 0xF);
  return (uint16_t)((lchk << 12) | lenid);
}

// Must be called after all INFO bytes have been pushed but before CHKSUM/EOI.
static uint16_t calc_checksum(const std::vector<uint8_t>& v, uint16_t lenid) {
  uint32_t sum = 0;
  for (size_t i = 1; i < (size_t)(13 + lenid); i++) sum += v[i];
  return (uint16_t)((~(sum & 0xFFFF) + 1) & 0xFFFF);
}

// Binary INFO bytes hex-encoded into a complete valid ASCII wire response frame.
static std::vector<uint8_t> make_response(uint8_t addr, uint8_t rtn,
                                          const uint8_t* info, size_t n) {
  uint16_t lenid = (uint16_t)(n * 2);
  std::vector<uint8_t> v;
  v.push_back(TB_SOI);
  push_hex_byte(v, TB_PROTOCOL_VERSION);
  push_hex_byte(v, addr);
  push_hex_byte(v, TB_CID1_BATTERY_DATA);
  push_hex_byte(v, rtn);
  push_hex_u16(v, calc_lenid_field(lenid));
  for (size_t i = 0; i < n; i++) push_hex_byte(v, info[i]);
  push_hex_u16(v, calc_checksum(v, lenid));
  v.push_back(TB_EOI);
  return v;
}

// Raw ASCII INFO bytes (CID2=0x51 style — not hex-encoded) into a valid frame.
static std::vector<uint8_t> make_response_raw(uint8_t addr, uint8_t rtn,
                                               const uint8_t* info, size_t n) {
  uint16_t lenid = (uint16_t)n;
  std::vector<uint8_t> v;
  v.push_back(TB_SOI);
  push_hex_byte(v, TB_PROTOCOL_VERSION);
  push_hex_byte(v, addr);
  push_hex_byte(v, TB_CID1_BATTERY_DATA);
  push_hex_byte(v, rtn);
  push_hex_u16(v, calc_lenid_field(lenid));
  for (size_t i = 0; i < n; i++) v.push_back(info[i]);
  push_hex_u16(v, calc_checksum(v, lenid));
  v.push_back(TB_EOI);
  return v;
}

// ── 1. parse_response_header: happy path ─────────────────────────────────────

TEST_CASE("header parses valid response frame") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Analog16Cell,
                             synth::kInfo_Analog16Cell_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);
  CHECK(bms_id == 0x00);
  CHECK(rtn == TB_RTN_OK);
  CHECK(plen == synth::kInfo_Analog16Cell_len * 2);
  CHECK(payload == frame.data() + 13);
}

// ── 2. parse_response_header: wrong SOI ──────────────────────────────────────

TEST_CASE("header rejects bad SOI byte") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Analog16Cell,
                             synth::kInfo_Analog16Cell_len);
  frame[0] = 0x00;
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::BadSoi);
}

// ── 3. parse_response_header: truncated buffer ───────────────────────────────

TEST_CASE("header rejects truncated frame") {
  uint8_t buf[] = { TB_SOI, '2', '1', '0', '0' };  // 5 bytes < 18 minimum
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(buf, sizeof(buf),
                                bms_id, rtn, &payload, plen) == ParseError::TruncatedHeader);
}

// ── 4. parse_response_header: corrupted checksum ─────────────────────────────

TEST_CASE("header rejects frame with corrupt checksum") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Analog16Cell,
                             synth::kInfo_Analog16Cell_len);
  // Corrupt first INFO byte: stored CHKSUM no longer matches recomputed value.
  frame[13] ^= 0xFF;
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::BadChecksum);
}

// ── 5. parse_response_header: wrong VER byte ─────────────────────────────────

TEST_CASE("header rejects frame with wrong VER byte") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Analog16Cell,
                             synth::kInfo_Analog16Cell_len);
  // Overwrite VER "21" with "22" (0x22 != TB_PROTOCOL_VERSION=0x21).
  // VER check fires before CRC check so BadVer is returned first.
  frame[1] = '2';
  frame[2] = '2';
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::BadVer);
}

// ── 6. analog values: 16-cell synthetic ──────────────────────────────────────

TEST_CASE("analog 16-cell synthetic frame decodes correctly") {
  auto frame = make_response(0x01, TB_RTN_OK,
                             synth::kInfo_Analog16Cell,
                             synth::kInfo_Analog16Cell_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_analog_values_fixed_point av{};
  REQUIRE(interpret_analog_values_fixed_point(payload, plen, av) == ParseError::Ok);

  CHECK(av.cell_count == 16);
  for (int i = 0; i < 16; i++)
    CHECK(av.cells[i] == Approx(3.350f).epsilon(0.001));

  CHECK(av.temp_count == 8);
  for (int i = 0; i < 8; i++)
    CHECK(av.temps[i] == Approx(25.0f).epsilon(0.1));

  CHECK(av.pack_current == Approx(0.0f).epsilon(0.01));
  CHECK(av.pack_voltage == Approx(536.48f).epsilon(0.5));
  CHECK(av.rem_ah  == Approx(20.0f).epsilon(0.01));
  CHECK(av.full_ah == Approx(20.0f).epsilon(0.01));
  CHECK(av.cycles == 1);
  CHECK(av.soc    == 100);
  CHECK(av.soh    == 100);
}

// ── 7. alarm info: all-clear ─────────────────────────────────────────────────

TEST_CASE("alarm info all-clear") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_AlarmClean,
                             synth::kInfo_AlarmClean_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_alarm_info ai{};
  REQUIRE(interpret_alarm_info(payload, plen, ai) == ParseError::Ok);

  CHECK(ai.alarm_bits == 0);
  CHECK(ai.cell_count == 15);
  CHECK(ai.temp_count == 7);
  for (int i = 0; i < 15; i++) CHECK(ai.cell_v_alarm[i] == 0);
}

// ── 8. alarm info: cell over-voltage ─────────────────────────────────────────

TEST_CASE("alarm info cell over-voltage bit set") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_AlarmOvervolt,
                             synth::kInfo_AlarmOvervolt_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_alarm_info ai{};
  REQUIRE(interpret_alarm_info(payload, plen, ai) == ParseError::Ok);

  CHECK((ai.alarm_bits & (uint64_t)TB_ALRMS::CELL_OVER_VOLTAGE_PROTECT) != 0);
  CHECK(ai.cell_v_alarm[2] == 0x01);
}

// ── 9. alarm info: temp charge-stop ──────────────────────────────────────────

TEST_CASE("alarm info cell temp charge-stop bit set") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_AlarmTempChargeStop,
                             synth::kInfo_AlarmTempChargeStop_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_alarm_info ai{};
  REQUIRE(interpret_alarm_info(payload, plen, ai) == ParseError::Ok);

  CHECK((ai.alarm_bits & (uint64_t)TB_ALRMS::CELL_TEMP_CHARGE_PROTECT) != 0);
}

// ── 10. alarm info: critical combo bits 0/13/23 ──────────────────────────────

TEST_CASE("alarm info critical combo bits 0/13/23") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_AlarmCritical,
                             synth::kInfo_AlarmCritical_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_alarm_info ai{};
  REQUIRE(interpret_alarm_info(payload, plen, ai) == ParseError::Ok);

  CHECK((ai.alarm_bits & (uint64_t)TB_ALRMS::CELL_OVER_VOLTAGE_PROTECT) != 0);
  CHECK((ai.alarm_bits & (uint64_t)TB_ALRMS::PACK_UNDER_VOLTAGE_PROTECT) != 0);
  CHECK((ai.alarm_bits & (uint64_t)TB_ALRMS::CHARGING) != 0);
}

// ── 11. system parameter: 15S LFP defaults ───────────────────────────────────

TEST_CASE("system parameter 15S LFP defaults decode correctly") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Sysparam,
                             synth::kInfo_Sysparam_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_system_parameter sp{};
  REQUIRE(interpret_system_parameter(payload, plen, sp) == ParseError::Ok);

  CHECK(sp.cell_high_v      == Approx(3.650f).epsilon(0.001));
  CHECK(sp.cell_low_v       == Approx(2.500f).epsilon(0.001));
  CHECK(sp.cell_under_v     == Approx(2.400f).epsilon(0.001));
  CHECK(sp.charge_high_t    == Approx(45.0f).epsilon(0.1));
  CHECK(sp.charge_low_t     == Approx(0.0f).epsilon(0.1));
  CHECK(sp.charge_max_a     == Approx(75.0f).epsilon(0.1));
  CHECK(sp.module_high_v    == Approx(54.75f).epsilon(0.01));
  CHECK(sp.module_low_v     == Approx(37.50f).epsilon(0.01));
  CHECK(sp.module_under_v   == Approx(36.00f).epsilon(0.01));
  CHECK(sp.discharge_high_t == Approx(60.0f).epsilon(0.1));
  CHECK(sp.discharge_low_t  == Approx(-20.0f).epsilon(0.1));
  CHECK(sp.discharge_max_a  == Approx(150.0f).epsilon(0.1));
}

// ── 12. manufacturer info ─────────────────────────────────────────────────────

TEST_CASE("manufacturer info strings parse correctly") {
  auto frame = make_response_raw(0x00, TB_RTN_OK,
                                  synth::kInfo_Manufacturer,
                                  synth::kInfo_Manufacturer_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_manufacturer_info mi{};
  REQUIRE(interpret_manufacturer_info(payload, plen, mi) == ParseError::Ok);

  CHECK(std::string(mi.hw) == "TopBand");
  CHECK(std::string(mi.sw) == "V1.0");
  CHECK(std::string(mi.id) == "TBLT-15S");
}

// ── 13. battery date ──────────────────────────────────────────────────────────

TEST_CASE("date 2024-01-15 decodes correctly") {
  auto frame = make_response(0x00, TB_RTN_OK,
                             synth::kInfo_Date,
                             synth::kInfo_Date_len);
  uint8_t bms_id, rtn;
  const uint8_t* payload;
  size_t plen;
  REQUIRE(parse_response_header(frame.data(), frame.size(),
                                bms_id, rtn, &payload, plen) == ParseError::Ok);

  tb_date d{};
  REQUIRE(interpret_date(payload, plen, d) == ParseError::Ok);

  CHECK(d.year   == 2024);
  CHECK(d.month  == 1);
  CHECK(d.day    == 15);
  CHECK(d.hour   == 0);
  CHECK(d.minute == 0);
  CHECK(d.second == 0);
}

// ── 14. build_request ─────────────────────────────────────────────────────────

TEST_CASE("build_request produces correct frame structure") {
  uint8_t buf[32];
  // Empty INFO field (standard analog query)
  size_t n = build_request(0x01, TB_CID2_ANALOG_VALUES_FIXED_POINT,
                           nullptr, 0, buf, sizeof(buf));
  REQUIRE(n == 18);        // SOI(1) + 12 header + 0 INFO + 4 CHKSUM + EOI(1)
  CHECK(buf[0]     == TB_SOI);
  CHECK(buf[n - 1] == TB_EOI);
  CHECK(buf[1] == '2');    // VER high nibble
  CHECK(buf[2] == '1');    // VER low nibble  (0x21)
  CHECK(buf[3] == '0');    // ADR high nibble
  CHECK(buf[4] == '1');    // ADR low nibble  (0x01)
  CHECK(buf[5] == '4');    // CID1 high nibble
  CHECK(buf[6] == '6');    // CID1 low nibble (0x46)
  CHECK(buf[7] == '4');    // CID2 high nibble
  CHECK(buf[8] == '2');    // CID2 low nibble (0x42)
}

// ── 15. real captured frames ──────────────────────────────────────────────────

TEST_CASE("all 40 real captured frames parse and decode without error") {
  std::ifstream f(FIXTURE_DIR "/captured_frames.txt");
  REQUIRE(f.is_open());

  int count = 0;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;

    // Format: <timestamp_ms> <bms_id> <current_A> <voltage_V> <hex_payload>
    std::istringstream ss(line);
    std::string ts, bid, cur, volt, hex;
    if (!(ss >> ts >> bid >> cur >> volt >> hex)) continue;

    // hex_payload is the ASCII wire frame without SOI/EOI; wrap it.
    std::vector<uint8_t> frame;
    frame.reserve(hex.size() + 2);
    frame.push_back(TB_SOI);
    for (char c : hex) frame.push_back((uint8_t)c);
    frame.push_back(TB_EOI);

    uint8_t bms_id, rtn;
    const uint8_t* payload;
    size_t plen;
    auto err = parse_response_header(frame.data(), frame.size(),
                                     bms_id, rtn, &payload, plen);
    INFO("line " << (count + 1) << ": " << line.substr(0, 80));
    REQUIRE(err == ParseError::Ok);

    tb_analog_values_fixed_point av{};
    auto ierr = interpret_analog_values_fixed_point(payload, plen, av);
    REQUIRE(ierr == ParseError::Ok);
    CHECK(av.cell_count >= 1);
    CHECK(av.cell_count <= 16);

    count++;
  }
  REQUIRE(count == 40);
}
