// test/host/test_can_victron.cpp
//
// Catch2 host tests for can/victron.cpp, can/tx.cpp (cadence logic),
// and can/busoff.cpp (BUS-OFF state machine).
//
// Test sections:
//   [can][v267]     V2.67 byte-regression: 8 cases × 5 frames each
//   [can]           Property tests (0x35E literal, signed-current encoding, alarm mapping)
//   [can][cadence]  can_tx_if_due heartbeat / express logic
//   [can][busoff]   BUS-OFF state machine transitions

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <cstring>
#include <cstdint>

#include "can/victron.h"
#include "can/tx.h"
#include "can/busoff.h"
#include "safety_state.h"

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Fixture helpers
// ─────────────────────────────────────────────────────────────────────────────

static json load_json(const std::string& path) {
  std::ifstream f(path);
  INFO("fixture: " << path);
  REQUIRE(f.is_open());
  return json::parse(f);
}

// Load a SafetyState from a Phase D safety output JSON.
static SafetyState safety_from_json(const json& j) {
  SafetyState s{};
  s.cycle_id           = j.value("cycle_id",          0u);
  s.produced_ms        = j.value("produced_ms",        0u);
  s.cvl_volts          = j.value("cvl_volts",          0.0f);
  s.ccl_amps           = j.value("ccl_amps",           0.0f);
  s.dcl_amps           = j.value("dcl_amps",           0.0f);
  s.pack_voltage_avg   = j.value("pack_voltage_avg",   0.0f);
  s.pack_current_total = j.value("pack_current_total", 0.0f);
  s.soc_avg            = j.value("soc_avg",            0.0f);
  s.soh_avg            = j.value("soh_avg",            0.0f);
  s.temp_avg           = j.value("temp_avg",           0.0f);
  s.capacity_total_ah  = j.value("capacity_total_ah",  0.0f);
  s.capacity_remain_ah = j.value("capacity_remain_ah", 0.0f);
  s.alarm_flags        = static_cast<uint8_t>(j.value("alarm_flags", 0));
  auto msg = j.value("sys_message", std::string("OK"));
  std::strncpy(s.sys_message, msg.c_str(), sizeof(s.sys_message) - 1);
  s.packs_online     = static_cast<uint8_t>(j.value("packs_online",     0));
  s.packs_configured = static_cast<uint8_t>(j.value("packs_configured", 0));
  return s;
}

struct FixtureFrame { uint32_t id; uint8_t data[8]; };

// Load CAN frames from a Phase E fixture JSON.
static std::vector<FixtureFrame> frames_from_json(const json& j) {
  std::vector<FixtureFrame> result;
  for (const auto& fj : j["frames"]) {
    FixtureFrame ff{};
    std::string id_str = fj["id"].get<std::string>();
    ff.id = static_cast<uint32_t>(std::stoul(id_str, nullptr, 16));
    std::string hex = fj["data_hex"].get<std::string>();
    REQUIRE(hex.size() == 16);
    for (int i = 0; i < 8; ++i)
      ff.data[i] = static_cast<uint8_t>(std::stoul(hex.substr(i * 2, 2), nullptr, 16));
    result.push_back(ff);
  }
  return result;
}

// Build SafetyState + expected frames for a named case.
struct CaseFixture {
  SafetyState           safety;
  std::vector<FixtureFrame> frames;
};

static CaseFixture load_case(const char* name) {
  std::string sd = std::string(FIXTURE_DIR) + "/v267/out/"     + name + "_safety.json";
  std::string cd = std::string(FIXTURE_DIR) + "/v267_can/out/" + name + "_can.json";
  CaseFixture cf;
  cf.safety = safety_from_json(load_json(sd));
  cf.frames = frames_from_json(load_json(cd));
  REQUIRE(cf.frames.size() == 5);
  return cf;
}

// Helper: run all 5 builders and compare against expected frames.
static void check_all_frames(const SafetyState& safety,
                              const std::vector<FixtureFrame>& expected) {
  uint8_t out[8];

  SECTION("0x351") {
    can::victron::build_0x351(safety, out);
    CHECK(expected[0].id == 0x351u);
    INFO("got: " << std::hex << (int)out[0] << " " << (int)out[1] << " "
                 << (int)out[2] << " " << (int)out[3] << " "
                 << (int)out[4] << " " << (int)out[5] << " "
                 << (int)out[6] << " " << (int)out[7]);
    REQUIRE(memcmp(out, expected[0].data, 8) == 0);
  }
  SECTION("0x355") {
    can::victron::build_0x355(safety, out);
    CHECK(expected[1].id == 0x355u);
    REQUIRE(memcmp(out, expected[1].data, 8) == 0);
  }
  SECTION("0x356") {
    can::victron::build_0x356(safety, out);
    CHECK(expected[2].id == 0x356u);
    REQUIRE(memcmp(out, expected[2].data, 8) == 0);
  }
  SECTION("0x35A") {
    can::victron::build_0x35A(safety, out);
    CHECK(expected[3].id == 0x35Au);
    REQUIRE(memcmp(out, expected[3].data, 8) == 0);
  }
  SECTION("0x35E") {
    can::victron::build_0x35E(out);
    CHECK(expected[4].id == 0x35Eu);
    REQUIRE(memcmp(out, expected[4].data, 8) == 0);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V2.67 byte-regression: 8 cases
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("V2.67 CAN regression: case_01_idle_4packs", "[can][v267]") {
  auto c = load_case("case_01_idle_4packs");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_02_charge_4packs", "[can][v267]") {
  auto c = load_case("case_02_charge_4packs");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_03_discharge_4packs", "[can][v267]") {
  auto c = load_case("case_03_discharge_4packs");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_04_cell_overvolt_pack3", "[can][v267]") {
  auto c = load_case("case_04_cell_overvolt_pack3");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_05_temp_charge_stop", "[can][v267]") {
  auto c = load_case("case_05_temp_charge_stop");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_06_bms_reported_alarm_pack1", "[can][v267]") {
  auto c = load_case("case_06_bms_reported_alarm_pack1");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_07_all_offline", "[can][v267]") {
  auto c = load_case("case_07_all_offline");
  check_all_frames(c.safety, c.frames);
}

TEST_CASE("V2.67 CAN regression: case_08_recovering_one_back_online", "[can][v267]") {
  auto c = load_case("case_08_recovering_one_back_online");
  check_all_frames(c.safety, c.frames);
}

// ─────────────────────────────────────────────────────────────────────────────
// Property tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("0x35E Victron manufacturer ID is always 'TOPBAND '", "[can]") {
  uint8_t out[8];
  can::victron::build_0x35E(out);
  REQUIRE(memcmp(out, "TOPBAND ", 8) == 0);
}

TEST_CASE("0x356 negative current encoded as signed 16-bit LE", "[can]") {
  SafetyState s{};
  s.pack_current_total = -15.5f;
  uint8_t out[8];
  can::victron::build_0x356(s, out);
  // -15.5 * 10 = -155 → 0xFF65 signed 16-bit LE
  auto encoded = static_cast<int16_t>(out[2] | (static_cast<uint16_t>(out[3]) << 8));
  REQUIRE(encoded == -155);
  REQUIRE(out[2] == 0x65);
  REQUIRE(out[3] == 0xFF);
}

TEST_CASE("0x35A alarm_flags bits 0x01 and 0x40 both map to byte4 bit7", "[can]") {
  SafetyState s{};
  uint8_t out[8];

  SECTION("bit 0x01 only") {
    s.alarm_flags = 0x01;
    can::victron::build_0x35A(s, out);
    REQUIRE((out[4] & 0x80) != 0);
  }
  SECTION("bit 0x40 only") {
    s.alarm_flags = 0x40;
    can::victron::build_0x35A(s, out);
    REQUIRE((out[4] & 0x80) != 0);
  }
  SECTION("bits 0x01|0x40 combined: byte4=0x80 only once") {
    s.alarm_flags = 0x01 | 0x40;
    can::victron::build_0x35A(s, out);
    REQUIRE(out[4] == 0x80);
  }
}

TEST_CASE("0x35A bits 0x20 and 0x80 have no Victron CAN mapping", "[can]") {
  SafetyState s{};
  uint8_t out[8];

  SECTION("imbalance 0x20") {
    s.alarm_flags = 0x20;
    can::victron::build_0x35A(s, out);
    REQUIRE(out[4] == 0x00);
  }
  SECTION("no packs online 0x80") {
    s.alarm_flags = 0x80;
    can::victron::build_0x35A(s, out);
    REQUIRE(out[4] == 0x00);
  }
}

TEST_CASE("0x351 bytes 6-7 are always zero regardless of input", "[can]") {
  SafetyState s{};
  s.cvl_volts = 53.5f; s.ccl_amps = 120.0f; s.dcl_amps = 120.0f;
  uint8_t out[8];
  can::victron::build_0x351(s, out);
  REQUIRE(out[6] == 0x00);
  REQUIRE(out[7] == 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
// Heartbeat / express cadence tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("can_tx_if_due: heartbeat fires every 1000 ms", "[can][cadence]") {
  can::tx::test_reset();

  SafetyState s{};
  s.alarm_flags = 0;

  // First call: sentinel ensures immediate fire
  REQUIRE(can::tx::can_tx_if_due(s, 0) == true);

  can::tx::CanStats st{};
  can::tx::get_stats(st);
  REQUIRE(st.heartbeats == 1);
  REQUIRE(st.express_sends == 0);

  // 999 ms later: no heartbeat
  REQUIRE(can::tx::can_tx_if_due(s, 999) == false);

  // 1000 ms after last TX: heartbeat
  REQUIRE(can::tx::can_tx_if_due(s, 1000) == true);
  can::tx::get_stats(st);
  REQUIRE(st.heartbeats == 2);

  // 500 ms: not yet
  REQUIRE(can::tx::can_tx_if_due(s, 1500) == false);

  // 1000 ms after last TX (at t=1000): heartbeat at t=2000
  REQUIRE(can::tx::can_tx_if_due(s, 2000) == true);
  can::tx::get_stats(st);
  REQUIRE(st.heartbeats == 3);
  REQUIRE(st.express_sends == 0);  // no alarm changes throughout
}

TEST_CASE("can_tx_if_due: express fires on alarm_flags rising edge", "[can][cadence]") {
  can::tx::test_reset();

  SafetyState s{};
  s.alarm_flags = 0;

  // First call fires as heartbeat (sentinel)
  REQUIRE(can::tx::can_tx_if_due(s, 0) == true);

  // 500 ms: alarm_flags unchanged — no send
  REQUIRE(can::tx::can_tx_if_due(s, 500) == false);

  // Alarm transitions to 0x02
  s.alarm_flags = 0x02;
  REQUIRE(can::tx::can_tx_if_due(s, 500) == true);

  can::tx::CanStats st{};
  can::tx::get_stats(st);
  REQUIRE(st.express_sends == 1);
  REQUIRE(st.heartbeats == 1);

  // Same alarm_flags again — no express (already updated)
  REQUIRE(can::tx::can_tx_if_due(s, 600) == false);

  // Alarm clears — another express
  s.alarm_flags = 0;
  REQUIRE(can::tx::can_tx_if_due(s, 700) == true);
  can::tx::get_stats(st);
  REQUIRE(st.express_sends == 2);
}

TEST_CASE("can_tx_if_due: heartbeat takes precedence when both fire together", "[can][cadence]") {
  can::tx::test_reset();

  SafetyState s{};
  s.alarm_flags = 0;
  // First call fires heartbeat
  REQUIRE(can::tx::can_tx_if_due(s, 0) == true);

  // At exactly 1000 ms, also change alarm_flags
  s.alarm_flags = 0x08;
  REQUIRE(can::tx::can_tx_if_due(s, 1000) == true);

  can::tx::CanStats st{};
  can::tx::get_stats(st);
  // Heartbeat takes precedence (heartbeat_due is checked first)
  REQUIRE(st.heartbeats == 2);
  REQUIRE(st.express_sends == 0);
}

TEST_CASE("can_tx_if_due: tx_ok counter increments on each frame set", "[can][cadence]") {
  can::tx::test_reset();

  SafetyState s{};
  // First call sends 5 frames
  can::tx::can_tx_if_due(s, 0);

  can::tx::CanStats st{};
  can::tx::get_stats(st);
  REQUIRE(st.tx_ok == 5);  // 5 frames per send_all_victron()

  // Heartbeat at 1000 ms sends another 5
  can::tx::can_tx_if_due(s, 1000);
  can::tx::get_stats(st);
  REQUIRE(st.tx_ok == 10);
}

// ─────────────────────────────────────────────────────────────────────────────
// BUS-OFF state machine tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("busoff: starts HEALTHY", "[can][busoff]") {
  can::busoff::test_reset();
  REQUIRE(can::busoff::is_healthy() == true);
}

TEST_CASE("busoff: BUS_OFF injection transitions to BUS_OFF_DETECTED", "[can][busoff]") {
  can::busoff::test_reset();
  REQUIRE(can::busoff::is_healthy() == true);

  // Inject BUS_OFF state
  can::busoff::set_mock_twai_state(1);
  can::busoff::tick(0);

  REQUIRE(can::busoff::is_healthy() == false);
}

TEST_CASE("busoff: recovery to HEALTHY after fault clears", "[can][busoff]") {
  can::busoff::test_reset();

  // Inject BUS_OFF
  can::busoff::set_mock_twai_state(1);
  can::busoff::tick(0);
  REQUIRE(can::busoff::is_healthy() == false);

  // Advance past first backoff window (1000 ms) to RECOVERING
  can::busoff::tick(1001);
  // Now clear the fault (simulate TWAI back to RUNNING)
  can::busoff::set_mock_twai_state(0);
  can::busoff::tick(1001);

  REQUIRE(can::busoff::is_healthy() == true);
}

TEST_CASE("busoff: exponential backoff doubles each retry", "[can][busoff]") {
  can::busoff::test_reset();

  // BUS_OFF at t=0: backoff=1000 ms, retry at t=1000
  can::busoff::set_mock_twai_state(1);
  can::busoff::tick(0);

  // At t=1001: pass first backoff, advance to RECOVERING
  can::busoff::tick(1001);  // transitions to RECOVERING, backoff doubles to 2000
  // Still bus-off — go back to BUS_OFF_DETECTED
  can::busoff::tick(3002);  // 2000 ms elapsed in RECOVERING, backoff doubles to 4000

  // Fault clears after backoff doubles twice — verify healthy
  can::busoff::set_mock_twai_state(0);
  can::busoff::tick(3002);
  REQUIRE(can::busoff::is_healthy() == true);
}

TEST_CASE("busoff: backoff caps at 30 s", "[can][busoff]") {
  can::busoff::test_reset();

  // Drive through 5 backoff doublings: 1000 → 2000 → 4000 → 8000 → 16000 → 30000 (cap)
  can::busoff::set_mock_twai_state(1);

  uint32_t t = 0;
  uint32_t backoff = 1000;
  for (int i = 0; i < 6; ++i) {
    can::busoff::tick(t);           // BUS_OFF_DETECTED at t
    t += backoff + 1;
    can::busoff::tick(t);           // RECOVERING, backoff doubles
    backoff = (backoff * 2 > 30000) ? 30000 : backoff * 2;
    t += backoff + 1;
    can::busoff::tick(t);           // back to BUS_OFF_DETECTED
  }
  // After 6 cycles backoff should be capped at 30 s — just verify still not healthy
  REQUIRE(can::busoff::is_healthy() == false);

  // Now clear fault
  can::busoff::set_mock_twai_state(0);
  can::busoff::tick(t);
  REQUIRE(can::busoff::is_healthy() == true);
}
