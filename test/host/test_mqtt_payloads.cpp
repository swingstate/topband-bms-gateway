#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <string>
#include <cstring>

#include "mqtt/payloads.h"

// ── Helpers ───────────────────────────────────────────────────────────────────

static BmsSystemSnapshot make_snap(uint8_t n_packs = 2) {
  BmsSystemSnapshot s{};
  s.pack_count_configured = n_packs;
  s.pack_count_online     = n_packs;

  for (uint8_t i = 0; i < n_packs; ++i) {
    s.pack[i].bms_id       = i;
    s.pack[i].online       = true;
    s.pack[i].cell_count   = 16;
    s.pack[i].temp_count   = 7;
    s.pack[i].pack_voltage = 52.0f;
    s.pack[i].pack_current = 5.0f;
    s.pack[i].soc          = 75;
    s.pack[i].soh          = 98;
    s.pack[i].alarm_bits   = 0;
    s.pack[i].cell_min_v   = 3.300f;
    s.pack[i].cell_max_v   = 3.350f;
    s.pack[i].cell_drift_v = 0.050f;
    s.pack[i].temp_max_c   = 30.0f;
    s.pack[i].temp_avg_c   = 25.0f;
    for (int c = 0; c < 16; ++c) s.pack[i].cell_v[c] = 3.325f;
    for (int t = 0; t < 7; ++t)  s.pack[i].temp_c[t] = 25.0f;
  }
  return s;
}

static SafetyState make_safety() {
  SafetyState ss{};
  ss.alarm_flags         = 0;
  ss.ccl_amps            = 80.0f;
  ss.dcl_amps            = 80.0f;
  ss.cvl_volts           = 56.0f;
  ss.soc_avg             = 75.0f;
  ss.soc_display         = 73.0f;
  ss.soc_source_shunt    = true;
  ss.soh_avg             = 98.0f;
  ss.pack_voltage_avg    = 52.0f;
  ss.pack_current_total  = 10.0f;
  ss.temp_avg            = 25.0f;
  ss.packs_online        = 2;
  ss.packs_configured    = 2;
  ss.event_count         = 0;
  ss.sys_message[0]      = '\0';
  return ss;
}

// ── build_data ────────────────────────────────────────────────────────────────

TEST_CASE("build_data: produces valid JSON with required fields") {
  BmsSystemSnapshot snap = make_snap(2);
  SafetyState safety = make_safety();

  char buf[1024];
  size_t n = mqtt::payloads::build_data(snap, safety, 1000000ULL, 1000U, buf, sizeof(buf));

  REQUIRE(n > 0);
  REQUIRE(n < sizeof(buf));

  std::string s(buf, n);
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"ts_ms\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"uptime_s\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"soc_avg\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"soc_display\":73"));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"soc_source\":\"shunt\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"pack_voltage_avg\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"pack_current_total\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"pack_power_w\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cvl_v\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"ccl_a\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"dcl_a\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"alarm_flags\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"bms_count_online\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cell_v_min\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cell_v_max\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cell_v_drift\""));
}

TEST_CASE("build_data: payload fits in 1024 bytes") {
  BmsSystemSnapshot snap = make_snap(16);
  SafetyState safety = make_safety();
  safety.packs_online = 16;
  safety.packs_configured = 16;

  char buf[1024];
  size_t n = mqtt::payloads::build_data(snap, safety, 9999999ULL, 9999U, buf, sizeof(buf));
  REQUIRE(n > 0);
  REQUIRE(n < 1024);
}

TEST_CASE("build_data: returns 0 on too-small buffer") {
  BmsSystemSnapshot snap = make_snap(1);
  SafetyState safety = make_safety();

  char buf[8];
  size_t n = mqtt::payloads::build_data(snap, safety, 1ULL, 1U, buf, sizeof(buf));
  REQUIRE(n == 0);
}

// ── build_cells ───────────────────────────────────────────────────────────────

TEST_CASE("build_cells: produces valid JSON with cells_v array") {
  BmsSystemSnapshot snap = make_snap(1);
  const BmsPackSnapshot& pack = snap.pack[0];

  char buf[1024];
  size_t n = mqtt::payloads::build_cells(pack, 2000000ULL, buf, sizeof(buf));

  REQUIRE(n > 0);
  REQUIRE(n < sizeof(buf));

  std::string s(buf, n);
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"ts_ms\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cells_v\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"alarm_bits\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cell_count\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"bms_id\""));
}

// Regression: the per-cell-level HA discovery PACK_ENTITIES (Pack N
// Voltage/Current/SOC) read value_json.{voltage,current,soc} from this same
// Cells JSON. If those keys go missing the entities show "Unknown" in HA while
// Pack N Alarms (value_json.alarm_bits) keeps working. Guard all three keys
// with the exact names the value_templates expect, plus their values.
TEST_CASE("build_cells: includes pack voltage/current/power/soc for HA value_templates") {
  BmsSystemSnapshot snap = make_snap(1);
  const BmsPackSnapshot& pack = snap.pack[0];  // voltage 52.0, current 5.0, soc 75

  char buf[1024];
  size_t n = mqtt::payloads::build_cells(pack, 2000000ULL, buf, sizeof(buf));
  REQUIRE(n > 0);

  std::string s(buf, n);
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"voltage\":52"));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"current\":5"));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"soc\":75"));
  // power = voltage * current = 52.0 * 5.0 = 260; feeds Pack N Power entity.
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"power\":260"));
}

TEST_CASE("build_cells: fits in 1024 bytes for 16 cells") {
  BmsSystemSnapshot snap = make_snap(1);
  snap.pack[0].cell_count = 16;

  char buf[1024];
  size_t n = mqtt::payloads::build_cells(snap.pack[0], 1ULL, buf, sizeof(buf));
  REQUIRE(n > 0);
  REQUIRE(n < 1024);
}

// ── build_alarm_event ─────────────────────────────────────────────────────────

TEST_CASE("build_alarm_event: produces valid JSON with event fields") {
  SafetyState::EventEntry evt{};
  evt.type       = SafetyState::SafetyEvent::CellOvervoltStart;
  evt.bms_id     = 2;
  evt.alarm_bits = 0x0001;

  char buf[512];
  size_t n = mqtt::payloads::build_alarm_event(evt, 3000000ULL, buf, sizeof(buf));

  REQUIRE(n > 0);
  REQUIRE(n < sizeof(buf));

  std::string s(buf, n);
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"ts_ms\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"event\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"bms_id\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"alarm_bits\""));
}

// ── build_diag ────────────────────────────────────────────────────────────────

TEST_CASE("build_diag: produces valid JSON with diag fields") {
  BmsSystemSnapshot snap = make_snap(2);
  SafetyState safety = make_safety();
  bms::poller::PollerStats ps{};
  ps.cycles_completed = 100;
  ps.cycle_avg_ms     = 50;
  ps.cycle_max_ms     = 120;
  can::tx::CanStats cs{};
  cs.tx_ok   = 1000;
  cs.tx_fail = 0;

  char buf[1024];
  size_t n = mqtt::payloads::build_diag(snap, safety, ps, cs, 5000000ULL, 5000U, buf, sizeof(buf));

  REQUIRE(n > 0);
  REQUIRE(n < sizeof(buf));

  std::string s(buf, n);
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"ts_ms\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cycles\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"cycle_avg_ms\""));
  REQUIRE_THAT(s, Catch::Matchers::ContainsSubstring("\"can_tx_ok\""));
}
