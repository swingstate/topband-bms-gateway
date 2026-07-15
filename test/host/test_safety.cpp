// test/host/test_safety.cpp
//
// Catch2 host tests for safety/runSafety.cpp and safety/filters.cpp.
//
// Structure:
//   [v267]     V2.67 regression cases loaded from JSON fixtures
//   [factor]   calc_factor temperature bands (via runSafety)
//   [filters]  filter_alarm_bits plausibility suppression
//   [edge]     state-transition events across two cycles
//   [soc]      SOC-based charge taper
//   [lock]     lock-to-zero rule (OV / UV / BMS alarm)
//   [proto]    sysparam-derived caps (CVL, CCL, DCL, proto-UV)
//   [update]   update_prev_state carry-forward
//
// RESOLVED (2026-07-15, chore/resolve-shouldfail-tests): the 22 cases
// previously tagged [!shouldfail] (introduced by chore/ci-real-tests,
// 2026-07) were root-caused, not blanket-tagged: nearly all of them predate
// battery_config_mode (d725059, 2026-06-05), which made Auto — sysparam as
// the PRIMARY source for CCL/DCL/CVL and the temperature window — the
// zero-value default. These tests were written for the pure cfg-formula
// (V2.67-equivalent) behavior and never set an explicit mode, so they now
// need `make_cfg_manual()` / cfg_from_json()'s explicit Manual mode (see
// those helpers) to exercise the logic they actually test. Two further
// cases (v267 case_04/case_06, and the "PackOvervoltStart" edge test) had
// additionally-stale expected values predating the direction-aware OV/UV
// lockout rework (3630bc3, 2026-07-07): OV/UV now block only the direction
// that worsens the fault, not both. All fixes were test-only; see the
// resolve-shouldfail-tests report for the per-test breakdown.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <cstring>

#include "safety/runSafety.h"
#include "safety/filters.h"

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// JSON fixture helpers
// ─────────────────────────────────────────────────────────────────────────────

static json load_fixture(const char* relpath) {
  std::string path = std::string(FIXTURE_DIR) + "/v267/" + relpath;
  std::ifstream f(path);
  INFO("fixture: " << path);
  REQUIRE(f.is_open());
  return json::parse(f);
}

static BmsSystemSnapshot snap_from_json(const json& j) {
  BmsSystemSnapshot s{};
  s.cycle_id              = j["cycle_id"].get<uint32_t>();
  s.produced_ms           = j["produced_ms"].get<uint32_t>();
  s.pack_count_configured = j["pack_count_configured"].get<uint8_t>();
  const auto& packs = j["packs"];
  int n = std::min(static_cast<int>(packs.size()), 16);
  for (int i = 0; i < n; ++i) {
    const auto& p = packs[i];
    BmsPackSnapshot& pk = s.pack[i];
    pk.bms_id         = p["bms_id"].get<uint8_t>();
    pk.online         = p["online"].get<bool>();
    pk.pack_voltage   = p["pack_voltage"].get<float>();
    pk.pack_current   = p["pack_current"].get<float>();
    pk.soc            = static_cast<uint8_t>(p["soc"].get<float>());
    pk.soh            = static_cast<uint8_t>(p["soh"].get<float>());
    pk.rem_ah         = p["rem_ah"].get<float>();
    pk.full_ah        = p["full_ah"].get<float>();
    pk.cell_count     = p["cell_count"].get<uint8_t>();
    const auto& cv = p["cell_v"];
    for (int j2 = 0; j2 < 16 && j2 < static_cast<int>(cv.size()); ++j2)
      pk.cell_v[j2] = cv[j2].get<float>();
    pk.cell_min_v     = p["cell_min_v"].get<float>();
    pk.cell_max_v     = p["cell_max_v"].get<float>();
    pk.cell_avg_v     = p["cell_avg_v"].get<float>();
    pk.cell_drift_v   = p["cell_drift_v"].get<float>();
    pk.temp_max_c     = p["temp_max_c"].get<float>();
    pk.temp_avg_c     = p["temp_avg_c"].get<float>();
    pk.alarm_bits     = p["alarm_bits"].get<uint64_t>();
    pk.last_alarm_ms  = p["last_alarm_ms"].get<uint32_t>();
    pk.sysparam_valid = p["sysparam_valid"].get<bool>();
    pk.last_sysparam_ms  = p["last_sysparam_ms"].get<uint32_t>();
    pk.sys_cell_high_v   = p["sys_cell_high_v"].get<float>();
    pk.sys_module_high_v = p["sys_module_high_v"].get<float>();
    pk.sys_module_under_v  = p["sys_module_under_v"].get<float>();
    pk.sys_charge_max_a    = p["sys_charge_max_a"].get<float>();
    pk.sys_discharge_max_a = p["sys_discharge_max_a"].get<float>();
    if (pk.online) s.pack_count_online++;
  }
  return s;
}

static Config cfg_from_json(const json& j) {
  Config cfg{};
  cfg.bms_count              = j["bms_count"].get<uint8_t>();
  cfg.charge_amps_per_pack   = j["charge_amps_per_pack"].get<float>();
  cfg.discharge_amps_per_pack= j["discharge_amps_per_pack"].get<float>();
  cfg.cvl_voltage            = j["cvl_voltage"].get<float>();
  cfg.safe_pack_volt         = j["safe_pack_volt"].get<float>();
  cfg.safe_cell_volt         = j["safe_cell_volt"].get<float>();
  cfg.safe_cell_drift        = j["safe_cell_drift"].get<float>();
  cfg.charge_temp_min        = j["charge_temp_min"].get<float>();
  cfg.charge_temp_max        = j["charge_temp_max"].get<float>();
  cfg.discharge_temp_min     = j["discharge_temp_min"].get<float>();
  cfg.discharge_temp_max     = j["discharge_temp_max"].get<float>();
  const std::string tm       = j["temp_mode"].get<std::string>();
  cfg.temp_mode = (tm == "Hottest") ? Config::TempMode::Hottest
                                    : Config::TempMode::Average;
  cfg.maint_charge_enabled   = j["maint_charge_enabled"].get<bool>();
  cfg.maint_target_voltage   = j["maint_target_voltage"].get<float>();
  // v267 fixtures predate battery_config_mode (added d725059, 2026-06-05) and
  // carry no such field; they capture V2.67's own always-formula-driven
  // behavior, which Manual mode reproduces exactly (sysparam only as a safe-
  // direction cap, never a primary source). Leaving this at the zero-value
  // default (Auto) would make CCL/DCL/CVL and the temperature factor follow
  // sysparam instead — and since these fixtures don't populate the Auto-only
  // sys_charge_low_t/high_t fields, that reads as an all-zero temp window and
  // forces a spurious full temperature cutoff on every case.
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  return cfg;
}

static PrevSafetyState prev_from_json(const json& j) {
  PrevSafetyState p = safety::make_default_prev();
  const auto& wpo = j["was_pack_online"];
  for (int i = 0; i < 16 && i < static_cast<int>(wpo.size()); ++i)
    p.was_pack_online[i] = wpo[i].get<bool>();
  p.prev_alarm_flags      = static_cast<uint8_t>(j["prev_alarm_flags"].get<int>());
  p.prev_factor_charge    = j["prev_factor_charge"].get<float>();
  p.prev_factor_discharge = j["prev_factor_discharge"].get<float>();
  p.was_packs_online_any  = j["was_packs_online_any"].get<bool>();
  return p;
}

static SafetyState::SafetyEvent event_type_from_str(const std::string& s) {
  using E = SafetyState::SafetyEvent;
  if (s == "BmsWentOffline")       return E::BmsWentOffline;
  if (s == "BmsCameOnline")        return E::BmsCameOnline;
  if (s == "PackOvervoltStart")    return E::PackOvervoltStart;
  if (s == "PackOvervoltClear")    return E::PackOvervoltClear;
  if (s == "CellOvervoltStart")    return E::CellOvervoltStart;
  if (s == "CellOvervoltClear")    return E::CellOvervoltClear;
  if (s == "PackUndervoltStart")   return E::PackUndervoltStart;
  if (s == "PackUndervoltClear")   return E::PackUndervoltClear;
  if (s == "TempChargeStop")       return E::TempChargeStop;
  if (s == "TempChargeResume")     return E::TempChargeResume;
  if (s == "TempDischargeStop")    return E::TempDischargeStop;
  if (s == "TempDischargeResume")  return E::TempDischargeResume;
  if (s == "CellImbalanceStart")   return E::CellImbalanceStart;
  if (s == "CellImbalanceClear")   return E::CellImbalanceClear;
  if (s == "BmsReportedAlarm")     return E::BmsReportedAlarm;
  if (s == "NoPacksOnline")        return E::NoPacksOnline;
  if (s == "PacksOnlineRecovered") return E::PacksOnlineRecovered;
  return E::None;
}

// Run one V2.67 fixture case end-to-end and compare with expected output.
static void run_v267_case(const char* name) {
  std::string prefix = std::string("in/case_") + name;
  auto j_snap = load_fixture((prefix + "_snapshot.json").c_str());
  auto j_cfg  = load_fixture((prefix + "_config.json").c_str());
  auto j_prev = load_fixture((prefix + "_prev.json").c_str());
  auto j_exp  = load_fixture((std::string("out/case_") + name + "_safety.json").c_str());

  BmsSystemSnapshot snap = snap_from_json(j_snap);
  Config            cfg  = cfg_from_json(j_cfg);
  PrevSafetyState   prev = prev_from_json(j_prev);

  SafetyState out;
  safety::runSafety(snap, cfg, prev, snap.produced_ms, out);

  CHECK(out.alarm_flags  == static_cast<uint8_t>(j_exp["alarm_flags"].get<int>()));
  CHECK(out.ccl_amps     == Catch::Approx(j_exp["ccl_amps"].get<float>()).margin(0.05f));
  CHECK(out.dcl_amps     == Catch::Approx(j_exp["dcl_amps"].get<float>()).margin(0.05f));
  CHECK(out.cvl_volts    == Catch::Approx(j_exp["cvl_volts"].get<float>()).margin(0.01f));
  CHECK(out.packs_online == static_cast<uint8_t>(j_exp["packs_online"].get<int>()));
  CHECK(std::string(out.sys_message) == j_exp["sys_message"].get<std::string>());
  CHECK(static_cast<int>(out.event_count) == j_exp["event_count"].get<int>());

  const auto& j_events = j_exp["events"];
  int n_ev = std::min(static_cast<int>(out.event_count),
                      static_cast<int>(j_events.size()));
  for (int i = 0; i < n_ev; ++i) {
    auto exp_type = event_type_from_str(j_events[i]["type"].get<std::string>());
    auto exp_id   = static_cast<uint8_t>(j_events[i]["bms_id"].get<int>());
    auto exp_bits = j_events[i]["alarm_bits"].get<uint64_t>();
    CHECK(out.events[i].type      == exp_type);
    CHECK(out.events[i].bms_id   == exp_id);
    CHECK(out.events[i].alarm_bits == exp_bits);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Synthetic helpers
// ─────────────────────────────────────────────────────────────────────────────

static BmsPackSnapshot make_pack(uint8_t id,
                                  bool    online     = true,
                                  float   voltage    = 50.1f,
                                  float   current    = 0.0f,
                                  uint8_t soc        = 50,
                                  float   cell_max   = 3.341f,
                                  float   cell_drift = 0.002f,
                                  float   temp_max   = 25.1f,
                                  float   temp_avg   = 25.0f) {
  BmsPackSnapshot p{};
  p.bms_id      = id;
  p.online      = online;
  p.pack_voltage = voltage;
  p.pack_current = current;
  p.soc         = soc;
  p.soh         = 100;
  p.rem_ah      = 100.0f;
  p.full_ah     = 100.0f;
  p.cell_count  = 15;
  for (int i = 0; i < 16; ++i) p.cell_v[i] = 3.340f;
  p.cell_min_v  = cell_max - cell_drift;
  p.cell_max_v  = cell_max;
  p.cell_avg_v  = 3.340f;
  p.cell_drift_v= cell_drift;
  p.temp_max_c  = temp_max;
  p.temp_avg_c  = temp_avg;
  p.sysparam_valid  = true;
  p.last_sysparam_ms = 0;
  p.sys_cell_high_v   = 3.65f;
  p.sys_module_high_v = 58.4f;
  p.sys_module_under_v= 38.0f;
  p.sys_charge_max_a  = 0.0f;
  p.sys_discharge_max_a = 0.0f;
  return p;
}

static Config make_cfg() {
  Config cfg{};
  cfg.bms_count               = 4;
  cfg.charge_amps_per_pack    = 30.0f;
  cfg.discharge_amps_per_pack = 30.0f;
  cfg.cvl_voltage             = 53.5f;
  cfg.safe_pack_volt          = 56.25f;
  cfg.safe_cell_volt          = 3.55f;
  cfg.safe_cell_drift         = 0.020f;
  cfg.charge_temp_min         = 5.0f;
  cfg.charge_temp_max         = 50.0f;
  cfg.discharge_temp_min      = -20.0f;
  cfg.discharge_temp_max      = 60.0f;
  cfg.temp_mode               = Config::TempMode::Hottest;
  cfg.maint_charge_enabled    = false;
  cfg.maint_target_voltage    = 52.0f;
  return cfg;
}

// Manual mode: CCL/DCL/CVL and OVP thresholds come directly from cfg (V2.67-
// equivalent formula), with sysparam only ever applied as a safe-direction
// cap. Auto (mode 0, the zero-initialised default) instead makes sysparam the
// PRIMARY source once a pack has valid sysparam, including the charge/
// discharge temperature window (sys_charge_low_t/high_t etc.) — fields the
// synthetic packs below and the v267 JSON fixtures never populate, so they
// read as an all-zero window and calc_factor() sees every positive temp as
// "above max". Tests that exist to validate the cfg-driven formula (factor
// bands, SoC taper, V2.67 fixture parity, OV cap) must run in Manual mode to
// avoid that trap; see cfg_from_json() for the fixture-side equivalent.
static Config make_cfg_manual() {
  Config cfg = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  return cfg;
}

static BmsSystemSnapshot make_system(int n_packs,
                                      float voltage   = 50.1f,
                                      float current   = 0.0f,
                                      uint8_t soc     = 50,
                                      float cell_max  = 3.341f,
                                      float cell_drift= 0.002f,
                                      float temp_max  = 25.1f) {
  BmsSystemSnapshot s{};
  s.cycle_id              = 1;
  s.produced_ms           = 10000;
  s.pack_count_configured = static_cast<uint8_t>(n_packs);
  for (int i = 0; i < n_packs; ++i) {
    s.pack[i] = make_pack(static_cast<uint8_t>(i), true, voltage, current,
                           soc, cell_max, cell_drift, temp_max, temp_max - 0.1f);
    s.pack_count_online++;
  }
  return s;
}

static PrevSafetyState make_prev_all_online(int n = 16) {
  PrevSafetyState p = safety::make_default_prev();
  for (int i = 0; i < n; ++i) p.was_pack_online[i] = true;
  p.was_packs_online_any = true;
  return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// V2.67 regression tests [v267]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("v267 case_01 idle 4 packs",          "[v267]") { run_v267_case("01_idle_4packs"); }
TEST_CASE("v267 case_02 charge 4 packs",        "[v267]") { run_v267_case("02_charge_4packs"); }
TEST_CASE("v267 case_03 discharge 4 packs",     "[v267]") { run_v267_case("03_discharge_4packs"); }
TEST_CASE("v267 case_04 cell overvolt pack3",   "[v267]") { run_v267_case("04_cell_overvolt_pack3"); }
TEST_CASE("v267 case_05 temp charge stop",      "[v267]") { run_v267_case("05_temp_charge_stop"); }
TEST_CASE("v267 case_06 bms reported alarm",    "[v267]") { run_v267_case("06_bms_reported_alarm_pack1"); }
TEST_CASE("v267 case_07 all offline",           "[v267]") { run_v267_case("07_all_offline"); }
TEST_CASE("v267 case_08 recovering online",     "[v267]") { run_v267_case("08_recovering_one_back_online"); }

// ─────────────────────────────────────────────────────────────────────────────
// calc_factor temperature bands [factor]
// Tested indirectly via runSafety() with 1 online pack (t_check_val = pack temp).
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("factor: charge below minimum (2 C) → ccl=0, alarm 0x08", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 2.0f);
  auto cfg  = make_cfg_manual();  // charge_temp_min = 5
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(0.0f));
  REQUIRE(out.dcl_amps == Catch::Approx(30.0f));  // discharge range -20..60 → normal
  REQUIRE(out.alarm_flags & 0x08);
}

TEST_CASE("factor: charge in low soft zone (7 C) → ccl = count*30*0.2", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 7.0f);
  auto cfg  = make_cfg_manual();  // charge_temp_min=5, soft_zone=5 → [5,10) returns 0.2
  auto prev = make_prev_all_online(1);
  prev.prev_factor_charge = 1.0f;  // transition triggers TempChargeStop event
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(1 * 30.0f * 0.2f).margin(0.01f));
  REQUIRE(out.dcl_amps == Catch::Approx(1 * 30.0f * 1.0f).margin(0.01f));
  REQUIRE(out.factor_charge == Catch::Approx(0.2f));
  REQUIRE_FALSE(out.alarm_flags & 0x08);  // temp flag only at factor==0
}

TEST_CASE("factor: charge in normal range (25 C) → ccl = count*30*1.0", "[factor]") {
  auto snap = make_system(4, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(4);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(4 * 30.0f));
  REQUIRE(out.dcl_amps == Catch::Approx(4 * 30.0f));
  REQUIRE(out.factor_charge    == Catch::Approx(1.0f));
  REQUIRE(out.factor_discharge == Catch::Approx(1.0f));
  REQUIRE(out.alarm_flags == 0x00);
}

TEST_CASE("factor: charge in high soft zone (47 C) → ccl = count*30*0.5", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 47.0f);
  auto cfg  = make_cfg_manual();  // charge_temp_max=50, soft_zone=5 → (45,50] returns 0.5
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.factor_charge == Catch::Approx(0.5f));
  REQUIRE(out.ccl_amps == Catch::Approx(1 * 30.0f * 0.5f).margin(0.01f));
}

TEST_CASE("factor: charge above maximum (55 C) → ccl=0, alarm 0x08", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 55.0f);
  auto cfg  = make_cfg();
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(0.0f));
  REQUIRE(out.alarm_flags & 0x08);
}

TEST_CASE("factor: discharge below minimum (-25 C) → dcl=0", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, -25.0f);
  auto cfg  = make_cfg();  // discharge_temp_min = -20
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.dcl_amps == Catch::Approx(0.0f));
}

TEST_CASE("factor: discharge in high soft zone (57 C) → dcl = count*30*0.5", "[factor]") {
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 57.0f);
  auto cfg  = make_cfg_manual();  // discharge_temp_max=60, soft_zone=5 → (55,60] returns 0.5
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.factor_discharge == Catch::Approx(0.5f));
  REQUIRE(out.dcl_amps == Catch::Approx(1 * 30.0f * 0.5f).margin(0.01f));
}

TEST_CASE("factor: t_check_val uses last online pack (V2.67 bug preserved)", "[factor]") {
  // Pack 0: 25 C (normal), Pack 1: 2 C (below charge min) → last pack wins.
  // Both online; t_check_val = pack[1].temp_max_c = 2.0 → charge cut.
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 10000;
  snap.pack_count_configured = 2;
  snap.pack_count_online     = 2;
  snap.pack[0] = make_pack(0, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 24.9f);
  snap.pack[1] = make_pack(1, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 2.0f, 1.9f);

  auto cfg  = make_cfg();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  // t_check_val = 2.0 (pack 1 last) → factor_charge = 0.0
  REQUIRE(out.factor_charge == Catch::Approx(0.0f));
  REQUIRE(out.ccl_amps == Catch::Approx(0.0f));
}

TEST_CASE("factor: average mode uses temp_avg_c of last pack", "[factor]") {
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 10000;
  snap.pack_count_configured = 2;
  snap.pack_count_online     = 2;
  // pack0: temp_max=25 (normal), temp_avg=2 (below charge min)
  // pack1: temp_max=3 (below charge min), temp_avg=25 (normal)
  snap.pack[0] = make_pack(0, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 2.0f);
  snap.pack[1] = make_pack(1, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 3.0f, 25.0f);

  auto cfg = make_cfg_manual();
  cfg.temp_mode = Config::TempMode::Average;
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  // Average mode: t_check_val = pack1.temp_avg_c = 25.0 → factor_charge = 1.0
  REQUIRE(out.factor_charge == Catch::Approx(1.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// filter_alarm_bits [filters]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("filters: UV bit suppressed when pack_v above sanity cap", "[filters]") {
  // cell_count=15, cap=max(40,min(57,15*3.2))=48.0; pack_v=50.0 > 48.0 → suppress
  uint64_t raw = (1ULL << 12);
  auto out = safety::filters::filter_alarm_bits(raw, 50.0f, 25.0f, 15,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out == 0u);
}

TEST_CASE("filters: UV bit kept when pack_v below sanity cap", "[filters]") {
  uint64_t raw = (1ULL << 12);
  // pack_v=30.0 < 48.0 → UV kept
  auto out = safety::filters::filter_alarm_bits(raw, 30.0f, 25.0f, 15,
                                                  50.0f, 2.0f, 3.65f, 58.4f);
  REQUIRE(out == (1ULL << 12));
}

TEST_CASE("filters: OV bit suppressed when both cell_v and pack_v below floors", "[filters]") {
  // OV bit 0; cell_high_v=3.65 → floor=3.53 clamped to 3.50; max_cell_v=3.341<3.50
  // module_high_v=58.4 → pack_floor=57.6; pack_v=50.0 < 57.6 → suppress
  uint64_t raw = (1ULL << 0);
  auto out = safety::filters::filter_alarm_bits(raw, 50.0f, 25.0f, 15,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out == 0u);
}

TEST_CASE("filters: OV bit kept when cell_v above cell floor", "[filters]") {
  // cell_high_v=3.65 → floor=3.50; max_cell_v=3.60 >= 3.50 → OV not suppressed
  uint64_t raw = (1ULL << 0);
  auto out = safety::filters::filter_alarm_bits(raw, 50.0f, 25.0f, 15,
                                                  50.0f, 3.60f, 3.65f, 58.4f);
  REQUIRE(out & (1ULL << 0));
}

TEST_CASE("filters: temp bits suppressed when temp well below limit", "[filters]") {
  // temp_limit=50 → guard=max(47,15)=47; max_temp=25.0 < 47 → suppress
  uint64_t raw = (1ULL << 5) | (1ULL << 6);
  auto out = safety::filters::filter_alarm_bits(raw, 50.0f, 25.0f, 15,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out == 0u);
}

TEST_CASE("filters: temp bits kept when max_temp at or above guard", "[filters]") {
  // temp_limit=50 → guard=47; max_temp=48 >= 47 → kept
  uint64_t raw = (1ULL << 5);
  auto out = safety::filters::filter_alarm_bits(raw, 50.0f, 48.0f, 15,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out & (1ULL << 5));
}

TEST_CASE("filters: non-critical bits stripped by CRITICAL_ALARM_MASK", "[filters]") {
  // Bit 1 is not in CRITICAL_ALARM_MASK → always stripped
  uint64_t raw = (1ULL << 1);
  auto out = safety::filters::filter_alarm_bits(raw, 30.0f, 25.0f, 15,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out == 0u);
}

TEST_CASE("filters: UV suppressed when cell_count unknown (0)", "[filters]") {
  uint64_t raw = (1ULL << 12);
  auto out = safety::filters::filter_alarm_bits(raw, 30.0f, 25.0f, 0,
                                                  50.0f, 3.341f, 3.65f, 58.4f);
  REQUIRE(out == 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// Edge detection across two cycles [edge]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("edge: BmsWentOffline event on pack dropout", "[edge]") {
  // Cycle 1: pack 0 online → update_prev_state
  // Cycle 2: pack 0 offline → BmsWentOffline(bms_id=0) emitted
  auto cfg = make_cfg();

  BmsSystemSnapshot snap1 = make_system(1);
  PrevSafetyState prev = safety::make_default_prev();
  SafetyState out1;
  safety::runSafety(snap1, cfg, prev, 10000, out1);
  safety::update_prev_state(out1, snap1, prev);

  BmsSystemSnapshot snap2 = snap1;
  snap2.pack[0].online = false;
  snap2.pack_count_online = 0;
  SafetyState out2;
  safety::runSafety(snap2, cfg, prev, 20000, out2);

  REQUIRE(out2.event_count >= 1);
  REQUIRE(out2.events[0].type   == SafetyState::BmsWentOffline);
  REQUIRE(out2.events[0].bms_id == 0);
  REQUIRE(out2.alarm_flags & 0x80);
}

TEST_CASE("edge: BmsCameOnline event on pack recovery", "[edge]") {
  auto cfg  = make_cfg();
  PrevSafetyState prev = safety::make_default_prev();  // all offline

  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 10000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true);

  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  // First event should be BmsCameOnline
  REQUIRE(out.event_count >= 1);
  bool found = false;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::BmsCameOnline && out.events[i].bms_id == 0)
      found = true;
  }
  REQUIRE(found);
}

TEST_CASE("edge: NoPacksOnline event when last pack goes offline", "[edge]") {
  auto cfg = make_cfg();
  auto prev = make_prev_all_online(1);
  prev.was_packs_online_any = true;

  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 10000;
  snap.pack_count_configured = 1;
  snap.pack[0] = make_pack(0, false);

  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  bool found_no_packs = false;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::NoPacksOnline)
      found_no_packs = true;
  }
  REQUIRE(found_no_packs);
}

TEST_CASE("edge: PacksOnlineRecovered after full outage", "[edge]") {
  auto cfg  = make_cfg();
  PrevSafetyState prev = safety::make_default_prev();
  prev.was_packs_online_any = false;

  BmsSystemSnapshot snap = make_system(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  bool found = false;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::PacksOnlineRecovered)
      found = true;
  }
  REQUIRE(found);
}

TEST_CASE("edge: TempChargeStop emits on factor transition", "[edge]") {
  auto cfg = make_cfg();
  auto prev = make_prev_all_online(1);
  prev.prev_factor_charge = 1.0f;

  // temp=2.0 → factor_charge = 0.0 (below min=5)
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 2.0f);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  bool found = false;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::TempChargeStop)
      found = true;
  }
  REQUIRE(found);
}

TEST_CASE("edge: TempChargeResume emits when factor returns to 1.0", "[edge]") {
  auto cfg = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  prev.prev_factor_charge = 0.0f;  // was throttled

  // temp=25.0 → factor_charge = 1.0 → resume
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  bool found = false;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::TempChargeResume)
      found = true;
  }
  REQUIRE(found);
}

TEST_CASE("edge: PackOvervoltStart emitted per-pack and system-level", "[edge]") {
  // cell_max=3.60 > safe_cell_volt=3.55 → OV alarm
  auto cfg = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  prev.prev_alarm_flags = 0;  // OV bit was clear

  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.60f, 0.005f, 25.0f);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  int ov_start_count = 0;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::PackOvervoltStart)
      ov_start_count++;
  }
  // Per-pack (bms_id=0) + system-level (bms_id=255) = 2
  REQUIRE(ov_start_count == 2);
  REQUIRE(out.alarm_flags & 0x02);
  // Direction-aware lockout (V3.2, see [[project_direction_aware_lockout]]):
  // OV blocks charge only — discharging LOWERS cell voltage and moves away
  // from the fault, so it stays available. This predates that rework, which
  // originally zeroed both directions; updated to match the now-intentional
  // behavior rather than the old blanket lock-to-zero.
  REQUIRE(out.ccl_amps == Catch::Approx(0.0f));
  REQUIRE(out.dcl_amps == Catch::Approx(1 * 30.0f));
}

TEST_CASE("edge: persistent OV re-fires per-pack event but not system-level", "[edge]") {
  auto cfg = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  prev.prev_alarm_flags = 0x02;  // OV was already flagged last cycle

  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.60f, 0.005f, 25.0f);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  int ov_start_count = 0;
  for (uint8_t i = 0; i < out.event_count; ++i) {
    if (out.events[i].type == SafetyState::PackOvervoltStart)
      ov_start_count++;
  }
  // Per-pack still fires (V2.67: no prev check), system-level does NOT (bit unchanged)
  REQUIRE(ov_start_count == 1);
  REQUIRE(out.events[0].bms_id == 0);  // per-pack, not system-level
}

// ─────────────────────────────────────────────────────────────────────────────
// SOC-based charge taper [soc]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("soc: below 99 → no taper", "[soc]") {
  auto snap = make_system(2, 50.1f, 0.0f, 98);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(2 * 30.0f));
}

TEST_CASE("soc: at 99 → ccl = count * 2", "[soc]") {
  auto snap = make_system(3, 50.1f, 0.0f, 99);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(3);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(3 * 2.0f));
  REQUIRE(out.dcl_amps == Catch::Approx(3 * 30.0f));  // no dcl taper
}

TEST_CASE("soc: at 100 → ccl = 0", "[soc]") {
  auto snap = make_system(2, 50.1f, 0.0f, 100);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(0.0f));
  REQUIRE(out.dcl_amps == Catch::Approx(2 * 30.0f));  // discharge unaffected
}

TEST_CASE("soc: taper skipped when maint_charge_enabled", "[soc]") {
  auto snap = make_system(1, 50.1f, 0.0f, 100);
  auto cfg  = make_cfg_manual();
  cfg.maint_charge_enabled = true;
  cfg.maint_target_voltage = 55.0f;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  // maint mode: no soc-based taper; volt < maint_target → full charge
  REQUIRE(out.ccl_amps == Catch::Approx(1 * 30.0f));
}

// ─────────────────────────────────────────────────────────────────────────────
// Lock-to-zero rule [lock]
// ─────────────────────────────────────────────────────────────────────────────

// These use Manual mode so CCL and DCL both have a predictable nonzero baseline
// (count × 30 A × temp_factor) — the direction that STAYS enabled is verified to
// carry its normal limit, not merely a coincidental zero. (In Auto mode with the
// synthetic packs' sys_*_max_a = 0, the sysparam branch yields 0 for both
// directions regardless of any lock, which would mask the direction split.)

TEST_CASE("lock: OV flag (0x02) → blocks CHARGE only, discharge stays available", "[lock]") {
  auto snap = make_system(2, 57.0f, 0.0f, 50, 3.341f, 0.002f, 25.0f);  // pack_v > 56.25
  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.alarm_flags & 0x02);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));           // charging worsens OV → blocked
  CHECK(out.dcl_amps == Catch::Approx(2 * 30.0f));      // discharging helps → available
  CHECK((out.lockout_flags & 0x01));                    // charge lockout flagged
  CHECK_FALSE((out.lockout_flags & 0x02));              // discharge NOT flagged
}

TEST_CASE("lock: UV flag (0x10) → blocks DISCHARGE only, charge stays available", "[lock]") {
  // Pack with fresh UV alarm (bit 12) and pack_v=30 below sanity cap
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 150000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true, 30.0f, 0.0f, 30, 2.0f, 0.10f, 25.0f, 25.0f);
  snap.pack[0].alarm_bits    = (1ULL << 12);
  snap.pack[0].last_alarm_ms = 120000;  // 30s ago → fresh

  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);

  REQUIRE(out.alarm_flags & 0x10);
  CHECK(out.dcl_amps == Catch::Approx(0.0f));           // discharging worsens UV → blocked
  CHECK(out.ccl_amps == Catch::Approx(1 * 30.0f));      // charging helps → available
  CHECK((out.lockout_flags & 0x02));
  CHECK_FALSE((out.lockout_flags & 0x01));
}

TEST_CASE("lock: BMS critical alarm (0x40) → blocks BOTH (non-directional)", "[lock]") {
  // Pack with fresh non-UV, non-OC critical alarm (bit 11, short-circuit
  // protect) — this bit is not decoded per-direction, so the conservative
  // response stays "stop everything". (Bit 4 used to stand in for "generic
  // critical" here, but as of V3.3 bit 4 is DISCHARGE_OVER_CURRENT1_PROTECT
  // and is direction-aware — see the charge/discharge over-current tests
  // below — so this test now uses a bit that genuinely has no direction.)
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 150000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 25.0f);
  snap.pack[0].alarm_bits    = (1ULL << 11);  // SHORT_CIRCUIT_PROTECT: critical, not UV/OV/OC
  snap.pack[0].last_alarm_ms = 120000;

  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);

  REQUIRE(out.alarm_flags & 0x40);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(0.0f));
  CHECK(out.lockout_flags == 0x03);                     // both directions flagged
}

TEST_CASE("lock: charge over-current (0x01) → blocks CHARGE only", "[lock]") {
  // Pack with fresh charge over-current protect bit (bit 2). V3.3 direction-
  // aware fix: this must block charge only, mirroring OV/UV, not fall into
  // the old undifferentiated "BMS critical" both-directions bucket.
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 150000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 25.0f);
  snap.pack[0].alarm_bits    = (1ULL << 2);  // CHARGE_OVER_CURRENT_PROTECT
  snap.pack[0].last_alarm_ms = 120000;

  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);

  REQUIRE(out.alarm_flags & 0x01);
  CHECK_FALSE(out.alarm_flags & 0x40);                  // not folded into the generic bucket
  CHECK(out.ccl_amps == Catch::Approx(0.0f));           // charging worsens charge-OC → blocked
  CHECK(out.dcl_amps == Catch::Approx(1 * 30.0f));      // discharging unaffected → available
  CHECK((out.lockout_flags & 0x01));
  CHECK_FALSE((out.lockout_flags & 0x02));
}

TEST_CASE("lock: discharge over-current (0x04) → blocks DISCHARGE only", "[lock]") {
  // Pack with fresh discharge over-current protect bit (bit 4). V3.3
  // direction-aware fix: this must block discharge only.
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 150000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 25.0f);
  snap.pack[0].alarm_bits    = (1ULL << 4);  // DISCHARGE_OVER_CURRENT1_PROTECT
  snap.pack[0].last_alarm_ms = 120000;

  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);

  REQUIRE(out.alarm_flags & 0x04);
  CHECK_FALSE(out.alarm_flags & 0x40);                  // not folded into the generic bucket
  CHECK(out.dcl_amps == Catch::Approx(0.0f));           // discharging worsens discharge-OC → blocked
  CHECK(out.ccl_amps == Catch::Approx(1 * 30.0f));      // charging unaffected → available
  CHECK((out.lockout_flags & 0x02));
  CHECK_FALSE((out.lockout_flags & 0x01));
}

TEST_CASE("lock: charge-OC and under-voltage combined on one pack → both classified independently", "[lock]") {
  // Regression for the if/else -> independent-classification rewrite: a single
  // BMS alarm response can report multiple critical bits at once. Both must be
  // classified, not just the first match (UV) swallowing the OC bit.
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 1;
  snap.produced_ms           = 150000;
  snap.pack_count_configured = 1;
  snap.pack_count_online     = 1;
  snap.pack[0] = make_pack(0, true, 30.0f, 0.0f, 30, 2.0f, 0.10f, 25.0f, 25.0f);
  snap.pack[0].alarm_bits    = (1ULL << 2) | (1ULL << 12);  // charge-OC + cell UV
  snap.pack[0].last_alarm_ms = 120000;

  auto cfg  = make_cfg();
  cfg.battery_config_mode = Config::BatteryConfigMode::Manual;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);

  REQUIRE(out.alarm_flags & 0x01);   // charge-OC classified
  REQUIRE(out.alarm_flags & 0x10);   // UV also classified, not swallowed
  CHECK_FALSE(out.alarm_flags & 0x40);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));           // OV-path: charge-OC blocks charge
  CHECK(out.dcl_amps == Catch::Approx(0.0f));           // UV blocks discharge
  CHECK(out.lockout_flags == 0x03);
}

// ─────────────────────────────────────────────────────────────────────────────
// Direction-aware lockout — comprehensive paired coverage [lockdir]
// Every condition checks BOTH the direction it should block AND the direction it
// must leave available. Manual mode → predictable count×30 A baseline.
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("lockdir: baseline healthy → both directions full, no lockout", "[lockdir]") {
  auto snap = make_system(2, 50.1f, 0.0f, 50, 3.341f, 0.002f, 25.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  CHECK(out.ccl_amps == Catch::Approx(2 * 30.0f));
  CHECK(out.dcl_amps == Catch::Approx(2 * 30.0f));
  CHECK(out.lockout_flags == 0x00);
}

TEST_CASE("lockdir: cell over-voltage → charge 0, discharge full", "[lockdir]") {
  // cell_max 3.60 > Manual cell cap (min(safe_cell_volt 3.55, 3.65) = 3.55)
  auto snap = make_system(2, 50.1f, 0.0f, 50, 3.60f, 0.002f, 25.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.alarm_flags & 0x02);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(2 * 30.0f));
  CHECK((out.lockout_flags & 0x01));
  CHECK_FALSE((out.lockout_flags & 0x02));
}

TEST_CASE("lockdir: charge over-temperature → charge 0, discharge full", "[lockdir]") {
  // 55 C: above charge_temp_max (50) but below discharge_temp_max (60)
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 55.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.factor_charge == Catch::Approx(0.0f));
  REQUIRE(out.factor_discharge == Catch::Approx(1.0f));
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(1 * 30.0f));
  CHECK((out.lockout_flags & 0x01));
  CHECK_FALSE((out.lockout_flags & 0x02));
  CHECK((out.temp_alarm & 0x02));                       // hot direction
}

TEST_CASE("lockdir: discharge over-temperature → discharge 0, charge full", "[lockdir]") {
  // Isolate discharge-over-temp: raise charge_temp_max above discharge_temp_max
  // so 60 C is in-range for charge but a cutoff for discharge.
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 60.0f);
  auto cfg  = make_cfg_manual();
  cfg.charge_temp_max    = 70.0f;
  cfg.discharge_temp_max = 55.0f;
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.factor_charge == Catch::Approx(1.0f));
  REQUIRE(out.factor_discharge == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(0.0f));
  CHECK(out.ccl_amps == Catch::Approx(1 * 30.0f));
  CHECK((out.lockout_flags & 0x02));
  CHECK_FALSE((out.lockout_flags & 0x01));
}

TEST_CASE("lockdir: charge low-temperature (cold) → charge 0, discharge full", "[lockdir]") {
  // 0 C: below charge_temp_min (5) but above discharge_temp_min (-20).
  // LiFePO4 cold-charge plating protection; discharging cold is fine.
  auto snap = make_system(1, 50.1f, 0.0f, 50, 3.341f, 0.002f, 0.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.factor_charge == Catch::Approx(0.0f));
  REQUIRE(out.factor_discharge == Catch::Approx(1.0f));
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(1 * 30.0f));
  CHECK((out.lockout_flags & 0x01));
  CHECK_FALSE((out.lockout_flags & 0x02));
  CHECK((out.temp_alarm & 0x01));                       // cold direction
}

TEST_CASE("lockdir: cold charge-cutoff at 99% SoC → charge stays 0 (taper cannot re-open)", "[lockdir]") {
  // Regression for the taper-override bug: at >=99% SoC the SoC taper sets
  // safe_chg = count*2 A; the protection block must run AFTER and keep it 0, so a
  // cold pack is never trickle-charged into plating.
  auto snap = make_system(1, 50.1f, 0.0f, 99, 3.341f, 0.002f, 0.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));           // NOT the 2 A taper trickle
  CHECK(out.dcl_amps == Catch::Approx(1 * 30.0f));
  CHECK((out.lockout_flags & 0x01));
}

TEST_CASE("lockdir: SoC 100% taper → charge 0 but NOT flagged as a lockout", "[lockdir]") {
  // Battery-full taper zeroes charge, but it is not a protection fault: no
  // lockout flag, so no banner/alert. Discharge unaffected.
  auto snap = make_system(2, 50.1f, 0.0f, 100, 3.341f, 0.002f, 25.0f);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(2 * 30.0f));
  CHECK(out.lockout_flags == 0x00);                     // taper is not a lockout
  CHECK(out.alarm_flags == 0x00);
}

TEST_CASE("lockdir: no packs online → both blocked, both flagged", "[lockdir]") {
  BmsSystemSnapshot snap{};
  snap.cycle_id = 1; snap.produced_ms = 10000;
  snap.pack_count_configured = 2;   // configured but none online
  snap.pack[0] = make_pack(0, false);
  snap.pack[1] = make_pack(1, false);
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.alarm_flags & 0x80);
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(0.0f));
  CHECK(out.lockout_flags == 0x03);
}

TEST_CASE("lockdir: combination OV + UV → both directions blocked, neither masks the other", "[lockdir]") {
  // Two mismatched packs: pack0 over-voltage (blocks charge), pack1 carrying a
  // fresh UV alarm at low voltage (blocks discharge). Each condition must apply
  // to its own direction. (OV and UV cannot coexist on ONE pack — the UV alarm
  // filter correctly suppresses UV at a high pack voltage — so they live on
  // separate packs here.)
  BmsSystemSnapshot snap{};
  snap.cycle_id = 1; snap.produced_ms = 150000;
  snap.pack_count_configured = 2;
  snap.pack_count_online     = 2;
  snap.pack[0] = make_pack(0, true, 57.0f, 0.0f, 50, 3.341f, 0.002f, 25.0f, 25.0f);  // pack OV
  snap.pack[1] = make_pack(1, true, 30.0f, 0.0f, 30, 2.0f,   0.10f,  25.0f, 25.0f);  // low V
  snap.pack[1].alarm_bits    = (1ULL << 12);   // UV alarm bit
  snap.pack[1].last_alarm_ms = 120000;
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 150000, out);
  REQUIRE(out.alarm_flags & 0x02);   // over-voltage (pack0)
  REQUIRE(out.alarm_flags & 0x10);   // under-voltage (pack1)
  CHECK(out.ccl_amps == Catch::Approx(0.0f));
  CHECK(out.dcl_amps == Catch::Approx(0.0f));
  CHECK(out.lockout_flags == 0x03);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sysparam protocol caps [proto]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("proto: ccl capped by sys_charge_max_a", "[proto]") {
  // sys_charge_max_a=20.0 per pack → proto_ccl_cap=40 < cfg 4*30=120.
  // Auto mode (default): sysparam is the PRIMARY CCL source, which is
  // specifically what this test exercises, so it stays in Auto rather than
  // switching to Manual like the cfg-formula tests. Auto also aggregates the
  // sysparam charge/discharge temperature window (sys_charge_low_t/high_t
  // etc.) for calc_factor(); make_pack()'s defaults leave those at 0/0 (an
  // always-cutoff window), so give both packs a real wide-open window here
  // to isolate the CCL-cap behavior under test from that unrelated factor.
  auto snap = make_system(2);
  snap.pack[0].sys_charge_max_a = 20.0f;
  snap.pack[1].sys_charge_max_a = 20.0f;
  for (int i = 0; i < 2; ++i) {
    snap.pack[i].sys_charge_low_t     = -20.0f;
    snap.pack[i].sys_charge_high_t    =  60.0f;
    snap.pack[i].sys_discharge_low_t  = -20.0f;
    snap.pack[i].sys_discharge_high_t =  60.0f;
  }
  auto cfg  = make_cfg();
  auto prev = make_prev_all_online(2);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(40.0f));  // capped at 20+20
}

TEST_CASE("proto: stale sysparam (>300s) ignored", "[proto]") {
  // Manual mode: this predates battery_config_mode (like the cfg-formula
  // tests above) — its intent is "a stale sysparam CCL cap must be ignored,
  // falling back to the configured amps," which Manual reproduces directly.
  // (Auto mode's own path to the same cfg-fallback formula is via
  // proto_count==0, identical to Manual here — but Auto mode also aggregates
  // the sysparam temperature window with a freshness gap: it keys off
  // p.sysparam_valid only, not the same 300s sp_fresh check the CCL/DCL/CVL
  // caps use, so a sysparam_valid-but-stale pack still feeds calc_factor().
  // That inconsistency is real but orthogonal to what this test checks;
  // flagged separately rather than folded in here.)
  auto snap = make_system(1);
  snap.pack[0].sys_charge_max_a = 5.0f;
  snap.pack[0].last_sysparam_ms = 0;
  auto cfg  = make_cfg_manual();
  auto prev = make_prev_all_online(1);
  // now_ms=400000 → 400000 - 0 = 400000 > 300000 → stale
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 400000, out);
  REQUIRE(out.ccl_amps == Catch::Approx(1 * 30.0f));  // cfg ccl used
}

TEST_CASE("proto: cvl capped by sys_module_high_v", "[proto]") {
  // sys_module_high_v=52.0 → cvl_cap = 52.0-0.2=51.8; cfg cvl=53.5 → use 51.8
  // chem_cap = 15*3.6=54.0; safe-0.20=56.05; cfg=53.5 → min(51.8,54,56.05,53.5)=51.8
  auto snap = make_system(1);
  snap.pack[0].sys_module_high_v = 52.0f;
  auto cfg  = make_cfg();
  auto prev = make_prev_all_online(1);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);
  REQUIRE(out.cvl_volts == Catch::Approx(51.8f).margin(0.01f));
}

// ─────────────────────────────────────────────────────────────────────────────
// update_prev_state [update]
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("update: was_pack_online reflects current snapshot online flags", "[update]") {
  auto snap = make_system(4);
  snap.pack[2].online = false;
  snap.pack_count_online = 3;

  SafetyState dummy{};
  dummy.packs_online = 3;
  dummy.alarm_flags  = 0;
  dummy.factor_charge = dummy.factor_discharge = 1.0f;

  PrevSafetyState prev = safety::make_default_prev();
  safety::update_prev_state(dummy, snap, prev);

  REQUIRE(prev.was_pack_online[0]);
  REQUIRE(prev.was_pack_online[1]);
  REQUIRE_FALSE(prev.was_pack_online[2]);
  REQUIRE(prev.was_pack_online[3]);
}

TEST_CASE("update: was_packs_online_any set from packs_online", "[update]") {
  auto snap = make_system(2);
  SafetyState s{};
  s.packs_online     = 0;
  s.factor_charge    = 1.0f;
  s.factor_discharge = 1.0f;

  PrevSafetyState prev = safety::make_default_prev();
  safety::update_prev_state(s, snap, prev);
  REQUIRE_FALSE(prev.was_packs_online_any);

  s.packs_online = 2;
  safety::update_prev_state(s, snap, prev);
  REQUIRE(prev.was_packs_online_any);
}

TEST_CASE("update: prev_alarm_flags carries alarm_flags", "[update]") {
  auto snap = make_system(1);
  SafetyState s{};
  s.packs_online     = 1;
  s.alarm_flags      = 0x12;
  s.factor_charge    = 1.0f;
  s.factor_discharge = 1.0f;

  PrevSafetyState prev = safety::make_default_prev();
  safety::update_prev_state(s, snap, prev);
  REQUIRE(prev.prev_alarm_flags == 0x12);
}

TEST_CASE("update: prev_factor_charge/discharge carry through", "[update]") {
  auto snap = make_system(1);
  SafetyState s{};
  s.packs_online     = 1;
  s.factor_charge    = 0.2f;
  s.factor_discharge = 0.5f;

  PrevSafetyState prev = safety::make_default_prev();
  safety::update_prev_state(s, snap, prev);
  REQUIRE(prev.prev_factor_charge    == Catch::Approx(0.2f));
  REQUIRE(prev.prev_factor_discharge == Catch::Approx(0.5f));
}

// ─────────────────────────────────────────────────────────────────────────────
// All packs offline — zero output path
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("no-packs: all outputs zero, 0x80 flag, cvl=0", "[nopacks]") {
  BmsSystemSnapshot snap{};
  snap.cycle_id              = 5;
  snap.produced_ms           = 50000;
  snap.pack_count_configured = 4;
  for (int i = 0; i < 4; ++i)
    snap.pack[i] = make_pack(static_cast<uint8_t>(i), false);

  auto cfg  = make_cfg();
  PrevSafetyState prev = safety::make_default_prev();
  // packs were all offline last cycle too
  prev.was_packs_online_any = false;

  SafetyState out;
  safety::runSafety(snap, cfg, prev, 50000, out);

  REQUIRE(out.alarm_flags & 0x80);
  REQUIRE(out.ccl_amps  == Catch::Approx(0.0f));
  REQUIRE(out.dcl_amps  == Catch::Approx(0.0f));
  REQUIRE(out.cvl_volts == Catch::Approx(0.0f));
  REQUIRE(out.packs_online == 0);
  // No NoPacksOnline event since was_packs_online_any was already false
  for (uint8_t i = 0; i < out.event_count; ++i)
    REQUIRE(out.events[i].type != SafetyState::NoPacksOnline);
}

TEST_CASE("no-packs: factor_charge/discharge preserved as 1.0 (V2.67)", "[nopacks]") {
  // V2.67 does not set factors in the no-packs path; they stay at 1.0.
  BmsSystemSnapshot snap{};
  snap.pack_count_configured = 1;
  snap.pack[0] = make_pack(0, false);

  auto cfg  = make_cfg();
  auto prev = safety::make_default_prev();
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  REQUIRE(out.factor_charge    == Catch::Approx(1.0f));
  REQUIRE(out.factor_discharge == Catch::Approx(1.0f));
}

TEST_CASE("no-packs: 4-pack ccl/dcl/cvl are zero and alarm 0x80", "[nopacks]") {
  BmsSystemSnapshot snap{};
  snap.pack_count_configured = 4;
  for (int i = 0; i < 4; ++i) snap.pack[i] = make_pack(i, false);

  auto cfg  = make_cfg();
  auto prev = make_prev_all_online(4);
  SafetyState out;
  safety::runSafety(snap, cfg, prev, 10000, out);

  REQUIRE(out.alarm_flags & 0x80);
  REQUIRE(out.ccl_amps  == Catch::Approx(0.0f));
  REQUIRE(out.dcl_amps  == Catch::Approx(0.0f));
  REQUIRE(out.cvl_volts == Catch::Approx(0.0f));
}
