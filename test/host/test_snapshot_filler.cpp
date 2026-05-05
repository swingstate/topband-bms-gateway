#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cstring>

#include "bms/snapshot.h"

// ── Helpers ──────────────────────────────────────────────────────────────────

// Build a minimal tb_analog_values_fixed_point from the first captured frame
// (frame 0 from captured_frames.txt, BMS address 0):
//   16579963 0 4.39 50.36  ... 15 cells, 7 temps
// Cell 0 = 0x0D1D = 3357 → 3.357 V
// Temp 4 = 0x0B93 = 2963 → (2963-2731)/10 = 23.2 °C  (0-indexed: temps[4])
// pack_voltage: 0xD07C = 53372... wait, pack_v in the fixture = 50.36 V.
// From Phase B test knowledge: the fixture line shows displayed_voltage = 50.36 V.
// We'll construct a synthetic struct that is consistent with the displayed values.
static bms::protocol::tb_analog_values_fixed_point make_frame0_analog() {
  bms::protocol::tb_analog_values_fixed_point p{};
  p.dataflag    = 0;
  p.cell_count  = 15;
  // First-frame cell values from captured_frames.txt (verified in Phase B tests):
  // 0x0D1D=3357→3.357V, 0x0D1E=3358→3.358V, 0x0D1F=3359→3.359V, 0x0D1C=3356→3.356V,
  // 0x0D1F=3359→3.359V, 0x0D1C=3356→3.356V, 0x0D1C=3356→3.356V, 0x0D21=3361→3.361V,
  // 0x0D1B=3355→3.355V, 0x0D1F=3359→3.359V, 0x0D19=3353→3.353V, 0x0D19=3353→3.353V,
  // 0x0D1F=3359→3.359V, 0x0D1B=3355→3.355V, 0x0D20=3360→3.360V
  float cv[] = {3.357f, 3.358f, 3.359f, 3.356f, 3.359f,
                3.356f, 3.356f, 3.361f, 3.355f, 3.359f,
                3.353f, 3.353f, 3.359f, 3.355f, 3.360f};
  for (int i = 0; i < 15; ++i) p.cells[i] = cv[i];

  // 7 temps: 4 cell temps + balancer/env/MOS (last 3 are special when count >= 3)
  p.temp_count = 7;
  // Temps from captured frame (°C decoded):
  // 0x0B80=2944→21.3°C, 0x0B7E=2942→21.1°C, 0x0B80=2944→21.3°C,
  // 0x0B81=2945→21.4°C, 0x0B93=2963→23.2°C, 0x0B83=2947→21.6°C, 0x0B90=2960→22.9°C
  float tv[] = {21.3f, 21.1f, 21.3f, 21.4f, 23.2f, 21.6f, 22.9f};
  for (int i = 0; i < 7; ++i) p.temps[i] = tv[i];
  // Last 3 are balancer/env/MOS per protocol.h annotation
  p.balancer_temp_c    = tv[4];  // temps[temp_count-3] = temps[4]
  p.environment_temp_c = tv[5];  // temps[temp_count-2] = temps[5]
  p.mosfet_temp_c      = tv[6];  // temps[temp_count-1] = temps[6]

  p.pack_current = 4.39f;    // displayed_current_A from fixture (positive = charge)
  p.pack_voltage = 50.36f;   // displayed_voltage_V from fixture
  p.rem_ah       = 10.35f;   // from 0x040B=1035→10.35 Ah
  p.full_ah      = 48.00f;   // nominal
  p.cycles       = 0x0042;
  p.soc          = 100;
  p.soh          = 100;
  return p;
}

// ── Tests ────────────────────────────────────────────────────────────────────

TEST_CASE("filler init_pack_snapshot_offline zeros struct and marks offline") {
  BmsPackSnapshot pack;
  // Poison the struct before init.
  memset(&pack, 0xFF, sizeof(pack));

  bms::init_pack_snapshot_offline(pack, 3);

  REQUIRE(pack.bms_id == 3);
  REQUIRE_FALSE(pack.online);
  REQUIRE_FALSE(pack.sysparam_valid);
  REQUIRE(pack.pack_voltage   == Catch::Approx(0.0f));
  REQUIRE(pack.pack_current   == Catch::Approx(0.0f));
  REQUIRE(pack.alarm_bits     == 0u);
  REQUIRE(pack.cell_count     == 0u);
  REQUIRE(pack.last_seen_ms   == 0u);
}

TEST_CASE("filler fill_from_analog produces correct derived fields from captured frame 0") {
  auto p = make_frame0_analog();
  BmsPackSnapshot snap;
  memset(&snap, 0, sizeof(snap));
  snap.bms_id = 0;

  bms::fill_from_analog(p, 1000u, snap);

  // Online + timestamp
  REQUIRE(snap.online);
  REQUIRE(snap.last_seen_ms == 1000u);

  // Voltage passthrough
  REQUIRE(snap.pack_voltage == Catch::Approx(50.36f));
  REQUIRE(snap.cell_count   == 15u);

  // Min cell: index 10 or 11 (both 3.353 V)
  REQUIRE(snap.cell_min_v == Catch::Approx(3.353f).epsilon(0.001));
  // Max cell: index 7 (3.361 V)
  REQUIRE(snap.cell_max_v == Catch::Approx(3.361f).epsilon(0.001));
  REQUIRE(snap.cell_max_idx == 7u);
  // Drift = max - min
  REQUIRE(snap.cell_drift_v == Catch::Approx(3.361f - 3.353f).epsilon(0.001));
  // Average of all 15 cells
  float expected_avg = (3.357f + 3.358f + 3.359f + 3.356f + 3.359f +
                        3.356f + 3.356f + 3.361f + 3.355f + 3.359f +
                        3.353f + 3.353f + 3.359f + 3.355f + 3.360f) / 15.0f;
  REQUIRE(snap.cell_avg_v == Catch::Approx(expected_avg).epsilon(0.001));

  // Current (non-zero, not held)
  REQUIRE(snap.pack_current == Catch::Approx(4.39f));
  REQUIRE_FALSE(snap.current_held);
}

TEST_CASE("filler fill_from_analog temperature aggregates (frame 0, 7 temps)") {
  auto p = make_frame0_analog();
  BmsPackSnapshot snap;
  memset(&snap, 0, sizeof(snap));

  bms::fill_from_analog(p, 2000u, snap);

  // temp_count = 7; cell temps = first 4 (temps[0..3]), special = last 3
  REQUIRE(snap.temp_count == 7u);

  // Phase B test verifies: temp index 4 (0-indexed) = 23.2°C.
  // In the snap, temp_c[4] is stored directly from parsed.temps[4].
  REQUIRE(snap.temp_c[4] == Catch::Approx(23.2f).epsilon(0.1f));

  // temp_max_c is the max over cell temps (first 4 = temps[0..3]):
  // 21.3, 21.1, 21.3, 21.4 → max = 21.4
  REQUIRE(snap.temp_max_c == Catch::Approx(21.4f).epsilon(0.1f));
  // temp_avg_c = avg of cell temps only
  float expected_avg = (21.3f + 21.1f + 21.3f + 21.4f) / 4.0f;
  REQUIRE(snap.temp_avg_c == Catch::Approx(expected_avg).epsilon(0.1f));

  // Special sensors stored separately
  REQUIRE(snap.balancer_temp_c    == Catch::Approx(23.2f).epsilon(0.1f));
  REQUIRE(snap.environment_temp_c == Catch::Approx(21.6f).epsilon(0.1f));
  REQUIRE(snap.mosfet_temp_c      == Catch::Approx(22.9f).epsilon(0.1f));
}

TEST_CASE("filler hold-last-value for pack_current") {
  auto p = make_frame0_analog();
  BmsPackSnapshot snap;
  memset(&snap, 0, sizeof(snap));

  // Step 1: non-zero current → stored, no hold
  p.pack_current = 2.5f;
  bms::fill_from_analog(p, 1000u, snap);
  REQUIRE(snap.pack_current == Catch::Approx(2.5f));
  REQUIRE_FALSE(snap.current_held);
  REQUIRE(snap.current_held_value == Catch::Approx(2.5f));
  // Hold window: 1000 + 120000 = 121000 ms
  REQUIRE(snap.current_held_until_ms == 121000u);

  // Step 2: BMS reports 0A within 120 s window → held value displayed
  p.pack_current = 0.0f;
  bms::fill_from_analog(p, 5000u, snap);  // 5s < 120s
  REQUIRE(snap.pack_current == Catch::Approx(2.5f));
  REQUIRE(snap.current_held);

  // Step 3: 0A after hold window expires (>120 s later) → display 0
  bms::fill_from_analog(p, 200000u, snap); // 200s > 120s hold
  REQUIRE(snap.pack_current == Catch::Approx(0.0f));
  REQUIRE_FALSE(snap.current_held);
}

TEST_CASE("filler decay_online_status transitions pack offline") {
  BmsPackSnapshot snap;
  memset(&snap, 0, sizeof(snap));
  snap.bms_id      = 2;
  snap.online      = true;
  snap.last_seen_ms = 1000u;  // seen at T=1s

  // Check at T=5s (4s elapsed): still within 10s threshold → remains online
  bool transitioned = bms::decay_online_status(snap, 5000u, 10000u);
  REQUIRE_FALSE(transitioned);
  REQUIRE(snap.online);

  // Check at T=15s (14s elapsed): exceeds 10s threshold → goes offline
  transitioned = bms::decay_online_status(snap, 15000u, 10000u);
  REQUIRE(transitioned);
  REQUIRE_FALSE(snap.online);

  // Calling again when already offline → returns false (no further transition)
  transitioned = bms::decay_online_status(snap, 20000u, 10000u);
  REQUIRE_FALSE(transitioned);
}

TEST_CASE("filler update_system_aggregates counts online packs correctly") {
  BmsSystemSnapshot sys;
  memset(&sys, 0, sizeof(sys));
  sys.pack_count_configured = 16;

  // Mark 4 packs online, 12 offline
  for (int i = 0; i < 16; ++i) {
    sys.pack[i].bms_id = static_cast<uint8_t>(i);
    sys.pack[i].online = (i < 4);
  }

  bms::update_system_aggregates(sys);

  REQUIRE(sys.pack_count_online      == 4u);
  REQUIRE(sys.pack_count_configured  == 16u);
}
