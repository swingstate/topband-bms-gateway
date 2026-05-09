"""
Self-tests for v267_reference.py.

3 hand-traced cases to verify the Python reference before it becomes the
truth source for the 8 generated test cases. Each case was computed by
reading V2.67 source line by line.

Run: cd test/host/fixtures/v267 && python -m pytest test_v267_reference.py -v
"""

import pytest
from v267_reference import (
    PackSnapshot, SystemSnapshot, Config, PrevState,
    SafetyState, calc_factor, filter_alarm_bits,
    calculate_victron_data,
    CRITICAL_ALARM_MASK, UV_ALARM_BITS, OV_ALARM_BITS,
)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def make_pack(bms_id, online=True, voltage=50.10, current=0.0, soc=50.0,
              soh=100.0, max_cell=3.341, min_cell=3.339, cell_drift=0.002,
              temp_max=25.1, temp_avg=25.0, alarm_bits=0,
              last_alarm_ms=0,
              sysparam_valid=True, last_sysparam_ms=0,
              sys_cell_high_v=3.65, sys_module_high_v=58.40,
              sys_module_under_v=38.0,
              sys_charge_max_a=0.0, sys_discharge_max_a=0.0):
    cells = [3.340] * 15 + [0.0]
    cells[0] = min_cell
    cells[1] = max_cell
    return PackSnapshot(
        bms_id=bms_id, online=online,
        pack_voltage=voltage, pack_current=current,
        soc=soc, soh=soh, rem_ah=100.0, full_ah=100.0,
        cell_count=15, cell_v=cells,
        cell_min_v=min_cell, cell_max_v=max_cell,
        cell_avg_v=3.340, cell_drift_v=cell_drift,
        temp_max_c=temp_max, temp_avg_c=temp_avg,
        alarm_bits=alarm_bits, last_alarm_ms=last_alarm_ms,
        sysparam_valid=sysparam_valid, last_sysparam_ms=last_sysparam_ms,
        sys_cell_high_v=sys_cell_high_v, sys_module_high_v=sys_module_high_v,
        sys_module_under_v=sys_module_under_v,
        sys_charge_max_a=sys_charge_max_a,
        sys_discharge_max_a=sys_discharge_max_a,
    )


def make_default_cfg(**overrides):
    cfg = Config(
        bms_count=4,
        charge_amps_per_pack=30.0,
        discharge_amps_per_pack=30.0,
        cvl_voltage=53.5,
        safe_pack_volt=56.25,
        safe_cell_volt=3.55,
        safe_cell_drift=0.020,
        charge_temp_min=5.0, charge_temp_max=50.0,
        discharge_temp_min=-20.0, discharge_temp_max=60.0,
        temp_mode="Hottest",
    )
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def make_default_prev(packs_online=None):
    if packs_online is None:
        packs_online = [True] * 4 + [False] * 12
    return PrevState(
        was_pack_online=packs_online,
        prev_alarm_flags=0,
        prev_factor_charge=1.0,
        prev_factor_discharge=1.0,
        was_packs_online_any=any(packs_online),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Self-test A: calc_factor correctness (hand-verified)
# ─────────────────────────────────────────────────────────────────────────────

class TestCalcFactor:
    """Hand-traced against V2.67 calcFactor() lines 2034-2040."""

    def test_below_min_returns_zero(self):
        # t < t_min → 0.0
        assert calc_factor(4.9, 5.0, 50.0) == 0.0

    def test_above_max_returns_zero(self):
        # t > t_max → 0.0
        assert calc_factor(50.1, 5.0, 50.0) == 0.0

    def test_at_min_enters_low_soft_zone(self):
        # t == t_min: not < t_min, not > t_max, t < t_min+5 → 0.2
        assert calc_factor(5.0, 5.0, 50.0) == 0.2

    def test_in_low_soft_zone(self):
        # t = 7.0; t_min=5, t_min+5=10; 5 <= 7 < 10 → 0.2
        assert calc_factor(7.0, 5.0, 50.0) == 0.2

    def test_at_top_of_low_soft_zone(self):
        # t = 9.99; 5 <= 9.99 < 10 → 0.2
        assert calc_factor(9.99, 5.0, 50.0) == 0.2

    def test_just_inside_normal_range(self):
        # t = 10.0; not < 10 → falls through to "not > t_max-5=45" → 1.0
        assert calc_factor(10.0, 5.0, 50.0) == 1.0

    def test_in_normal_range(self):
        assert calc_factor(25.0, 5.0, 50.0) == 1.0

    def test_at_top_of_normal_range(self):
        # t = 45.0; t_max-5=45; 45 > 45 is False → 1.0
        assert calc_factor(45.0, 5.0, 50.0) == 1.0

    def test_just_inside_high_soft_zone(self):
        # t = 45.001; 45.001 > 45 → 0.5
        assert calc_factor(45.001, 5.0, 50.0) == 0.5

    def test_in_high_soft_zone(self):
        # t = 48.0 → 0.5
        assert calc_factor(48.0, 5.0, 50.0) == 0.5

    def test_at_max_stays_in_high_soft_zone(self):
        # t = 50.0; not > t_max → soft zone check → > 45.0 → 0.5
        assert calc_factor(50.0, 5.0, 50.0) == 0.5

    def test_discharge_normal(self):
        # t=25, min=-20, max=60; not < -20, not > 60; not < -15; not > 55 → 1.0
        assert calc_factor(25.0, -20.0, 60.0) == 1.0

    def test_discharge_too_cold(self):
        # t=-21 < -20 → 0.0
        assert calc_factor(-21.0, -20.0, 60.0) == 0.0

    def test_discharge_cold_soft(self):
        # t=-18, min=-20, min+5=-15; -18 < -15 → 0.2
        assert calc_factor(-18.0, -20.0, 60.0) == 0.2


# ─────────────────────────────────────────────────────────────────────────────
# Self-test B: Idle 1 pack, all sysparam valid
# ─────────────────────────────────────────────────────────────────────────────

class TestIdle1Pack:
    """
    Hand-traced case:
    1 pack online, all values normal, no alarms, no transitions.
    V2.67 line 2082: sum_i=0, sum_v=50.10, count=1.
    t_check_val = pack[0].temp_max_c = 25.1 (Hottest mode).
    factor_charge = calcFactor(25.1, 5, 50) = 1.0 (25.1 in [10, 45]).
    factor_discharge = calcFactor(25.1, -20, 60) = 1.0.
    safe_chg = 1 * 30.0 * 1.0 = 30.0 (no proto cap since sys_charge_max_a=0).
    safe_dis = 1 * 30.0 * 1.0 = 30.0.
    cvl: proto_cvl_cap from sysparam:
      cvl_cap = 58.40 - 0.20 = 58.20
      chem_cap = 15 * 3.60 = 54.0 → cvl_cap = 54.0
      safe_pack_volt - 0.20 = 56.05 → cvl_cap = min(54.0, 56.05) = 54.0
      cvl_voltage = 53.5 → cvl_cap = min(54.0, 53.5) = 53.5
      proto_cvl_cap = 53.5 < 900 → cvl_volts = 53.5
    alarm_flags = 0x00, sys_message = "OK".
    SOC = 50 < 99, no taper.
    """

    def setup_method(self):
        snap = SystemSnapshot(
            cycle_id=1, produced_ms=100000, pack_count_configured=1,
            packs=[make_pack(0)] + [make_pack(i, online=False) for i in range(1, 16)],
        )
        cfg = make_default_cfg(bms_count=1)
        prev = make_default_prev(packs_online=[True] + [False] * 15)
        self.result = calculate_victron_data(snap, cfg, prev)

    def test_cvl_volts(self):
        assert abs(self.result.cvl_volts - 53.5) < 0.001

    def test_ccl_amps(self):
        assert abs(self.result.ccl_amps - 30.0) < 0.001

    def test_dcl_amps(self):
        assert abs(self.result.dcl_amps - 30.0) < 0.001

    def test_factor_charge(self):
        assert self.result.factor_charge == 1.0

    def test_factor_discharge(self):
        assert self.result.factor_discharge == 1.0

    def test_alarm_flags_zero(self):
        assert self.result.alarm_flags == 0x00

    def test_sys_message_ok(self):
        assert self.result.sys_message == "OK"

    def test_packs_online(self):
        assert self.result.packs_online == 1

    def test_no_events(self):
        # No transitions from previous state (pack was already online)
        relevant = [e for e in self.result.events
                    if e.type not in ("CellImbalanceStart",)]
        assert len(relevant) == 0


# ─────────────────────────────────────────────────────────────────────────────
# Self-test C: Cell over-voltage — filter suppresses OV alarm bits
# ─────────────────────────────────────────────────────────────────────────────

class TestFilterSuppressesOVAlarm:
    """
    Pack 0 has alarm_bits with bit 0 (OV protect) set.
    But max_cell_v = 3.45 V — well below ov_cell_floor.
    pack_v = 50.10 — well below ov_pack_floor.

    Hand-trace of filter:
      ov_cell_floor = tbOverVoltCellSanityFloor(3.65) = 3.65 - 0.12 = 3.53
      ov_pack_floor = tbOverVoltPackSanityFloor(58.40, 15) = 58.40 - 0.80 = 57.60
      max_cell_v (3.45) < 3.53  AND  pack_v (50.10) < 57.60 → OV bits SUPPRESSED
      → crit_bits = (alarm_bits & CRITICAL_ALARM_MASK) & ~OV_ALARM_BITS

    Bit 0 is OV; it's NOT in UV_ALARM_BITS, so the BMS-alarm path triggers.
    But after OV bit is suppressed, crit_bits = 0 → no alarm set!
    Result: alarm_flags = 0x00, ccl/dcl = 120.0.
    """

    def setup_method(self):
        # last_alarm_ms = 50000, produced_ms = 100000: fresh (50 s < 60 s)
        p0 = make_pack(0, alarm_bits=(1 << 0), last_alarm_ms=50000,
                       max_cell=3.45, min_cell=3.43, cell_drift=0.020)
        snap = SystemSnapshot(
            cycle_id=2, produced_ms=100000, pack_count_configured=4,
            packs=[p0] + [make_pack(i) for i in range(1, 4)]
                  + [make_pack(i, online=False) for i in range(4, 16)],
        )
        cfg = make_default_cfg(bms_count=4)
        prev = make_default_prev()
        self.result = calculate_victron_data(snap, cfg, prev)

    def test_ov_alarm_bit_suppressed(self):
        # OV bit in alarm_bits was filtered out → alarm_flags & 0x40 == 0
        # AND alarm_flags & 0x02 == 0 (no config-level overvolt either, 3.45 < 3.55)
        assert (self.result.alarm_flags & 0x40) == 0
        assert (self.result.alarm_flags & 0x02) == 0

    def test_alarm_flags_zero(self):
        assert self.result.alarm_flags == 0x00

    def test_ccl_not_zeroed(self):
        # No OV/UV/BMS-alarm → lock-to-zero rule doesn't fire → full CCL
        assert abs(self.result.ccl_amps - 120.0) < 0.001

    def test_sys_message_ok(self):
        assert self.result.sys_message == "OK"


# ─────────────────────────────────────────────────────────────────────────────
# Self-test D: All packs offline → alarm_flags = 0x80
# ─────────────────────────────────────────────────────────────────────────────

class TestAllOffline:
    """
    V2.67 line 2267 else-branch: alarm_flags |= 0x80, all currents = 0.
    Previous state had 4 packs online → BmsWentOffline events for 0-3.
    NoPacksOnline event emitted.
    """

    def setup_method(self):
        snap = SystemSnapshot(
            cycle_id=3, produced_ms=165000, pack_count_configured=4,
            packs=[make_pack(i, online=False) for i in range(16)],
        )
        cfg = make_default_cfg(bms_count=4)
        prev = make_default_prev(packs_online=[True]*4 + [False]*12)
        self.result = calculate_victron_data(snap, cfg, prev)

    def test_alarm_flags_no_packs(self):
        assert self.result.alarm_flags == 0x80

    def test_ccl_zero(self):
        assert self.result.ccl_amps == 0.0

    def test_dcl_zero(self):
        assert self.result.dcl_amps == 0.0

    def test_cvl_zero(self):
        assert self.result.cvl_volts == 0.0

    def test_packs_online_zero(self):
        assert self.result.packs_online == 0

    def test_no_packs_online_event(self):
        types = [e.type for e in self.result.events]
        assert "NoPacksOnline" in types

    def test_went_offline_events(self):
        offline_events = [e for e in self.result.events if e.type == "BmsWentOffline"]
        # Packs 0-3 were online, now offline → 4 BmsWentOffline events
        assert len(offline_events) == 4
        assert {e.bms_id for e in offline_events} == {0, 1, 2, 3}


# ─────────────────────────────────────────────────────────────────────────────
# Self-test E: filter_alarm_bits unit tests (direct)
# ─────────────────────────────────────────────────────────────────────────────

class TestFilterAlarmBits:
    """Verify the filter function against hand-computed expectations."""

    def test_unknown_cells_clears_uv_bits(self):
        # cells=0 → UV bits always cleared
        raw = UV_ALARM_BITS | (1 << 4)
        result = filter_alarm_bits(raw, 20.0, 25.0, 0, 50.0, 3.4, 3.65, 58.4)
        assert (result & UV_ALARM_BITS) == 0

    def test_pack_above_uv_cap_clears_uv_bits(self):
        # 15 cells, sanity_cap = 15*3.2 = 48.0
        # pack_v = 50.0 > 48.0 → UV bits cleared
        raw = UV_ALARM_BITS
        result = filter_alarm_bits(raw, 50.0, 25.0, 15, 50.0, 3.4, 3.65, 58.4)
        assert (result & UV_ALARM_BITS) == 0

    def test_pack_below_uv_cap_preserves_uv_bits(self):
        # pack_v = 30.0 <= 48.0 → UV bits preserved
        raw = UV_ALARM_BITS
        result = filter_alarm_bits(raw, 30.0, 25.0, 15, 50.0, 3.1, 3.65, 58.4)
        assert (result & UV_ALARM_BITS) != 0

    def test_ov_bits_suppressed_when_both_below_floor(self):
        # max_cell_v=3.45 < floor(3.65-0.12=3.53); pack_v=50 < ov_pack_floor(58.4-0.8=57.6)
        raw = OV_ALARM_BITS
        result = filter_alarm_bits(raw, 50.0, 25.0, 15, 50.0, 3.45, 3.65, 58.4)
        assert (result & OV_ALARM_BITS) == 0

    def test_ov_bits_preserved_when_cell_v_above_floor(self):
        # max_cell_v=3.60 >= floor=3.53 → NOT suppressed
        raw = OV_ALARM_BITS
        result = filter_alarm_bits(raw, 50.0, 25.0, 15, 50.0, 3.60, 3.65, 58.4)
        assert (result & OV_ALARM_BITS) != 0

    def test_temp_bits_suppressed_well_below_limit(self):
        # temp_limit=50, temp_guard=max(47, 15)=47; max_temp=25 < 47 → temp bits cleared
        raw = (1 << 5) | (1 << 6) | (1 << 59)
        result = filter_alarm_bits(raw, 50.0, 25.0, 15, 50.0, 3.4, 3.65, 58.4)
        assert (result & ((1 << 5) | (1 << 6) | (1 << 59))) == 0

    def test_temp_bits_preserved_near_limit(self):
        # max_temp=48 > guard=47 → temp bits NOT suppressed
        raw = (1 << 5)
        result = filter_alarm_bits(raw, 50.0, 48.0, 15, 50.0, 3.4, 3.65, 58.4)
        assert (result & (1 << 5)) != 0

    def test_bits_outside_critical_mask_always_stripped(self):
        # bit 7 is not in CRITICAL_ALARM_MASK → always 0 in output
        raw = (1 << 7) | OV_ALARM_BITS
        result = filter_alarm_bits(raw, 60.0, 48.0, 15, 50.0, 3.70, 3.65, 58.4)
        assert (result & (1 << 7)) == 0
