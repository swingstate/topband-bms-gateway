"""
Generates 8 V2.67 regression test cases as JSON fixtures.

Each case writes four files:
  in/case_NN_<name>_snapshot.json
  in/case_NN_<name>_config.json
  in/case_NN_<name>_prev.json
  out/case_NN_<name>_safety.json

Outputs are computed by v267_reference.calculate_victron_data().
These files are committed to the repo; C++ tests read them without Python.
"""

import json
import os
import sys

from v267_reference import (
    PackSnapshot, SystemSnapshot, Config, PrevState,
    calculate_victron_data,
    snapshot_to_json, config_to_json, prev_to_json, safety_to_json,
)

OUT_DIR_IN  = "in"
OUT_DIR_OUT = "out"


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


def offline_packs(start, end):
    """Return offline packs for range [start, end)."""
    return [make_pack(i, online=False) for i in range(start, end)]


# ─────────────────────────────────────────────────────────────────────────────
# The 8 cases
# ─────────────────────────────────────────────────────────────────────────────

def case_01_idle_4packs():
    """4 packs online, no current, normal temperature. No transitions."""
    snap = SystemSnapshot(
        cycle_id=50, produced_ms=150000, pack_count_configured=4,
        packs=[make_pack(i) for i in range(4)] + offline_packs(4, 16),
    )
    return "01_idle_4packs", snap, make_default_cfg(), make_default_prev()


def case_02_charge_4packs():
    """4 packs online, 5A charging each, normal temperature."""
    snap = SystemSnapshot(
        cycle_id=51, produced_ms=153000, pack_count_configured=4,
        packs=[make_pack(i, current=5.0, soc=55.0) for i in range(4)]
              + offline_packs(4, 16),
    )
    return "02_charge_4packs", snap, make_default_cfg(), make_default_prev()


def case_03_discharge_4packs():
    """4 packs online, -10A discharging each (signed convention)."""
    snap = SystemSnapshot(
        cycle_id=52, produced_ms=156000, pack_count_configured=4,
        packs=[make_pack(i, current=-10.0, soc=45.0) for i in range(4)]
              + offline_packs(4, 16),
    )
    return "03_discharge_4packs", snap, make_default_cfg(), make_default_prev()


def case_04_cell_overvolt_pack3():
    """Pack 3 max cell at 3.60 V > safe_cell_volt=3.55. OV flag. Lock to zero."""
    packs = [make_pack(i) for i in range(4)]
    packs[3] = make_pack(3, max_cell=3.60, cell_drift=0.020)  # 3.60 > 3.55
    snap = SystemSnapshot(
        cycle_id=53, produced_ms=159000, pack_count_configured=4,
        packs=packs + offline_packs(4, 16),
    )
    return "04_cell_overvolt_pack3", snap, make_default_cfg(), make_default_prev()


def case_05_temp_charge_stop():
    """All pack temps at 2 °C < charge_temp_min=5. factor_charge=0, alarm_flags|=0x08."""
    snap = SystemSnapshot(
        cycle_id=54, produced_ms=162000, pack_count_configured=4,
        packs=[make_pack(i, temp_max=2.0, temp_avg=2.0) for i in range(4)]
              + offline_packs(4, 16),
    )
    return "05_temp_charge_stop", snap, make_default_cfg(), make_default_prev()


def case_06_bms_reported_alarm_pack1():
    """
    Pack 1 reports UV alarm bit (bit 12) via 0x44 response.
    Pack_v=30.0 is below UV sanity cap (15*3.2=48), so alarm is NOT suppressed.
    Results: alarm_flags |= 0x10, safe_dis=0, safe_chg=0 (lock rule), ccl=dcl=0.
    """
    packs = [make_pack(i) for i in range(4)]
    # Bit 12 = TB_ALRMS UV protection (see V2.67 TB_CRITICAL_ALARM_MASK)
    # Pack voltage 30.0 V to satisfy should_flag (below sanity cap of 48.0)
    packs[1] = make_pack(1, alarm_bits=(1 << 12),
                         last_alarm_ms=120000,   # fresh (30s < 60s before 150000ms)
                         voltage=30.0,
                         max_cell=2.0, min_cell=1.9, cell_drift=0.10)
    snap = SystemSnapshot(
        cycle_id=55, produced_ms=150000, pack_count_configured=4,
        packs=packs + offline_packs(4, 16),
    )
    return "06_bms_reported_alarm_pack1", snap, make_default_cfg(), make_default_prev()


def case_07_all_offline():
    """All packs offline. Previous: 4 packs online. BmsWentOffline × 4 + NoPacksOnline."""
    snap = SystemSnapshot(
        cycle_id=56, produced_ms=165000, pack_count_configured=4,
        packs=[make_pack(i, online=False) for i in range(16)],
    )
    return "07_all_offline", snap, make_default_cfg(), \
           make_default_prev(packs_online=[True]*4 + [False]*12)


def case_08_recovering_one_back_online():
    """
    Previous cycle: all offline (was_packs_online_any=False).
    Current: pack 0 just came online. PacksOnlineRecovered + BmsCameOnline events.
    """
    packs = [make_pack(0)] + [make_pack(i, online=False) for i in range(1, 16)]
    snap = SystemSnapshot(
        cycle_id=57, produced_ms=168000, pack_count_configured=4, packs=packs,
    )
    prev = make_default_prev(packs_online=[False] * 16)
    return "08_recovering_one_back_online", snap, make_default_cfg(), prev


# ─────────────────────────────────────────────────────────────────────────────
# Driver
# ─────────────────────────────────────────────────────────────────────────────

CASES = [
    case_01_idle_4packs,
    case_02_charge_4packs,
    case_03_discharge_4packs,
    case_04_cell_overvolt_pack3,
    case_05_temp_charge_stop,
    case_06_bms_reported_alarm_pack1,
    case_07_all_offline,
    case_08_recovering_one_back_online,
]


def main():
    os.makedirs(OUT_DIR_IN,  exist_ok=True)
    os.makedirs(OUT_DIR_OUT, exist_ok=True)

    for case_fn in CASES:
        name, snap, cfg, prev = case_fn()
        result = calculate_victron_data(snap, cfg, prev)

        stem = f"case_{name}"
        with open(f"{OUT_DIR_IN}/{stem}_snapshot.json", "w") as f:
            json.dump(snapshot_to_json(snap), f, indent=2)
        with open(f"{OUT_DIR_IN}/{stem}_config.json", "w") as f:
            json.dump(config_to_json(cfg), f, indent=2)
        with open(f"{OUT_DIR_IN}/{stem}_prev.json", "w") as f:
            json.dump(prev_to_json(prev), f, indent=2)
        with open(f"{OUT_DIR_OUT}/{stem}_safety.json", "w") as f:
            json.dump(safety_to_json(result), f, indent=2)

        print(f"  {stem}: alarm=0x{result.alarm_flags:02X} "
              f"ccl={result.ccl_amps:.1f} dcl={result.dcl_amps:.1f} "
              f"cvl={result.cvl_volts:.2f} "
              f"packs={result.packs_online} msg='{result.sys_message}'")


if __name__ == "__main__":
    main()
