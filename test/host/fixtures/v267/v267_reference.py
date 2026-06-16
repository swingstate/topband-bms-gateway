"""
V2.67 calculateVictronData() reference implementation.

Translated from legacy/Topband_BMS_Gateway_v2_67_2.ino, lines 2034-2267.
This is the canonical reference for Phase D regression tests.  V3.0's C++
runSafety() must produce equivalent results for the same inputs.

Key V2.67 specifics preserved exactly:
  - calcFactor(): 5 bands, fixed soft_zone=5, outputs {0,0.2,0.5,1.0}
    Low soft zone → 0.2 (not 0.5). High soft zone → 0.5 (not 0.2).
  - t_check_val: overwritten per online pack; last online pack wins (line 2148).
  - Alarm flag bit assignments: 0x02=OV, 0x08=temp, 0x10=UV, 0x20=drift,
    0x40=BMS alarm, 0x80=no packs (line 869).
  - "Lock to zero" rule (line 2211): OV|UV|BMS alarm → both currents zero.
  - sys_error_msg: first-writer-wins for "OK" check; some conditions overwrite
    without checking (see comments at each site).
  - SOC taper: soc>=99 → chg=count*2; soc==100 → chg=0 (lines 2226-2228).
  - No expert-mode floor handling: test cases supply cfg values already floored.
  - No auto-balance watchdog (requires real time; excluded from runSafety).
  - No easy-mode auto-apply (config mutation; excluded from runSafety).

Author: Claude Code, Phase D
Source: V2.67.2 (commit at point of fork)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import List
import json
import math


# ─────────────────────────────────────────────────────────────────────────────
# Data types matching the C++ structs
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PackSnapshot:
    bms_id: int
    online: bool
    pack_voltage: float
    pack_current: float
    soc: float               # 0-100
    soh: float               # 0-100
    rem_ah: float
    full_ah: float
    cell_count: int
    cell_v: List[float]      # length 16; unused slots = 0.0
    cell_min_v: float
    cell_max_v: float
    cell_avg_v: float
    cell_drift_v: float
    temp_max_c: float
    temp_avg_c: float
    alarm_bits: int           # uint64
    last_alarm_ms: int        # >0 means alarm data is present
    sysparam_valid: bool
    last_sysparam_ms: int
    sys_cell_high_v: float
    sys_module_high_v: float
    sys_module_under_v: float
    sys_charge_max_a: float
    sys_discharge_max_a: float


@dataclass
class SystemSnapshot:
    cycle_id: int
    produced_ms: int
    pack_count_configured: int
    packs: List[PackSnapshot]   # length 16


@dataclass
class Config:
    bms_count: int
    charge_amps_per_pack: float
    discharge_amps_per_pack: float
    cvl_voltage: float
    safe_pack_volt: float
    safe_cell_volt: float
    safe_cell_drift: float
    charge_temp_min: float
    charge_temp_max: float
    discharge_temp_min: float
    discharge_temp_max: float
    temp_mode: str            # "Hottest" or "Average"
    maint_charge_enabled: bool = False
    maint_target_voltage: float = 52.0


@dataclass
class PrevState:
    was_pack_online: List[bool]       # length 16
    prev_alarm_flags: int = 0         # previous cycle alarm_flags byte
    prev_factor_charge: float = 1.0
    prev_factor_discharge: float = 1.0
    was_packs_online_any: bool = True


@dataclass
class Event:
    type: str
    bms_id: int = 0xFF
    alarm_bits: int = 0


@dataclass
class SafetyState:
    cycle_id: int
    produced_ms: int
    cvl_volts: float
    ccl_amps: float
    dcl_amps: float
    pack_voltage_avg: float
    pack_current_total: float
    soc_avg: float
    soh_avg: float
    temp_avg: float
    capacity_total_ah: float
    capacity_remain_ah: float
    factor_charge: float
    factor_discharge: float
    alarm_flags: int
    sys_message: str
    packs_online: int
    packs_configured: int
    events: List[Event] = field(default_factory=list)
    events_overflowed: bool = False


# ─────────────────────────────────────────────────────────────────────────────
# V2.67's tbFilterCriticalAlarmBits helpers (lines 1367-1426)
# ─────────────────────────────────────────────────────────────────────────────

CRITICAL_ALARM_MASK = (
    (1 << 0)  | (1 << 2)  | (1 << 4)  | (1 << 5)  |
    (1 << 6)  | (1 << 11) | (1 << 12) | (1 << 13) |
    (1 << 50) | (1 << 51) | (1 << 52) | (1 << 53) |
    (1 << 54) | (1 << 55) | (1 << 58) | (1 << 59)
)
UV_ALARM_BITS   = (1 << 12) | (1 << 13)
OV_ALARM_BITS   = (1 << 0)  | (1 << 58)
TEMP_ALARM_BITS = (1 << 5)  | (1 << 6)  | (1 << 59)


def _under_volt_sanity_cap(cells: int) -> float:
    """V2.67 tbUnderVoltSanityCap (line 1373)."""
    if cells <= 0 or cells > 32:
        return 57.0
    cap = cells * 3.20
    if cap < 40.0: cap = 40.0
    if cap > 57.0: cap = 57.0
    return cap


def _should_flag_proto_under_volt(pack_v: float, module_under_v: float, cells: int) -> bool:
    """V2.67 tbShouldFlagProtoUnderVolt (line 1381)."""
    if module_under_v <= 1.0: return False
    if cells <= 0 or cells > 32: return False
    trigger_v = module_under_v + 0.05
    sanity_cap = _under_volt_sanity_cap(cells)
    if trigger_v > sanity_cap: return False
    return pack_v <= trigger_v


def _ov_cell_sanity_floor(cell_high_v: float) -> float:
    """V2.67 tbOverVoltCellSanityFloor (line 1390)."""
    floor_v = cell_high_v - 0.12
    if cell_high_v < 3.2 or cell_high_v > 4.5:
        floor_v = 3.58
    if floor_v < 3.50: floor_v = 3.50
    if floor_v > 4.05: floor_v = 4.05
    return floor_v


def _ov_pack_sanity_floor(module_high_v: float, cells: int) -> float:
    """V2.67 tbOverVoltPackSanityFloor (line 1398)."""
    floor_v = module_high_v - 0.80
    if module_high_v < 30.0 or module_high_v > 70.0:
        floor_v = cells * 3.55 if (0 < cells <= 32) else 50.0
    if floor_v < 40.0: floor_v = 40.0
    if floor_v > 61.0: floor_v = 61.0
    return floor_v


def filter_alarm_bits(raw_bits: int, pack_v: float, max_temp_c: float,
                      cells: int, temp_limit_c: float,
                      max_cell_v: float, cell_high_v: float,
                      module_high_v: float) -> int:
    """V2.67 tbFilterCriticalAlarmBits (line 1409)."""
    bits = raw_bits & CRITICAL_ALARM_MASK

    # Undervolt bits — suppress if cells unknown or pack_v above sanity cap
    if cells <= 0 or cells > 32:
        bits &= ~UV_ALARM_BITS
    elif pack_v > _under_volt_sanity_cap(cells):
        bits &= ~UV_ALARM_BITS

    # Overvolt bits — suppress when both cell and pack voltage below floors
    ov_cell_floor = _ov_cell_sanity_floor(cell_high_v)
    ov_pack_floor = _ov_pack_sanity_floor(module_high_v, cells)
    if max_cell_v < ov_cell_floor and pack_v < ov_pack_floor:
        bits &= ~OV_ALARM_BITS

    # Temperature bits — suppress when max_temp well below trip temperature
    temp_guard = max(temp_limit_c - 3.0, 15.0)
    if max_temp_c < temp_guard:
        bits &= ~TEMP_ALARM_BITS

    return bits


# ─────────────────────────────────────────────────────────────────────────────
# V2.67's calcFactor() — line 2034
# ─────────────────────────────────────────────────────────────────────────────

def calc_factor(t: float, t_min: float, t_max: float) -> float:
    """
    V2.67 lines 2034-2040: temperature throttle factor.
    Returns exactly one of {0.0, 0.2, 0.5, 1.0}.

    Boundary choices (verbatim from V2.67 source, < vs >, not <=/>= ):
      t < t_min              → 0.0   (below minimum: cutoff)
      t > t_max              → 0.0   (above maximum: cutoff)
      t < (t_min + 5)        → 0.2   (low soft zone, closed at t_min)
      t > (t_max - 5)        → 0.5   (high soft zone, open at t_max-5)
      otherwise              → 1.0   (normal operating range)

    Note: the high soft zone returns 0.5, NOT 0.2 (common misconception).
    The low soft zone returns 0.2, NOT 0.5.
    """
    if t < t_min: return 0.0
    if t > t_max: return 0.0
    soft_zone = 5.0
    if t < (t_min + soft_zone): return 0.2
    if t > (t_max - soft_zone): return 0.5
    return 1.0


# ─────────────────────────────────────────────────────────────────────────────
# V2.67's calculateVictronData() — line 2081
# ─────────────────────────────────────────────────────────────────────────────

def _emit(events: List[Event], events_overflowed: list,
          etype: str, bms_id: int = 0xFF, alarm_bits: int = 0) -> None:
    """Append event, set overflow flag if > 16."""
    if len(events) >= 16:
        events_overflowed[0] = True
        return
    events.append(Event(type=etype, bms_id=bms_id, alarm_bits=alarm_bits))


def calculate_victron_data(snap: SystemSnapshot, cfg: Config,
                           prev: PrevState) -> SafetyState:
    """
    V2.67 lines 2081-2267: main safety logic.

    Preserves V2.67 behaviour exactly, including:
    - t_check_val overwritten per online pack (last wins, line 2148)
    - sys_error_msg only changed if "OK" for low-priority conditions,
      always overwritten for OV/temp-stop (lines 2146-2208)
    - Lock-to-zero rule (line 2211)
    - SOC taper (lines 2226-2228)
    """
    sum_i = sum_v = sum_soc = sum_soh = 0.0
    sum_t = sum_cap = sum_rem = 0.0
    proto_ccl_cap = 0.0
    proto_dcl_cap = 0.0
    proto_cvl_cap = 999.0   # sentinel: no sysparam-based cap yet
    proto_under_v_hit = False
    proto_count = 0
    count = 0
    alarm_flags = 0
    sys_error_msg = "OK"

    # t_check_val: V2.67 initialises to 25.0 then overwrites per online pack.
    t_check_val = 25.0

    events: List[Event] = []
    events_overflowed = [False]  # mutable container so _emit can set it
    now_ms = snap.produced_ms

    # ── Per-pack loop (V2.67 lines 2094-2151) ────────────────────────────────
    for i in range(cfg.bms_count):
        p = snap.packs[i]
        on = p.online

        # Edge: BMS went offline (line 2098)
        if prev.was_pack_online[i] and not on:
            # sys_error_msg overwritten without "OK" check (line 2100)
            if sys_error_msg == "OK":
                sys_error_msg = f"ALARM: BMS{i} OFFLINE"
            _emit(events, events_overflowed, "BmsWentOffline", bms_id=i)

        # Edge: BMS came online
        if not prev.was_pack_online[i] and on:
            _emit(events, events_overflowed, "BmsCameOnline", bms_id=i)

        if not on:
            continue

        # ── Alarm filter (lines 2104-2127) ────────────────────────────────
        alarm_fresh = (p.last_alarm_ms > 0) and ((now_ms - p.last_alarm_ms) < 60000)
        if alarm_fresh:
            cells_for_filter = p.cell_count if p.cell_count > 0 else 0
            temp_limit = min(cfg.charge_temp_max, cfg.discharge_temp_max)
            crit_bits = filter_alarm_bits(
                p.alarm_bits, p.pack_voltage, p.temp_max_c,
                cells_for_filter, temp_limit,
                p.cell_max_v, p.sys_cell_high_v, p.sys_module_high_v)
            if crit_bits != 0:
                if crit_bits & UV_ALARM_BITS:
                    alarm_flags |= 0x10
                    if sys_error_msg == "OK":
                        sys_error_msg = "ALARM: PACK UNDERVOLT"
                else:
                    alarm_flags |= 0x40
                    if sys_error_msg == "OK":
                        sys_error_msg = "ALARM: BMS STATUS"
                _emit(events, events_overflowed, "BmsReportedAlarm",
                      bms_id=i, alarm_bits=crit_bits)

        # ── Sysparam protocol caps (lines 2128-2145) ───────────────────────
        sp_fresh = p.sysparam_valid and ((now_ms - p.last_sysparam_ms) < 300000)
        if sp_fresh:
            if p.sys_charge_max_a > 0.1:
                proto_ccl_cap += p.sys_charge_max_a
            if p.sys_discharge_max_a > 0.1:
                proto_dcl_cap += p.sys_discharge_max_a

            cells_for_cap = p.cell_count if p.cell_count > 0 else 0
            cvl_cap = p.sys_module_high_v - 0.20
            if cells_for_cap > 0:
                chem_cap = cells_for_cap * 3.60
                if cvl_cap > chem_cap:
                    cvl_cap = chem_cap
            # Cap against safe_pack_volt and cvl_voltage (lines 2139-2141)
            if cfg.safe_pack_volt > 40.0 and cvl_cap > (cfg.safe_pack_volt - 0.20):
                cvl_cap = cfg.safe_pack_volt - 0.20
            if cfg.cvl_voltage > 40.0 and cvl_cap > cfg.cvl_voltage:
                cvl_cap = cfg.cvl_voltage
            if cvl_cap > 40.0 and cvl_cap < proto_cvl_cap:
                proto_cvl_cap = cvl_cap

            cells_for_uv = cells_for_cap
            if _should_flag_proto_under_volt(p.pack_voltage, p.sys_module_under_v, cells_for_uv):
                proto_under_v_hit = True

            proto_count += 1

        # ── Pack / cell overvolt (lines 2146-2147) ────────────────────────
        if p.pack_voltage > cfg.safe_pack_volt:
            alarm_flags |= 0x02
            sys_error_msg = "ALARM: PACK OVERVOLT"   # no "OK" check; always overwrite
        if p.cell_max_v > cfg.safe_cell_volt:
            alarm_flags |= 0x02
            sys_error_msg = "ALARM: CELL OVERVOLT"   # no "OK" check; always overwrite

        # Overvolt edge events
        ov_now = (p.pack_voltage > cfg.safe_pack_volt) or (p.cell_max_v > cfg.safe_cell_volt)
        if ov_now:
            _emit(events, events_overflowed, "PackOvervoltStart", bms_id=i)

        # ── t_check_val update (line 2148) ────────────────────────────────
        # V2.67 overwrites with each online pack; last online pack wins.
        if cfg.temp_mode == "Hottest":
            t_check_val = p.temp_max_c
        else:
            t_check_val = p.temp_avg_c

        # ── Cell drift (line 2149) ────────────────────────────────────────
        if p.cell_drift_v > cfg.safe_cell_drift:
            alarm_flags |= 0x20
            if sys_error_msg == "OK":
                sys_error_msg = "WARN: CELL IMBALANCE"
            _emit(events, events_overflowed, "CellImbalanceStart", bms_id=i)

        # ── Accumulate (line 2150) ────────────────────────────────────────
        sum_i   += p.pack_current
        sum_v   += p.pack_voltage
        sum_soc += p.soc
        sum_soh += p.soh
        sum_t   += p.temp_avg_c
        sum_cap += p.full_ah
        sum_rem += p.rem_ah
        count   += 1

    # ── count > 0 path (lines 2154-2266) ─────────────────────────────────────
    if count > 0:
        pack_voltage_avg   = sum_v / count
        pack_current_total = sum_i
        soc_avg            = sum_soc / count
        soh_avg            = sum_soh / count
        temp_avg           = sum_t / count
        capacity_total_ah  = sum_cap
        capacity_remain_ah = sum_rem

        # Packs-online-recovered edge
        if not prev.was_packs_online_any:
            _emit(events, events_overflowed, "PacksOnlineRecovered")

        # ── Temperature factors (lines 2188-2189) ─────────────────────────
        factor_charge    = calc_factor(t_check_val, cfg.charge_temp_min, cfg.charge_temp_max)
        factor_discharge = calc_factor(t_check_val, cfg.discharge_temp_min, cfg.discharge_temp_max)
        safe_chg = count * cfg.charge_amps_per_pack * factor_charge
        safe_dis = count * cfg.discharge_amps_per_pack * factor_discharge

        # ── Protocol caps from sysparam (lines 2194-2205) ─────────────────
        if proto_count > 0:
            if proto_ccl_cap > 0.1 and safe_chg > proto_ccl_cap:
                safe_chg = proto_ccl_cap
            if proto_dcl_cap > 0.1 and safe_dis > proto_dcl_cap:
                safe_dis = proto_dcl_cap
            cvl_volts = proto_cvl_cap if proto_cvl_cap < 900.0 else cfg.cvl_voltage
        else:
            cvl_volts = cfg.cvl_voltage

        # ── Temperature stop flags (lines 2207-2208) ──────────────────────
        # Note: these OVERWRITE sys_error_msg without checking "OK" (V2.67 behaviour).
        if factor_charge == 0.0:
            alarm_flags |= 0x08
            sys_error_msg = "INFO: TEMP CHARGE STOP"
        if factor_discharge == 0.0:
            alarm_flags |= 0x08
            sys_error_msg = "INFO: TEMP DISCHG STOP"

        # Throttle edge events (V2.67 lines 2220-2223)
        if factor_charge < 1.0 and prev.prev_factor_charge >= 1.0:
            _emit(events, events_overflowed, "TempChargeStop")
        if factor_charge >= 1.0 and prev.prev_factor_charge < 1.0:
            _emit(events, events_overflowed, "TempChargeResume")
        if factor_discharge < 1.0 and prev.prev_factor_discharge >= 1.0:
            _emit(events, events_overflowed, "TempDischargeStop")
        if factor_discharge >= 1.0 and prev.prev_factor_discharge < 1.0:
            _emit(events, events_overflowed, "TempDischargeResume")

        # Alarm-flag edge events (V2.67 lines 2215-2219)
        if (alarm_flags & 0x02) and not (prev.prev_alarm_flags & 0x02):
            _emit(events, events_overflowed, "PackOvervoltStart")
        if not (alarm_flags & 0x02) and (prev.prev_alarm_flags & 0x02):
            _emit(events, events_overflowed, "PackOvervoltClear")
        if (alarm_flags & 0x10) and not (prev.prev_alarm_flags & 0x10):
            _emit(events, events_overflowed, "PackUndervoltStart")
        if not (alarm_flags & 0x10) and (prev.prev_alarm_flags & 0x10):
            _emit(events, events_overflowed, "PackUndervoltClear")
        if (alarm_flags & 0x08) and not (prev.prev_alarm_flags & 0x08):
            if factor_charge == 0.0:
                _emit(events, events_overflowed, "TempChargeStop")
            if factor_discharge == 0.0:
                _emit(events, events_overflowed, "TempDischargeStop")

        # ── Proto undervolt (lines 2209-2210) ─────────────────────────────
        if proto_under_v_hit:
            alarm_flags |= 0x10
            safe_dis = 0.0
            if sys_error_msg == "OK":
                sys_error_msg = "ALARM: PACK UNDERVOLT"

        # ── Lock to zero rule (line 2211) ─────────────────────────────────
        if (alarm_flags & 0x02) or (alarm_flags & 0x10) or (alarm_flags & 0x40):
            safe_chg = 0.0
            safe_dis = 0.0

        # ── SOC-based charge taper (lines 2226-2228) ──────────────────────
        if not cfg.maint_charge_enabled:
            if soc_avg >= 99.0:
                safe_chg = count * 2.0
            if soc_avg >= 100.0:
                safe_chg = 0.0
        else:
            maint_target = max(40.0, min(cfg.maint_target_voltage, cfg.safe_pack_volt))
            if pack_voltage_avg >= maint_target and safe_chg > count * 2.0:
                safe_chg = count * 2.0

        return SafetyState(
            cycle_id=snap.cycle_id,
            produced_ms=snap.produced_ms,
            cvl_volts=cvl_volts,
            ccl_amps=safe_chg,
            dcl_amps=safe_dis,
            pack_voltage_avg=pack_voltage_avg,
            pack_current_total=pack_current_total,
            soc_avg=soc_avg,
            soh_avg=soh_avg,
            temp_avg=temp_avg,
            capacity_total_ah=capacity_total_ah,
            capacity_remain_ah=capacity_remain_ah,
            factor_charge=factor_charge,
            factor_discharge=factor_discharge,
            alarm_flags=alarm_flags,
            sys_message=sys_error_msg,
            packs_online=count,
            packs_configured=snap.pack_count_configured,
            events=events,
            events_overflowed=events_overflowed[0],
        )

    else:
        # ── No packs online (line 2267) ────────────────────────────────────
        alarm_flags |= 0x80
        if prev.was_packs_online_any:
            _emit(events, events_overflowed, "NoPacksOnline")
        return SafetyState(
            cycle_id=snap.cycle_id,
            produced_ms=snap.produced_ms,
            cvl_volts=0.0,
            ccl_amps=0.0,
            dcl_amps=0.0,
            pack_voltage_avg=0.0,
            pack_current_total=0.0,
            soc_avg=0.0,
            soh_avg=0.0,
            temp_avg=0.0,
            capacity_total_ah=0.0,
            capacity_remain_ah=0.0,
            factor_charge=1.0,
            factor_discharge=1.0,
            alarm_flags=alarm_flags,
            sys_message=sys_error_msg,
            packs_online=0,
            packs_configured=snap.pack_count_configured,
            events=events,
            events_overflowed=events_overflowed[0],
        )


# ─────────────────────────────────────────────────────────────────────────────
# JSON serialisation helpers (for generate_cases.py / C++ test loader)
# ─────────────────────────────────────────────────────────────────────────────

def pack_to_dict(p: PackSnapshot) -> dict:
    return {
        "bms_id": p.bms_id,
        "online": p.online,
        "pack_voltage": p.pack_voltage,
        "pack_current": p.pack_current,
        "soc": p.soc,
        "soh": p.soh,
        "rem_ah": p.rem_ah,
        "full_ah": p.full_ah,
        "cell_count": p.cell_count,
        "cell_v": p.cell_v,
        "cell_min_v": p.cell_min_v,
        "cell_max_v": p.cell_max_v,
        "cell_avg_v": p.cell_avg_v,
        "cell_drift_v": p.cell_drift_v,
        "temp_max_c": p.temp_max_c,
        "temp_avg_c": p.temp_avg_c,
        "alarm_bits": p.alarm_bits,
        "last_alarm_ms": p.last_alarm_ms,
        "sysparam_valid": p.sysparam_valid,
        "last_sysparam_ms": p.last_sysparam_ms,
        "sys_cell_high_v": p.sys_cell_high_v,
        "sys_module_high_v": p.sys_module_high_v,
        "sys_module_under_v": p.sys_module_under_v,
        "sys_charge_max_a": p.sys_charge_max_a,
        "sys_discharge_max_a": p.sys_discharge_max_a,
    }


def snapshot_to_json(snap: SystemSnapshot) -> dict:
    return {
        "cycle_id": snap.cycle_id,
        "produced_ms": snap.produced_ms,
        "pack_count_configured": snap.pack_count_configured,
        "packs": [pack_to_dict(p) for p in snap.packs],
    }


def config_to_json(cfg: Config) -> dict:
    return {
        "bms_count": cfg.bms_count,
        "charge_amps_per_pack": cfg.charge_amps_per_pack,
        "discharge_amps_per_pack": cfg.discharge_amps_per_pack,
        "cvl_voltage": cfg.cvl_voltage,
        "safe_pack_volt": cfg.safe_pack_volt,
        "safe_cell_volt": cfg.safe_cell_volt,
        "safe_cell_drift": cfg.safe_cell_drift,
        "charge_temp_min": cfg.charge_temp_min,
        "charge_temp_max": cfg.charge_temp_max,
        "discharge_temp_min": cfg.discharge_temp_min,
        "discharge_temp_max": cfg.discharge_temp_max,
        "temp_mode": cfg.temp_mode,
        "maint_charge_enabled": cfg.maint_charge_enabled,
        "maint_target_voltage": cfg.maint_target_voltage,
    }


def prev_to_json(prev: PrevState) -> dict:
    return {
        "was_pack_online": prev.was_pack_online,
        "prev_alarm_flags": prev.prev_alarm_flags,
        "prev_factor_charge": prev.prev_factor_charge,
        "prev_factor_discharge": prev.prev_factor_discharge,
        "was_packs_online_any": prev.was_packs_online_any,
    }


def event_to_dict(e: Event) -> dict:
    return {"type": e.type, "bms_id": e.bms_id, "alarm_bits": e.alarm_bits}


def safety_to_json(s: SafetyState) -> dict:
    return {
        "cycle_id": s.cycle_id,
        "produced_ms": s.produced_ms,
        "cvl_volts": s.cvl_volts,
        "ccl_amps": s.ccl_amps,
        "dcl_amps": s.dcl_amps,
        "pack_voltage_avg": s.pack_voltage_avg,
        "pack_current_total": s.pack_current_total,
        "soc_avg": s.soc_avg,
        "soh_avg": s.soh_avg,
        "temp_avg": s.temp_avg,
        "capacity_total_ah": s.capacity_total_ah,
        "capacity_remain_ah": s.capacity_remain_ah,
        "factor_charge": s.factor_charge,
        "factor_discharge": s.factor_discharge,
        "alarm_flags": s.alarm_flags,
        "sys_message": s.sys_message,
        "packs_online": s.packs_online,
        "packs_configured": s.packs_configured,
        "event_count": len(s.events),
        "events": [event_to_dict(e) for e in s.events],
        "events_overflowed": s.events_overflowed,
    }
