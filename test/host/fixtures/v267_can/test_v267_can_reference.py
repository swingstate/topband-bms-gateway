"""
Self-tests for v267_can_reference.py — 4 hand-traced cases.
Run: python -m pytest test_v267_can_reference.py -v
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from v267_can_reference import (
    SafetyStateInput,
    build_0x351, build_0x355, build_0x356, build_0x35A,
    build_0x35E_victron, build_all_victron,
)


def _state(**kwargs) -> SafetyStateInput:
    """Build a SafetyStateInput with sensible defaults; caller overrides via kwargs."""
    defaults = dict(
        cvl_volts=53.5, ccl_amps=120.0, dcl_amps=120.0,
        soc_avg=50.0, soh_avg=100.0, capacity_total_ah=400.0,
        pack_voltage_avg=50.1, pack_current_total=0.0,
        temp_avg=25.0, alarm_flags=0,
    )
    defaults.update(kwargs)
    return SafetyStateInput(**defaults)


# ── Case 1: Idle (hand-traced) ────────────────────────────────────────────────

def test_idle_0x351():
    """cv=int(53.5*10)=535=0x0217 → LE [0x17,0x02]; ccl=dcl=1200=0x04B0 → [0xB0,0x04]."""
    f = build_0x351(_state())
    assert f.id == 0x351
    assert f.data == bytes([0x17, 0x02, 0xB0, 0x04, 0xB0, 0x04, 0x00, 0x00]), \
        f"got {f.data.hex().upper()}"


def test_idle_0x355():
    """soc=50=0x32, soh=100=0x64, cap=int(400*10)=4000=0x0FA0 → LE [0xA0,0x0F]."""
    f = build_0x355(_state())
    assert f.id == 0x355
    assert f.data == bytes([0x32, 0x00, 0x64, 0x00, 0xA0, 0x0F, 0x00, 0x00]), \
        f"got {f.data.hex().upper()}"


def test_idle_0x356():
    """v=int(50.1*100)=5010=0x1392 → [0x92,0x13]; i=0; t=int(25*10)=250=0xFA → [0xFA,0x00]."""
    f = build_0x356(_state())
    assert f.id == 0x356
    assert f.data == bytes([0x92, 0x13, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00]), \
        f"got {f.data.hex().upper()}"


def test_idle_0x35A_no_alarms():
    """No alarm bits set → all bytes zero."""
    f = build_0x35A(_state())
    assert f.id == 0x35A
    assert f.data == bytes(8), f"got {f.data.hex().upper()}"


def test_0x35E_always_topband():
    """Manufacturer ID is always 'TOPBAND ' for Victron protocol."""
    f = build_0x35E_victron()
    assert f.id == 0x35E
    assert f.data == b"TOPBAND "


# ── Case 2: Charge case ───────────────────────────────────────────────────────

def test_charge_0x351_cv_bytes():
    """0x351 bytes 0-1: int(53.5*10)=535=0x0217 → LE [0x17, 0x02]."""
    f = build_0x351(_state(soc_avg=55.0, pack_current_total=20.0))
    assert f.data[0] == 0x17, f"byte0={f.data[0]:#04x}"
    assert f.data[1] == 0x02, f"byte1={f.data[1]:#04x}"


def test_charge_0x356_positive_current():
    """i=int(20.0*10)=200=0x00C8 → LE [0xC8, 0x00]."""
    f = build_0x356(_state(pack_current_total=20.0))
    assert f.data[2] == 0xC8, f"byte2={f.data[2]:#04x}"
    assert f.data[3] == 0x00, f"byte3={f.data[3]:#04x}"


# ── Case 3: Alarm case ────────────────────────────────────────────────────────

def test_alarm_overvolt_and_temp_stop():
    """alarm_flags=0x02|0x08: byte4 of 0x35A = 0x40|0x20 = 0x60."""
    f = build_0x35A(_state(alarm_flags=0x02 | 0x08, ccl_amps=0.0, dcl_amps=0.0))
    assert f.data[4] == 0x60, f"byte4={f.data[4]:#04x}"
    # Other bytes must be zero
    for i in range(8):
        if i != 4:
            assert f.data[i] == 0x00, f"byte{i}={f.data[i]:#04x} (expected 0)"


def test_alarm_bms_reported_and_undervolt():
    """alarm_flags=0x40|0x10=0x50: bit6→byte4|=0x80, bit4→byte4|=0x10 → byte4=0x90."""
    f = build_0x35A(_state(alarm_flags=0x40 | 0x10))
    assert f.data[4] == 0x90, f"byte4={f.data[4]:#04x}"


def test_alarm_no_packs_online():
    """alarm_flags=0x80: no mapping → byte4=0x00 (0x80 has no Victron CAN bit)."""
    f = build_0x35A(_state(alarm_flags=0x80))
    assert f.data[4] == 0x00, f"byte4={f.data[4]:#04x}"


def test_alarm_imbalance_only():
    """alarm_flags=0x20 (imbalance): no Victron CAN mapping → byte4=0x00."""
    f = build_0x35A(_state(alarm_flags=0x20))
    assert f.data[4] == 0x00, f"byte4={f.data[4]:#04x}"


# ── Case 4: Negative current (discharging) ────────────────────────────────────

def test_negative_current_encoding():
    """pack_current_total=-15.5 A: i=int(-155.0)=-155.
    Signed 16-bit LE two's-complement: -155=0xFF65 → bytes [0x65, 0xFF].
    """
    f = build_0x356(_state(pack_current_total=-15.5))
    assert f.data[2] == 0x65, f"byte2={f.data[2]:#04x}"
    assert f.data[3] == 0xFF, f"byte3={f.data[3]:#04x}"
    # Verify round-trip via int16 interpretation
    encoded = int.from_bytes(f.data[2:4], byteorder='little', signed=True)
    assert encoded == -155, f"decoded={encoded}"


def test_large_negative_current():
    """-40 A (case_03_discharge_4packs): i=int(-400.0)=-400=0xFE70 → [0x70, 0xFE]."""
    f = build_0x356(_state(pack_current_total=-40.0, soc_avg=45.0))
    encoded = int.from_bytes(f.data[2:4], byteorder='little', signed=True)
    assert encoded == -400, f"decoded={encoded}"
    assert f.data[2] == 0x70, f"byte2={f.data[2]:#04x}"
    assert f.data[3] == 0xFE, f"byte3={f.data[3]:#04x}"


# ── build_all_victron ordering ────────────────────────────────────────────────

def test_frame_order_and_ids():
    """build_all_victron returns 5 frames in order: 0x351, 0x355, 0x356, 0x35A, 0x35E."""
    frames = build_all_victron(_state())
    assert len(frames) == 5
    expected_ids = [0x351, 0x355, 0x356, 0x35A, 0x35E]
    for i, (f, eid) in enumerate(zip(frames, expected_ids)):
        assert f.id == eid, f"frame[{i}].id={f.id:#05x} expected {eid:#05x}"
        assert len(f.data) == 8, f"frame[{i}] has {len(f.data)} bytes"
