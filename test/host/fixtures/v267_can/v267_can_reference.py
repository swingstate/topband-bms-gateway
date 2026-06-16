"""
V2.67 sendVictronCAN() reference — frame composition logic only.
Translated from legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350.

This is the canonical reference for Phase E byte-regression tests.
V3.0's C++ build_0xNNN() functions must produce byte-equivalent results
for the same SafetyState input.

Author: Claude Code, Phase E
Source: V2.67.2 sendVictronCAN() (lines 2332-2350)
"""

from dataclasses import dataclass
from typing import List


@dataclass
class SafetyStateInput:
    """Subset of SafetyState fields used by CAN TX (mirrors C++ definition)."""
    cvl_volts: float
    ccl_amps: float
    dcl_amps: float
    soc_avg: float
    soh_avg: float
    capacity_total_ah: float
    pack_voltage_avg: float
    pack_current_total: float
    temp_avg: float
    alarm_flags: int  # uint8


@dataclass
class CanFrame:
    id: int
    data: bytes  # exactly 8 bytes


def build_0x351(s: SafetyStateInput) -> CanFrame:
    """V2.67 lines 2333: charge voltage / charge-discharge current limits.

    cv  = int(cvl * 10)   stored LE bytes 0-1
    ccl = int(ccl * 10)   stored LE bytes 2-3
    dcl = int(dcl * 10)   stored LE bytes 4-5
    bytes 6-7 = 0
    """
    cv  = int(s.cvl_volts * 10)
    ccl = int(s.ccl_amps  * 10)
    dcl = int(s.dcl_amps  * 10)
    d = bytearray(8)
    d[0] = cv  & 0xFF;  d[1] = (cv  >> 8) & 0xFF
    d[2] = ccl & 0xFF;  d[3] = (ccl >> 8) & 0xFF
    d[4] = dcl & 0xFF;  d[5] = (dcl >> 8) & 0xFF
    return CanFrame(id=0x351, data=bytes(d))


def build_0x355(s: SafetyStateInput) -> CanFrame:
    """V2.67 lines 2334-2335: SOC / SOH / total capacity.

    soc = int(avgSOC)           stored LE bytes 0-1
    soh = int(avgSOH)           stored LE bytes 2-3
    cap = int(totalCapacity*10) stored LE bytes 4-5
    bytes 6-7 = 0
    """
    soc = int(s.soc_avg)
    soh = int(s.soh_avg)
    cap = int(s.capacity_total_ah * 10)
    d = bytearray(8)
    d[0] = soc & 0xFF;  d[1] = (soc >> 8) & 0xFF
    d[2] = soh & 0xFF;  d[3] = (soh >> 8) & 0xFF
    d[4] = cap & 0xFF;  d[5] = (cap >> 8) & 0xFF
    return CanFrame(id=0x355, data=bytes(d))


def build_0x356(s: SafetyStateInput) -> CanFrame:
    """V2.67 lines 2336-2337: pack voltage / current / temperature.

    v = int(avgVoltage * 100)    stored LE bytes 0-1 (unsigned)
    i = int(totalCurrent * 10)   stored LE bytes 2-3 (SIGNED 16-bit)
    t = int(avgTemp * 10)        stored LE bytes 4-5 (unsigned)
    bytes 6-7 = 0

    int() truncates toward zero — matches C's (int) cast.
    For negative i (discharging): Python & 0xFF and >> 8 on negative ints
    produce the same two's-complement bytes as C on ARM Cortex (sign-extending
    arithmetic right shift).  e.g. i=-155: bytes 2-3 = [0x65, 0xFF].
    """
    v = int(s.pack_voltage_avg   * 100)
    i = int(s.pack_current_total * 10)   # may be negative
    t = int(s.temp_avg           * 10)
    d = bytearray(8)
    d[0] = v & 0xFF;  d[1] = (v >> 8) & 0xFF
    d[2] = i & 0xFF;  d[3] = (i >> 8) & 0xFF   # Python & 0xFF handles negative two's-complement
    d[4] = t & 0xFF;  d[5] = (t >> 8) & 0xFF
    return CanFrame(id=0x356, data=bytes(d))


def build_0x35A(s: SafetyStateInput) -> CanFrame:
    """V2.67 line 2338: Victron alarm flag translation (Victron protocol only).

    Only byte 4 is written; bytes 0-3 and 5-7 remain 0.

    Bit mapping (alarm_flags → byte 4):
      0x01 → bit 7 (0x80)   — also set by 0x40 (both map to same output bit)
      0x40 → bit 7 (0x80)
      0x02 → bit 6 (0x40)   — pack/cell overvolt
      0x08 → bit 5 (0x20)   — temperature stop
      0x10 → bit 4 (0x10)   — pack/cell undervolt

    Bits 0x20 (imbalance) and 0x80 (no packs online) have no Victron CAN mapping.
    SMA-protocol-only extra bits (d[4] |= 0x02 / 0x01 for zero CCL/DCL) are
    skipped — Victron only in Phase E.
    """
    d = bytearray(8)
    af = s.alarm_flags
    if af & 0x01: d[4] |= 0x80
    if af & 0x40: d[4] |= 0x80
    if af & 0x02: d[4] |= 0x40
    if af & 0x08: d[4] |= 0x20
    if af & 0x10: d[4] |= 0x10
    return CanFrame(id=0x35A, data=bytes(d))


def build_0x35E_victron() -> CanFrame:
    """V2.67 line 2341: manufacturer ID = 'TOPBAND ' (8 bytes, trailing space)."""
    return CanFrame(id=0x35E, data=b"TOPBAND ")


def build_all_victron(s: SafetyStateInput) -> List[CanFrame]:
    """Returns all 5 frames in V2.67's send order: 0x351, 0x355, 0x356, 0x35A, 0x35E."""
    return [
        build_0x351(s),
        build_0x355(s),
        build_0x356(s),
        build_0x35A(s),
        build_0x35E_victron(),
    ]
