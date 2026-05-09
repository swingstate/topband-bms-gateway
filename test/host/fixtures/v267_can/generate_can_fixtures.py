"""Generate V2.67 CAN frame fixtures for byte-regression tests.

Reads each Phase D safety output JSON from fixtures/v267/out/,
extracts the SafetyState fields, runs them through
v267_can_reference.build_all_victron(), and writes 5 frames as JSON
for C++ (Catch2) test consumption.

Usage:
    cd test/host/fixtures/v267_can
    python generate_can_fixtures.py
    git add out/
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from v267_can_reference import SafetyStateInput, build_all_victron

# Phase D's safety output lives here (relative to this script's directory)
PHASE_D_OUT = os.path.join(os.path.dirname(__file__), "../v267/out")
OUT_DIR     = os.path.join(os.path.dirname(__file__), "out")


def safety_json_to_input(d: dict) -> SafetyStateInput:
    return SafetyStateInput(
        cvl_volts         = float(d["cvl_volts"]),
        ccl_amps          = float(d["ccl_amps"]),
        dcl_amps          = float(d["dcl_amps"]),
        soc_avg           = float(d["soc_avg"]),
        soh_avg           = float(d["soh_avg"]),
        capacity_total_ah = float(d["capacity_total_ah"]),
        pack_voltage_avg  = float(d["pack_voltage_avg"]),
        pack_current_total= float(d["pack_current_total"]),
        temp_avg          = float(d["temp_avg"]),
        alarm_flags       = int(d["alarm_flags"]),
    )


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)

    cases = sorted(f for f in os.listdir(PHASE_D_OUT) if f.endswith("_safety.json"))
    if not cases:
        print(f"No _safety.json files found in {PHASE_D_OUT}", file=sys.stderr)
        sys.exit(1)

    for case_file in cases:
        case_name = case_file.replace("_safety.json", "")   # e.g. "case_01_idle_4packs"

        with open(os.path.join(PHASE_D_OUT, case_file)) as f:
            safety = json.load(f)

        inp    = safety_json_to_input(safety)
        frames = build_all_victron(inp)

        out = {
            "case": case_name,
            "alarm_flags": inp.alarm_flags,
            "frames": [
                {
                    "id":       f"0x{frame.id:03X}",
                    "data_hex": frame.data.hex().upper(),
                }
                for frame in frames
            ],
        }

        out_path = os.path.join(OUT_DIR, f"{case_name}_can.json")
        with open(out_path, "w") as f:
            json.dump(out, f, indent=2)

        hex_strs = [frame.data.hex().upper() for frame in frames]
        print(f"  {case_name}:")
        for fid, hx in zip(["0x351","0x355","0x356","0x35A","0x35E"], hex_strs):
            print(f"    {fid} = {hx}")

    print(f"\n{len(cases)} fixtures written to {OUT_DIR}")


if __name__ == "__main__":
    main()
