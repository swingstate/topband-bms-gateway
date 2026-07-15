# Release Notes — v3.2.0-preview.3

**Status: PREVIEW — for hardware evaluation, not the final 3.2.0**

This is a preview/test build of the 3.2 line, published for evaluation on real
hardware. It is not the final, stable 3.2.0 release. This note summarizes the
delta since `v3.2.0-preview.2` (an internal build); if you are coming from
`v3.2.0-preview.1`, all of the changes below are new to you.

Build: `3.2.0-preview.3` | Branch: `develop`

This is a focused round of Pylontech CAN fixes, all driven by real-world reports
and cross-checked against the actual Pylontech CAN parser that Deye / OpenDTU
users run (OpenDTU-onBattery).

---

## Fixes in this build

### Pylontech: discharge could appear disabled on a perfectly healthy battery

The Pylontech charge/discharge request frame (`0x35C`) had its **discharge-enable
and force-charge bits swapped**. The standard Pylontech layout that
Victron / Deye / SMA / OpenDTU decode is: bit 7 = charge enable, **bit 6 =
discharge enable**, **bit 5 = request force charge**. The firmware was writing
discharge-enable to bit 5 and force-charge to bit 6.

Against the standard decode, a completely healthy battery (no undervolt, discharge
current available) reported **discharge enabled = No** and **immediate charge
requested = Yes** at the same time — both wrong. On some inverters this also
produced a **permanent discharge lockout**, because the inverter only ever saw
discharge-enable asserted while the battery was actually in undervolt. Each flag
now sits on its correct bit. The underlying safety logic (when discharge is
genuinely disabled) is unchanged: a real cutoff still clears discharge-enable.

### Pylontech: discharge voltage limit was always 0.0 V

The Pylontech limits frame (`0x351`) left the **discharge voltage limit** field
(bytes 6-7) at zero, which an inverter reads as a 0.0 V floor — effectively
"discharge all the way to empty." It now carries the **configured pack
low-voltage cutoff** (the same value used for the pack's own low-voltage
protection), including during a discharge-disable state, where the separate
discharge-enable flag already signals the lockout. This matches how
OpenDTU-onBattery reads the field (bytes 6-7, scaled by 0.1).

### Pylontech: battery module count showed 0 ("Batteriemodule: 0")

The Pylontech alarm/status frame (`0x359`) never populated **byte 4, the battery
module count**, so inverters and monitors displayed a module count of 0. Verified
against OpenDTU-onBattery's Pylontech provider — the parser Deye / OpenDTU users
run — which reads that byte directly as its "Module Count" entity. The firmware
now reports the number of packs currently online, so a healthy 3-pack system
reads 3. The count tracks the packs actually communicating, consistent with the
online set the other frames aggregate over.

(For the technically curious: a widely-cited forum note pointed at a
`0x4200`/`0x7320` frame for module count. That is the Pylontech SC0500
high-voltage console protocol, which the low-voltage CAN parser these inverters
use does not read; the correct low-voltage field is `0x359` byte 4, which is what
this build populates.)

---

## Change: CAN state-of-charge now matches the dashboard

The state of charge (SOC) reported to the inverter over CAN now follows the same
**Combined SOC** shown on the dashboard — the Battery Value Sources fused value
(SmartShunt-led when its reading is fresh, BMS as the fallback) — across all three
inverter protocols (Victron, Pylontech, SMA). Previously CAN SOC was held to the
BMS average only.

Two things deliberately do **not** change:

- **State of health (SOH)** over CAN stays BMS-only — the shunt does not measure
  state of health.
- **The safety charge-taper logic stays strictly on the raw BMS SOC.** The fused
  value affects the *reported* SOC only; it never feeds the taper or any pack-level
  safety decision. These are intentionally separate.

If you do not have a SmartShunt configured, the CAN SOC is the BMS value exactly
as before.

---

## Footprint (3.2.0-preview.3)

| | Used | Available |
|---|---|---|
| Flash | 2 051 613 B (48.9 %) | 4 194 304 B |
| RAM (static) | 44 776 B (13.7 %) | 327 680 B |

Static RAM is unchanged from preview.1/preview.2.

---

## Upgrading

OTA upgrade is supported from any 3.0.x, 3.1.x, or earlier 3.2 preview build.
Config schema migrates automatically.

- These fixes affect the **Pylontech** CAN protocol specifically. Victron and SMA
  users get the CAN-SOC change; the Pylontech frame fixes do not apply to them.
- The CAN-SOC change only differs from previous behavior if you have a SmartShunt
  enabled and leading. With no shunt, CAN behavior is identical to before.
- Upgrading from V2.67.x is via USB factory image only (OTA from V2 to V3 is not
  supported). Back up your V2 settings first, then restore after V3 first boot.

### Images in this release

- `Topband-bms-gateway-ota-v3.2.0-preview.3.bin` — OTA update (upload via the
  gateway's web UI or `pio run -t upload --upload-port <gateway-ip>`).
- `Topband-bms-gateway-factory-v3.2.0-preview.3.bin` — full factory image for
  USB flashing. **App partition offset is 0x20000.**
- `SHA256SUMS-v3.2.0-preview.3.txt` — SHA-256 checksums for both images.

---

*This is a preview build for evaluation, not the final 3.2.0 release.*
