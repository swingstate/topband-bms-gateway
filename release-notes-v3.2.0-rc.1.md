# v3.2.0-rc.1

Release candidate for the 3.2 line. This is the consolidated, feature-complete
build for 3.2 and is now undergoing final validation before the stable `3.2.0`.
It rolls up everything since `v3.2.0-preview.4` plus the changes below.

## New in this release

- **CAN current now follows the same fused/shunt-led source as the dashboard.**
  The instantaneous current reported over CAN (0x356, all three protocols:
  Victron, Pylontech, SMA) now uses the fused Battery Value Sources value:
  shunt-led when the SmartShunt is fresh, BMS-derived otherwise. This fixes the
  inverter showing "0.0 A" while real sub-amp current was flowing (the BMS
  reports 0.0 A below ~0.5 A) even though the dashboard already read the true
  current. Reporting only: charge/discharge limits and the charge-taper safety
  logic remain strictly on the raw BMS values and are unchanged.

- **SmartShunt "Consumed Ah" now available over MQTT and on the Diagnostics
  page.** The shunt's own hardware Coulomb counter (negative = discharged) is
  published on `{base}/shunt/consumed_ah` (Home Assistant entity
  `shunt_consumed_ah`) as a bank-level, read-only reference. It is deliberately
  not fused into any dashboard, CAN, or display value. When the shunt is stale or
  not yet synced the publish is skipped so the entity goes unavailable rather than
  reporting a stale number.

- **CAN status pill in the top bar now shows the correct health color.** A
  healthy CAN bus renders green like the WiFi/BMS/MQTT/MPPT/SHUNT pills instead of
  the source-badge palette. Cosmetic only.

## Notes

- No configuration changes needed. Safe to update from any 3.2.0-preview build.
- This is a release candidate. Please report any issues so they can be addressed
  before the stable `3.2.0`.
