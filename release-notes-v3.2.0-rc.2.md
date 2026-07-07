# v3.2.0-rc.2

Second release candidate for the 3.2 line. Feature-complete for 3.2 and seeking
wider real-world validation before the stable `3.2.0`. This build rolls up
everything in `v3.2.0-rc.1` plus the changes below (the delta since rc.1 is the
Pylontech conformance audit, the direction-aware safety lockout, and the
Diagnostics page restructure).

## New in this release (delta since rc.1)

- **Safety: direction-aware charge/discharge lockout.** A protection alarm no
  longer locks the pack to zero in both directions. Cell over-voltage now blocks
  charging only, leaving discharge available so the pack can self-correct
  (discharging lowers cell voltage). Cell under-voltage mirrors this: it blocks
  discharging only, leaving charging available so the pack can recover. A latent
  bug where near-full SoC charge tapering could override a cold-temperature
  charge cutoff and trickle-charge a too-cold pack was also fixed. The dashboard
  and alert log now surface which direction is locked out and why.

  **NOT YET HARDWARE-VERIFIED BY THE MAINTAINER.** This change ships in this RC
  deliberately, for wider testing, ahead of the maintainer's own hardware
  validation. Testers should specifically exercise charge/discharge behavior
  during any voltage or temperature alarm (cell over-voltage, cell under-voltage,
  cold-charge cutoff) and confirm the inverter is allowed to move current in the
  safe direction and blocked only in the unsafe one. Please report results.

- **Pylontech CAN protocol conformance audit.** Every transmitted frame was
  checked byte-for-byte against the reference decoder used throughout this
  project's Pylontech work. No encoder bugs were found. This confirms the earlier
  fixes (the discharge/charge alarm-bit swap and the false undertemperature
  alarm) were the real issues, not surface symptoms of deeper protocol drift.

- **Diagnostics page restructure.** New and strengthened RS485, Battery/BMS, and
  CAN sections. Per-pack sysparam and cell-high-voltage fields are now visible in
  the UI (previously available over the API only). Duplicate fields were removed.

## Rolled up from rc.1 (already shipped in the 3.2 RC line)

- **CAN current follows the fused/shunt-led source.** Instantaneous current over
  CAN (0x356, all three protocols) uses the fused Battery Value Sources value,
  fixing the inverter showing "0.0 A" during real sub-amp current. Reporting
  only; charge/discharge limits and taper logic remain on raw BMS values.
- **SmartShunt "Consumed Ah"** available over MQTT (`{base}/shunt/consumed_ah`,
  HA entity `shunt_consumed_ah`) and on the Diagnostics page, as a read-only
  bank-level reference. Not fused into any dashboard, CAN, or display value.
- **CAN status pill** in the top bar shows the correct health color (green when
  healthy, matching the other status pills). Cosmetic only.

## Notes

- This is a release candidate. It is feature-complete for 3.2 and is seeking
  wider real-world validation before the stable `3.2.0`, especially for the
  direction-aware safety lockout above, which has not yet been hardware-verified
  by the maintainer.
- No configuration changes needed. Safe to update from any 3.2.0-preview or
  3.2.0-rc.1 build.
