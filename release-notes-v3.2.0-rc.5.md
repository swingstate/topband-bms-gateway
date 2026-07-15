# v3.2.0-rc.5

Third release candidate rollup for the 3.2 line since `v3.2.0-rc.2`, driven by a
real end-user report of a 24-48h unreachability issue. This build is under active
field-testing, particularly the WiFi reconnect fix — not yet the stable `3.2.0`.

## New in this release (delta since rc.2)

- **WiFi: fixed a bug where the gateway could become permanently unreachable
  after a WiFi outage longer than ~30 seconds**, requiring a manual power-cycle.
  The gateway now retries reconnecting indefinitely with capped exponential
  backoff and never auto-switches network mode at runtime. A new Diagnostics
  page section shows live WiFi state, retry count, and outage duration; the
  Alerts log records outage start, extended-outage check-ins, and recovery.

  **UNDER ACTIVE FIELD-TESTING.** This fix ships in this RC specifically to
  validate it against the original 24-48h unreachability report. Testers should
  exercise a WiFi outage well past 30 seconds (router reboot, AP power-cycle)
  and confirm the gateway reconnects on its own without a manual power-cycle.
  Please report results.

- **Fixed a bug where the persisted Alerts log was silently wiped on every
  boot.** A LittleFS read-path bug — unrelated to the WiFi fix itself but found
  while field-testing it — meant alert history did not survive a power-cycle.
  Alert history now correctly persists across reboots.

- **Diagnostics/Battery page: "Fills first" per-cell drift indicator restyled.**
  No longer overlaps or clips the cell bar; now plain inline text matching the
  existing "worst cell" convention. Hardware-verified by the maintainer.

## Notes

- This is a release candidate under active field-testing, particularly the
  WiFi reconnect fix. Not yet the stable `3.2.0`.
- No configuration changes needed. Safe to update from any 3.2.0-rc.1 or
  3.2.0-rc.2 build.
