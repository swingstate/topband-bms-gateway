# Release Notes — v3.1.0

**Status: STABLE — production release**

This is the final, stable 3.1.0 release. It supersedes the `v3.1.0-preview.1` through `v3.1.0-preview.4` pre-release builds published for hardware evaluation. No functional changes were made for the final cut beyond version finalization; this note summarizes the complete delta since `v3.0.0`.

Build: `3.1.0 (14b9653)` | Branch: `main`

---

## What's new in 3.1

### Bluetooth LE / Victron MPPT solar charger

The gateway can now read a Victron MPPT solar charger directly over Bluetooth LE — no Cerbo, no extra wiring. Enable it from **Settings > BLE Sources**, enter the device MAC and encryption key. Once paired, PV power, PV voltage, PV current, yield today, charger output, and charger state show up on the dashboard, the new Solar page, and are published to MQTT with Home Assistant auto-discovery.

BLE scanning is coexistence-hardened against WiFi: the scanner pauses during TLS handshakes, and NimBLE runs observer-only (no advertising or connection overhead). Several rounds of coexistence fixes during the preview cycle eliminated WiFi association drops seen in early dev builds.

### Solar page

A dedicated **Solar** page shows a day chart of solar power, the MPPT charger's DC output, and a Solar-Passthrough indicator for setups running OpenDTU-onBattery. The gateway only reads the charger — it never controls it.

The MQTT topic and HA entity previously named `solar_batt_*` are renamed to `solar_output_*` to correctly reflect that this is the charger's output to the DC bus, not the PV input. Existing HA automations referencing the old entity IDs will need updating; a ghost-entity cleanup runs automatically on first boot after upgrade.

### Battery Drift Details

Each pack card on the Battery page has a collapsible **Details** panel showing per-cell drift over the last 5 days, measured where it actually matters on LiFePO4 — near full charge and near empty, not the flat middle of the curve. Metrics are SoC-gated (recorded only at pack SoC >= 95% for "at full" and <= 5% for "at empty") so they capture real imbalance instead of mid-range noise.

The panel went through a redesign during the preview cycle to fix a confusing interaction: the status pill (Balanced / Monitor / Imbalanced) used to be based on the last full-charge spread, so a pack that was perfectly balanced right now could still show "IMBALANCED at full" — contradicting the live battery card next to it. The pill now always reflects the **live spread**, matching what the battery card shows, and can never contradict the present state. The expanded view is now **Now / At full / At empty / Trend**, always in that order, with the per-cell live-reading dot enlarged and given a contrasting ring so it's easy to find against the drift band color. The 5-day per-cell band, day-aware drift rate, and completed-day ring persist across reboots.

### Diagnostics page reorganized

The `/diag` page groups fields into collapsible sections — Bluetooth LE, WiFi, MPPT, Shunt — instead of one long flat list. Units are stated honestly (no implied precision the underlying sensor doesn't have), mislabeled and misplaced fields were corrected (BSSID lock indicator moved into the WiFi section, total current moved into Shunt), and the ESP32-S3 CPU temperature was added. Coredump detection now probes once at boot instead of on every `/api/diag` request, and stale coredumps that can't be decoded are erased automatically instead of lingering.

### WiFi: strongest-AP selection 

When multiple access points share the same SSID, the gateway connects to the strongest one instead of the first one seen. On disconnect it re-scans and reconnects to the current best AP.

### RS485 responder-address validation (safety-relevant correctness fix)

A checksum-valid RS485 frame can still be a **late reply** — a pack times out, then its response arrives while the gateway is already polling the next pack. Without an address check, that stale frame was silently attributed to the wrong pack, feeding foreign cell/alarm/sysparam data into the safety aggregation for that slot. All three RS485 reply paths (analog, alarm, sysparam) now verify the responder address matches the pack that was actually polled and discard the frame otherwise. Discards are counted in a new `wrong_addr` stat, visible at `/api/diag` (`poller.wrong_addr`) and on the Diagnostics page — expected to stay at 0 on a healthy bus.

### UI and stability fixes

- **No chart flicker** — chart cards update data in place each poll instead of tearing down and rebuilding the DOM, eliminating the visible flash on Battery and Solar pages.
- **Persistent session cookie** — the login session now carries an explicit lifetime, so you stay logged in across browser restarts instead of being logged out whenever the tab closes.
- **No build-version churn** — `UI_VERSION` is derived from a SHA-256 content hash of the web assets. Rebuilding firmware without touching web files leaves the generated provisioner header unchanged, so the working tree stays git-clean after a routine `pio run`.
- Mobile layout improvements: single-column grid on narrow viewports, horizontal scroll in landscape, better subpage centering.

### Memory

- `BmsSystemSnapshot` and the `/api/live` JSON document moved to PSRAM, freeing internal DRAM previously under pressure.
- The app-side MQTT task stack was raised from 6144 to 8192 bytes after field high-water-mark data showed a 256-byte overflow risk.
- A TLS regression introduced mid-preview-cycle (`SPIRAM_MALLOC_ALWAYSINTERNAL` lowered to 8192, which silently moved the mbedTLS input buffer to PSRAM and broke TLS on some AP configurations) was caught and reverted to 16384 before this release.

---

## Footprint (3.1.0 final)

| | Used | Available |
|---|---|---|
| Flash | 2 025 277 B (48.3 %) | 4 194 304 B |
| RAM (static) | 44 696 B (13.6 %) | 327 680 B |

---

## Upgrading from 3.0.x or a 3.1.0 preview

OTA upgrade is supported from any 3.0.x or 3.1.0-preview build. Config schema migrates automatically. After upgrading:

1. If you use Home Assistant, trigger a MQTT discovery cycle (restart the gateway or toggle the HA MQTT integration) to pick up the renamed `solar_output_*` entities.
2. The BLE Sources page is new since 3.0 — existing installs have BLE disabled by default; enable only if you have a compatible Victron MPPT.
3. If you were running a 3.1.0-preview build with BLE/WiFi coexistence issues, this release includes the full set of coexistence fixes; no special action needed beyond the OTA upgrade itself.

Upgrading from V2.67.x is via USB factory image only (OTA from V2 to V3 is not supported). Back up your V2 settings first (General → Maintenance → Export settings), then restore them after V3 first boot.

---

*This is the final 3.1.0 release, not a preview build.*
