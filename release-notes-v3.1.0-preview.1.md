# Release Notes — v3.1.0-preview.1

**Status: TEST / PREVIEW — not for production use**

This is the first pre-release build for v. 3.1.0. It is intended for hardware evaluation only. Flash it, poke it, break it. The final 3.1.0 release follows after hardware sign-off.

Build: `3.1.0-preview.1 (0f3e644)` | Branch: `develop`

---

## What's new in 3.1

### Victron BLE integration (MPPT in v3.1,  SmartShunt planned for v3.2)

The gateway can now pull live data directly from Victron MPPT Solar Charger devices over Bluetooth LE — no additional hardware or Cerbo required. Enable from **Settings > BLE Sources**, enter the device MAC and optional encryption key. Once paired:

- MPPT readings (PV power, PV voltage, PV current, yield today, charger state) appear on the Solar page and Dashboard pill.
- All values are published to MQTT and auto-discovered in Home Assistant.
- v.3.2 SmartShunt readings (voltage, current, SoC, power) appear on the Dashboard and can be used as SOC source (vs. BMS).

### Solar day-chart

The updated  **Solar** page shows todays MPPT output. Data is stored in a PSRAM-backed ring buffer — survives reboots that don't cut power, zeroes cleanly on cold boot. The chart reuses the same renderer as the existing Cell Voltage chart, with a y-axis that auto-scales from zero.

### Battery Drift Details

Each pack card on the Battery page now has a **Details** panel (collapsible, defaults closed). It shows:

- Per-cell drift band over the last 5 days (min / max voltage seen, shown as a small bar)
- **Top-of-charge (ToC) spread** — cell delta observed at the highest SoC in the window (only recorded when pack SoC ≥ 95 %)
- **Bottom-of-discharge (BoD) spread** — cell delta at the lowest SoC in the window (only recorded when pack SoC ≤ 5 %)
- **First full / First empty** — timestamp of first ToC / BoD sample in the window

<img width="756" height="565" alt="Bildschirmfoto 2026-06-27 um 13 25 50" src="https://github.com/user-attachments/assets/00ce406e-9816-4a01-b525-410f92837cbf" />

ToC and BoD spreads are SoC-gated so they only capture meaningful imbalance data, not mid-range noise. The 5-day band gives a quick visual of which cells drift high or low over a charge cycle.

### Diagnostics page reorganized

The `/diag` page now groups fields into collapsible sections:

| Section | Contents |
|---|---|
| Bluetooth LE | Scanner state, device presence, connection quality |
| WiFi | SSID, BSSID, RSSI, channel, IP, reconnect count, BSSID lock status |
| MPPT | All PV fields, charger state, yield |
| Shunt v3.2 | Voltage, current, SoC, power, mode |

Several field labels were corrected and two fields were moved to their correct sections (BMS total current moved to Shunt; BSSID lock icon moved into WiFi row).

### WiFi: strongest-AP selection

When multiple access points share the same SSID, the gateway now connects to the one with the strongest signal rather than the first one found. On disconnect the gateway re-scans and picks the current best AP.

### MPPT Charger Output rename + Solar-Passthrough display + OpenDTU compatibility

The MQTT topic and HA entity previously named `solar_batt_*` are renamed to `solar_output_*` to accurately reflect that this is the charger's DC output to the bus, not the PV input. 
For Setups that run on **OpenDTU (onBattery)** -  A passthrough tile on the Solar page shows instantaneous solar-to-load power with a staleness indicator. 
Existing HA automations referencing the old entity IDs will need updating (HA ghost-cleanup runs automatically on first boot after upgrade).

### UI improvements

- **No chart flicker** — chart cards update data in place each poll cycle instead of tearing down and rebuilding the DOM node. Eliminates the visible flash on the Battery and Solar pages.
- **Persistent session cookie** — the login cookie now carries an explicit lifetime so you stay logged in across browser restarts. Previously the session expired whenever the tab closed.
- **Mobile layout** — single-column grid on narrow viewports, horizontal scroll on landscape, improved subpage centering.
- **Dashboard** — Battery column and Solar column swapped to match visual priority; MPPT pill turns green when charging.

---

## Footprint (3.1.0-preview.1)

| | Used | Available |
|---|---|---|
| Flash | 1 998 653 B (47.7 %) | 4 194 304 B |
| RAM (static) | 46 512 B (14.2 %) | 327 680 B |

---

## Known limitations / not yet in 3.1

- BLE pairing requires manual MAC entry (no scan-and-select UI yet).
- Solar day-chart data does not survive a cold power cut (PSRAM is volatile).
- No cell-level drift history export yet.

---

## Upgrading from 3.0.x

OTA upgrade is supported. Config schema migrates automatically. After upgrading:

1. If you use Home Assistant, trigger a MQTT discovery cycle (restart the gateway or toggle HA MQTT integration) to pick up the renamed `solar_output_*` entities.
2. The BLE Settings Subpage is new — existing installs have BLE disabled by default; enable only if you have a compatible Victron device.

---

*This is a preview build. Flash via OTA or USB. Do not deploy to a production system without hardware validation.*
