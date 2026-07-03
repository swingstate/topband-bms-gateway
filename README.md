# TopBand BMS Gateway

ESP32-S3 firmware that bridges TopBand LiFePO4 BMS battery packs to Victron, Pylontech, or SMA inverters via CAN bus and to Home Assistant via MQTT. Includes a web dashboard, live charts, energy tracking, and Telegram notifications for safety alerts.

Compatible with TopBand-based batteries including EET, Power Queen, and others using the TopBand RS485 protocol.

![Version](https://img.shields.io/badge/version-3.1.0--preview.1-blue)
![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-orange)

> **New in V3.1: Bluetooth support.** The gateway can now read a Victron MPPT solar charger over Bluetooth LE. Its data shows up on the dashboard, is published to MQTT, and feeds into the energy view. More BLE devices (such as Victron SmartShunt) are planned for a later release.

## What's new in V3.1

V3.1 builds on V3.0 and adds:

- **Bluetooth LE / Victron MPPT.** Read solar charger data (PV power, charger output, yield, charger state) over BLE. No extra wiring — the ESP32-S3 reads the MPPT's advertisements directly.
- **Solar page.** A dedicated page with a day chart of solar power, the MPPT charger output, and a Solar-Passthrough indicator (for setups running OpenDTU-onBattery). All read-only; the gateway never controls the charger.
- **Battery Drift Details.** A per-pack, per-cell view of how the cells balance and drift over the last 5 days. Drift is measured where it actually matters on LiFePO4 — near full charge and near empty — instead of the flat middle of the charge curve, where all cells look the same.
- **Reworked Diagnostics page.** Bluetooth, WiFi, and MPPT are now in clearly separated sections.
- **Smaller fixes.** Charts update in place instead of redrawing (no more flicker), the login session stays valid across browser restarts, and internal memory use was reduced.

The CAN output, BMS polling, safety logic, and MQTT/Home Assistant integration are unchanged from V3.0. If you don't use a Victron MPPT, V3.1 behaves like V3.0.

## Architecture

![Architecture Overview](docs/bms_gateway_architecture_3_1.png)

## Screenshots

![Dashboard v. 3.1](docs/bms_gateway_dashboard_3_1.png)

Dashboard with live SOC, power, voltage, cell balance bars, and history charts. Glassmorphism UI with light and dark mode.

A standalone HTML demo is available at `docs/dashboard-demo.html`. Open it directly in any browser to preview the dashboard with synthetic data (no hardware needed). Useful for UI previews.

## Features

- **Multi-pack support** up to 16 BMS packs on one RS485 bus, with per-pack and per-cell data
- **Bluetooth LE** read a Victron MPPT solar charger; data flows into the dashboard, MQTT, and energy tracking
- **Web dashboard** with glassmorphism sidebar UI, light/dark mode, responsive mobile layout
- **Live charts** for power, voltage, SOC, temperature, and cell drift — with persistent history (2-hour fine, 7-day coarse)
- **Solar page** day chart of solar power, MPPT charger output, and Solar-Passthrough status
- **Battery Drift Details** per-cell balance and drift over 5 days, measured at the charge extremes
- **Energy tracking** rolling today / 7-day / monthly counters
- **MQTT publishing** with Home Assistant auto-discovery, per-pack and per-cell topics
- **Safety logic** cell voltage, drift, temperature cutoffs with hysteresis
- **CAN output** Victron, Pylontech, or SMA protocol — selectable at runtime
- **Battery config modes** Auto (from BMS parameters), Auto+Margin, or Manual
- **OTA firmware updates** with 5-minute self-test and automatic rollback on failure
- **Telegram notifications** for safety alerts — configurable debounce to prevent alert floods
- **Settings backup/restore** as JSON
- **CSV history export**
- **mDNS** access via `topband-gateway-macID.local`
- **Cookie-based authentication** with SHA-256 hashed password and rate limiting
- **Alert log** persisted to flash, with per-event severity and timestamp
- **Board selector** Waveshare preset or Manual pin entry for any qualifying ESP32-S3 board
- **Per-BMS communication statistics** polls/ok/timeout/errors
- **Tiered MQTT detail levels** off / per-pack statistics / per-cell voltages
- **Diagnostics panel** with gateway self-monitoring counters, stack high-water marks, and coredump capture

## Supported Hardware

V3.x requires an ESP32-S3 with **16 MB flash and 8 MB PSRAM**. Board type and GPIO pins are configured at runtime via the web UI. Bluetooth uses the ESP32-S3's built-in radio — no extra hardware needed.

### Tested Boards

| Board | RS485 | CAN | Preset |
|---|---|---|---|
| Waveshare ESP32-S3-RS485-CAN | GPIO 17/18/21 | GPIO 15/16 | Built-in |
| Custom ESP32-S3 | User-defined | User-defined | Manual pin entry |

For custom boards: select `Manual` in Settings → Hardware and enter your GPIO pin assignments. Set `DIR = -1` if your RS485 transceiver has automatic direction control.

> LilyGo T-CAN485 (classic ESP32, 4 MB flash, no PSRAM) is not compatible with V3.x.

## Installation

### Pre-built Binary (Recommended)

Download the release files from the [Releases page](../../releases).

**First install (USB):** write the factory image, which bundles the bootloader, partition table, and firmware in a single file.

```bash
# macOS / Linux
esptool.py --chip esp32s3 --port /dev/cu.usbserial-XXXX write_flash 0x0 Topband-bms-gateway-factory-v3.1.0-preview.1.bin

# Windows (adjust COM port)
esptool.py --chip esp32s3 --port COM3 write_flash 0x0 Topband-bms-gateway-factory-v3.1.0-preview.1.bin
```

If upgrading from V2.67.x, erase the flash first (new partition layout):

```bash
esptool.py --chip esp32s3 --port /dev/cu.usbserial-XXXX erase_flash
```

**OTA update (existing V3.x install):** open the web dashboard, go to Settings → OTA Firmware Update, and upload `Topband-bms-gateway-ota-v3.1.0-preview.1.bin`. The device reboots, runs a 5-minute self-test in the background, and rolls back automatically if the self-test fails.

> Upgrading from V2.67.x to V3.x via OTA is not supported. Use the USB factory image. Back up your V2 settings first (General → Maintenance → Export settings in V2), then restore them after V3 first boot.

### First Boot

1. The device starts a WiFi captive portal (SSID: `Topband-Setup-XXXX`)
2. Connect and configure your WiFi credentials at `192.168.4.1`
3. Access the dashboard at `http://topband-gateway.local` or via the device IP
4. Go to Settings → Hardware and select your board type
5. Save and reboot

## Configuration

### Basic Setup

Navigate to the Battery tab and configure:

- BMS count (1 to 16)
- Cells per BMS (0 = auto-detect)
- Battery config mode (Auto / Auto+Margin / Manual)
- Charge/discharge current limits
- Charge voltage limit (CVL)
- Safety cutoffs (cell max, pack max, drift)
- Temperature ranges for charge and discharge

### Auto-Config from BMS

Settings → Battery → Apply auto-config reads the BMS system parameter frame (0x47) and suggests safe values based on the manufacturer's limits. Choose Auto to apply directly, Auto+Margin to apply with a safety margin, or Manual to set all limits yourself.

### Bluetooth / Victron MPPT

Navigate to Network → Bluetooth:

- Enable Bluetooth and enter the MPPT's encryption key (from the VictronConnect app)
- Once paired, the MPPT's solar data appears on the Solar page and the dashboard
- This is read-only — the gateway never sends commands to the charger

### MQTT and Home Assistant

Navigate to Network → MQTT:

- Enable MQTT, set broker IP, port, credentials, and base topic
- Set detail level: off / per-pack statistics / per-cell voltages
- Enable HA discovery to register entities automatically
- Click Send HA discovery to push discovery messages immediately
- Optional: Configure the SolarPassThrough Topic if you have an OpenDTU Setup and want to see the status in the Dashboard. 

### Telegram Notifications

Navigate to Network → Notifications:

- Enable Telegram, enter your bot token and chat ID
- Use the Test button to verify delivery before saving
- Notifications are sent for safety events (overvoltage, undervoltage, temperature cutoff, imbalance) with configurable debounce to prevent alert floods

## Architecture

### Dual-Core Design

- **Core 0** `ControlTask` runs BMS RS485 polling, safety evaluation, and CAN TX in sequential phases. No network I/O touches Core 0.
- **Core 1** runs HTTP, MQTT, Bluetooth, history accumulation, and housekeeping tasks concurrently. Tasks are event-driven and independent of each other.

### Lock-Free Data Path

BMS snapshots travel from Core 0 to Core 1 through a seqlock double-buffer in PSRAM. Core 1 tasks read the latest snapshot without holding a mutex, eliminating the watchdog-reboot class of bugs that affected V2.67.

### Storage

- **NVS:** one versioned Config blob (~600 B), CRC-protected. Schema-migrated automatically on upgrade.
- **LittleFS:** history ring files (2-hour fine / 7-day coarse), persisted alert log, energy counters, and web UI assets.
- **PSRAM:** large history buffers, the solar day-ring, and per-cell drift history live in PSRAM, keeping internal RAM free.
- Session tokens are RAM-only and regenerated each boot.

### Web UI

Static files served from LittleFS via `esp_http_server`. The dashboard polls `/api/live` every 1.5 seconds by default. Charts render client-side and update in place on each poll. All configuration changes go through POST endpoints with CSRF protection.

### Alert System

Safety events are generated by `runSafety()` — a pure function on Core 0 with no I/O and no globals, fully unit-testable on host. Events are routed to the persisted alert log and, when configured, to Telegram via the notify module. Rising-edge debounced to suppress transient noise.

## Protocol

Based on reverse-engineering work from [linedot/topbands-bms](https://github.com/linedot/topbands-bms).

### Supported Commands

| CID2 | Function | Used by |
|---|---|---|
| 0x42 | Analog data (cell voltages, temperatures, SOC) | Main polling loop |
| 0x44 | Alarm/status bitmap | Round-robin every 15s |
| 0x47 | System parameters (limits from manufacturer) | Round-robin every 30s |
| 0x4F | Manufacturer info (HW/SW version) | Service page |
| 0xA1 | Historical events | Service page |
| 0xB1 | Date/time read | Service page |
| 0xB2 | Date/time write | Service page (manual sync) |

### CAN Output Frames

| ID | Content | Protocol |
|---|---|---|
| 0x351 | Charge/discharge voltage and current limits | Victron |
| 0x355 | SOC, SOH | Victron |
| 0x356 | Pack voltage, current, temperature | Victron |
| 0x359 | Alarm and warning flags | Victron |
| 0x35A | Protection and alarm bits | Victron |
| 0x35E | Manufacturer string | Victron |

Pylontech and SMA frame formats are selectable via Settings → Battery → CAN Protocol.

The Victron MPPT is read over Bluetooth, not CAN — it does not use these frames.

## Troubleshooting

### BMS not detected

- Check RS485 wiring (A to A, B to B, GND common)
- Verify 120 ohm termination at both ends of the bus
- Check BMS address dip switches (addresses 0 to 15)
- View the Diagnostics panel to see raw RS485 frames and per-BMS poll statistics

### CAN bus errors

- Check 120 ohm termination at both ends
- Verify CAN H and CAN L are not swapped
- Ensure inverter and gateway share ground
- Check CAN status in the dashboard header pill

### Bluetooth / MPPT not showing data

- Confirm the encryption key matches the one in the VictronConnect app
- The MPPT must be within Bluetooth range of the gateway
- Solar data only appears once the MPPT is actively charging or reporting; check the Bluetooth section on the Diagnostics page for advertisement counts
- The first solar chart point appears about 5 minutes after boot (the chart samples every 5 minutes)

### Safety lockout

- Triggered by cell voltage, pack voltage, drift, or temperature exceeding configured limits
- Clears automatically once the underlying condition resolves within normal range
- Check the Alert log for the specific event and timestamp
- If limits are triggering incorrectly, review the thresholds in the Battery tab

### Factory reset / password reset

Factory reset and password reset are done via the web UI: Settings → Factory Reset. If you cannot reach the UI, reflash the factory image via USB — this resets all settings to defaults.

## Compatibility Notes

### OTA Upgrades

OTA upgrades are supported between V3.x releases, including V3.0 to V3.1. The first install of V3.x requires a USB reflash with the factory image (new partition layout). After that, all updates can be done via OTA.

### Home Assistant Entities

V3.1 adds new MQTT entities for the Victron MPPT (solar power, charger output, yield, charger state). They register automatically via HA discovery. If you upgrade from V3.0, use Settings → Send HA Discovery to register the new solar entities.

V3.x MQTT topic names and payload fields differ from V2.67.x. Existing V2 HA dashboards need to be updated.

### Known Limitations

- No TLS for HTTP or MQTT connections (planned for a future release)
- Telegram is the only supported notification channel for now
- SD card logging is not supported
- The Victron SmartShunt is not yet integrated (planned for a later release)
- Drift history: completed days persist across reboots; the current day's partial data rebuilds after a restart

## License

MIT License. See [LICENSE](LICENSE) file for details.

## Credits

- Protocol reverse-engineering: [linedot/topband-bms](https://github.com/linedot/topband-bms)
- Original groundwork: [atomi23/Topband-BMS-to-CAN](https://github.com/atomi23/Topband-BMS-to-CAN)
- Earlier firmware lineage used [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) for captive portal; V3.x uses a custom implementation

## Contributing

Issues and pull requests welcome. Please include:

- Firmware version
- Board type and preset used
- Reproduction steps
- Serial log output if applicable

## Disclaimer

This is a DIY project. Battery systems involve high currents and can cause fire, injury, or death if misconfigured. Verify all safety limits before connecting to a live battery system. Test thoroughly with your own hardware before relying on the firmware for protection. The author and contributors accept no liability for damage, injury, or loss arising from use of this firmware.
