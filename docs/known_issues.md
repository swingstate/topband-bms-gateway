# Known Issues and Limitations — V3.0.0

This document covers known limitations, deferred features, and one-time
migration behaviours for the V3.0.0 release. It is intended to accompany the
GitHub release notes.

---

## Deferred features (planned for V3.1+)

**LED driver not active.**
The LED pin is configurable in Settings and is blocked against reserved GPIOs,
but no NeoPixel / RMT driver is included in V3.0. The pin assignment is stored
and will be used when LED support is added in a future release.

**ntfy notification provider not included.**
Only Telegram is supported in V3.0. ntfy push notifications are planned for
V3.1. The notification architecture is provider-agnostic; adding ntfy requires
only a new `INotifyProvider` implementation.

**Board selector not in the captive portal.**
The board preset (Waveshare / Manual) and GPIO pin map are configurable only
from the Settings page of the main dashboard. They are not accessible during
the first-boot captive portal setup. After initial WiFi configuration, navigate
to Settings → General → Hardware to set pins.

---

## Platform constraints

**ESP32-S3 only (16 MB flash, 8 MB PSRAM required).**
V3.0 targets ESP32-S3 with 16 MB flash and 8 MB PSRAM. Classic ESP32 (no PSRAM),
ESP32-C3, and S3 boards without PSRAM are not supported and will not boot
correctly. The LilyGo T-CAN485 (classic ESP32, 4 MB flash, no PSRAM) is not
supported.

**Manual mode requires physical GPIO knowledge.**
When using Manual board preset, incorrect GPIO assignments can cause bus lockup
or hardware damage. Use the reserved-GPIO blocklist (shown in the UI) and verify
your board schematic before saving.

**No TLS for web UI or MQTT.**
All HTTP and MQTT communication is plain (port 80 / port 1883). This is
appropriate for LAN use but is not suitable for direct internet exposure without
a VPN or TLS-terminating reverse proxy.

---

## Test suite

**Host Catch2 test suite has a known build issue.**
`cmake -B build_host -S test/host && cmake --build build_host` fails due to a
Catch2 CMake integration issue. This does not affect the firmware binary.
The embedded firmware test (`pio test -e native`) is also affected. Deferred
per owner decision. The firmware itself is not affected.

---

## One-time migration behaviours (upgrade to 3.0.0)

**Cell drift chart history is cleared once on upgrade.**
V3.0 uses a different storage format for the cell drift history ring buffer
(PSRAM companion arrays, persisted via LittleFS). On the first boot after
upgrading from any earlier version, the drift chart history will be empty.
Normal drift history accumulation resumes immediately and is persistent
thereafter.

**HA stale-entity cleanup on first MQTT connect.**
When the gateway first connects to the broker after a firmware upgrade,
it automatically publishes tombstone (empty payload) messages for any
HA discovery entities from the previous firmware version that no longer exist
in V3.0. This clears stale sensor/binary_sensor entries from Home Assistant.
The process runs once per firmware version and takes a few seconds.

**Config schema migration is automatic and non-destructive.**
On first boot after upgrade, the config schema is migrated from the old version
to v5. All settings from the previous firmware are preserved; new fields are
initialised to safe defaults. A backup export before upgrading is always
recommended.

---

## Operational notes

**MQTT password is not included in backup exports.**
For security, the MQTT broker password is never stored in the backup JSON.
After restoring a backup on a new device, re-enter the MQTT password in
Settings → Network → MQTT.

**WiFi password is not included in backup exports.**
Same rationale. WiFi credentials are not exported or imported. Re-enter on
the new device via the captive portal or Settings → Network → WiFi.

**Factory reset requires UI access.**
The V2.67 power-cycle-5× gesture has been replaced with a software rapid-reset
gesture (5× ESP_RST_SW resets in quick succession, which clears credentials and
WiFi). A full factory reset is available via Settings → System → Factory Reset
in the web dashboard.
