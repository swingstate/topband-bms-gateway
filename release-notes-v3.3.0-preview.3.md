# Release Notes — v3.3.0-preview.3

**Status: PREVIEW — for field testing, not the stable 3.3.0**

Build: `3.3.0-preview.3 (b22e70a)` | Branch: `develop`

---

## What's in this release

- CI now actually runs the full automated test suite on every change, catching more bugs before release.
- Overcurrent conditions now only block the affected direction (charging or discharging), not both — matching the earlier fix for over-voltage and under-voltage. Still seeking field confirmation from `preview.1` testers.
- The Voltage chart on the dashboard now follows the same data source (SmartShunt or BMS) as the rest of the battery readings, instead of always showing BMS data.
- Cleaned up internal test coverage — no functional change.
- Fixed a bug where stale sensor data could keep influencing temperature-based safety limits longer than intended; it now expires the same way other safety data already did.

---

## Upgrading

OTA upgrade is supported from any 3.2.x or 3.3.0-preview build. No configuration changes are needed.

### Images in this release

- `Topband-bms-gateway-ota-v3.3.0-preview.3.bin` — OTA update (upload via the gateway's web interface).
- `Topband-bms-gateway-factory-v3.3.0-preview.3.bin` — full factory image for USB flashing. App partition offset is 0x20000.
- `SHA256SUMS-v3.3.0-preview.3.txt` — SHA-256 checksums for both images.

---

*This is a preview build for field testing, not the final 3.3.0 release.*
