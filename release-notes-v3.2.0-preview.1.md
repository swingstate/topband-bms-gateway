# Release Notes — v3.2.0-preview.1

**Status: PREVIEW — for hardware evaluation, not the final 3.2.0**

This is a preview/test build of the 3.2 line, published for evaluation on real
hardware. It is not the final, stable 3.2.0 release. This note summarizes the
complete delta since `v3.1.0`.

Build: `3.2.0-preview.1 (5cd9d30)` | Branch: `develop`

---

## What's new in 3.2

### SmartShunt integration (Bluetooth LE)

The gateway can now read a Victron SmartShunt directly over Bluetooth LE, giving
it a true bank-level view of current, voltage, and state of charge to sit
alongside the per-pack BMS data. Enable it from **Settings > Bluetooth LE** by
entering the shunt's MAC and encryption key.

Which value gets shown is governed by a new **Battery Value Sources** policy
(**Settings > Battery**): in Auto mode the shunt leads for display when its
reading is fresh, and the BMS is always the fallback when the shunt is disabled,
absent, or stale; Manual mode lets you fix the source. A small source badge
("SHUNT" / "BMS") on the affected dashboard and Battery tiles always discloses
which one is currently feeding the number.

One rule never changes regardless of source policy: the **safety-critical
charge-taper logic, per-pack SOC, and CAN transmission to the inverter always
use the BMS value**, never the fused shunt value. The shunt influences display
and bank-level SOC only; it can never override the pack-level safety
aggregation.

### SmartShunt decode fix (found during integration testing)

Victron's 2022+ SmartShunt firmware broadcasts a bit-packed advertisement rather
than the older byte-aligned layout. The initial decode read the legacy layout
for both, so on a real 48 V bank the voltage came out roughly half the true
value (it was actually reading the shunt's time-to-go field) and the SOC could
stick at 0.0%. This is now decoded correctly with a dedicated bit-packed
decoder, verified against a known-good reference implementation with host unit
tests. A shunt that has genuinely never been synchronized via VictronConnect now
shows an honest **Not synced** state instead of a misleading "0.0%", and an
unsynced reading is never promoted to the primary bank SOC.

### Manual WiFi BSSID pinning, with safe auto-fallback

**Settings > Network** now exposes a "Preferred Access Point" pin: with multiple
APs sharing one SSID (or a mesh), you can lock the gateway to a specific AP's
BSSID. The WiFi scan list shows each network's BSSID with a one-click "pin this
AP" action, and the connection status reads as a clear three-state
(not configured / pinned and connected / pinned but unreachable — falling back
to auto-select). If the pinned AP becomes unreachable the gateway automatically
falls back to strongest-AP auto-select and re-arms the pin on the next
successful reconnect, so it can never get stuck offline because of a pin. The
underlying pin mechanism shipped in 3.1; 3.2 adds the UI and the automatic
re-pin.

### Dedicated CAN settings page

Inverter/CAN protocol settings now live on their own **Settings > CAN** page,
separated from the Battery settings they used to share, so the two concerns no
longer crowd each other.

### Network settings folded into Settings

The former standalone Network page is now a section within **Settings**,
consistent with the other settings groups. Old `/network` links and bookmarks
redirect to it automatically.

### Dashboard and Battery page honesty fixes

- The charge/discharge state pill now reflects the **active current source**.
  Previously it could show "Idle" while the shunt was reporting real current.
- Source badges now appear on **Energy Today**, **Combined Power**, and the
  history charts, so it's always clear which source produced a given figure.
  Energy Today follows the active Battery Value Sources policy; the Cell Drift
  and Voltage history charts are labeled "BMS" to reflect the data they
  actually plot (drift is per-cell and bank-level shunt data can't inform it).
- Misleading "0%" / "0.00 V" placeholders are replaced with honest **no-data**
  states when a source is offline.
- The top status-bar SHUNT pill now uses the standard health-pill green like
  every other pill in that bar, instead of borrowing the purple source-badge
  color.

### Bluetooth LE key entry and diagnostics

- The encryption-key fields (both SmartShunt and MPPT) now extract the 32-hex
  key out of noisy pasted input — a label, spaces, a trailing period picked up
  from a phone photo's Live Text / OCR — validate it live as you type, and
  reject input with no recoverable key instead of silently storing junk.
- The diagnostics BLE panel gains a per-device **Key valid** row and a shunt
  decode funnel (type / MAC / decrypt counters), so a misconfigured shunt is
  distinguishable at a glance from a dead radio.

---

## Important fix: config could silently reset to factory defaults on upgrade

A config-migration bug could **silently reset the entire device configuration to
factory defaults** when upgrading across a schema version that changed the config
struct's size.

`loadConfig()` sized its NVS read buffer from `sizeof(Config)` — the struct size
of the *running* firmware. Schema v11 (Battery Value Sources) is the first schema
change to ever shrink the struct (884 B to 880 B, a mid-struct field removal
that closed an alignment gap), so an upgrading device's still-larger v10 blob no
longer fit the read buffer. The read was rejected, treated identically to
"config not found," and the whole config — not just the couple of fields that
happened to get noticed — reset to defaults on first boot after the upgrade.

The fix probes the stored blob's actual size before reading instead of assuming
it fits the current struct. New regression coverage exercises the **real NVS
storage path** end to end (the previous migration tests only exercised the
in-memory struct deserializer with a correctly-sized buffer, which is exactly why
they never caught this), and the v10-to-v11 test now uses a fully populated v10
fixture and asserts every field survives.

**If you upgraded through any of the affected 3.2 dev builds and lost settings,
re-check your configuration once on this preview.1 build; from here forward the
migration path is covered.**

---

## Footprint (3.2.0-preview.1)

| | Used | Available |
|---|---|---|
| Flash | 2 051 529 B (48.9 %) | 4 194 304 B |
| RAM (static) | 44 776 B (13.7 %) | 327 680 B |

---

## Upgrading

OTA upgrade is supported from any 3.0.x or 3.1.x build. Config schema migrates
automatically, and with the fix above that migration no longer risks a factory
reset. After upgrading, re-check your configuration once if you had passed
through an affected dev build.

- SmartShunt support is new in 3.2 and is off by default; enable it under
  **Settings > Bluetooth LE** only if you have a SmartShunt. With it disabled,
  behavior is identical to 3.1 — the BMS remains the sole source.
- Upgrading from V2.67.x is via USB factory image only (OTA from V2 to V3 is not
  supported). Back up your V2 settings first, then restore after V3 first boot.

### Images in this release

- `Topband-bms-gateway-ota-v3.2.0-preview.1.bin` — OTA update (upload via the
  gateway's web UI or `pio run -t upload --upload-port <gateway-ip>`).
- `Topband-bms-gateway-factory-v3.2.0-preview.1.bin` — full factory image for
  USB flashing. **App partition offset is 0x20000.**
- `SHA256SUMS-v3.2.0-preview.1.txt` — SHA-256 checksums for both images.

---

*This is a preview build for evaluation, not the final 3.2.0 release.*
