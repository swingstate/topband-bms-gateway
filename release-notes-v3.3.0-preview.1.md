# Release Notes — v3.3.0-preview.1

**Status: PREVIEW — for field testing, not the stable 3.3.0**

This is a preview/test build, published to get real-world feedback on one
change in particular before it becomes the stable 3.3.0 release.

Build: `3.3.0-preview.1 (7befc4b)` | Branch: `develop`

---

## What's in this release

**Automated tests now run on every change.** This is a behind-the-scenes
project improvement: the test suite that checks the gateway's logic before a
release now actually runs automatically, instead of silently failing to run.
It doesn't change anything you'll see, but it should mean fewer bugs slip
through in future releases.

**Overcurrent protection now only blocks the direction that's actually a
problem.** If the battery reports too much current while charging, the
gateway now stops charging but leaves discharging alone. If it reports too
much current while discharging, only discharging stops and charging still
works. Previously, an overcurrent reading in either direction stopped both
charging and discharging at once, even when only one of them was the actual
problem. This follows the same fix already made for over-voltage and
under-voltage protection in an earlier release.

---

## Needs field testing: the overcurrent fix

This change has **not yet been tested against a real overcurrent event**.
There isn't a safe way for the maintainer to trigger genuine overcurrent on a
real battery to verify it. The logic has been checked against unit tests and
a clean build, but not against real hardware seeing a real overcurrent
condition.

**If you experience — or can safely simulate — an overcurrent event on this
build, please check the Diagnostics page and report what you see.**
Specifically: did only the correct direction (charging or discharging) get
disabled, with the other direction still working? Please report both
successes and anything that looks wrong.

---

## Upgrading

OTA upgrade is supported from any 3.2.x build. No configuration changes are
needed.

This is a preview build for testing, not the stable `3.3.0`. It's safe to
run, but treat the overcurrent behavior above as unverified on real hardware
until confirmed by field reports.

### Images in this release

- `Topband-bms-gateway-ota-v3.3.0-preview.1.bin` — OTA update (upload via the
  gateway's web interface).
- `Topband-bms-gateway-factory-v3.3.0-preview.1.bin` — full factory image for
  USB flashing. App partition offset is 0x20000.
- `SHA256SUMS-v3.3.0-preview.1.txt` — SHA-256 checksums for both images.

---

*This is a preview build for field testing, not the final 3.3.0 release.*
