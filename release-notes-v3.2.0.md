# Release Notes — v3.2.0

This is the stable 3.2.0 release. It replaces all earlier 3.2 preview and
release-candidate builds and covers everything that changed since the last
stable release, v3.1.0.

Build: `3.2.0`

---

## New

**Bluetooth battery monitor support.** The gateway can now pair with a Victron
SmartShunt battery monitor over Bluetooth and use it alongside your existing
battery packs. Once paired, it gives you a whole-bank reading of current,
voltage, and charge percentage that can be more accurate than the packs alone,
especially at very low currents where the packs tend to round small readings
down to zero. A setting lets you choose whether the shunt or the packs lead on
the dashboard, or set it automatically so the shunt leads whenever it has a
fresh reading and the packs take over if it doesn't. A small label on the
dashboard always tells you which one is currently being shown. Later in the
3.2 line, a running total of amp-hours consumed (from the shunt's own
counter) was added as an extra reference number, available in Home Assistant
and on the Diagnostics page.

Charging and discharging decisions are never affected by the shunt — the
safety logic that protects your packs always uses the packs' own numbers, no
matter what the dashboard is showing.

**Preferred WiFi access point.** If you have more than one WiFi access point
using the same network name, you can now pin the gateway to a specific one
from the network settings, with a one-click option when scanning. If that
access point ever goes away, the gateway automatically falls back to
whichever one has the strongest signal, and re-locks to your preferred one as
soon as it's reachable again.

**Reorganized settings and diagnostics pages.** Inverter/CAN settings now
have their own page instead of sharing space with battery settings. Network
settings moved into the main Settings area (old bookmarks still work). The
Diagnostics page was reorganized into clearer sections for the battery link,
the network connection, and the inverter connection, and now shows a few
useful numbers that previously weren't visible anywhere in the interface.

**Clearer safety lockout behavior.** If one cell's voltage goes too high or
too low, the gateway now only blocks the direction that's actually unsafe.
For example, a cell that's too full now blocks charging but still allows
discharging, so the pack can bring itself back into a safe range on its own.
Previously a problem in either direction could block both charging and
discharging at once. The dashboard and alert history now show which
direction is currently blocked and why.

---

## Fixed

**A software update could silently wipe your settings.** Under specific
conditions, updating the firmware could reset the entire configuration back
to factory defaults without any warning — WiFi, MQTT, battery settings, all
of it. This is fixed, and updating no longer carries this risk.

**The gateway could go permanently unreachable after a WiFi outage.** If WiFi
was down for more than about 30 seconds, the gateway would eventually stop
trying to reconnect altogether, leaving it offline until someone manually
power-cycled it. It now keeps retrying indefinitely, with a backoff so it
doesn't hammer the network, until the connection comes back on its own.

**Alert history didn't survive a power cycle.** The log of past alerts (low
voltage, disconnects, and so on) was silently lost every time the gateway
rebooted, instead of just when the device was reset. Alert history now
correctly carries over across reboots and power outages.

**Several correctness bugs in the data sent to Pylontech-compatible
inverters (as used by Deye and others).** A perfectly healthy battery could
be reported to the inverter as refusing to discharge, or as demanding an
immediate charge, at the same time — both wrong, and on some inverters this
caused the battery to get permanently locked out of discharging even though
nothing was actually wrong. A related bug meant a healthy, room-temperature
battery could be reported as too cold to charge. The lowest safe discharge
voltage sent to the inverter was also always reported as zero instead of
your actual configured limit, and the number of connected battery packs
always showed as zero. All of these have been corrected and checked against
real inverter software, not just against our own code.

**A cold pack could still get a trickle charge it shouldn't.** A separate
safety rule meant to stop charging a too-cold battery could, in a narrow
case, be overridden by the "topping off a nearly full battery" logic. Fixed
so the cold-battery protection always wins.

**Per-pack numbers showing as "Unknown" in Home Assistant.** Some per-pack
current, charge-percentage, and power readings weren't being sent correctly
over MQTT, so Home Assistant showed them as unavailable even though the
gateway had good data.

**A newer battery monitor could report roughly half the correct voltage, and
its charge percentage could get stuck at 0%.** This affected shunts running
current Victron firmware; older units were unaffected. A shunt that has
genuinely never been set up in the Victron app now shows an honest "not yet
set up" status instead of a misleading 0%.

**Smaller fixes:** a few status indicators in the top bar used an
inconsistent color when healthy; a "fills first" label on the per-cell
balance chart could overlap the chart it was labeling; repeated warnings
appeared in Home Assistant logs when the solar charger was temporarily out
of range; and the aggregate charge percentage could briefly disagree between
MQTT, the dashboard, and individual pack values.

---

## Changed

**The numbers sent to your inverter can now use the more accurate battery
monitor reading, when one is connected.** The battery percentage and current
sent over the inverter connection now follow the same reading shown on the
dashboard — the battery monitor's, when it's connected and its data is
fresh, and the packs' otherwise. This mainly fixes very small currents (for
example, a battery quietly discharging at under half an amp) that the packs
alone would round down to zero and report to the inverter as "no current
flowing" even though the dashboard already showed the real number. This
only changes what gets reported — the safety limits that control how much
the inverter is allowed to charge or discharge remain governed strictly by
the battery packs, unaffected by any battery monitor.

If you don't have a battery monitor connected, none of this changes anything
for you.

---

## Upgrading

OTA (over-the-network) update is supported from any 3.0.x or 3.1.x build, and
from any 3.2 preview or release-candidate build. Settings carry over
automatically.

If you're coming from the older V2.67.x firmware, upgrade with a USB factory
image, not OTA — back up your V2 settings first and restore them after the
first boot on V3.

Bluetooth battery monitor support is off by default. If you don't have a
SmartShunt, nothing changes for you — the gateway behaves exactly as it did
in 3.1.

### Images in this release

- `Topband-bms-gateway-ota-v3.2.0.bin` — OTA update (upload via the gateway's
  web interface).
- `Topband-bms-gateway-factory-v3.2.0.bin` — full factory image for USB
  flashing. App partition offset is 0x20000.
- `SHA256SUMS-v3.2.0.txt` — checksums for both images.
