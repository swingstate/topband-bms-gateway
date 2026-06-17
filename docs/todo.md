# TODO — Deferred Items

## ~~MQTT settings change without reboot~~ — likely resolved, verify

The original deferred-restart-on-MQTT-change item was addressed during the MQTT-crash
fix work: `mqtt::reconfigure()` was moved off the HTTP handler thread (the
"defer MQTT reconfigure out of HTTP handler" commit, per architecture §10 R9
post-Phase-I finding), and HA discovery was routed through `q_mqtt_publish`.

**Action:** verify whether an MQTT settings change (host/port/password/enable) still
forces a reboot, or whether it now reconfigures live. If it still reboots, the original
fix below still applies; if not, delete this item.

Relevant files:
- `src/web/handlers_config.cpp` — `handle_config_post()`, the `if (mqtt_changed)` block
- `src/app/housekeeping.cpp` — flag check + call to `reconfigure()`
- `src/mqtt/publisher.cpp` — `reconfigure()` / `stop()` already exist

---

## Drift chart persistence across reboots (optional)

Cell Drift is now a selectable chart metric, but unlike the other metrics it is
**ephemeral** — held in PSRAM companion arrays, lost on reboot. The other chart
metrics persist (history rings in LittleFS). This is an inconsistency: after a
reboot/OTA, Power/Voltage/etc. charts retain their history but Drift starts empty.

**To make Drift persistent:** add a `uint16_t` drift field (mV) to the on-disk
`HistoryFinePoint`/`HistoryCoarsePoint` structs. Cost: ~5.5 KB flash (LittleFS),
~5.5 KB PSRAM (buffers already in PSRAM). Requires on-disk format versioning so
existing history isn't discarded on upgrade (version the ring header + migrate, or
accept one-time history loss on the update that ships it).

Decision pending: worth it for consistency, or leave ephemeral (rebuilds in ~2 h
after a reboot, which is rare now that crashes are fixed).

---

## Phase J4 — Brand palette refresh (Indigo/Cream/Teal-Bright/Teal-Soft)

Replace the current Aubergine palette with the new Indigo-based one.
Patches only `web/ui/style.css` (CSS variables); all components already
use `var(--*)` tokens, so the change is centralized.

> Note: UI_VERSION is currently at `beta-mvp-12`. The palette refresh will
> need a bump (-> `beta-mvp-13` or similar) to bust the CSS cache.

### Brand core

```css
--brand-indigo:       #170C79;   /* sidebar / headings / primary dark */
--brand-cream:        #EFE3CA;   /* card surfaces / light theme bg */
--brand-teal-bright:  #56B6C6;   /* primary accent / info / active */
--brand-teal-soft:    #8ACBD0;   /* secondary accent / hover */
```

### Semantic colors

```css
--color-success:      #22A06B;   /* OK / online / charging-positive */
--color-warning:      #E89C5C;   /* caution / soft warning */
--color-alarm:        #E2574C;   /* critical alarm / error / bad value */
--color-critical:     #9B1B30;   /* severe critical / emergency */
--color-info:         var(--brand-teal-bright);
```

### Neutrals — light theme

```css
--neutral-50:         #FAF6E8;   /* page bg */
--neutral-100:        #EFE3CA;   /* card bg */
--neutral-200:        #E5D6B8;   /* hover / dividers */
--neutral-300:        #C9B898;   /* borders */
--neutral-500:        #807055;   /* muted text */
--neutral-700:        #3F3520;   /* secondary text */
--neutral-900:        #170C79;   /* primary text */
```

### Neutrals — dark theme

Dark-theme page background uses a desaturated indigo-near-black for
eye comfort over long viewing periods (full indigo `#170C79` was
visually fatiguing).

```css
--neutral-50:         #100C28;   /* deepest indigo-black */
--neutral-100:        #1A1638;   /* page bg */
--neutral-200:        #251E50;   /* card bg */
--neutral-300:        #352B68;   /* hover / borders */
--neutral-500:        #6B6088;   /* muted text */
--neutral-700:        #B0A8C8;   /* secondary text */
--neutral-900:        #EFE3CA;   /* primary text = cream */
```

### Semantic role mapping (both themes)

```css
--bg-page:            var(--neutral-50);
--bg-card:            var(--neutral-100);
--bg-card-hover:      var(--neutral-200);
--bg-sidebar:         var(--brand-indigo);  /* light theme */
/* Dark theme overrides --bg-sidebar to var(--neutral-50) */
--text-primary:       var(--neutral-900);
--text-secondary:     var(--neutral-700);
--text-muted:         var(--neutral-500);
--text-on-dark:       var(--brand-cream);
--text-on-dark-muted: rgba(239, 227, 202, 0.55);
--border-subtle:      var(--neutral-300);
--accent-primary:     var(--brand-teal-bright);
--accent-secondary:   var(--brand-teal-soft);
--shadow-card:        0 1px 3px rgba(23, 12, 121, 0.06),
                      0 4px 12px rgba(23, 12, 121, 0.04);
```

### Component-level use

- **Sidebar:** indigo bg, cream text, teal-bright active-stripe (3px left edge)
- **Status pills:** Charging=success-green, Discharging=indigo, Idle=neutral-300, Alarm=alarm-coral, Critical=critical-crimson
  - Note: topbar WiFi/BMS/MQTT/CAN pills currently follow a green=connected pattern — keep consistent with the new palette (success-green for connected states).
- **Cell bargraph:** normal=success-green, min=teal-bright, max=alarm-coral
- **Charts:** primary=teal-bright, secondary=teal-soft, threshold=alarm-coral
  - Note: charts now have 5 selectable metrics (Power/SOC/Voltage/Temp/Drift) across 2 tiles — ensure the palette covers whichever metric lands in either tile.
- **Buttons:** primary=indigo, destructive=alarm-coral
- **Bad-value cards:** 2px alarm-coral border for Warn/Error, 2px critical-crimson border for Critical
- **New since palette spec written:** Settings -> Hardware (board selector) section, OTA panel, per-pack/per-cell HA-related UI — all use `var(--*)` tokens, should pick up the palette automatically, but spot-check after.

### Update section 10.10 of architecture doc

The "Branding and visual identity" section currently documents
Cream/Aubergine. Replace with Indigo/Cream/Teal-Bright/Teal-Soft and
the dark-theme indigo-near-black bg.

---

## Phase H3a UI polish (deferred)

Three small iterations, pickable any time:

### 1. Show timestamps directly in alert rows

Currently timestamps only show on mouse-hover. Make them visible permanently.

- Right-aligned, muted color (`var(--text-muted)`)
- Relative format: "5 min ago", "2 h ago", "3 d ago"
- Keep mouse-hover with absolute time as secondary detail

### 2. Sidebar Alerts badge counter

Badge is missing.

- Count of CRITICAL + ERROR alerts in the last 24h
- Only count alerts the user hasn't seen yet
- "Seen" = localStorage timestamp updated each time the Alerts tab is opened
- Update on each `/api/alerts` poll
- Coral background for the badge

### 3. Click alert row -> detail modal

Rows currently aren't clickable. Add modal on click:

- Severity icon + source pill at top
- Full timestamp: wall-clock + monotonic uptime in ms
- Full message (untruncated)
- "Copy to clipboard" button
- Suppressed-duplicates note if applicable
- Brand-styled modal (same pattern as factory-reset confirm)

---

## Phase J5 — Release prep (V3.0.0)

- Final documentation pass: `api.md`, `mqtt.md`, `safety.md` updated to match shipped state
  (the MQTT docs especially — plain-text per-pack/per-cell topics, publish levels, HA sub-device model)
- `pio test -e native` Catch2 dependency fix
- Edge-case sweeps
- Release tag `v3.0.0` on `main`, GitHub release with SHA-256 in notes

---

## Candidate features / roadmap (post-3.0, not scoped yet)

Captured so they don't get lost. None are V3.0.0 release blockers — the system is
functionally complete as a monitoring + inverter-bridge gateway. Ranked roughly by
"how much a finished gateway is expected to have it." Whether each matters depends on:
(a) is this only for the owner's own install, or distributed to other users?
(b) should the gateway stay passive (monitor + forward + alert) or also actively control?
(c) do target users run Home Assistant (HA automations cover a lot) or need standalone?

### Higher value (failure/fault handling — relevant for an energy store)

- **Config backup / restore (own V3 config).** Export the current config as a JSON
  download and import it back (or onto a replacement device) so a dead unit doesn't mean
  re-entering everything by hand. Infrastructure is half-present (`/api/restore` existed
  for the dropped V2.67 migrator). This is the one most expected of a shipped device.
- **External notifications (no-HA path).** Alerts currently surface in the UI and over
  MQTT. A user without HA open won't know a pack went offline or cell drift went critical.
  Add an outbound push path: Telegram bot / Pushover / ntfy.sh / email. HA users get this
  via MQTT→HA automations already; this covers the standalone case. Energy-store faults
  unnoticed can get expensive/dangerous, so this rates high.

### Medium

- **Long-term history export / streaming.** History rings are bounded (720 fine /
  ~2000 coarse). Not enough for months-long capacity-degradation / SOH-trend analysis.
  CSV export exists; automated push to a TSDB (InfluxDB) or similar would close the gap
  for non-HA users. (HA Recorder covers HA users.)
- **Active control / conditioning (design-boundary decision).** Today the gateway is
  passive: reads, forwards, alarms — safety logic exists but no active regulation. Possible
  future: dynamic CVL/CCL adjustment, time-windowed charge limiting, throttle charge power
  on over-temperature. This is a deliberate boundary right now; revisit only if the gateway
  should become a regulator, not just a bridge. Needs careful safety review if pursued.

### Lower / nice-to-have

- **UI i18n.** UI is English-only; owner works in German. Relevant only if distributed.
- **Mobile-optimized UI / PWA.** UI is responsive (desktop-tested); a true mobile pass +
  installable PWA would help for phone checks. HA app covers HA users.
- **Multi-gateway aggregation.** Overview across several storage systems. HA solves this
  better than the gateway could — likely never in-scope.
- **Transport security (TLS).** Web UI is plain HTTP (hence the `crypto.subtle` fallback
  work), MQTT is plaintext 1883. Fine for a LAN device; becomes a real topic only if the
  gateway is ever exposed over the internet. HTTPS + MQTT-TLS would be the work.

---

## Optional follow-ups (no commitment)

- **LED driver:** the board selector exposes an LED pin (GPIO38) but V3 has no
  NeoPixel/RMT driver — the pin is stored and validated but inert. If a status LED
  is wanted, implement the driver; no config change needed when it ships.
- **Board selector in captive portal:** currently Settings-only. Could be added to the
  first-boot AP/captive-portal flow so board + WiFi are configured together on initial
  setup (saves one reboot). Not required — the device always boots reachable with
  Waveshare defaults, so pins are always settable from normal Settings.
- **WebSocket** over the snapshot bus (V3.1 — see section 12.5).

---

## Done (moved out of TODO)

- ~~Pylontech / SMA CAN protocols~~ — shipped (Phase J1+J2)
- ~~V2.67 backup JSON migrator~~ — **dropped** per owner decision
- ~~MQTT-load panic~~ — fixed, coredump-confirmed, 9h44m+ stable
- ~~RAM headroom~~ — ~127 KB DRAM reclaimed (history->PSRAM)
- ~~Chart timezone offset~~ — fixed (anchor t0_epoch to NTP)
- ~~Cell Drift chart metric~~ — shipped (ephemeral; persistence is the optional item above)
- ~~Task-HWM diag table~~ — shipped (was the H3a "deferred to V3.1" item, done early)
- ~~Board selector~~ — shipped (Waveshare/Manual; LilyGo excluded as incompatible)
- ~~Power-cut WiFi-wipe~~ — fixed (POWERON/BROWNOUT excluded from rapid-reset tally)
- ~~Config wiped on schema bump~~ — fixed (field-preserving v1->v2 migration)
