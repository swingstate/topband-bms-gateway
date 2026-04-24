# Changelog

All notable changes to TopBand BMS Gateway are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).

## [2.67.2] - 2026-04-24

GUI-Cleanup-Release. Reine UI-Arbeit, keine funktionalen Änderungen am MQTT-, RS485-, CAN- oder HA-Discovery-Pfad. OTA-kompatibel, keine Breaking Changes, keine NVS-Schema-Änderungen.

### Changed

- **Content skaliert jetzt auf volle Fensterbreite.** Die `.pg` Klasse hatte `max-width:1400px` hardcoded, was auf breiten Monitoren einen leeren rechten Rand erzeugte. Constraint entfernt. Metric Cards, Charts und BMS-Pack-Cards nutzen das verfügbare Fenster jetzt vollständig über die existierenden `auto-fit` Grids.
- **Einheitliche Button-Größen überall.** Zuvor hatten diverse Sekundär-Buttons inline `style="padding:4px 10px;font-size:11px"` Overrides, Primär-Buttons hatten `.bo` Standard-Padding. Das führte zu visuell ungleichen Button-Paaren. Gelöst via zwei neuer Klassen: `.bo` bekommt `min-height:34px` für konsistente vertikale Ausrichtung, `.bo.sm` ist die offizielle kompakte Variante für Copy-/Close-Actions. Alle inline-style Button-Overrides entfernt.
- **MQTT Section neu strukturiert (Network-Tab).** Fünf klare Subsections: **Broker** (Connection Config), **Home Assistant** (Auto-Discovery + Send/Clear Buttons), **Battery data (always on)** (Erklärung was immer publisht wird), **Extended battery monitoring (optional)** (drei vertikal gestackte Level-Optionen mit Beschreibung unter jedem Radio, keine Überlappungen mehr), **Gateway self-monitoring (optional, separate)** (klar abgegrenzt als ESP32-Health-Daten, nicht Batterie-Daten). Neue `.lvl-opt` Klasse für Radio-mit-Beschreibung-Pattern, neue `.btn-row` Klasse für Button-plus-Description-Zeilen.
- **Level-Labels in Klartext.** Bisher zeigte die UI "L1 lean (default)", "L2 + per-BMS stats", "L3 + cells topic (20s)". Jetzt "Off", "Per-pack statistics", "Additional per-cell voltages" mit ausführlicher Beschreibung darunter. Interner NVS-Key `mq_level` bleibt unverändert bei 0/1/2.
- **General Diagnostics Section neu strukturiert.** Log bekommt jetzt eine Überschrift ("Log (live)") und den Copy-Button rechtsbündig auf gleicher Höhe. Frame-Spy und Runtime-Counters Buttons stehen jetzt in `.btn-row` Grid mit fester linker Spaltenbreite (180px), Beschreibungstext rechts davon, beide Buttons exakt gleich breit. Frame-Spy Button kürzer: "Start RS485 Frame Spy" zu "Frame spy".
- **"Show diag counters" zu "Runtime counters" umbenannt.** Klarerer Name, Runtime-Counters-Panel bekommt jetzt auch einen eigenen Header im Panel selbst.
- **About Section in drei Subsections geteilt.** Build info (fix, ändert sich nur bei Release), Runtime state (Uptime, Heap, Flash, NVS, Boot-Reason, alles was sich ändert), Hardware (Device-ID, Chip, GPIO, alles statisch an dieses Board gebunden). `NVS usage` Row aus dem Diagnostics-Block nach About verschoben.
- **Redundanz zwischen Runtime Counters und About Section eliminiert.** Diag-Counters-Panel zeigte 8 Werte die bereits in About standen (Firmware, Uptime, Boot-Reason, Free-Heap, Heap-Min, NVS-Used, NVS-Free, Session-Age-Days). Aus dem UI-Panel entfernt. Server-seitige Publishes auf MQTT `/diag` und HA Auto-Discovery bleiben unverändert bei 24 Keys, nur die Web-UI-Ansicht zeigt jetzt 16 Runtime-spezifische Counter.
- **Board-Name präzisiert.** Dropdown-Label "Waveshare ESP32-S3" zu "Waveshare ESP32-S3 RS485/CAN Controller" erweitert, entspricht dem vollen Produktnamen.
- **Maintenance Section als `.btn-row` Grid.** Firmware update, Download backup, Restore backup, Export history standen bisher als horizontale Button-Reihe mit unterschiedlichen Breiten. Jetzt vier vertikale Grid-Zeilen mit je 180px breitem Button links und Beschreibungstext rechts. Konsistent mit Network > Home Assistant und General > Tools.
- **Battery Tab Buttons ebenfalls harmonisiert.** "BMS diagnostics" (Toggle für Service-Panel) und "Auto-configure from BMS" in `.btn-row` Grid umgesetzt, letzterer zu "Auto-configure" gekürzt.
- **Alerts Clear Button auf `.bo.sm`.** Konsistent mit dem Header-Action-Pattern der Copy-Log und Copy-Hex Buttons.

### Button-Architektur final

| Pattern | Klasse | Verwendung |
|---|---|---|
| Main action | `.bo` in `.btn-row` Grid, feste 180px | Haupt-Actions auf Tab-Ebene, mit Description |
| Compact action | `.bo.sm` (4px 10px) | Header-aligned oder kompakte inline-Actions |
| Primary in task panel | `.bp` (grün) | Hauptaktion innerhalb eines expanded Tool-Panels |
| Secondary in task panel | `.bo` | Sekundäraktion neben einem `.bp` |

### Breaking changes

Keine. NVS-Keys unverändert, gespeicherte Settings und Backups laden ohne Migration. MQTT-Topics unverändert, HA Entity IDs unverändert. Sessions bleiben gültig.

### Deploy-Hinweise

- Direkt-OTA von V2.67 oder V2.67.1 unterstützt.
- Harter Browser-Refresh (Ctrl+Shift+R oder Cmd+Shift+R) empfohlen nach dem ersten Login, damit aktualisierte HTML/CSS/JS geladen werden. `Cache-Control: max-age=600` aus V2.66.2 E2 bedeutet sonst bis zu 10 Minuten alter UI-Cache.

### Memory footprint

- Static RAM: unverändert.
- Flash: +~2.7 KB für HTML-Restrukturierung und neue CSS-Klassen.

---

## [2.67.1] - 2026-04-24

Hotfix ausgelöst durch field-beobachteten TASK_WDT-Reboot am 2026-04-24 02:42 auf einem V2.67-Device mit aktiviertem L3 (`mq_level=2`). OTA-kompatibel, keine Breaking Changes, keine NVS-Schema-Änderungen.

### Fixed

- **F1/L3 Mutex-Hold über synchrone MQTT-Publishes in `sendMqttCells()`.** Der in V2.67 F1 eingeführte L3-Publish-Pfad hielt `dataMutex` über bis zu 16 aufeinanderfolgende synchrone `mqtt.publish()` TCP-Sends. Im Regelfall (gesunde WiFi/Broker-Verbindung) sind das ~40-160ms Mutex-Hold und unauffällig. Bei WiFi- oder Broker-Stall kann ein einzelner `mqtt.publish()` bis zum PubSubClient-Default-Timeout von 15s blockieren. Bei mehreren Packs summiert sich das. Folge: rs485Task auf Core 0 blockierte am Mutex, Main-Loop überschritt das 60s TASK_WDT-Fenster, Gerät rebootete. Field-Evidenz: `handler_max_ms=8854`, `loop_max_ms=8864` post-recovery, matcht einem Worst-Case von ~1.1s pro publish über 8 Packs. Ein weiterer Stall und das Fenster wäre überschritten. Fix folgt dem H4/C2-Snapshot-Pattern: ~170 bytes pro Pack (valid, cell_count, temp_count, cells[32], temps[8]) werden unter dem Mutex in einen static `CellSnap[MAX_BMS]` Buffer kopiert, Mutex wird sofort released, JSON-Build und `mqtt.publish()` passieren außerhalb des Locks. Zusätzlich `esp_task_wdt_reset()` zwischen den Publishes und `mqtt.connected()` Guard innerhalb der Publish-Loop damit eine mid-batch verlorene Verbindung nicht in 16 TCP-Timeouts kaskadiert.

### Root-Cause-Analyse

Pre-V2.67 existierte dieser Pfad nicht. V2.67 F1 führte `sendMqttCells()` ein, übernahm aber das Mutex-Take-over-Publish Muster das die V2.67 Roadmap (H4) explizit als Anti-Pattern für V2.68 markiert hatte. Das fiel im V2.67 Review nicht auf weil H4 auf V2.68 verschoben wurde und `sendMqttCells()` als neue Funktion nicht im H4-Scope stand. V2.67.1 zieht das Pattern für diesen einen neuen Pfad vor.

### Breaking changes

Keine. Schema von `{base}/cells/bms{n}` unverändert (`"v":1`). Keine NVS-Key-Änderungen. `mq_level` Verhalten unverändert.

### Memory footprint

- Static RAM: +2.7 KB (.bss) für `snap[16]` Buffer.
- Stack: nicht betroffen, snap ist static statt lokal.
- Flash: +~500 bytes durch zusätzliche Bounds-Checks und Kommentare.

### Deploy-Hinweise

- Direkt-OTA von V2.67 unterstützt.
- Wer nach V2.67-Instabilität auf L1/L2 zurückgeschaltet hat, kann nach diesem Upgrade gefahrlos wieder auf L3.

---

## [2.67] - 2026-04-23

Erstes GA-Release der V2.67-Linie. Vier Features, alle additiv, alle OTA-kompatibel, keine Breaking Changes, keine NVS-Schema-Migration. Setzt die in V2.66.3 begonnene Diagnostics-Sichtbarkeit konsequent fort: User bekommen ihre Zustandsdaten nicht nur auf MQTT, sondern auch direkt in der Web-UI und in Home Assistant, mit Erklärungen und mit einer Reset-Funktion damit Langzeit-Counter nach einem Vorfall sinnvoll wieder bei Null anfangen. Außerdem erstmals ein Tiered-MQTT-Modell (L1/L2/L3) damit kleinere HA-Instanzen nicht von 100+ Entitäten erschlagen werden.

### Added

- **F1 Tiered MQTT (L1/L2/L3) mit level-gated HA-Discovery.** Neuer NVS-Key `mq_level` (int, default 0). L1 (lean, default) publisht nur die Pack-Aggregate wie bisher, keine Per-BMS-Entitäten. L2 ergänzt pro BMS die 6 Core-Entitäten (soc, v, i, min_cell, max_cell, online) plus 6 neue Stat-Entitäten (`drift_mv`, `polls`, `timeouts`, `errors`, `spikes`, `current_holds`). L3 fügt zusätzlich das retained Topic `{base}/cells/bms{n}` im 20s-Takt hinzu, inkl. bis zu 16 Cell-Voltages und 4 Temps pro Pack. Level-Switch in der UI (MQTT-Tab) triggert sofortiges Re-Publish der HA-Discovery-Configs plus gezieltes Remove der nicht mehr benötigten Entitäten. Zero cost auf L1, deterministische Publish-Cadence auf L2/L3.
- **F2 About-Section im General-Tab.** Compiled-Date, Git-SHA, Chip-Info, Flash-Usage, Free-Heap, PSRAM (wenn vorhanden), Uptime, Last-Boot-Time, Boot-Reason, GPIO-Pin-Set, Projekt-Link, Lizenz. Git-SHA wird über externes Build-Script `tools/git_sha_gen.sh` in `git_sha.h` injiziert; Fallback `"unknown"` wenn Header fehlt (Arduino-IDE Ad-hoc-Builds). Neuer globaler `g_boot_epoch` (uint32) erfasst Unix-Epoch des Boot-Zeitpunkts lazy bei erstem NTP-Sync, Cutoff 2023-01-01 als Pre-NTP-Guard.
- **F3 Diagnostics-Panel mit Tooltips und `/diag` HTTP-Endpoint.** Neue `DIAG_KEY_HELP`-Tabelle als Single Source of Truth für Key-Erklärungen. Genutzt von (a) einem neuen `"_help"`-Objekt im MQTT `/diag`-Payload und (b) Hover-Tooltips im General-Tab der Web-UI. Jede Hilfszeile <=80 chars, plain English, Akronyme bei erster Nutzung ausgeschrieben. Neuer HTTP-Endpoint `GET /diag` gibt denselben JSON-Payload zurück wie das MQTT-Topic (inkl. `_help`), auth-geschützt, für UI-Live-Anzeige genutzt. Helper `buildDiagPayloadJson()` eliminiert die Duplikation zwischen MQTT- und HTTP-Pfad.
- **F4 Counter-Reset-Button + 7-Tage-Auto-Rollover.** Neuer "Reset counters"-Button im Diagnostics-Panel mit Confirm-Dialog. Rolling-7-Day Auto-Reset seit letztem Reset, nicht kalender-wöchentlich. NTP-aware: wenn bei Rollover-Zeitpunkt NTP nicht synchronisiert ist, wird der Reset verschoben bis zum nächsten synchronisierten 60s-Check. Neuer additiver NVS-Key `lrst_ts` (ulong) persistiert den letzten Reset-Zeitstempel, RAM-Counter selbst bleiben RAM-only und resetten bei Reboot (Reboot bumpt `lrst_ts` nicht). Neuer `last_reset_ts` Key in Diag-Payload, HA-Entity und Web-UI. Neuer HTTP-Endpoint `POST /svc/reset_counters` (plus GET-Fallback). Reset zero-t: `bms_polls`, `can_tx_ok`, `can_tx_fail`, `stream_aborts`, `current_holds` (aggregate), `spike_rejects` (aggregate), `rl_rejects`, `mqtt_fail`, plus per-BMS `polls`, `timeouts`, `errors`, `spikes`. Preserved bleiben: `heap_min`, `handler_max_ms`, `loop_max_ms`, `rs485_hwm`, `wdt_warnings`, `can_tx_fail_streak`.

### Changed

- **MQTT `/diag` Buffer-Budget von 768 auf 3072 Bytes erhöht.** Nötig für das neue `_help`-Objekt (24 Keys × ~75 Bytes). Buffer lebt auf dem Main-Loop-Task-Stack (>=8 KB), Stack-Impact bleibt deutlich unter Wasserlinie.

### Breaking changes

Keine. Alle NVS-Keys additiv (`mq_level`, `lrst_ts`). Existierende Settings laden unverändert. Default für `mq_level` ist 0 (L1), d.h. Verhalten auf dem MQTT-Topic ist auf dem ersten Boot nach Upgrade identisch zu V2.66.3 bis der User explizit auf L2 oder L3 wechselt. HA-Discovery-Entities bestehend aus V2.66.3 bleiben bestehen.

### Deploy-Hinweise

- **OTA-Upgrade direkt von V2.66.3 unterstützt.** Kein USB-Flash nötig.
- **Tiered MQTT (F1) opt-in.** User muss im MQTT-Tab `Publish level` aktiv umschalten um L2- oder L3-Features zu nutzen. Default bleibt L1 um bestehende HA-Installationen nicht zu überraschen.
- **Counter-Reset-Window (F4) erst aktiv sobald NTP synchronisiert ist.** Auf einem fresh-flashed Device ohne WiFi/NTP bleibt `g_last_reset_ts = 0`. Beim ersten Loop-Tick mit valider Systemzeit (>2023-01-01) wird das Fenster einmalig verankert und in NVS geschrieben.
- **HA-Diag-Discovery muss einmal neu getoggelt werden** damit die neue `last_reset_ts`-Entity erscheint. MQTT-Tab: Diagnostics off -> Save -> wieder on -> Save. Existierende 23 Diag-Entities bleiben davon unberührt.
- **Git-SHA in About-Section (F2) nur sichtbar wenn Build-Script läuft.** `tools/git_sha_gen.sh` ausführen vor jedem Release-Build, dann taucht die SHA unter "Build" auf. Bei lokalen Arduino-IDE-Builds ohne Script steht dort "unknown"; das ist Absicht.

### Known issue (fixed in V2.67.1)

V2.67 L3 hatte unter WiFi/Broker-Stall ein TASK_WDT-Risiko wegen synchroner `mqtt.publish()`-Schleife unter `dataMutex`. Field-reproduced am 2026-04-24. Hotfix V2.67.1 eliminiert das Problem via Snapshot-Pattern. V2.67-User auf L2 oder L3 sollten auf V2.67.1 oder V2.67.2 upgraden.

### Memory footprint

- Static RAM: +16 bytes (`g_last_reset_ts`, `g_last_rollover_check`, `g_boot_epoch`, `g_mqtt_level`, `last_mqtt_cells`) plus DIAG_KEY_HELP-Tabelle (~24 × 16 bytes im Flash/ROM, nicht im RAM). Vernachlässigbar.
- Stack: Diag-Payload-Buffer 768 -> 3072 bytes, einmal pro Publish auf Main-Loop-Task.
- Flash: UI-HTML +~1.2 KB.
- NVS: 2 neue Keys additiv.

---

## [2.66.3] - 2026-04-20

Home Assistant Auto-Discovery für den in V2.66.2 eingeführten `{base}/diag` Topic. Pre-existing JSON-Bugfix als Bonus. OTA-kompatibel, keine Breaking Changes.

### Added

- **D2 HA Auto-Discovery für Diagnostic-Entities.** V2.66.2 D1 publishte den Diag-Topic bereits sichtbar im MQTT, aber ohne Auto-Discovery mussten User die 21 Sensoren manuell in `configuration.yaml` konfigurieren. V2.66.3 ergänzt `sendMqttDiagDiscovery()`, die 21 retained Discovery-Configs auf `homeassistant/sensor/{uid}_diag_{key}/config` publisht. Device-Gruppierung: alle 21 Entities erscheinen unter dem bestehenden "Topband BMS (xxxxxxxx)" Device. Korrekte `device_class` (duration, data_size) und `state_class` (measurement, total_increasing) für HA-native Graphen und Statistiken. Publish-Trigger: bei jedem MQTT-Reconnect sowie bei Toggle-Enable. Cleanup-Funktion `removeMqttDiagDiscovery()` entfernt die Configs bei Toggle-Disable, verhindert Zombie-Sensoren in HA.

### Fixed

- **Duplicate JSON Key `rl_rejects` in `/data` Response.** Pre-existing Bug aus V2.66 Copy-Paste-Altlast, niemand hatte es bemerkt weil der Wert doppelt geschrieben wurde und Parser meist "letzter Wert gewinnt" verwenden. Jetzt single emission.

### Deploy-Hinweise

- Beim ersten Boot nach OTA muss der User im MQTT-Tab `Home Assistant Auto-Discovery` UND `Publish diagnostics` aktivieren, dann Save. HA registriert die 21 Entities automatisch.
- Mosquitto-Persistence weiterhin empfohlen (`persistence true` in Broker-Config), damit die Retained Discovery-Configs nach Broker-Restart bestehen bleiben.
- Falls V2.66.2 ohne HA-Discovery benutzt wurde: existierende manuell konfigurierte Sensoren in `configuration.yaml` können entfernt werden, sobald die Auto-Discovery-Entities angekommen sind.

---

## [2.66.2] - 2026-04-19

C3 Hardening plus MQTT Diagnostics Topic. Ausgelöst durch reproduzierbaren TASK_WDT-Reboot auf V2.66.1 beim Page-Load trotz C3-Fix, sowie einen unexplained WDT-Reboot nach 6h30m Uptime ohne verwertbare Post-Mortem-Daten (Log-Ringbuffer ist RAM-only). OTA-kompatibel, keine Breaking Changes.

### Changed

- **C3+ C3 Härtung mit globalem Abort-Flag.** V2.66.1 C3 Gate im `sendHist` Lambda returnte nur aus dem Lambda, nicht aus `handleData`. Bei Disconnect mitten im Chart-Stream konnte `handleData` die nächsten 5 `sendHist`-Aufrufe starten bevor der nachgelagerte Gate griff. Jedes Lambda versuchte zuerst einen `sendContent()` ins tote Socket bevor es den Disconnect bemerkte. Cumulative TCP-Timeouts haben über 60s WDT-Fenster akkumuliert. Fix: `static bool g_stream_abort` Flag wird von `streamAlive()` gesetzt, nach jedem `sendHist`-Aufruf sowie in der BMS-Array-Loop geprüft. Lambda-Return propagiert sofort bis Handler-Exit. Flag wird am Anfang jedes `handleData`-Aufrufs zurückgesetzt.
- **E2 Cache-Header für Main UI.** `handleRoot` sendete die ~900-Zeilen-HTML mit `Cache-Control: no-store`, was jeden Page-Load erzwang die komplette HTML neu zu serialisieren. Jetzt `max-age=600` (10 Minuten), Live-Daten weiter via `/data` Poll alle 2.5s. Reduziert Gateway-Last bei Page-Refresh signifikant.

### Added

- **D1 MQTT Diagnostics Topic `{base}/diag`.** Opt-in via UI-Toggle im MQTT-Tab. 30s Cadence. 21 Diagnostic-Keys als JSON für Post-Mortem-Analyse und Trend-Tracking: `fw`, `uptime`, `boot_reason` (String), `heap_free`, `heap_min`, `nvs_used`, `nvs_free`, `can_tx_ok`, `can_tx_fail`, `can_tx_fail_streak`, `can_status`, `rs485_hwm` (Stack Watermark), `bms_polls`, `bms_timeouts`, `current_holds`, `spike_rejects`, `rl_rejects`, `mqtt_fail`, `session_age_d`, `stream_aborts`, `handler_max_ms`, `loop_max_ms`, `wdt_warnings`. Retained-Flag gesetzt damit letzter Snapshot nach Reboot auf dem Broker verfügbar bleibt. NVS-Key `mq_diag` (bool) persistiert UI-State. snprintf-basiert in 768-Byte Stack-Buffer, kein String-Churn. Off-by-default, zero-cost wenn deaktiviert.

### Fixed (über V2.66.1)

Keine neuen Bugfixes über das in V2.66.1 hinaus Adressierte. V2.66.2 ist ausschließlich Hardening + Observability.

### Deploy-Hinweis

Mosquitto-Persistence aktivieren (`persistence true` in Broker-Config) damit retained diag-Messages einen Broker-Restart überleben.

---

## [2.66.1] - 2026-04-18

Hotfix ausgelöst durch Code-Review (CODE_REVIEW_V2.66.md) und Field-Observation eines reproduzierbaren TASK_WDT-Reboots beim Web-UI Page-Refresh. OTA-kompatibel, keine Breaking Changes.

### Fixed

- **C3 /data Streaming TASK_WDT-Trigger behoben.** Field-beobachtet 2026-04-18 21:54:05: Dashboard-Refresh triggerte TASK_WDT-Reboot. Ursache: `handleData` streamte ~60-80 `sendContent()` Chunks nach Mutex-Release. Bei Client-Disconnect mitten im Stream blockierten subsequent Sends im TCP-Retransmit-Timeout. Akkumulierte Stalls überschritten 60s WDT-Fenster. W1-Reset nach `server.handleClient()` in `loop()` half nicht weil der Handler selbst nicht zurückkehrte. Fix: neue `streamAlive()` Helper-Funktion prüft `server.client().connected()` und ruft `esp_task_wdt_reset()` auf. An 5 strategischen Punkten im Streaming-Block aufgerufen (nach victron+config, vor Charts, in Chart-Loop alle 32 Punkte, vor BMS-Array, vor Alerts-Array). Handler bricht sauber ab bei Disconnect.
- **C1 xSemaphoreGive() ohne bestätigtes Take an 3 Sites behoben.** Pattern `if (dataMutex) xSemaphoreTake(...)` prüfte den Return-Value nicht. Bei Mutex-Timeout lief der kritische Abschnitt ohne Lock, danach `Give` auf nicht-gehaltener Mutex = FreeRTOS undefined behavior. Betroffen: `sendMqttData` (Line 2572), `handleData` (Line 3288), `rs485Task::calculateVictronData` (Line 4967). Fix: lokale `bool locked`, `Give` nur wenn `locked=true`, Skip mit `addToLog` bei Timeout. Pattern von `alertDetectTick` (Line 2263) als Template übernommen.
- **C2 H1 Hold-Last-Value Cross-Core-Read behoben.** `bms[addr].current` und `.valid` wurden außerhalb des `dataMutex` gelesen, obwohl 30 Zeilen vorher bereits ein Snapshot unter Mutex stattfand. Fix: Snapshot erweitert um `prev_cur_for_hold` und `prev_valid_for_hold` unter Mutex, Hold-Logic nutzt captured locals.
- **H1 M6 Rate-Limit LRU-Loop Short-Circuit behoben.** `break` auf ersten leeren Slot verhinderte das Finden eines existierenden IP-Eintrags in einem späteren Slot. Folge: Client bekam frisches 4-Token-Bucket bei jedem Request wenn Slot 0 leer war, Rate-Limit-Effektivität kompromittiert. Fix: Loop scannt alle 8 Slots komplett, leerer Slot wird nur als Fallback gemerkt wenn kein Match.

---

## [2.66] - 2026-04-18

Sammelrelease mit einem kritischen Stabilitätsfix, einem wichtigen BMS-Workaround und drei aufgestauten Bugfixes aus dem V2.64-Backlog. Komplett OTA-kompatibel, keine Breaking Changes.

### Warum dieses Release

Ein TASK_WDT-Reboot (System-Watchdog) trat während Web-UI-Nutzung auf, Feld-Daten zeigten zwei Reboots in 10 Stunden. Zusätzlich bestätigte die V2.65.1 Frame-Spy-Analyse, dass TopBand BMS nur alle ~90 Sekunden Stromwerte liefert was zu flackernden Dashboard-Werten und Energie-Unterzählung bei kleinen Lasten führt. V2.66 adressiert beides plus die ältesten offenen Punkte aus dem V2.64 Code Review.

### Changed

- **W1 TASK_WDT-Reboots behoben.** Im `loop()` wurde der Watchdog-Reset bisher nur einmal pro Iteration aufgerufen. Bei ungünstigem Timing konnten sich `server.handleClient()` (große `/data`-Payload), MQTT-Publish, NVS-Save und MQTT-Reconnect auf über 30 Sekunden summieren und einen Reset auslösen. Jetzt wird `esp_task_wdt_reset()` an vier kritischen Stellen in `loop()` aufgerufen. Zusätzlich wurde das WDT-Timeout von 30 auf 60 Sekunden erhöht als zusätzliches Sicherheitsnetz.
- **H1 BMS current hold-last-value.** Wenn der BMS 0.00 A meldet und der vorherige Wert nicht null war und weniger als 120 Sekunden alt ist, wird der alte Wert beibehalten. Das glättet das Flackern und verbessert die Energy-Integration für kleine Dauerlasten wie Wechselrichter-Eigenverbrauch. Echte 0-Ampere-Zustände werden nach 120s Gap akzeptiert. Pro BMS wird ein `current_holds` Zähler geführt und in `/data` ausgegeben. V3.2 (SmartShunt) macht diesen Workaround langfristig obsolet.
- **H2 Session-Expiry serverseitig erzwungen.** V2.64 setzte zwar Cookie-Max-Age auf 30 Tage, aber der Server validierte das Alter nicht. Ein kompromittierter Cookie war damit praktisch unbegrenzt gültig. Jetzt wird Token-Erstellungszeit im NVS-Key `sess_ts` gespeichert, `checkAuth()` lehnt Sessions älter als 30 Tage ab und rotiert das Token. Bei NTP-Problemen zum Erstellungszeitpunkt wird `millis()/1000` als Fallback gespeichert und nach erster NTP-Synchronisation re-verankert.
- **M4 Spike-Filter-Thresholds konfigurierbar.** Die Schwellwerte für Voltage-Jump (5V), Current-Jump (250A) und SOC-Jump (20%) waren hardcoded. Jetzt über Battery-Tab konfigurierbar, mit sinnvollen Clamping-Grenzen. Defaults entsprechen den bisherigen Werten, Verhalten bleibt backward-kompatibel. Neue NVS-Keys `sp_v`, `sp_a`, `sp_s`.
- **M6 Rate-Limit auf `/data` Endpoint.** Einfacher Token-Bucket pro Client-IP: 4 Requests burst, 2 pro Sekunde Refill. Bei Überschreitung HTTP 429 mit `Retry-After: 1`. Schützt vor Runaway-Clients (falsch konfigurierte Dashboards, aggressive Third-Party-Tools) ohne Auswirkung auf normales 2.5s Poll-Intervall. 8 IP-Slots LRU, IPv4-only (LAN-Deployment-Annahme). Reject-Counter via `rl_rejects` in `/data` einsehbar.

### Added

- **`/data` Feld `current_holds`** pro BMS (unter `stats`-Objekt). Zeigt wie oft Hold-Last-Value gegriffen hat seit Boot.
- **`/data` Feld `rl_rejects`** auf Geräte-Level. Gesamtzahl Rate-Limit-Rejections seit Boot.
- **Battery-Tab: Abschnitt "Spike filter"** mit drei konfigurierbaren Feldern plus Erklärtext.
- **Log-Line bei erster Hold-Last-Value Aktivierung pro BMS** (`BMS0: current hold-last-value active (V2.66 H1)`) — einmalig pro Boot, damit man weiß der Mechanismus ist aktiv.

### Breaking changes

Keine. Alle Änderungen additiv. Bestehende Sessions bleiben gültig (bis sie 30 Tage alt sind). Settings-Backup-Files von V2.65.x laden sauber.

### Upgrade notes

- Erster Boot nach Upgrade: Bestehende Session-Cookies bekommen einen `sess_ts`-Zeitstempel ab jetzt. Die 30-Tage-Uhr beginnt praktisch neu. Wer ganz aktuellen Cookie-Reset will: 5× Power-Cycle auslöst Session + Auth-Reset.
- TASK_WDT-Reboots sollten ab sofort ausbleiben. Falls sie trotz V2.66 weiter auftreten, ist ein separates Ticket nötig — dann steckt die Ursache tiefer als in der Loop-Struktur.

### Memory footprint

- Static RAM: +64 bytes (RateLimitSlot × 8) + 24 bytes (spike filter globals + counters). Vernachlässigbar.
- Free heap at boot: erwartet unverändert bei ~145 KB.

### Dependencies for next versions

V2.66 ist Voraussetzung für V3.0 (Multi-File-Refactor). Hold-Last-Value aus V2.66 bleibt in V3.0 als Fallback für User ohne SmartShunt. In V3.2 wird SmartShunt zur primären Strom-Quelle und Hold-Last-Value zum Backup-Pfad.

## [2.65.4] - 2026-04-18

**Architectural decision: chart history leaves NVS.**

### Why this release exists

The NVS partition is designed to hold configuration data, not time-series data that gets rewritten every few minutes. V2.65.1 through V2.65.3 saved the four chart history arrays (power, voltage, SOC, temperature) to NVS every 15 minutes. Each save marks about 123 NVS entries as deleted and writes 123 new ones. The ESP32 NVS garbage collector only reclaims space when an entire 4 KB flash page is fully invalidated, which happens rarely under this pattern. Result: NVS usage grows continuously until the partition fills up and saves begin to fail silently.

V2.65.2 consolidated the four blobs into one atomic blob, which reduced but did not eliminate the problem. Field data over a single night showed the partition climbing from 305 used entries back up to 504 (of 630 total), with save failures resuming and the cycle repeating.

The root cause is architectural: NVS is the wrong tool for this job. V2.65.4 removes chart history from NVS entirely. In V3.0, history comes back, then stored via LittleFS on a dedicated flash partition where repeated large writes are the expected access pattern.

### What behaves differently

- **Charts start empty after every reboot.** They populate over 48 hours as new data points are recorded. A small banner below the charts shows `Chart data collects since last boot (X% filled)` until the window is at least half full, then the banner disappears automatically.
- **Energy counters are preserved.** Daily, weekly, and monthly kWh totals survive reboots as before. Only the graph lines reset.
- **NVS writes drop dramatically.** The hourly save now only writes six small scalars (about 15 NVS entries total). Expected NVS usage on a working device: stable around 80 entries out of 630, approximately 13% utilization with no growth over time.
- **Chart resolution improves.** Back to 3-minute sampling intervals (was 6-minute in V2.65.2). Frontend downsampling tightened from DS=5 to DS=3. Result: 320 chart points rendered per line instead of 96. Three times more visual detail, the zigzag effect from coarse sampling is gone.
- **Chart X-axis labels are now accurate.** The frontend reads the minutes-per-point value from the backend instead of hardcoding 15. Labels will match the actual data resolution for any future schema changes.

### Trade-off explicitly accepted

Losing chart history on reboot is the cost of keeping NVS stable. For a device that reboots only on intentional OTA or user-initiated save-and-reboot events, this is acceptable. V3.0 will restore persistence via LittleFS.

### What's still persisted in NVS

- All settings (BMS count, safety limits, CVL, pin config, MQTT credentials, auth hash)
- Alert ring buffer (last 25 alerts)
- Session token (cookie auth survives reboots)
- Energy counters: today, rolling 7 days, monthly totals

Total NVS footprint after V2.65.4: approximately 80 entries, no growth over time.

### Changed

- **R1** `loadHistory()` and `saveHistory()` rewritten. No more `includeGraph` parameter, no malloc, no blob serialization. Simple energy-counter read/write only.
- **R2** Legacy NVS key `hblob` from V2.65.2/V2.65.3 automatically deleted on first V2.65.4 boot. Reclaimed entries become available to the NVS garbage collector.
- **R3** `HISTORY_LEN` back to 960 slots at 3-minute intervals (48h depth preserved). Chart downsampling constant `DS` changed from 5 to 3. Three times more chart resolution.
- **R4** `/data` endpoint now emits `minPer`, `histLen`, and `histFilled` fields. Frontend X-axis labels read `minPer` from backend instead of using a hardcoded value.
- **R5** Periodic NVS save interval raised from 15 minutes to 60 minutes. Day rollover and pre-OTA saves unchanged.
- **R6** Dashboard shows a `Chart data collects since last boot` info banner below the chart area while less than 50% of the 48-hour window is filled. Auto-hides once the window is half populated.
- **R7** Boot log now explicitly states `History: RAM-only, 48h depth at 3-min resolution, 320 chart points`.

### Breaking

- Any external tooling that expected `hdat`/`hvlt`/`hsoc`/`htmp`/`hblob` keys in NVS backups will find those keys missing. Settings backup/restore continues to work identically.
- Downgrading to V2.65.2 or earlier after running V2.65.4 will start with empty chart arrays until the old firmware repopulates NVS on its next save cycle.

### Memory footprint

- Free heap at boot: approximately 148 KB (down from 156 KB in V2.65.2, because chart arrays grew from 11.5 KB to 23 KB as a consequence of doubling the slot count back to 960)
- Min-heap warning threshold remains 20 KB
- Safety margin: approximately 128 KB of headroom before warning, 148 KB before OOM

### Removed

- Entire blob serialization / deserialization pathway (HIST_BLOB_VERSION, HIST_BLOB_HDR_SIZE, HIST_BLOB_ARR_BYTES, HIST_BLOB_TOTAL_SIZE and all associated malloc/free/memcpy code)
- `includeGraph` parameter from `saveHistory()`
- All related validation, size-mismatch logging, and schema-upgrade branches for chart arrays
- About 160 lines of code net

## [2.65.3] - 2026-04-18

UI quality-of-life release. No behavioural changes to backend, polling, or safety logic.

### Added
- **U1** Debug log: `Copy log` button above the log pane serializes visible entries as plain text and copies them to the clipboard. Useful for pasting into bug reports or sharing reboot sequences.
- **U2** Frame Spy: `Copy hex` button serializes all captured frames as a tab-separated block with columns `ts_ms`, `bms`, `cur_A`, `volt_V`, `hex` (oldest-first). Copies to clipboard for analysis in spreadsheets or scripts.
- **U3** Frame Spy: close (×) button hides the panel without affecting the server-side ring buffer. Pressing Start re-renders current state.

### Internal
- Dual-path clipboard write: `navigator.clipboard` when available (HTTPS), `execCommand('copy')` fallback for plain-HTTP dashboards, which is the typical deployment.

## [2.65.2] - 2026-04-18

NVS schema consolidation hotfix. Targets the partially-saved history blob pattern exposed by V2.65.1 diagnostics: one user's partition was 497/630 entries used (78.9%), with the last write in the four-blob save sequence (`htmp`) failing silently. This release eliminates the partial-save failure mode and reclaims ~130 NVS entries.

### Changed
- **E1** History persistence consolidated into a single atomic NVS blob named `hblob`. Replaces the four separate blobs (`hdat`, `hvlt`, `hsoc`, `htmp`) plus `hidx`/`hfill` scalars with one 3856-byte write. Either all four history arrays and their metadata persist together or none do. Blob format is versioned (v1 header) for future migrations.
- **E2** Chart history depth preserved at 48 hours, but sampling interval doubled from 3 minutes to 6 minutes. Slot count halved from 960 to 480. Net NVS saving: approximately 130 entries freed from the `h` namespace.
- **E4** CSV history export timestamps updated to reflect the new 6-minute interval.

### Breaking
- **One-time history reset on upgrade.** V2.65.2 cannot load V2.65.1-or-earlier history blobs. On first boot after upgrade, chart history starts fresh (log line: `NVS: schema upgrade from V2.65.1 or earlier, history reset`). Energy counters (daily / weekly / monthly kWh) are preserved.
- **Chart visual resolution halves** (30-minute-per-rendered-point instead of 15-minute). Caused by `DS=5` downsampling constant remaining unchanged while source interval doubled. If finer chart resolution is preferred, `DS` in the `sendHist` helper can be tuned to `3` in a follow-up release.

### Removed
- **E3** Legacy NVS keys (`hdat`, `hvlt`, `hsoc`, `htmp`, `hidx`, `hfill`) automatically deleted on first V2.65.2 boot to reclaim partition space. Migration is one-way: downgrading to V2.65.1 after running V2.65.2 will result in empty history arrays.

### Memory footprint
- NVS: ~130 entries freed. On the reference device (630-entry partition), usage drops from ~497 to ~370 (59%).
- Heap: `saveHistory()` now allocates and frees a 3856-byte buffer per graph save (every 15 min + on OTA). Transient, no persistent footprint.
- Flash image: no significant change.

## [2.65.1] - 2026-04-18

Diagnostic release. Adds instrumentation to surface two issues reported during V2.65 field soak: intermittent history loss across OTA reboots (affecting voltage and temperature charts but not power) and BMS current readings flickering between real values and zero. No behavioural changes to safety logic, CAN output, polling cadence, or MQTT.

### Added
- **D1** NVS partition monitoring. `getNvsStats()` queries the ESP-IDF NVS stats API; `/data` now includes `nvs_used` / `nvs_free` / `nvs_total` entry counts, rendered in General > Diagnostics with colour coding (amber >70%, red >85%). Boot-time snapshot logged.
- **D2** RS485 Frame Spy. 20-entry ring buffer (~4 KB) captures raw hex responses from `parseTopband()` before any early-return path, so truncated and malformed frames are included. Toggle via General > Diagnostics > Start RS485 Frame Spy, or `/spy?act=1` over HTTP. Auto-disables after 5 minutes. Frontend decodes the TopBand 0x42 frame client-side to show cell count, temp count, current, and voltage alongside the raw hex.

### Changed
- `saveHistory()` now checks the return value of every `putBytes` / `putFloat` / `putInt` call and logs the specific key on failure (`NVS save: hvlt (volt) FAILED` etc). On any failure, a follow-up log line dumps current NVS used/free entries so space exhaustion is obvious. Return type changed from `void` to `bool` (callers unaffected).
- `loadHistory()` now logs blob-size mismatches per key instead of silently zeroing. Distinguishes missing-key (low log level) from corrupt-size (error level) cases.

### Added endpoints
- `GET /spy` returns spy state + frame ring buffer as JSON
- `GET /spy?act=1` starts capture (clears ring, sets 5 min deadline)
- `GET /spy?act=0` stops capture

### Memory footprint
- Frame Spy ring buffer: 20 × 228 bytes = ~4.6 KB static RAM
- NVS stats call: no persistent allocation

## [2.65] - 2026-04-17

### Added
- CAN TX backpressure counters (`can_ok`, `can_fail`, `can_fail_streak`) exposed in `/data` for bus-health monitoring.
- Consecutive CAN TX failure warnings at 50 and 500 streak thresholds.
- Deferred restart mechanism (`g_restart_at`) polled in `loop()`. Save, OTA, and Restore handlers now return immediately and flush the TCP response cleanly before reboot.

### Changed
- **M3** Replaced unbounded `String debug_log` with a fixed 40-entry ring buffer (~4 KB, 92-byte message cap). Eliminates heap fragmentation from per-log String allocations. `/data` now emits `log` as a JSON array instead of pre-built HTML. Frontend renders client-side with XSS escaping.
- **H4** `debug_can_status` converted from cross-core `String` to a 64-byte char buffer under `canStatusMutex`. Removes the heap corruption vector from Core 0 writes / Core 1 reads.
- **M2** Save, OTA, and Restore handlers no longer call `delay()` before `ESP.restart()`. Restart is deferred via `g_restart_at` and triggered from `loop()`.
- **M7** `sendVictronCAN()` now gated on `!simulation_active`, matching the existing poll gates. Simulation mode no longer emits phantom CAN frames to the inverter.
- Mutex creation (`rs485Mutex`, `dataMutex`, `logMutex`, `canStatusMutex`) moved to the top of `setup()` so early-boot `addToLog()` and `setCanStatus()` calls are thread-safe.

### Fixed
- `alertDetectTick()` now snapshots `g_can_status` under `canStatusMutex` before substring matching, removing the last cross-core String access.

### Internal
- New `setCanStatus(const char*)` helper consolidates all `debug_can_status` writes behind the mutex.
- `LogEntry` struct (ts + flags + msg[92]) replaces dynamic String storage for log history.

### Breaking
- `/data` response: `log` field changed from HTML string to JSON array of `{t, e, m}` objects. Third-party consumers must update their parser.

## [2.64] - 2026-04

### Fixed
- **C1** `parseTopband()` writes now atomic via local-buffer + `memcpy` under `dataMutex`, eliminating torn reads.
- **C2** `alertDetectTick()` snapshots `victronData` and `bms[]` under `dataMutex` at tick start.
- **H1** Session token persisted in NVS (`sess_tok`), cookie `Max-Age=2592000` (30 days), rotation on auth change or 5x power cycle.

## [2.63] - 2026-03

### Fixed
- Arduino IDE auto-prototype compile errors: all struct definitions consolidated at top of file (lines 153–265) so generated prototypes resolve correctly.

### Added
- First public GitHub release.

## [2.61] - 2026-03

### Added
- Server-side alert ring buffer (25 entries, NVS-persistent) with `/alerts` endpoints.
- Alert detection on Core 1 with state-transition logic (rising-edge triggers, no spam).

### Fixed
- Mobile menu position for iOS Safari and Chrome.
- Mobile metric card layout gaps (`.mc-w` wide cards).
- Chart `ResizeObserver` and history-start tracking.

## [2.60] - 2026-02

### Added
- Runtime pin configuration via web UI (Waveshare / LilyGo / Custom presets), NVS-persistent.

### Changed
- All `#define` pin constants replaced with runtime variables.
- Power-display thresholds lowered (50 W → 10 W, 0.5 A → 0.15 A).
