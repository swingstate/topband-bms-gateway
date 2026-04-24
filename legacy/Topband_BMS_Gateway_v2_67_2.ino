/*
 * TOPBAND BMS GATEWAY V2.67.2
 * =========================
 *
 * V2.67.2 CHANGELOG (GUI cleanup, final V2.67 release)
 *
 *   UI-only release. No functional changes to MQTT, RS485, CAN, or HA
 *   discovery. OTA-compatible from V2.67 or V2.67.1.
 *
 *   - Content scales to full window width (removed 1400px cap on .pg)
 *   - Consistent button sizing across all pages via new .bo + .bo.sm
 *     classes. All main-action buttons use .btn-row grid (180px fixed
 *     button + inline description) for visual consistency.
 *   - MQTT section restructured with 5 subsections: Broker, Home Assistant,
 *     Battery data (always on), Extended battery monitoring, Gateway
 *     self-monitoring. Level labels in plain English instead of L1/L2/L3.
 *     Radios vertical-stacked with description per option, fixing the
 *     label overlap bug from V2.67.
 *   - General Diagnostics section: Log gets a header, Frame spy and
 *     Runtime counters buttons in equal-width grid with descriptions.
 *   - "Show diag counters" renamed to "Runtime counters".
 *   - About section split into Build info / Runtime state / Hardware.
 *     NVS usage moved here from Diagnostics.
 *   - Runtime counters panel deduplicated: 8 keys already in About
 *     (fw, uptime, boot_reason, heap_free, heap_min, nvs_used, nvs_free,
 *     session_age_d) removed from the UI panel. Server-side MQTT /diag
 *     and HA discovery keep all 24 keys.
 *   - Board dropdown label: "Waveshare ESP32-S3" ->
 *     "Waveshare ESP32-S3 RS485/CAN Controller".
 *   - Maintenance, Battery, and Alerts buttons harmonized.
 *
 *   No NVS schema changes. No breaking changes. OTA-ok from V2.67 and
 *   V2.67.1.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.67.1
 * =========================
 *
 * V2.67.1 CHANGELOG (critical L3 hotfix)
 *
 *   Hotfix released 2026-04-24 after a field-observed TASK_WDT reboot
 *   on a V2.67 device running mq_level=2 ("Additional per-cell
 *   voltages"). OTA-compatible from V2.67.
 *
 *   Root cause: sendMqttCells() held dataMutex across up to 16
 *   synchronous mqtt.publish() calls. On a healthy WiFi + broker this
 *   is 40-160 ms of mutex hold. Under a WiFi or broker stall each
 *   publish can block up to 15 s (PubSubClient default timeout).
 *   Cumulative hold exceeded the 60 s TASK_WDT window. Field evidence:
 *   handler_max_ms=8854, loop_max_ms=8864 post-recovery.
 *
 *   Fix: snapshot ~170 bytes per pack (valid, cell_count, temp_count,
 *   cells[32], temps[8]) into a static CellSnap[MAX_BMS] buffer under
 *   the mutex, release immediately, then build JSON and publish
 *   outside the lock. esp_task_wdt_reset() between publishes and
 *   mqtt.connected() guard inside the publish loop prevent cascading
 *   TCP timeouts on mid-batch disconnect.
 *
 *   Same H4/C2 pattern already catalogued in CODE_REVIEW_V2.66.md.
 *   V2.68 will extend it to sendMqttData as originally scoped.
 *
 *   No NVS changes. No schema changes. No breaking changes.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.67
 * =========================
 *
 * V2.67 CHANGELOG (F1 tiered MQTT, F2 About, F3 diag tooltips, F4 reset)
 *
 *   First GA release of the V2.67 line. Four additive features, all
 *   OTA-compatible from V2.66.3, no breaking changes, no NVS schema
 *   migration. Continues the V2.66.3 diagnostics visibility push into
 *   Web UI and Home Assistant, and introduces tiered MQTT so smaller
 *   HA instances are not flooded with 100+ entities.
 *
 *   F1  Tiered MQTT (L1/L2/L3) with level-gated HA discovery.
 *       New NVS key mq_level (int, default 0). L1 (lean, default)
 *       publishes pack aggregates as before, no per-BMS entities.
 *       L2 adds 6 core + 6 stat entities per pack (drift_mv, polls,
 *       timeouts, errors, spikes, current_holds). L3 additionally
 *       publishes retained {base}/cells/bms{n} at 20 s with up to
 *       16 cell voltages and 4 temps per pack. Level switch in the
 *       UI triggers immediate re-publish of HA discovery plus
 *       targeted removal of entities no longer needed. Zero cost on
 *       L1. Deterministic publish cadence on L2/L3.
 *
 *   F2  About section in the General tab.
 *       Compiled date, git SHA, chip info, flash usage, free heap,
 *       PSRAM (if present), uptime, last boot time, boot reason,
 *       GPIO pin set, project link, license. Git SHA injected via
 *       external build script tools/git_sha_gen.sh into git_sha.h;
 *       falls back to "unknown" for Arduino IDE ad-hoc builds.
 *       New global g_boot_epoch (uint32) captures Unix epoch of boot
 *       lazily on first NTP sync, 2023-01-01 pre-NTP cutoff guard.
 *
 *   F3  Diagnostics panel with tooltips and /diag HTTP endpoint.
 *       New DIAG_KEY_HELP table as single source of truth for key
 *       explanations. Used by (a) a new "_help" object in the MQTT
 *       /diag payload and (b) hover tooltips in the General tab.
 *       Each help line <= 80 chars, plain English, acronyms spelled
 *       out on first use. New HTTP endpoint GET /diag returns the
 *       same payload as the MQTT topic (including _help), auth-
 *       protected, used by the UI for live display. Helper
 *       buildDiagPayloadJson() eliminates duplication between the
 *       MQTT and HTTP paths. Diag buffer grown 768 -> 3072 bytes to
 *       fit the _help dictionary.
 *
 *   F4  Manual counter reset + 7-day rolling auto-rollover.
 *       New "Reset counters" button in the Diagnostics panel with
 *       confirm dialog. Rolling 7-day auto-reset since last reset,
 *       not calendar week. NTP-aware: if NTP is not synced at
 *       rollover time, the reset is deferred until the next NTP-
 *       synced 60 s check. New additive NVS key lrst_ts (ulong)
 *       persists the last reset timestamp. RAM counters stay
 *       RAM-only and reset on reboot (reboot does not bump lrst_ts).
 *       New last_reset_ts key in diag payload, HA entity, Web UI.
 *       New HTTP endpoint POST /svc/reset_counters (plus GET
 *       fallback). Reset zeroes: bms_polls, can_tx_ok, can_tx_fail,
 *       stream_aborts, current_holds, spike_rejects, rl_rejects,
 *       mqtt_fail, plus per-BMS polls/timeouts/errors/spikes.
 *       Preserved: heap_min, handler_max_ms, loop_max_ms, rs485_hwm,
 *       wdt_warnings, can_tx_fail_streak.
 *
 *   Known issue in V2.67 (fixed in V2.67.1): L3 mutex-over-publish
 *   path could trigger TASK_WDT under WiFi/broker stall. V2.67
 *   users on L2 or L3 should upgrade to V2.67.1 or V2.67.2 directly.
 *
 *   No breaking changes. Additive NVS keys only (mq_level, lrst_ts).
 *   Default mq_level = 0 matches V2.66.3 MQTT surface exactly.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.66.3
 * =========================
 *
 * V2.66.3 CHANGELOG (HA Auto-Discovery for diag topic)
 *
 *   D2  Home Assistant Auto-Discovery fuer {base}/diag Entities.
 *       V2.66.2 D1 publishte den Diag-Topic bereits sichtbar im MQTT,
 *       aber ohne HA-Discovery mussten User die 21 Sensoren manuell
 *       in configuration.yaml konfigurieren. Jetzt: sendMqttDiagDiscovery()
 *       publisht 21 retained Discovery-Config-Messages auf
 *       homeassistant/sensor/<dev_uid>_diag_<key>/config mit korrekten
 *       device_class, state_class und unit_of_measurement Werten.
 *       Publish-Trigger: nach jedem MQTT-Reconnect und beim Toggle-Enable.
 *       Nutzt HA Device-Gruppierung damit alle 21 Entities unter einem
 *       gemeinsamen Geraet erscheinen. Cleanup-Funktion entfernt Discovery
 *       Configs bei Toggle-Disable (sonst bleiben Zombie-Sensoren in HA).
 *       Abhaengig von existierender g_ha_enable Einstellung.
 *
 *   Kein Breaking Change. Zero-cost wenn diag oder ha_en deaktiviert. OTA-ok.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.66.2
 * =========================
 *
 * V2.66.2 CHANGELOG (C3 hardening + diagnostics)
 *
 *   C3+ C3 Haertung mit globalem Abort-Flag.
 *       V2.66.1 C3 Gate in sendHist Lambda returned nur aus dem Lambda,
 *       nicht aus handleData. Bei Disconnect mitten im Chart-Stream
 *       konnte handleData die naechsten 5 sendHist-Aufrufe starten
 *       bevor der AFTER-Chart Gate griff. Fix: static bool g_stream_abort
 *       wird von streamAlive() gesetzt, nach jedem sendHist-Aufruf in
 *       handleData geprueft. Lambda-Return propagiert sofort bis zum
 *       Handler-Exit. Reset am Anfang jedes handleData Aufrufs.
 *       Gleicher Flag-Check nach jedem BMS-Chunk im BMS-Array Loop (E1).
 *
 *   E2  WDT-Reset beim Dashboard HTML Serving.
 *       handleRoot sendet ~900 Zeilen inline HTML als eine grosse
 *       sendContent-Chain. Praeventiv esp_task_wdt_reset() nach dem
 *       Senden eingefuegt, deckt langsame Netzwerk-Szenarien ab.
 *
 *   D1  MQTT Diagnostics Topic {base}/diag (opt-in, retained).
 *       30s Cadence, jetzt 21 Diagnostic-Keys fuer Post-Mortem und
 *       Trend-Analyse. Zusaetzlich zu V1-Entwurf vier neue Counter:
 *       stream_aborts (C3 Trigger-Counter), handler_max_ms, loop_max_ms,
 *       wdt_warnings (loop >45s near-miss Events). Retained-Flag damit
 *       letzter Snapshot nach Reboot im Broker bleibt fuer Post-Mortem.
 *       Off-by-default. NVS-Key "mq_diag" (bool). UI-Toggle MQTT-Tab.
 *       snprintf-Buffer (kein String-Churn). Boot-Reason als String.
 *
 *   Kein Breaking Change. Zero-cost wenn deaktiviert. OTA-kompatibel.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.66.1
 * =========================
 *
 * V2.66.1 CHANGELOG (Hotfix - code review findings + field observation)
 *
 *   C3  /data streaming TASK_WDT trigger fixed.
 *       Field-beobachtet 18.04.2026 21:54: Page-Refresh auf Dashboard
 *       triggert TASK_WDT-Reboot. Ursache: handleData streamt ~60-80
 *       sendContent() Chunks nach Mutex-Release. Bei Client-Disconnect
 *       mitten im Stream blockieren subsequente Sends im TCP-Timeout.
 *       Akkumulierte Stalls ueberschreiten 60s WDT-Fenster. Fix: neue
 *       streamAlive() Helper prueft server.client().connected() und
 *       ruft esp_task_wdt_reset() auf. An 5 strategischen Punkten im
 *       Streaming-Block aufgerufen. Handler bricht sauber ab bei
 *       Disconnect. W1-Reset nach server.handleClient() in loop()
 *       hilft nicht weil der Handler selbst nicht zurueckkehrt.
 *
 *   C1  xSemaphoreGive() ohne bestaetigtes Take an 3 Sites gefixt.
 *       Pattern "if (dataMutex) xSemaphoreTake(...)" pruefte den
 *       Return-Value nicht. Bei Timeout lief der kritische Abschnitt
 *       ohne Lock, danach Give auf nicht-gehaltener Mutex = FreeRTOS
 *       undefined behavior. Betroffen: sendMqttData, handleData,
 *       rs485Task::calculateVictronData. Fix: lokale bool locked,
 *       Give nur wenn locked=true, Skip mit addToLog bei Timeout.
 *       Template alertDetectTick Zeile 2263 diente als Vorlage.
 *
 *   C2  H1 Hold-Last-Value Cross-Core-Read gefixt.
 *       bms[addr].current und .valid wurden ausserhalb des Mutex
 *       re-gelesen, obwohl 30 Zeilen vorher schon ein Snapshot unter
 *       Mutex stattfand. Fix: Snapshot erweitert um prev_cur_for_hold
 *       und prev_cur_available, Hold-Logic nutzt captured locals.
 *
 *   H1  M6 Rate-Limit LRU-Loop Short-Circuit gefixt.
 *       break auf ersten leeren Slot verhinderte das Finden eines
 *       existierenden IP-Eintrags in einem spaeteren Slot. Folge:
 *       Client bekam frisches 4-Token-Bucket bei jedem Request wenn
 *       Slot 0 leer war. Fix: Loop scannt alle 8 Slots komplett,
 *       leerer Slot wird nur gemerkt als Fallback wenn kein Match.
 *
 *   Kein Breaking Change. Alles additiv. OTA-kompatibel.
 *
 * =========================
 *
 * TOPBAND BMS GATEWAY V2.66
 *
 * V2.66 CHANGELOG (Bug fixes + BMS current workaround)
 *
 *   W1  TASK_WDT reboots fixed.
 *       Field data zeigte zwei TASK_WDT-Reboots in 10h, getriggert durch
 *       Web-UI-Nutzung. Ursache: loop() rief esp_task_wdt_reset() nur einmal
 *       am Anfang, kumulative Stalls (server.handleClient + MQTT publish +
 *       saveHistory + NVS writes) konnten 30s Watchdog-Grenze ueberschreiten
 *       bei ungluecklichem Timing. Fix: WDT-Reset an allen potenziell
 *       langen Stellen in loop() + rs485Task. Plus WDT_TIMEOUT von 30 auf
 *       60 Sekunden erhoeht als Sicherheitsnetz.
 *
 *   H1  BMS current "hold-last-value" Workaround.
 *       Frame-Spy-Daten haben gezeigt dass TopBand BMS Current-Samples nur
 *       alle ~90 Sekunden liefern. Zwischen Samples ist der Strom-Wert im
 *       Frame "00 00" egal ob wirklich Strom fliesst. Das fuehrt zu flickern-
 *       den Dashboard-Werten und Energy-Undercounting bei kleinen Lasten.
 *       Fix: Wenn BMS 0.00A meldet und der vorherige Wert war (a) nicht null
 *       und (b) weniger als 120 Sekunden alt, wird der alte Wert behalten.
 *       Per-BMS Counter "current_holds" zeigt wie oft der Hold gegriffen hat.
 *       Echte 0A-Zustaende werden nach 120s Gap akzeptiert. V3.2 macht das
 *       via SmartShunt obsolet.
 *
 *   H2  Session-Expiry Server-seitig enforced.
 *       V2.64 H1 setzte Cookie Max-Age=30-Tage, aber keine Server-Validierung.
 *       Ein kompromittierter Cookie war damit unbegrenzt gueltig. Fix: NVS-
 *       Key "sess_ts" speichert Token-Erstellungszeit. checkAuth() verweigert
 *       Sessions aelter als 30 Tage. Bei NTP-Problemen zum Erstellungs-
 *       zeitpunkt wird millis/1000 gespeichert, Revalidation auf erstes NTP.
 *
 *   M4  Konfigurierbare Spike-Filter Thresholds.
 *       Spike-Filter Hardcodes (5V, 250A, 20% SOC) sind jetzt ueber Battery
 *       tab konfigurierbar. NVS-Keys sp_v / sp_a / sp_s. Defaults = bisherige
 *       Werte, also backward-kompatibel. Sinnvoll fuer User mit anderen
 *       Pack-Groessen oder verrauschten RS485-Bussen.
 *
 *   M6  Rate-Limit auf /data Endpoint.
 *       Simpler Token-Bucket pro Client-IP: 4 Requests burst, 2 req/sec refill.
 *       Ueberschreiten liefert HTTP 429. Schuetzt vor Runaway-Clients ohne
 *       Auswirkung auf normalen 2.5s Dashboard-Poll. 8 IP-Slots LRU, kein
 *       Mutex noetig (alles auf Core 1). IPv4-only, LAN-Deployment-Annahme.
 *
 *   Kein Breaking Change. Alles additiv. OTA-kompatibel.
 *
 * =========================
 *
 * V2.65.4 CHANGELOG (Architektur-Entscheidung: History aus NVS raus)
 *
 *   WARUM DIESE AENDERUNG
 *   Die NVS-Partition (24 KB Flash) ist als Konfigurationsspeicher ausgelegt,
 *   nicht als Datenspeicher fuer periodisch geschriebene Zeitreihen. Jeder
 *   "save" der History-Arrays markiert ~123 NVS-Eintraege als invalid und
 *   schreibt die gleiche Menge neu. Garbage Collection laeuft erst bei ganzen
 *   4KB-Pages. Das Ergebnis: NVS-Auslastung waechst stetig bis zum Anschlag
 *   und Saves scheitern. V2.65.2 hat diesen Effekt reduziert, aber nicht
 *   beseitigt - Felddaten zeigten Wachstum von 305 auf 504 Eintraege in 8h.
 *
 *   NEUES VERHALTEN
 *   History-Arrays leben nur noch im RAM. Nach einem Reboot sind die Graphen
 *   leer und fuellen sich ueber 48 Stunden mit neuen Datenpunkten. Das ist
 *   ein bewusster Trade-off: Graphen-Persistenz gegen NVS-Stabilitaet. In
 *   V3.0 kommen Graphen zurueck, dann via LittleFS auf eigener Flash-Partition.
 *
 *   WAS BLEIBT PERSISTENT IN NVS
 *   Settings, Pin-Config, Auth-Hash, Session-Token, Alert-Ring,
 *   Energy-Zaehler (Tag/Woche/Monat kWh). Alles insgesamt ~80 Eintraege,
 *   kein Wachstum. NVS-Auslastung sollte dauerhaft unter 15% bleiben.
 *
 *   DETAILS
 *   R1  loadHistory() / saveHistory() bereinigt. Kein includeGraph-Parameter
 *       mehr. Keine malloc/free-Pfade. Keine Blob-Serialisierung. Nur noch
 *       Energy-Counter und Day/Month-Arrays werden in NVS geschrieben.
 *   R2  NVS-Key "hblob" aus V2.65.2 wird auf erstem V2.65.4-Boot geloescht
 *       (legacy cleanup), zusammen mit den schon entfernten V2.65.1-Keys.
 *   R3  Chart-Aufloesung erhoeht: zurueck auf 3-Minuten-Intervall (war
 *       bei V2.65.2 auf 6 Min vergroebert). HISTORY_LEN 960 Slots.
 *       Frontend-Downsampling DS=5 -> DS=3. Ergibt 320 Chart-Punkte statt
 *       96. Dreimal feinere Visualisierung. Kostet ~12 KB zusaetzlichen RAM.
 *   R4  /data liefert neues Feld "minPer" (Minuten pro Chart-Punkt). Frontend
 *       nutzt das fuer X-Achsen-Labels. Vorher war das hardcoded auf 15 Min
 *       was nach V2.65.2 falsch wurde. Jetzt rechnerisch korrekt.
 *   R5  Periodisches Save-Intervall von 15 Min auf 60 Min erhoeht. Da nur
 *       noch Energy-Counter geschrieben werden, reicht das dicke.
 *   R6  Dashboard-Hinweis: erscheint unter dem Charts-Bereich solange
 *       historyFilled < HISTORY_LEN/2 ("Chart data collects since last boot").
 *       Verschwindet automatisch wenn mehr als halbe Tiefe erreicht.
 *   R7  Startup-Log zeigt History-Parameter explizit:
 *       "History: RAM-only, 48h depth at 3-min resolution, 320 chart points".
 *
 *   RAM-AUSWIRKUNG
 *   Free heap bei Boot: ca. 148 KB (vorher ca. 156 KB, Reduktion durch
 *   groessere History-Arrays). Min-Heap-Warnschwelle bleibt 20 KB,
 *   Sicherheitspuffer ca. 128 KB. Weit entfernt von jeder Grenze.
 *
 * =========================
 *
 * V2.65.3 CHANGELOG (UI quality-of-life)
 *   U1  Dashboard log: "Copy" button next to the debug log pane copies all
 *       visible log entries as plain text (timestamp + message) to the
 *       clipboard. Works over plain HTTP via execCommand fallback.
 *   U2  Frame Spy: "Copy hex" button copies all captured frames as a
 *       tab-separated plain-text block (ts / BMS addr / raw hex / decoded
 *       current / voltage) for pasting into analysis tools or bug reports.
 *   U3  Frame Spy: close (X) button hides the panel without affecting the
 *       server-side ring buffer. Reopening re-renders from current state.
 *   No behavioural changes to backend, polling, or safety logic.
 *
 * =========================
 *
 * V2.65.2 CHANGELOG (NVS schema consolidation)
 *   E1  History schema consolidated into a single atomic NVS blob "hblob".
 *       Prior schema used four separate blobs (hdat/hvlt/hsoc/htmp) which
 *       could partially-save when the partition was near capacity - exactly
 *       the failure pattern observed at 497/630 entries. New format is one
 *       3856-byte write: either all four arrays persist or none do.
 *   E2  History depth preserved at 48h but interval doubled from 3 min to
 *       6 min. Slot count halves from 960 to 480. Net NVS saving: ~130
 *       entries freed from the "h" namespace.
 *   E3  Legacy keys (hdat, hvlt, hsoc, htmp, hidx, hfill) deleted on first
 *       V2.65.2 boot to reclaim NVS space. One-time history reset on upgrade.
 *   E4  CSV export timestamps match the new 6-minute interval.
 *
 * =========================
 *
 * V2.65.1 CHANGELOG (diagnostic release)
 *   D1  NVS history save/load now checks and logs every putBytes/putFloat/putInt
 *       result. Failures include the specific key name. loadHistory() logs any
 *       blob whose size doesn't match the expected schema instead of silently
 *       zeroing it. Exposes nvs_used / nvs_free / nvs_total in /data for
 *       continuous partition monitoring.
 *   D2  RS485 Frame Spy: optional 20-frame ring buffer in RAM (~4 KB) capturing
 *       raw hex responses from the BMS poll loop. Toggled via /spy endpoint or
 *       the General > Debug panel. Auto-disables after 5 minutes to protect
 *       the buffer. Used for diagnosing BMS current flicker (0A / real / 0A
 *       pattern) and short/malformed frames. Frontend decodes current and
 *       voltage fields client-side so no parsing cost on the device.
 *
 * =========================
 *
 * V2.65 CHANGELOG
 *   M3  Ring-buffer log (40 fixed entries) replaces unbounded String debug_log.
 *       Kills heap fragmentation from addToLog() String churn. Frontend renders
 *       from JSON array instead of server-built HTML.
 *   M2  Non-blocking restart: save/OTA/restore handlers no longer call delay()
 *       before ESP.restart(). Deferred via g_restart_at flag polled in loop().
 *   M1  CAN TX backpressure: per-frame ok/fail counters + consecutive-fail
 *       tracking in sendCanFrame(). Exposed in /data as can_ok, can_fail,
 *       can_fail_streak. debug_can_status now char[64] + canStatusMutex (H4).
 *   M7  simulation_active now also gates sendVictronCAN() (was polling-only).
 *   H4  debug_can_status is no longer a cross-core String. Fixed char buffer
 *       under canStatusMutex eliminates heap corruption vector.
 *
 * =========================
 * ESP32/ESP32-S3 firmware that bridges TopBand LiFePO4 BMS battery packs to
 * Victron/Pylontech/SMA inverters via CAN bus, with a web dashboard,
 * MQTT integration, and Home Assistant auto-discovery.
 *
 * Hardware
 *   Runs on any ESP32 or ESP32-S3 with RS485 and CAN transceivers.
 *   Pin assignments are configured at runtime via the web UI.
 *   Board presets: Waveshare ESP32-S3, LilyGo T-CAN485, or Custom.
 *
 * Architecture
 *   Core 0 (rs485Task): RS485 BMS polling, CAN output, alarm/sysparam monitoring
 *   Core 1 (loop):      WebServer, MQTT, WiFi, LED, energy tracking, NTP
 *
 * Thread safety
 *   rs485Mutex  - serializes RS485 bus access between polling (Core 0) and
 *                 web service requests (Core 1, /svc/* endpoints)
 *   dataMutex   - protects victronData, bms[], alarm_flags, sys_error_msg
 *                 during writes (Core 0) and reads (Core 1)
 *
 * Web UI
 *   Glassmorphism sidebar layout, 5 pages: Dashboard, Battery, Network,
 *   General, Alerts. Frosted glass cards, light + dark mode.
 *   Served as raw literal from handleRoot(), polls /data every 2.5s.
 *   BMS cards throttled to 15s refresh. Chart selection syncs across devices.
 *
 * Authentication
 *   Cookie-based session auth with SHA-256 hashed password stored in NVS.
 *   Login page at /login, session token regenerated on each boot.
 *   Rate limiting: 5 failed attempts trigger 60s lockout.
 *   Password reset via rapid power-cycle (5x within 15s).
 *
 * MQTT
 *   Publishes to {topic}/data every 5s. LWT for offline detection.
 *   Separate {topic}/alarm topic (retained) fires on state changes.
 *   HA auto-discovery re-sent on every connect (keeps FW version current).
 *
 * History & Energy
 *   4 chart types (power/voltage/SOC/temperature) persisted to NVS.
 *   960 points x 3min = 48h depth. Saved every 15 min + on OTA/reboot.
 *   CSV export via /export endpoint.
 *   Rolling energy counters: today + 7-day + monthly, NVS-persistent.
 *
 * Networking
 *   WiFiManager captive portal on first boot. mDNS (.local hostname).
 *   Smart reboot polling on OTA/save (no fixed timers).
 *
 * Diagnostics
 *   Per-BMS communication stats (polls/ok/timeout/errors/spikes).
 *   Reboot reason logged (esp_reset_reason).
 *   State-transition logging for safety events and temperature throttling.
 *
 * NVS layout
 *   "gateway" namespace - all settings (cnt, cells, pins, auth, mqtt, ...)
 *   "h" namespace       - chart history, energy counters (day/week/month)
 *
 * Protocol work based on https://github.com/linedot/topband-bms
 *
 * License: MIT
 */

// ==========================================
// COMPILE-TIME OPTIONS
// Pin assignments and board type are configured at runtime via the web UI.
// The only compile-time choice is SD card support (LilyGo only) and WiFi mode.
// ==========================================
// #define BOARD_LILYGO        // Uncomment ONLY if using LilyGo T-CAN485 with SD card
#define BOARD_WAVESHARE        // Default: no SD card (works for any ESP32/ESP32-S3)

// ==========================================
// COMMUNICATION MODE (uncomment one)
// ==========================================
#define MODE_SMART_WIFI        // WiFi + WebServer + MQTT + OTA
// #define MODE_SIMPLE_CABLE   // RS485 + CAN only, no WiFi
// ==========================================

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <driver/twai.h>
#include <esp_task_wdt.h>
#include <Update.h>
#include <mbedtls/sha256.h>    // On-chip SHA-256 for password hashing
#include <math.h>
#include <Preferences.h>
#include <nvs.h>               // V2.65.1 D1: nvs_get_stats() for partition monitoring
#if defined(BOARD_LILYGO)
  #include <SD.h>              // SD card only available on LilyGo board
  #include <SPI.h>
#endif

// ==========================================
// V2.63: FUNCTION FORWARD DECLARATIONS
// ==========================================
// Struct types are defined later, after the TIMING block. We still need
// forward-declare functions here so Arduino IDE's auto-prototype generator
// has them available. WiFiManager class is forward-declared because its
// full header is included only inside MODE_SMART_WIFI.
// ==========================================
#ifdef MODE_SMART_WIFI
class WiFiManager;
#endif
void setLed(int r, int g, int b);
void addToLog(String msg, bool error);
void spyCaptureFrame(const String &hex, uint8_t addr);   // V2.65.1 D2
bool getNvsStats(uint32_t &used, uint32_t &free, uint32_t &total);  // V2.65.1 D1
bool sendMqttDiagDiscovery();           // V2.66.3 D2
void removeMqttDiagDiscovery();         // V2.66.3 D2
void resetCountersNow(uint32_t new_reset_ts);  // V2.67 F4: zero RAM counters, stamp last_reset_ts
void handleResetCounters();             // V2.67 F4: /svc/reset_counters HTTP handler

// WiFi credentials: leave empty to use WiFiManager captive portal on first boot
const char* FIXED_SSID = ""; 
const char* FIXED_PASS = "";
const char *HOSTNAME = "Topband-Gateway"; 
const char *FW_VERSION = "2.67.2";

// V2.67 F2: Git SHA injected by external build script (git_sha.h).
// Arduino IDE ad-hoc builds without the script fall back to "unknown".
// The header is .gitignored and regenerated by tools/git_sha_gen.sh.
#if __has_include("git_sha.h")
  #include "git_sha.h"
#endif
#ifndef GIT_SHA
  #define GIT_SHA "unknown"
#endif

// V2.67 F2: Project links shown on About section.
const char *GITHUB_URL = "https://github.com/swingstate/topband-bms-gateway";
const char *LICENSE_STR = "MIT License";

// Pre-built RS485 poll commands for BMS address 0-15 (CID2=0x42, analog data request)
const char *POLL_CMDS[16] = {
    "~21004642E00200FD36\r", "~21014642E00201FD34\r", "~21024642E00202FD32\r",
    "~21034642E00203FD30\r", "~21044642E00204FD2E\r", "~21054642E00205FD2C\r",
    "~21064642E00206FD2A\r", "~21074642E00207FD28\r", "~21084642E00208FD26\r",
    "~21094642E00209FD24\r", "~210A4642E0020AFD14\r", "~210B4642E0020BFD12\r",
    "~210C4642E0020CFD10\r", "~210D4642E0020DFD0E\r", "~210E4642E0020EFD0C\r",
    "~210F4642E0020FFD0A\r"};

// ==========================================
// TIMING & LIMITS
// ==========================================
#define NUM_LEDS 1              // Onboard NeoPixel LED count
#define MAX_BMS 16              // Maximum supported BMS packs
#define RS485_BAUD 9600         // TopBand BMS protocol baud rate
#define POLL_INTERVAL 3000      // BMS data poll interval (ms)
#define RS485_TIMEOUT 600       // Response timeout per BMS (ms)
#define BUS_GUARD_TIME 200      // Delay between RS485 transactions (ms)
#define WDT_TIMEOUT 60          // V2.66 W1: Watchdog timeout raised from 30 to 60 seconds
#define HISTORY_LEN 960          // V2.65.4: 960 x 3min = 48h chart history depth (RAM only, not persisted)
#define HISTORY_INTERVAL 180000  // V2.65.4: Chart data point interval: 3 minutes (ms)
#define SD_LOG_INTERVAL 60000   // SD card log write interval (ms)
#define MQTT_INTERVAL 5000      // MQTT publish interval (ms)
#define MQTT_DIAG_INTERVAL 30000 // V2.66.2 D1: MQTT diagnostics publish interval (ms)
#define MQTT_CELLS_INTERVAL 20000 // V2.67 F1: L3 per-BMS cells topic publish cadence (ms)
#define MQTT_CELLS_SCHEMA_V 1     // V2.67 F1: Cells topic payload schema version
#define ALARM_POLL_INTERVAL 15000   // BMS alarm status poll (round-robin, ms)
#define SYSPARAM_POLL_INTERVAL 30000 // BMS system parameter poll (round-robin, ms)
#define TB_INFO_MAX 256         // Max TopBand protocol INFO field length
#define ALERT_RING_SIZE 25      // V2.61: alert ring buffer depth
#define ALERT_MSG_LEN 96        // V2.61: max chars per alert message

// ==========================================
// V2.63: ALL STRUCT DEFINITIONS CONSOLIDATED HERE
// ==========================================
// Structs are defined here, before any function declarations, so that
// Arduino IDE's auto-prototype generator sees complete types. Previously,
// forward-declarations were used but the IDE still emitted prototypes
// referencing incomplete types in some toolchain versions, causing
// "TBFrame has not been declared" errors. Defining structs upfront fixes
// that class of error entirely.
// ==========================================

// Per-BMS live data (updated by parseTopband on Core 0)
struct BMSData {
  bool valid; float voltage; float current; int soc; int soh; float rem_ah; float full_ah; float cells[32];
  int cell_count; float temps[8]; int temp_count; uint16_t cycles;
  float minCellV; float maxCellV; float avgCellV;
  int minCellIdx; int maxCellIdx; float maxTemp; float avgTemp; unsigned long last_seen;
};

// Aggregated data sent to inverter via CAN (updated by calculateVictronData on Core 0)
struct VictronType {
  float totalCurrent; float avgVoltage; float avgSOC; float avgSOH; float maxChargeCurrent; float maxDischargeCurrent;
  float avgTemp; float totalCapacity; float remainCapacity; float totalPower; int activePacks;
};

// TopBand RS485 protocol frame (SOI ~ VER ADR CID1 CID2 LENGTH INFO CHKSUM EOI)
struct TBFrame {
  bool valid;
  bool info_ascii_raw;
  uint8_t version;
  uint8_t adr;
  uint8_t cid1;
  uint8_t cid2;
  uint16_t length_field;
  uint16_t lenid;
  uint16_t checksum;
  uint8_t info[TB_INFO_MAX];
  size_t info_len;
};

struct TBDate {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

// BMS system parameters (read via CID2=0x47)
struct TBSysParam {
  float cell_high_v;
  float cell_low_v;
  float cell_under_v;
  float charge_high_t;
  float charge_low_t;
  float charge_current_max_a;
  float module_high_v;
  float module_low_v;
  float module_under_v;
  float discharge_high_t;
  float discharge_low_t;
  float discharge_current_max_a;
};

// BMS alarm/status bitmap (read via CID2=0x44)
struct TBAlarmInfoParsed {
  uint8_t cell_count;
  uint8_t temp_count;
  uint8_t cell_voltage_status[16];
  uint8_t cell_temp_status[16];
  uint8_t balancer_temp_status;
  uint8_t environment_temp_status;
  uint8_t mosfet_temp_status;
  uint8_t charge_current_status;
  uint8_t module_voltage_status;
  uint8_t status_count;
  uint64_t status_bits;
};

struct TBAnalogDiag {
  uint8_t id;
  uint8_t cell_count;
  uint8_t temp_count;
  float min_cell_v;
  float max_cell_v;
  float current_a;
  float pack_v;
  float rem_ah;
  float full_ah;
  int soc;
  int soh;
};

// Per-BMS communication statistics (reset on reboot)
struct BMSCommStats {
  uint32_t polls;          // Total poll attempts
  uint32_t ok;             // Successful responses
  uint32_t timeouts;       // No response within timeout
  uint32_t errors;         // Parse errors or bad frames
  uint32_t spikes;         // Spike filter rejections
  uint32_t current_holds;  // V2.66 H1: hold-last-value activations
};

// V2.61: alert ring buffer entry (NVS-persisted)
struct AlertEntry {
  uint32_t ts;                // Unix epoch (0 if NTP unavailable)
  uint32_t uptime_s;          // Fallback: seconds since boot when alert raised
  uint8_t sev;                // 0=info, 1=warn, 2=err
  uint8_t _pad[3];
  char msg[ALERT_MSG_LEN];    // Null-terminated
};

// V2.61: state-transition tracking to avoid alert spam on 10s tick
struct AlertState {
  bool no_packs;              // activePacks==0
  bool bms_offline[MAX_BMS];
  bool cell_max_near[MAX_BMS];
  bool cell_min_crit[MAX_BMS];
  bool drift_near[MAX_BMS];
  bool temp_near[MAX_BMS];
  bool soc_low[MAX_BMS];
  bool sys_soc_low;
  bool can_error_state;
  bool mqtt_fail_state;
};


// ==========================================
// PIN ASSIGNMENTS (runtime-configurable, persisted in NVS)
// Board type: 0=Waveshare, 1=LilyGo, 2=Custom
// Presets loaded at boot, overridable via web UI.
// ==========================================

// SD card support remains compile-time (controls #include)
#if defined(BOARD_LILYGO)
  #define HAS_SD_CARD true
  #define SD_MISO 2
  #define SD_MOSI 15
  #define SD_SCLK 14
  #define SD_CS 13
#else
  #define HAS_SD_CARD false
  #define SD_MISO -1
  #define SD_MOSI -1
  #define SD_SCLK -1
  #define SD_CS -1
#endif

// Pin variables (set from NVS or preset at boot)
int g_pin_rs485_tx = 17;
int g_pin_rs485_rx = 18;
int g_pin_rs485_dir = 21;     // Single direction pin (Waveshare/Custom mode)
int g_pin_can_tx = 15;
int g_pin_can_rx = 16;
int g_pin_led = 38;
// LilyGo-specific pins (active when board_type=1)
int g_pin_lilygo_pwr = 16;
int g_pin_lilygo_en = 19;
int g_pin_lilygo_cb = 17;
// Board type: 0=Waveshare, 1=LilyGo, 2=Custom
int g_board_type = 0;

// Presets: apply default pin values for a given board type
void applyBoardPreset(int type) {
  if (type == 1) {
    // LilyGo T-CAN485
    g_pin_rs485_tx = 22; g_pin_rs485_rx = 21; g_pin_rs485_dir = -1;
    g_pin_can_tx = 27; g_pin_can_rx = 26; g_pin_led = 4;
    g_pin_lilygo_pwr = 16; g_pin_lilygo_en = 19; g_pin_lilygo_cb = 17;
  } else {
    // Waveshare ESP32-S3 (also default for Custom)
    g_pin_rs485_tx = 17; g_pin_rs485_rx = 18; g_pin_rs485_dir = 21;
    g_pin_can_tx = 15; g_pin_can_rx = 16; g_pin_led = 38;
    g_pin_lilygo_pwr = -1; g_pin_lilygo_en = -1; g_pin_lilygo_cb = -1;
  }
}

#ifdef MODE_SMART_WIFI
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <WiFiManager.h>
  #include <PubSubClient.h>
#endif

// ==========================================
// HARDWARE OBJECTS
// ==========================================
Adafruit_NeoPixel pixels(NUM_LEDS, 0, NEO_GRB + NEO_KHZ800); // Pin set in setup()
HardwareSerial RS485(1);       // UART1 for RS485 communication
Preferences preferences;       // NVS storage for settings

// ==========================================
// WIFI / WEB / MQTT GLOBALS
// ==========================================
#ifdef MODE_SMART_WIFI
  WebServer server(80);
  WiFiClient espClient;
  PubSubClient mqtt(espClient);
  Preferences histStore;       // Separate NVS namespace "h" for history data
  
  // MQTT configuration (persisted in NVS)
  String g_mqtt_server = ""; int g_mqtt_port = 1883; String g_mqtt_user = ""; String g_mqtt_pass = "";
  String g_mqtt_topic = "Topband/BMS";
  String g_dev_uid = "tb_000000"; // Generated from MAC address at boot
  bool g_mqtt_enable = false; bool g_mqtt_full = false; bool g_ha_enable = false;
// V2.66.2 C3+: Global stream-abort flag. Set by streamAlive() when the
// TCP client has disconnected mid-response. Checked by handleData() after
// each major chunk-group so that a disconnect propagates all the way out
// of the handler instead of returning only from inner lambdas/loops.
// Reset at the start of every handleData() call. Single-writer on Core 1.
static bool g_stream_abort = false;

// V2.66.2 D1: Debug counters for MQTT diag publish.
// All single-writer on Core 1 (except last_error which parseTopband can set),
// so reads from Core 1 see consistent values without mutex.
uint32_t g_stream_aborts   = 0;  // Count of C3+ abort triggers since boot
uint32_t g_handler_max_ms  = 0;  // Longest HTTP handler runtime since boot
uint32_t g_loop_max_ms     = 0;  // Longest loop() iteration on Core 1
uint32_t g_wdt_warnings    = 0;  // Count of loop() iterations > 45s (near-WDT)
extern uint32_t g_rl_rejects;    // V2.66.2 D1: forward-decl, defined with rate-limit slots below

bool g_mqtt_diag = false;           // V2.66.2 D1: Publish {base}/diag every 30s (opt-in)
unsigned long last_mqtt_diag = 0;   // V2.66.2 D1: cadence tracker
uint8_t g_mqtt_level = 0;           // V2.67 F1: 0=L1 lean, 1=L2 +per-BMS stats, 2=L3 +cells topic
unsigned long last_mqtt_cells = 0;  // V2.67 F1: cadence tracker for cells topic (L3)
// V2.67 F4: Counter reset + 7-day rollover.
// g_last_reset_ts is the Unix epoch of the last manual/rollover counter reset.
// 0 = uninitialised (fresh flash or NTP never synced). First NTP-synced main
// loop after boot stamps it once and writes NVS. Rollover fires when
// (time(NULL) - g_last_reset_ts) >= 604800 AND NTP is synced. Reset writes
// NVS exactly once per event, not per check. RAM counters reset on reboot
// unconditionally; reboot itself never bumps g_last_reset_ts.
uint32_t g_last_reset_ts = 0;        // V2.67 F4: Unix epoch of last counter reset (persisted)
unsigned long g_last_rollover_check = 0;  // V2.67 F4: millis() of last rollover check (60s cadence)
#define RESET_ROLLOVER_SEC  604800UL   // V2.67 F4: 7 days = 7 * 86400
#define RESET_CHECK_INTERVAL 60000UL   // V2.67 F4: rollover check cadence (60 s)
#define NTP_SYNC_EPOCH_MIN 1672531200UL  // V2.67 F4: Jan 1 2023; matches existing NTP-synced threshold
  bool g_sd_enable = false; bool g_sd_spy = false; bool g_serial_debug = false;
  // V2.65 M3: Ring-buffer log replaces unbounded String debug_log.
  // V2.65 H4: debug_can_status is now a fixed char buffer, not a cross-core String.
  // Actual storage + mutex declared in the SHARED STATE block below.
  bool sd_ok = false; bool sd_error_flag = false;
  int g_theme_id = 0; String g_ntp_server = "pool.ntp.org"; int g_timezone_offset = 1;

  // Chart history: 4 data types x 960 points = 48 hours at 3-min intervals
  // Avg arrays are NVS-persisted; min/max rebuild from avg after reboot
  int16_t powerHistory[HISTORY_LEN]; int historyIdx = 0;
  int16_t voltHistory[HISTORY_LEN];  // Stored as V * 100 (int16)
  int16_t socHistory[HISTORY_LEN];   // Stored as % * 10 (int16)
  int16_t tempHistory[HISTORY_LEN];  // Stored as C * 10 (int16)
  int16_t powerMin[HISTORY_LEN]; int16_t powerMax[HISTORY_LEN];
  int16_t voltMin[HISTORY_LEN];  int16_t voltMax[HISTORY_LEN];
  int16_t socMin[HISTORY_LEN];   int16_t socMax[HISTORY_LEN];
  int16_t tempMin[HISTORY_LEN];  int16_t tempMax[HISTORY_LEN];
  int historyFilled = 0;             // V2.61: slots written since first boot, capped at HISTORY_LEN. Distinguishes real 0 from "no data".

  // Accumulators: collect min/max/avg between 3-min history intervals
  float acc_sum[4] = {0};  int acc_count = 0;
  float acc_min[4], acc_max[4];
  bool acc_reset = true;
  int g_chart1 = 0; int g_chart2 = 1;  // Chart type selection: 0=power, 1=voltage, 2=soc, 3=temp

  // Energy tracking: calculated by ESP (power * time), reset at midnight
  float energy_in_today = 0; float energy_out_today = 0;
  unsigned long last_energy_calc = 0;
  // Rolling energy history: 7 days + 12 months, persisted in NVS "h" namespace
  float energy_days_in[7] = {0};   // [0]=yesterday, [1]=2 days ago, ... [6]=7 days ago
  float energy_days_out[7] = {0};
  float energy_months_in[12] = {0}; // [0]=this month, [1]=last month, ... [11]=12 months ago
  float energy_months_out[12] = {0};
  int current_month = -1;           // Tracks month for rollover
  unsigned long last_hist_time = 0; unsigned long last_sd_time = 0; unsigned long last_mqtt_time = 0;
  unsigned long last_mqtt_reconnect_attempt = 0; unsigned long mqtt_reconnect_interval = 5000;
  uint16_t mqtt_fail_count = 0; uint32_t heap_min = UINT32_MAX;
  int current_day = -1;        // Tracks day-of-month for midnight energy reset
#endif

// ==========================================
// BMS & SAFETY CONFIGURATION (persisted in NVS "gateway" namespace)
// ==========================================
int g_bms_count = 2; int g_force_cell_count = 0; int g_detected_cells = 0; 
bool g_victron_enable = true;
float g_charge_amps = 30.0; float g_discharge_amps = 30.0;
float g_cvl_voltage = 52.5;       // Charge Voltage Limit sent to inverter
float g_safe_volt = 53.25;        // Safety cutoff: max pack voltage
float g_safe_cell = 3.55;         // Safety cutoff: max cell voltage
float g_safe_drift = 0.20;        // Safety warning: max cell drift (V)
// V2.66 M4: Configurable spike filter thresholds. Defaults match the hardcoded
// values from V2.65.x so behaviour is unchanged unless the user tunes them.
float g_spike_volt = 5.0;         // Max plausible voltage jump between polls (V)
float g_spike_cur  = 250.0;       // Max plausible current jump between polls (A)
int   g_spike_soc  = 20;          // Max plausible SOC jump between polls (%)
// Temperature cutoffs (C): charge/discharge are throttled in soft zones
float g_tc_min = 5.0; float g_tc_max = 50.0;   // Charge temperature range
float g_td_min = -20.0; float g_td_max = 60.0;  // Discharge temperature range
int g_temp_mode = 0;               // 0=use hottest sensor (safe), 1=use average
int g_can_protocol = 0;            // 0=Victron, 1=Pylontech, 2=SMA
bool g_expert_mode = true; bool g_easy_mode = false;          // Deprecated: kept for NVS compat
bool g_setup_done = true; bool g_easy_auto_applied = false;   // Deprecated: kept for NVS compat
int g_soc_source_mode = 2;        // SOC source: 0=calculated, 1=raw BMS, 2=hybrid
bool g_maint_charge_mode = false;
float g_maint_target_v = 51.00;   // Maintenance charge target voltage
bool g_auto_balance_enable = true; // Auto-balance watchdog (30 days)
uint32_t g_auto_balance_last_ts = 0;
bool g_runtime_auto_balance_due = false;
bool g_runtime_balance_reached_prev = false;
float factor_charge = 1.0; float factor_discharge = 1.0; // Temperature throttle factors (0.0-1.0)

// Web authentication (cookie-based session, SHA-256 hashed password in NVS)
bool g_auth_enable = false;
String g_auth_user = "admin";
String g_auth_hash = "";           // SHA-256 hex digest, never stored in plaintext

// ==========================================
// RUNTIME STATE (not persisted)
// ==========================================
unsigned long last_poll_time = 0; unsigned long last_can_time = 0;
unsigned long stat_rx_count = 0; unsigned long stat_tx_count = 0;
bool can_error_flag = false;
uint8_t alarm_flags = 0;          // Bitmask: 0x02=overvolt, 0x08=temp, 0x10=undervolt, 0x20=drift, 0x40=BMS alarm, 0x80=no packs
String sys_error_msg = "OK";
bool simulation_active = false;
String g_boot_reason = "UNKNOWN";   // Set once in setup() from esp_reset_reason()
uint32_t g_boot_epoch = 0;          // V2.67 F2: Captured lazily once NTP syncs. Unix epoch of boot instant. 0 means not yet synced.
SemaphoreHandle_t rs485Mutex = NULL;  // Protects RS485 bus access between cores
SemaphoreHandle_t dataMutex = NULL;   // Protects victronData/bms[]/alarm_flags between cores
SemaphoreHandle_t logMutex = NULL;         // V2.65 M3: Protects g_log_ring writes/reads
SemaphoreHandle_t canStatusMutex = NULL;   // V2.65 H4: Protects g_can_status char buffer
TaskHandle_t rs485TaskHandle = NULL;

// ==========================================
// V2.65 M3: FIXED-SIZE RING BUFFER LOG
// Replaces unbounded String debug_log. Eliminates heap fragmentation
// from per-call String allocations. ~4KB total, newest-first emission.
// ==========================================
#define LOG_RING_SIZE 40
#define LOG_MSG_MAX   92          // Total entry = 100 bytes (4-byte aligned)

struct LogEntry {
  uint32_t ts;                    // Unix time, or millis/1000 if NTP not synced
  uint8_t  flags;                 // bit0 = error
  char     msg[LOG_MSG_MAX];      // Null-terminated, truncated on overflow
};
LogEntry g_log_ring[LOG_RING_SIZE];
uint8_t  g_log_head = 0;          // Next write slot
uint8_t  g_log_count = 0;         // Valid entries (caps at LOG_RING_SIZE)

// V2.65 H4: Fixed char buffer for CAN status (was cross-core String).
char g_can_status[64] = "Init";

// V2.65 M1: CAN TX backpressure counters
uint32_t g_can_tx_ok = 0;
uint32_t g_can_tx_fail = 0;
uint32_t g_can_tx_fail_streak = 0;

// V2.65 M2: Deferred restart flag. Set by handlers, polled in loop().
// Eliminates delay() + ESP.restart() blocking in web callbacks.
unsigned long g_restart_at = 0;
unsigned long last_alarm_poll_time = 0;
int next_alarm_poll_bms = 0;
unsigned long last_sysparam_poll_time = 0;
int next_sysparam_poll_bms = 0;

// ==========================================
// V2.65.1 D2: RS485 FRAME SPY RING BUFFER
// Optional diagnostic capture of raw BMS responses. Used to diagnose BMS
// current-flicker behaviour without attaching a USB serial cable. Ring is
// populated inside parseTopband() when g_rs485_spy_enable is true. Auto-
// disables via g_rs485_spy_deadline in loop() after 5 minutes to prevent
// unbounded capture. Mutex-protected against cross-core access.
// ==========================================
#define SPY_RING_SIZE    20
#define SPY_FRAME_MAX   220     // Hex chars for a full TopBand 0x42 analog frame

struct SpyEntry {
  uint32_t ts;                  // millis() at capture
  uint8_t  addr;                // BMS address (0..15)
  uint16_t len;                 // Actual frame length in hex chars
  char     hex[SPY_FRAME_MAX];  // Raw hex response, null-terminated
};
SpyEntry g_spy_ring[SPY_RING_SIZE];
uint8_t  g_spy_head = 0;        // Next write slot
uint8_t  g_spy_count = 0;       // Valid entries (caps at SPY_RING_SIZE)

bool           g_rs485_spy_enable = false;    // Master toggle
unsigned long  g_rs485_spy_deadline = 0;      // millis() when auto-off triggers
uint32_t       g_spy_capture_count = 0;       // Total frames captured since enable

SemaphoreHandle_t spyMutex = NULL;            // Protects g_spy_ring writes/reads

// ==========================================
// GLOBAL STORAGE (structs defined earlier, after TIMING block)
// ==========================================

BMSData bms[MAX_BMS];
VictronType victronData;

// Per-BMS alarm and system parameter cache (updated by round-robin polling on Core 0)
uint64_t g_bms_alarm_bits[MAX_BMS];
bool g_bms_was_online[MAX_BMS];       // Tracks online->offline transitions for logging
unsigned long g_bms_alarm_last_seen[MAX_BMS];
uint8_t g_bms_alarm_status_count[MAX_BMS];
float g_bms_sys_module_high_v[MAX_BMS];
float g_bms_sys_module_under_v[MAX_BMS];
float g_bms_sys_charge_max_a[MAX_BMS];
float g_bms_sys_discharge_max_a[MAX_BMS];
float g_bms_sys_cell_high_v[MAX_BMS];
float g_bms_sys_charge_low_t[MAX_BMS];
float g_bms_sys_charge_high_t[MAX_BMS];
float g_bms_sys_discharge_low_t[MAX_BMS];
float g_bms_sys_discharge_high_t[MAX_BMS];
unsigned long g_bms_sys_last_seen[MAX_BMS];

BMSCommStats g_bms_stats[MAX_BMS];

// V2.61: alert ring buffer storage. Server-side detection runs every 10s on Core 1.
// Dedup: same message within 30s is suppressed. Clear: POST /alerts/clear wipes RAM + NVS.
AlertEntry g_alerts[ALERT_RING_SIZE];
int g_alert_count = 0;        // Total entries stored (capped at ALERT_RING_SIZE)
int g_alert_head = 0;         // Next write index (ring position)
bool g_alerts_dirty = false;
unsigned long g_alerts_last_save = 0;
unsigned long g_alerts_last_detect = 0;
AlertState g_alert_state = {};


// Runtime protocol-derived limits (capped from BMS 0x47 system params)
float g_runtime_cvl = 0.0f;          // Effective CVL (may be lower than config)
float g_runtime_proto_ccl_cap = 0.0f; // Protocol charge current cap (sum of all BMS)
float g_runtime_proto_dcl_cap = 0.0f; // Protocol discharge current cap
bool g_runtime_proto_uv_hit = false;   // True if any BMS near undervolt threshold

// ==========================================
// TOPBAND RS485 PROTOCOL HELPERS
// ==========================================
// Frame format: ~VER ADR CID1 CID2 LENGTH INFO CHKSUM CR
// All values are ASCII hex encoded. Checksum is modular complement.
// Protocol details: https://github.com/linedot/topband-bms
// ==========================================
void setLed(int r, int g, int b) { pixels.setPixelColor(0, pixels.Color(r, g, b)); pixels.show(); }
uint16_t get_u16(const uint8_t *b, int o) { return (b[o] << 8) | b[o + 1]; }
int16_t get_s16(const uint8_t *b, int o) { uint16_t v = get_u16(b, o); return (v > 32767) ? (v - 65536) : v; }
uint8_t parse_hex(char c) { if (c >= '0' && c <= '9') return c - '0'; if (c >= 'A' && c <= 'F') return c - 'A' + 10; if (c >= 'a' && c <= 'f') return c - 'a' + 10; return 0xFF; }

uint16_t tbBuildLengthField(uint16_t lenid) {
  lenid &= 0x0FFF;
  uint16_t sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  uint16_t lchk = ((~(sum & 0xF)) + 1) & 0xF;
  return (lchk << 12) | lenid;
}

bool tbCheckLengthField(uint16_t length_field) {
  uint16_t lenid = length_field & 0x0FFF;
  uint16_t lchk = (length_field >> 12) & 0xF;
  uint16_t sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  uint16_t expected = ((~(sum & 0xF)) + 1) & 0xF;
  return lchk == expected;
}

String toHex2(uint8_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", v);
  return String(b);
}

String toHex4(uint16_t v) {
  char b[5];
  snprintf(b, sizeof(b), "%04X", v);
  return String(b);
}

bool parseHexByteAt(const String &s, int pos, uint8_t &out) {
  if (pos < 0 || pos + 1 >= s.length()) return false;
  uint8_t hi = parse_hex(s[pos]);
  uint8_t lo = parse_hex(s[pos + 1]);
  if (hi == 0xFF || lo == 0xFF) return false;
  out = (hi << 4) | lo;
  return true;
}

bool parseHexU16At(const String &s, int pos, uint16_t &out) {
  if (pos < 0 || pos + 3 >= s.length()) return false;
  uint8_t b0, b1;
  if (!parseHexByteAt(s, pos, b0)) return false;
  if (!parseHexByteAt(s, pos + 2, b1)) return false;
  out = ((uint16_t)b0 << 8) | b1;
  return true;
}

String tbBuildCommand(uint8_t addr, uint8_t cid2, const uint8_t *info, size_t info_len_bytes) {
  uint16_t lenid = (uint16_t)(info_len_bytes * 2);
  uint16_t length_field = tbBuildLengthField(lenid);

  String body = "";
  body.reserve(32 + lenid);
  body += toHex2(0x21);  // protocol version
  body += toHex2(addr);
  body += toHex2(0x46);  // CID1 battery
  body += toHex2(cid2);
  body += toHex4(length_field);
  for (size_t i = 0; i < info_len_bytes; i++) body += toHex2(info[i]);

  uint16_t sum = 0;
  for (int i = 0; i < body.length(); i++) sum += (uint8_t)body[i];
  uint16_t chk = ((~(sum & 0xFFFF)) + 1) & 0xFFFF;

  String cmd = "~";
  cmd += body;
  cmd += toHex4(chk);
  cmd += "\r";
  return cmd;
}

// RS485 direction helpers (runtime pin config)
void rs485SetTX() {
  if (g_pin_rs485_dir >= 0) digitalWrite(g_pin_rs485_dir, HIGH);
}
void rs485SetRX() {
  if (g_pin_rs485_dir >= 0) digitalWrite(g_pin_rs485_dir, LOW);
}

// Send command and receive response over RS485 bus (mutex-protected)
bool rs485ExchangeFrame(const String &cmd, String &resp, uint32_t timeout_ms) {
  if (!rs485Mutex || !xSemaphoreTake(rs485Mutex, pdMS_TO_TICKS(3000))) return false;

  bool ok = false;
  resp = "";
  resp.reserve(512);

  rs485SetTX();
  RS485.flush();
  while (RS485.available()) RS485.read();
  RS485.print(cmd);
  RS485.flush();
  rs485SetRX();

  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (RS485.available()) {
      char c = RS485.read();
      if (resp.length() < 700) resp += c;
      if (c == '\r') { ok = true; break; }
    } else {
      vTaskDelay(1);
    }
  }

  xSemaphoreGive(rs485Mutex);
  return ok && resp.length() > 0;
}

bool tbDecodeFrame(const String &raw_resp, TBFrame &frame, String &err) {
  frame = {};
  err = "";
  int soi = raw_resp.indexOf('~');
  if (soi < 0) { err = "No SOI"; return false; }

  String s = raw_resp.substring(soi);
  int eoi = s.indexOf('\r');
  if (eoi < 0) eoi = s.indexOf('\n');
  if (eoi >= 0) s = s.substring(0, eoi + 1);

  if (s.length() < 18) { err = "Frame too short"; return false; }
  char eoi_ch = s[s.length() - 1];
  if (eoi_ch != '\r' && eoi_ch != '\n') { err = "No EOI"; return false; }

  uint8_t version, adr, cid1, cid2;
  uint16_t length_field, rx_chk;
  if (!parseHexByteAt(s, 1, version) || !parseHexByteAt(s, 3, adr) || !parseHexByteAt(s, 5, cid1) || !parseHexByteAt(s, 7, cid2)) {
    err = "Header hex parse failed"; return false;
  }
  if (!parseHexU16At(s, 9, length_field)) { err = "Length parse failed"; return false; }
  if (!tbCheckLengthField(length_field)) { err = "LCHKSUM invalid"; return false; }

  uint16_t lenid = length_field & 0x0FFF;
  int expected_len = 18 + lenid;
  if (s.length() < expected_len) { err = "Truncated frame"; return false; }
  if (!parseHexU16At(s, 13 + lenid, rx_chk)) { err = "CHKSUM parse failed"; return false; }

  uint16_t sum = 0;
  for (int i = 1; i < 13 + lenid; i++) sum += (uint8_t)s[i];
  uint16_t expected_chk = ((~(sum & 0xFFFF)) + 1) & 0xFFFF;
  if (rx_chk != expected_chk) { err = "CHKSUM invalid"; return false; }

  frame.valid = true;
  frame.version = version;
  frame.adr = adr;
  frame.cid1 = cid1;
  frame.cid2 = cid2;
  frame.length_field = length_field;
  frame.lenid = lenid;
  frame.checksum = rx_chk;
  frame.info_ascii_raw = false;

  if (lenid == 0) {
    frame.info_len = 0;
    return true;
  }
  if (lenid > TB_INFO_MAX) { err = "Info too long"; return false; }

  bool hex_ok = (lenid % 2) == 0;
  if (hex_ok) {
    size_t info_bytes = lenid / 2;
    for (size_t i = 0; i < info_bytes; i++) {
      uint8_t b;
      if (!parseHexByteAt(s, 13 + (int)(i * 2), b)) { hex_ok = false; break; }
      frame.info[i] = b;
    }
    if (hex_ok) {
      frame.info_len = info_bytes;
      return true;
    }
  }

  // Manufacturer info can be raw ASCII in protocol field.
  for (uint16_t i = 0; i < lenid; i++) frame.info[i] = (uint8_t)s[13 + i];
  frame.info_ascii_raw = true;
  frame.info_len = lenid;
  return true;
}

bool tbRequestFrame(uint8_t bms_id, uint8_t cid2, const uint8_t *info, size_t info_len_bytes, TBFrame &reply, String &err, bool allow_nonzero_cid2 = false) {
  String cmd = tbBuildCommand(bms_id, cid2, info, info_len_bytes);
  String raw;
  if (!rs485ExchangeFrame(cmd, raw, RS485_TIMEOUT + 200)) {
    err = "No response";
    return false;
  }
  if (!tbDecodeFrame(raw, reply, err)) return false;
  if (!allow_nonzero_cid2 && reply.cid2 != 0x00) {
    err = "BMS reply code 0x" + String(reply.cid2, HEX);
    return false;
  }
  return true;
}

float tbKelvin10ToC(uint16_t raw) {
  return ((float)raw - 2731.0f) / 10.0f;
}

bool tbParseDateFromFrame(const TBFrame &f, TBDate &d) {
  if (!f.valid || f.info_len < 7) return false;
  d.year = ((int)f.info[0] << 8) | (int)f.info[1];
  d.month = f.info[2];
  d.day = f.info[3];
  d.hour = f.info[4];
  d.minute = f.info[5];
  d.second = f.info[6];
  return true;
}

bool tbParseSystemParameterFromFrame(const TBFrame &f, TBSysParam &sp) {
  if (!f.valid || f.info_len < 24) return false;
  sp.cell_high_v = ((float)get_u16(f.info, 0)) / 1000.0f;
  sp.cell_low_v = ((float)get_u16(f.info, 2)) / 1000.0f;
  sp.cell_under_v = ((float)get_u16(f.info, 4)) / 1000.0f;
  sp.charge_high_t = tbKelvin10ToC(get_u16(f.info, 6));
  sp.charge_low_t = tbKelvin10ToC(get_u16(f.info, 8));
  sp.charge_current_max_a = ((float)((int16_t)get_u16(f.info, 10))) * 0.01f;
  sp.module_high_v = ((float)get_u16(f.info, 12)) / 100.0f;
  sp.module_low_v = ((float)get_u16(f.info, 14)) / 100.0f;
  sp.module_under_v = ((float)get_u16(f.info, 16)) / 100.0f;
  sp.discharge_high_t = tbKelvin10ToC(get_u16(f.info, 18));
  sp.discharge_low_t = tbKelvin10ToC(get_u16(f.info, 20));
  sp.discharge_current_max_a = ((float)((int16_t)get_u16(f.info, 22))) * 0.01f;
  return true;
}

bool tbParseAlarmInfoFromFrame(const TBFrame &f, TBAlarmInfoParsed &a) {
  if (!f.valid || f.info_len < 8) return false;
  memset(&a, 0, sizeof(a));

  size_t off = 1;
  if (off >= f.info_len) return false;
  a.cell_count = f.info[off];
  if (a.cell_count > 16) a.cell_count = 16;
  off += 1;
  if (off + a.cell_count > f.info_len) return false;
  for (uint8_t i = 0; i < a.cell_count; i++) a.cell_voltage_status[i] = f.info[off + i];
  off += a.cell_count;

  if (off >= f.info_len) return false;
  a.temp_count = f.info[off];
  off += 1;
  uint8_t cell_temp_bytes = (a.temp_count >= 3) ? (a.temp_count - 3) : 0;
  if (cell_temp_bytes > 16) cell_temp_bytes = 16;
  if (off + cell_temp_bytes > f.info_len) return false;
  for (uint8_t i = 0; i < cell_temp_bytes; i++) a.cell_temp_status[i] = f.info[off + i];
  off += cell_temp_bytes;

  if (off + 3 > f.info_len) return false;
  a.balancer_temp_status = f.info[off];
  a.environment_temp_status = f.info[off + 1];
  a.mosfet_temp_status = f.info[off + 2];
  off += 3;

  if (off + 3 > f.info_len) return false;
  a.charge_current_status = f.info[off];
  a.module_voltage_status = f.info[off + 1];
  a.status_count = f.info[off + 2];
  off += 3;

  if (off + a.status_count > f.info_len) return false;
  uint8_t status_bytes = a.status_count > 8 ? 8 : a.status_count;
  for (uint8_t i = 0; i < status_bytes; i++) a.status_bits |= ((uint64_t)f.info[off + i]) << (8 * i);
  return true;
}

bool tbParseAnalogDiagFromFrame(const TBFrame &f, TBAnalogDiag &a) {
  if (!f.valid || f.info_len < 16) return false;
  memset(&a, 0, sizeof(a));
  size_t p = 0;
  a.id = f.info[p++];
  a.cell_count = f.info[p++];
  if (a.cell_count > 32) a.cell_count = 32;

  float min_v = 99.0f, max_v = 0.0f;
  for (uint8_t i = 0; i < a.cell_count; i++) {
    if (p + 1 >= f.info_len) return false;
    float v = ((float)get_u16(f.info, (int)p)) / 1000.0f;
    if (v > 0.1f) {
      if (v < min_v) min_v = v;
      if (v > max_v) max_v = v;
    }
    p += 2;
  }
  a.min_cell_v = min_v < 90.0f ? min_v : 0.0f;
  a.max_cell_v = max_v;

  if (p >= f.info_len) return false;
  a.temp_count = f.info[p++];
  if (a.temp_count > 8) a.temp_count = 8;
  size_t temp_bytes = (size_t)a.temp_count * 2;
  if (p + temp_bytes > f.info_len) return false;
  p += temp_bytes;

  if (p + 10 >= f.info_len) return false;
  a.current_a = ((float)((int16_t)get_u16(f.info, (int)p))) * 0.01f; p += 2;
  a.pack_v = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 2;
  a.rem_ah = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 3;
  a.full_ah = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 2;

  int soc_calc = 0;
  if (a.full_ah > 0.01f) soc_calc = (int)((a.rem_ah / a.full_ah) * 100.0f);
  if (soc_calc > 100) soc_calc = 100;
  if (soc_calc < 0) soc_calc = 0;
  a.soc = soc_calc;
  a.soh = 100;

  // Newer Topband payloads carry cycle(2B), SOC(1B), SOH(1B).
  // Keep legacy fallback to avoid regressions with older frames.
  size_t meta = p;
  if (meta + 3 < f.info_len) {
    uint8_t raw_soc = f.info[meta + 2];
    uint8_t raw_soh = f.info[meta + 3];
    bool extended_layout = (meta + 7 < f.info_len);
    bool raw_valid = (raw_soc <= 100) && (raw_soh <= 100);
    a.soc = tbSelectSocValue(soc_calc, (int)raw_soc, raw_valid, extended_layout);
    if (raw_valid && (extended_layout || g_soc_source_mode == 1 || abs((int)raw_soc - soc_calc) <= 15)) {
      a.soh = (int)raw_soh;
    } else {
      uint16_t s16 = get_u16(f.info, (int)(meta + 2));
      if (s16 <= 100) a.soh = (int)s16;
    }
  }
  return true;
}

const char* tbAlarmBitName(uint8_t bit) {
  switch (bit) {
    case 0: return "Cell over voltage protect";
    case 1: return "Cell under voltage";
    case 2: return "Charge over current protect";
    case 3: return "Cell over voltage alarm";
    case 4: return "Discharge over current 1 protect";
    case 5: return "Cell temperature discharge protect";
    case 6: return "Cell temperature charge protect";
    case 7: return "Pack under voltage alarm";
    case 8: return "Open current limit";
    case 9: return "Charge MOSFET on";
    case 10: return "Discharge MOSFET on";
    case 11: return "Short circuit protect";
    case 12: return "Cell under voltage protect";
    case 13: return "Pack under voltage protect";
    case 14: return "Reverse protect";
    case 15: return "SOC low alarm";
    case 20: return "Charger connected";
    case 22: return "Discharging";
    case 23: return "Charging";
    case 28: return "Cell low force protect";
    case 40: return "Pack over voltage alarm";
    case 41: return "MOS NTC temperature alarm";
    case 42: return "Environment temperature low alarm";
    case 43: return "Environment temperature high alarm";
    case 44: return "Cell temperature low alarm";
    case 45: return "Cell temperature high alarm";
    case 46: return "Discharge current alarm";
    case 47: return "Charge current alarm";
    case 48: return "Balance temperature alarm";
    case 49: return "Balance temperature protect";
    case 50: return "Discharge MOSFET fault";
    case 51: return "Charge MOSFET fault";
    case 52: return "Current sensor fault";
    case 53: return "AFE fault";
    case 54: return "NTC fault";
    case 55: return "Cell fault";
    case 56: return "Discharge over current 2 protect";
    case 58: return "Pack over voltage protect";
    case 59: return "MOS temperature protect";
    case 60: return "Discharge MOS forced close";
    case 61: return "Charge MOS forced close";
    case 62: return "Environment protect discharging";
    case 63: return "Environment protect charging";
    default: return "Alarm/Status";
  }
}

const uint64_t TB_CRITICAL_ALARM_MASK =
    (1ULL << 0)  | (1ULL << 2)  | (1ULL << 4)  | (1ULL << 5)  |
    (1ULL << 6)  | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) |
    (1ULL << 50) | (1ULL << 51) | (1ULL << 52) | (1ULL << 53) |
    (1ULL << 54) | (1ULL << 55) | (1ULL << 58) | (1ULL << 59);

float tbUnderVoltSanityCap(int cells) {
  if (cells <= 0 || cells > 32) return 57.0f;
  float cap = ((float)cells) * 3.20f;
  if (cap < 40.0f) cap = 40.0f;
  if (cap > 57.0f) cap = 57.0f;
  return cap;
}

bool tbShouldFlagProtoUnderVolt(float pack_v, float module_under_v, int cells) {
  if (module_under_v <= 1.0f) return false;
  if (cells <= 0 || cells > 32) return false;
  float trigger_v = module_under_v + 0.05f;
  float sanity_cap = tbUnderVoltSanityCap(cells);
  if (trigger_v > sanity_cap) return false;
  return pack_v <= trigger_v;
}

float tbOverVoltCellSanityFloor(float cell_high_v) {
  float floor = cell_high_v - 0.12f;
  if (cell_high_v < 3.2f || cell_high_v > 4.5f) floor = 3.58f;
  if (floor < 3.50f) floor = 3.50f;
  if (floor > 4.05f) floor = 4.05f;
  return floor;
}

float tbOverVoltPackSanityFloor(float module_high_v, int cells) {
  float floor = module_high_v - 0.80f;
  if (module_high_v < 30.0f || module_high_v > 70.0f) {
    if (cells > 0 && cells <= 32) floor = ((float)cells) * 3.55f;
    else floor = 50.0f;
  }
  if (floor < 40.0f) floor = 40.0f;
  if (floor > 61.0f) floor = 61.0f;
  return floor;
}

uint64_t tbFilterCriticalAlarmBits(uint64_t status_bits, float pack_v, float max_temp_c, int cells, float temp_limit_c, float max_cell_v, float cell_high_v, float module_high_v) {
  uint64_t bits = status_bits & TB_CRITICAL_ALARM_MASK;
  const uint64_t uv_bits = (1ULL << 12) | (1ULL << 13);
  if (cells <= 0 || cells > 32) bits &= ~uv_bits;
  else if (pack_v > tbUnderVoltSanityCap(cells)) bits &= ~uv_bits;

  const uint64_t ov_bits = (1ULL << 0) | (1ULL << 58);
  float ov_cell_floor = tbOverVoltCellSanityFloor(cell_high_v);
  float ov_pack_floor = tbOverVoltPackSanityFloor(module_high_v, cells);
  if (max_cell_v < ov_cell_floor && pack_v < ov_pack_floor) bits &= ~ov_bits;

  const uint64_t temp_bits = (1ULL << 5) | (1ULL << 6) | (1ULL << 59);
  float temp_guard = temp_limit_c - 3.0f;
  if (temp_guard < 15.0f) temp_guard = 15.0f;
  if (max_temp_c < temp_guard) bits &= ~temp_bits;

  return bits;
}

bool tbHasCriticalAlarm(uint64_t status_bits) {
  return (status_bits & TB_CRITICAL_ALARM_MASK) != 0;
}

String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 16);
  const char *hexDigits = "0123456789ABCDEF";
  for (int i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          out += "\\u00";
          out += hexDigits[(c >> 4) & 0x0F];
          out += hexDigits[c & 0x0F];
        } else {
          out += (char)c;
        }
        break;
    }
  }
  return out;
}

String u64Hex(uint64_t v) {
  char b[19];
  snprintf(b, sizeof(b), "%016llX", (unsigned long long)v);
  return String(b);
}

int tbSelectSocValue(int soc_calc, int soc_raw, bool raw_valid, bool extended_layout) {
  if (soc_calc < 0) soc_calc = 0;
  if (soc_calc > 100) soc_calc = 100;
  if (soc_raw < 0) soc_raw = 0;
  if (soc_raw > 100) soc_raw = 100;

  if (g_soc_source_mode == 0) return soc_calc;
  if (g_soc_source_mode == 1) return raw_valid ? soc_raw : soc_calc;

  if (!raw_valid) return soc_calc;
  if (extended_layout || abs(soc_raw - soc_calc) <= 15) return soc_raw;
  return soc_calc;
}

bool tbParseManufacturerFromFrame(const TBFrame &f, String &hw, String &sw, String &id) {
  if (!f.valid || f.info_len < 24) return false;
  String raw = "";
  raw.reserve(f.info_len + 1);
  for (size_t i = 0; i < f.info_len; i++) raw += (char)f.info[i];
  if (raw.length() < 24) return false;
  hw = raw.substring(0, 20); hw.trim();
  sw = raw.substring(20, 24); sw.trim();
  id = raw.substring(24); id.trim();
  return true;
}

bool tbParseHistoricalFirstFromFrame(const TBFrame &f, TBDate &d, int &event_type, uint64_t &status_bits, float &current_a, float &pack_v, float &remain_ah) {
  if (!f.valid || f.info_len < 24) return false;
  if (f.info_len < 11) return false;

  d.year = ((int)f.info[2] << 8) | (int)f.info[3];
  d.month = f.info[4];
  d.day = f.info[5];
  d.hour = f.info[6];
  d.minute = f.info[7];
  d.second = f.info[8];
  event_type = f.info[9];
  uint8_t status_count = f.info[10];
  status_bits = 0;
  size_t off = 11;
  uint8_t copy_n = status_count > 8 ? 8 : status_count;
  if (off + copy_n <= f.info_len) {
    for (uint8_t i = 0; i < copy_n; i++) status_bits |= ((uint64_t)f.info[off + i]) << (8 * i);
  }
  if (f.info_len >= 24) {
    current_a = ((float)((int16_t)get_u16(f.info, 18))) / 100.0f;
    pack_v = ((float)get_u16(f.info, 20)) / 100.0f;
    remain_ah = ((float)get_u16(f.info, 22)) / 100.0f;
  } else {
    current_a = 0; pack_v = 0; remain_ah = 0;
  }
  return true;
}

void tbBuildAutoCfgSuggestion(const TBSysParam &sp, float buf_cell_mv, float buf_pack_v, float buf_temp_c, float buf_curr_pct,
                              float &safe_cell, float &safe_volt, float &tc_min, float &tc_max, float &td_min, float &td_max,
                              float &chg_a, float &dis_a, float &cvl_v) {
  float buf_cell_v = buf_cell_mv / 1000.0f;
  safe_cell = sp.cell_high_v - buf_cell_v;
  safe_volt = sp.module_high_v - buf_pack_v;

  tc_min = sp.charge_low_t + buf_temp_c;
  tc_max = sp.charge_high_t - buf_temp_c;
  td_min = sp.discharge_low_t + buf_temp_c;
  td_max = sp.discharge_high_t - buf_temp_c;

  // Safety-first: auto profiles never allow charging below +5 C.
  if (tc_min < 5.0f) tc_min = 5.0f;
  if (tc_max > 50.0f) tc_max = 50.0f;
  if (td_min < -20.0f) td_min = -20.0f;
  if (td_max > 60.0f) td_max = 60.0f;

  float scale = 1.0f - (buf_curr_pct / 100.0f);
  if (scale < 0.1f) scale = 0.1f;
  chg_a = sp.charge_current_max_a * scale;
  dis_a = sp.discharge_current_max_a * scale;

  if (safe_cell < 3.2f) safe_cell = 3.2f;
  if (safe_cell > 3.6f) safe_cell = 3.6f;
  if (safe_volt < 40.0f) safe_volt = 40.0f;
  if (safe_volt > 60.0f) safe_volt = 60.0f;

  int est_cells = 0;
  if (sp.cell_high_v > 2.5f) {
    float ratio = sp.module_high_v / sp.cell_high_v;
    if (ratio >= 8.0f && ratio <= 24.0f) est_cells = (int)(ratio + 0.5f);
  }
  if (est_cells > 0) {
    float pack_cap = safe_cell * (float)est_cells;
    if (safe_volt > pack_cap) safe_volt = pack_cap;
  }

  if (tc_min > tc_max - 1.0f) tc_min = tc_max - 1.0f;
  if (td_min > td_max - 1.0f) td_min = td_max - 1.0f;
  if (chg_a < 1.0f) chg_a = 1.0f;
  if (dis_a < 1.0f) dis_a = 1.0f;

  cvl_v = safe_volt - 0.20f;
  if (cvl_v < 48.0f) cvl_v = 48.0f;
  if (cvl_v > safe_volt) cvl_v = safe_volt;
}
bool tbSystemParamPlausible(const TBSysParam &sp) {
  if (sp.cell_high_v < 3.0f || sp.cell_high_v > 4.5f) return false;
  if (sp.cell_low_v < 2.0f || sp.cell_low_v > 3.6f) return false;
  if (sp.cell_under_v < 1.8f || sp.cell_under_v > 3.5f) return false;
  if (sp.module_high_v < 40.0f || sp.module_high_v > 65.0f) return false;
  if (sp.module_under_v < 20.0f || sp.module_under_v > 60.0f) return false;
  if (sp.module_under_v >= sp.module_high_v) return false;
  if (sp.charge_current_max_a < 1.0f || sp.charge_current_max_a > 500.0f) return false;
  if (sp.discharge_current_max_a < 1.0f || sp.discharge_current_max_a > 500.0f) return false;
  if (sp.charge_low_t < -40.0f || sp.charge_low_t > 40.0f) return false;
  if (sp.charge_high_t < 20.0f || sp.charge_high_t > 100.0f) return false;
  if (sp.discharge_low_t < -40.0f || sp.discharge_low_t > 40.0f) return false;
  if (sp.discharge_high_t < 20.0f || sp.discharge_high_t > 100.0f) return false;
  return true;
}

bool pollBmsAlarmStatus(int bms_id) {
  if (bms_id < 0 || bms_id >= MAX_BMS) return false;
  uint8_t req[1] = {(uint8_t)bms_id};
  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)bms_id, 0x44, req, 1, fr, err)) return false;
  TBAlarmInfoParsed a;
  if (!tbParseAlarmInfoFromFrame(fr, a)) return false;
  g_bms_alarm_bits[bms_id] = a.status_bits;
  g_bms_alarm_status_count[bms_id] = a.status_count;
  g_bms_alarm_last_seen[bms_id] = millis();
  return true;
}

bool pollBmsSystemParameter(int bms_id) {
  if (bms_id < 0 || bms_id >= MAX_BMS) return false;
  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)bms_id, 0x47, nullptr, 0, fr, err)) return false;
  TBSysParam sp;
  if (!tbParseSystemParameterFromFrame(fr, sp)) return false;
  if (!tbSystemParamPlausible(sp)) {
    addToLog("Spike Filter: SystemParam out of range BMS" + String(bms_id), true);
    return false;
  }
  bool prev_fresh = (g_bms_sys_last_seen[bms_id] > 0) && (millis() - g_bms_sys_last_seen[bms_id] < 300000);
  if (prev_fresh) {
    if (fabs(sp.module_high_v - g_bms_sys_module_high_v[bms_id]) > 5.0f) {
      addToLog("Spike Filter: module_high jump BMS" + String(bms_id), true);
      return false;
    }
    if (fabs(sp.charge_current_max_a - g_bms_sys_charge_max_a[bms_id]) > 100.0f) {
      addToLog("Spike Filter: charge_max jump BMS" + String(bms_id), true);
      return false;
    }
    if (fabs(sp.discharge_current_max_a - g_bms_sys_discharge_max_a[bms_id]) > 100.0f) {
      addToLog("Spike Filter: discharge_max jump BMS" + String(bms_id), true);
      return false;
    }
  }
  g_bms_sys_module_high_v[bms_id] = sp.module_high_v;
  g_bms_sys_module_under_v[bms_id] = sp.module_under_v;
  g_bms_sys_charge_max_a[bms_id] = sp.charge_current_max_a;
  g_bms_sys_discharge_max_a[bms_id] = sp.discharge_current_max_a;
  g_bms_sys_cell_high_v[bms_id] = sp.cell_high_v;
  g_bms_sys_charge_low_t[bms_id] = sp.charge_low_t;
  g_bms_sys_charge_high_t[bms_id] = sp.charge_high_t;
  g_bms_sys_discharge_low_t[bms_id] = sp.discharge_low_t;
  g_bms_sys_discharge_high_t[bms_id] = sp.discharge_high_t;
  g_bms_sys_last_seen[bms_id] = millis();
  return true;
}

// ==========================================
// LOGGING, AUTH, LED STATUS, AND LOGIN RATE LIMITING
// ==========================================
// V2.65 M3: Ring-buffer implementation replaces String debug_log.
// Writes are O(1), bounded heap, no per-call String concatenation.
// Serial output (if error or debug flag) is rebuilt from msg; original
// behaviour preserved.
void addToLog(String msg, bool error) {
  #ifdef MODE_SMART_WIFI
    // Pre-mutex fallback: if called before setup() creates the mutex,
    // fall back to Serial only. Prevents early-boot crash.
    if (!logMutex) {
      if (error || g_serial_debug) Serial.println(msg);
      return;
    }
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(50))) {
      LogEntry &e = g_log_ring[g_log_head];
      time_t now_t; time(&now_t);
      // If NTP hasn't synced (epoch < Jan 2023), store millis/1000 as fallback.
      // Frontend detects small values and renders as uptime.
      e.ts = (now_t > 1672531200) ? (uint32_t)now_t : (uint32_t)(millis() / 1000);
      e.flags = error ? 1 : 0;
      strncpy(e.msg, msg.c_str(), LOG_MSG_MAX - 1);
      e.msg[LOG_MSG_MAX - 1] = '\0';
      g_log_head = (g_log_head + 1) % LOG_RING_SIZE;
      if (g_log_count < LOG_RING_SIZE) g_log_count++;
      xSemaphoreGive(logMutex);
    }
    if (error || g_serial_debug) {
      String tm = getTimeStr();
      Serial.println("[" + tm + "] " + msg);
    }
  #else
    if(error) Serial.println(msg);
  #endif
}

String getTimeStr() {
  struct tm t; if (!getLocalTime(&t)) return String(millis() / 1000) + "s";
  char b[20]; strftime(b, sizeof(b), "%H:%M:%S", &t); return String(b);
}

String sha256Hash(const String &input) {
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)input.c_str(), input.length());
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);
  String hex; hex.reserve(64);
  for (int i = 0; i < 32; i++) hex += toHex2(hash[i]);
  return hex;
}

// Cookie-based session authentication (replaces browser Basic Auth popup)
// Session token is generated at boot, stored in RAM only. Reboot = new session.
#ifdef MODE_SMART_WIFI
String g_session_token = "";  // V2.64: Persisted in NVS, stable across reboots. Rotates on password change or explicit logout-all.
uint32_t g_session_ts = 0;    // V2.66 H2: Unix epoch (or millis/1000 pre-NTP) when token was created. Used for server-side age check.
#define SESSION_MAX_AGE_SEC (30UL * 24UL * 60UL * 60UL)   // V2.66 H2: 30-day session cap (matches cookie Max-Age)

void generateSessionToken() {
  g_session_token = "";
  for (int i = 0; i < 16; i++) g_session_token += toHex2((uint8_t)esp_random());
}

// V2.64: Load token from NVS on boot. If missing or wrong length, generate and persist a new one.
// Users stay logged in across reboots (previously every reboot invalidated all sessions).
// Capture token creation timestamp. If NTP hasn't synced yet, use
// millis()/1000 as a monotonic fallback. First NTP sync re-stamps.
static uint32_t currentSessionTs() {
  time_t now_t; time(&now_t);
  return (now_t > 1672531200) ? (uint32_t)now_t : (uint32_t)(millis() / 1000);
}

void loadOrCreateSessionToken() {
  Preferences p;
  p.begin("gateway", false);
  String saved = p.getString("sess_tok", "");
  if (saved.length() == 32) {
    g_session_token = saved;
    // V2.66 H2: sess_ts may not exist on upgrade from V2.64/V2.65 - default 0
    // means "no known age", which triggers a one-time rotation below.
    g_session_ts = p.getUInt("sess_ts", 0);
    if (g_session_ts == 0) {
      g_session_ts = currentSessionTs();
      p.putUInt("sess_ts", g_session_ts);
    }
  } else {
    generateSessionToken();
    g_session_ts = currentSessionTs();
    p.putString("sess_tok", g_session_token);
    p.putUInt("sess_ts", g_session_ts);
  }
  p.end();
}

// V2.64: Rotate token (called on password change or logout-all). Invalidates all existing sessions.
// V2.66 H2: Also stamps a fresh creation timestamp.
void rotateSessionToken() {
  generateSessionToken();
  g_session_ts = currentSessionTs();
  Preferences p;
  p.begin("gateway", false);
  p.putString("sess_tok", g_session_token);
  p.putUInt("sess_ts", g_session_ts);
  p.end();
}

String getSessionCookie() {
  if (!server.hasHeader("Cookie")) return "";
  String cookies = server.header("Cookie");
  int pos = cookies.indexOf("tbsid=");
  if (pos < 0) return "";
  int start = pos + 6;
  int end = cookies.indexOf(';', start);
  if (end < 0) end = cookies.length();
  return cookies.substring(start, end);
}

bool checkAuth(bool jsonMode = false) {
  if (!g_auth_enable || g_auth_hash.length() == 0) return true;
  // Check session cookie
  if (getSessionCookie() == g_session_token && g_session_token.length() > 0) {
    // V2.66 H2: Enforce 30-day session age server-side. If NTP has synced
    // since token creation but sess_ts is still a pre-NTP fallback (tiny
    // value relative to current epoch), restamp silently so the session
    // gets a proper 30-day window anchored to real wall-clock time.
    uint32_t now_ts = currentSessionTs();
    if (g_session_ts < 1672531200 && now_ts > 1672531200) {
      // Pre-NTP fallback met its first NTP; anchor the session now.
      g_session_ts = now_ts;
      Preferences p;
      p.begin("gateway", false);
      p.putUInt("sess_ts", g_session_ts);
      p.end();
      addToLog("Auth: session timestamp re-anchored after NTP sync", false);
      return true;
    }
    if (now_ts > g_session_ts && (now_ts - g_session_ts) > SESSION_MAX_AGE_SEC) {
      // Session too old. Rotate token so the stored cookie stops working.
      addToLog("Auth: session expired (age > 30 days), token rotated", true);
      rotateSessionToken();
      // Fall through to 401
    } else {
      return true;
    }
  }
  // No valid session
  if (jsonMode) {
    server.send(401, "application/json", "{\"auth\":false}");
  } else {
    server.sendHeader("Location", "/login");
    server.send(302, "text/plain", "Redirecting to login");
  }
  return false;
}

void handleLogin() {
  // Rate limiting: max 5 failed attempts, then 60s lockout
  static uint8_t login_fails = 0;
  static unsigned long lockout_until = 0;
  if (millis() < lockout_until) {
    server.send(429, "text/plain", "Too many attempts. Try again later.");
    return;
  }
  if (server.method() == HTTP_POST) {
    // Validate credentials
    String user = server.arg("user");
    String pass = server.arg("pass");
    if (user == g_auth_user && sha256Hash(pass) == g_auth_hash) {
      // Success: set session cookie and redirect to dashboard
      login_fails = 0;
      // V2.64: Max-Age=2592000 (30 days) keeps cookie across browser restarts.
      server.sendHeader("Set-Cookie", "tbsid=" + g_session_token + "; Path=/; HttpOnly; SameSite=Strict; Max-Age=2592000");
      server.sendHeader("Location", "/");
      server.send(302, "text/plain", "Login OK");
      addToLog("Auth: login success (" + user + ")", false);
      return;
    }
    login_fails++;
    addToLog("Auth: login failed (" + user + "), attempt " + String(login_fails), true);
    if (login_fails >= 5) {
      lockout_until = millis() + 60000;
      addToLog("Auth: lockout active for 60s after " + String(login_fails) + " failures", true);
      login_fails = 0;
    }
  }
  // Serve login form (GET, or failed POST)
  bool failed = (server.method() == HTTP_POST);
  String h = R"LOGIN(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Login</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--red:#ef4444;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:32px;max-width:340px;width:90%;box-shadow:0 4px 12px rgba(0,0,0,0.1)}
h2{font-size:16px;font-weight:500;margin-bottom:4px}
.s{color:var(--tx2);font-size:12px;margin-bottom:20px}
.e{color:var(--red);font-size:12px;margin-bottom:12px}
label{display:block;font-size:11px;color:var(--tx2);margin-bottom:3px}
input{width:100%;padding:9px 10px;border:1px solid rgba(128,128,128,0.3);border-radius:8px;background:var(--bg);color:var(--tx);font-size:13px;margin-bottom:12px;outline:none}
input:focus{border-color:var(--acc)}
button{width:100%;padding:10px;background:var(--acc);color:#fff;border:none;border-radius:8px;font-size:13px;font-weight:500;cursor:pointer}
button:hover{opacity:0.9}
</style></head><body><div class="b"><h2>TopBand BMS Gateway</h2><div class="s">Sign in to continue</div>)LOGIN";
  if (failed) h += "<div class=\"e\">Invalid username or password</div>";
  h += R"LOGIN2(<form method="POST" action="/login">
<label>Username</label><input type="text" name="user" autocomplete="username" autofocus required>
<label>Password</label><input type="password" name="pass" autocomplete="current-password" required>
<button type="submit">Sign in</button></form>
</div></body></html>)LOGIN2";
  server.send(200, "text/html", h);
}

void handleLogout() {
  server.sendHeader("Set-Cookie", "tbsid=; Path=/; Max-Age=0");
  server.sendHeader("Location", "/login");
  server.send(302, "text/plain", "Logged out");
}
#endif

void handleLedStatus() {
  unsigned long m = millis();
  #ifdef MODE_SMART_WIFI
  if(WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) { 
      int val = (int)((sin(m / 300.0) + 1.0) * 60.0); setLed(val, 0, val); return; 
  }
  #endif
  if (victronData.activePacks == 0) {
      int val = (int)((sin(m / 200.0) + 1.0) * 100.0); setLed(0, 0, val); 
  } else if( (alarm_flags & 0x02) || (alarm_flags & 0x08) || (alarm_flags & 0x10) || (alarm_flags & 0x40) ) {
      bool on = (m / 80) % 2; setLed(on ? 255 : 0, 0, 0); 
  } else if (alarm_flags & 0x20) {
      bool on = (m / 500) % 2; setLed(on ? 255 : 0, on ? 160 : 0, 0); 
  } else {
      int val = (int)((sin(m / 1500.0) + 1.0) * 60.0); setLed(0, val, 0); 
  }
}

// --- AP MODE CALLBACK ---
#ifdef MODE_SMART_WIFI
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("AP Mode Active");
  setLed(255, 0, 0); // Red = AP mode
}
#endif

// ==========================================
// BMS DATA PARSER (called on Core 0 after each RS485 response)
// Decodes TopBand analog data frame: cell voltages, temperatures,
// pack voltage/current/SOC/SOH. Includes spike filter to reject
// implausible jumps (>5V voltage, >250A current, >20% SOC).
// Updates per-BMS communication stats (ok/errors/spikes).
// ==========================================
void parseTopband(String raw, int addr) {
  // V2.65.1 D2: Spy capture fires BEFORE any early-return so that truncated
  // / malformed frames are also captured for diagnosis.
  if (g_rs485_spy_enable && addr >= 0 && addr < MAX_BMS) {
    spyCaptureFrame(raw, (uint8_t)addr);
  }
  #ifdef MODE_SMART_WIFI
    if(g_serial_debug) { Serial.print("RX BMS "); Serial.print(addr); Serial.print(": "); Serial.println(raw); }
    #if HAS_SD_CARD
    if(g_sd_spy && sd_ok) {
        File f = SD.open("/spy.txt", FILE_APPEND);
        if(f) { f.printf("[%s] BMS%d: %s\n", getTimeStr().c_str(), addr, raw.c_str()); f.close(); }
    }
    #endif
  #endif
  if (raw.length() < 20) { addToLog("Err: Short Frame", true); if(addr>=0&&addr<MAX_BMS)g_bms_stats[addr].errors++; return; }
  stat_rx_count++; uint8_t b[512]; int blen = 0;
  for (int i = 0; i < raw.length() - 1; i += 2) { if (blen >= 511) break; b[blen++] = (parse_hex(raw[i]) << 4) | parse_hex(raw[i + 1]); }
  int idx = -1; for (int i = 0; i < blen - 1; i++) { if (b[i] == 0xD0 && b[i + 1] == 0x7C) { idx = i; break; } }
  if (idx == -1) return;
  int p = idx + 2; p++; int cells = b[p++];
  if (g_force_cell_count > 0) cells = g_force_cell_count;
  if (cells > 32) cells = 32;
  if (cells < 0) cells = 0;
  g_detected_cells = cells;

  // V2.64: Build into local BMSData, then single memcpy under dataMutex.
  // Read previous state for spike filter under mutex, write final state under mutex.
  BMSData local = {};
  float vSum = 0, minV = 99, maxV = 0; int minI = 0, maxI = 0;
  for (int i = 0; i < cells; i++) {
    if (p + 1 >= blen) return;
    float v = get_u16(b, p) / 1000.0; local.cells[i] = v;
    if (v > 0.1) { vSum += v; if (v < minV) { minV = v; minI = i + 1; } if (v > maxV) { maxV = v; maxI = i + 1; } }
    p += 2;
  }
  local.cell_count = cells; local.minCellV = minV; local.maxCellV = maxV;
  local.minCellIdx = minI; local.maxCellIdx = maxI; local.avgCellV = (cells > 0) ? vSum / cells : 0;
  if (p < blen) {
    int t_cnt = b[p++]; if (t_cnt > 8) t_cnt = 8; local.temp_count = t_cnt;
    float max_t = -99; for (int i = 0; i < t_cnt; i++) {
      if (p + 1 >= blen) break; float t = (get_u16(b, p) - 2731) / 10.0;
      if (t < -50 || t > 150) t = 25.0; local.temps[i] = t; if (t > max_t) max_t = t; p += 2;
    }
    local.maxTemp = max_t; local.avgTemp = max_t;
  }
  if (p + 10 < blen) {
    float cur = get_s16(b, p) * 0.01; p += 2; float volt = get_u16(b, p) * 0.01; p += 2; float rem = get_u16(b, p) * 0.01; p += 3; float full = get_u16(b, p) * 0.01; p += 2;
    float v_min = (cells > 0) ? (cells * 2.0f) : 20.0f;
    float v_max = (cells > 0) ? (cells * 4.05f) : 65.0f;
    if (volt < v_min || volt > v_max || abs(cur) > 500 || rem < 0 || full < 0 || full > 2000) { addToLog("Bad Main Data", true); g_bms_stats[addr].errors++; return; }
    int new_soc_calc = (full > 0) ? (int)((rem / full) * 100.0) : 0; if (new_soc_calc > 100) new_soc_calc = 100; if (new_soc_calc < 0) new_soc_calc = 0;
    int new_soc = new_soc_calc;
    int new_soh = 100;

    // Newer payload variant: cycle(2B), SOC(1B), SOH(1B) after full_ah.
    int meta = p;
    uint16_t new_cycles = 0;
    if (meta + 3 < blen) {
      new_cycles = get_u16(b, meta);
      uint8_t raw_soc = b[meta + 2];
      uint8_t raw_soh = b[meta + 3];
      bool extended_layout = (meta + 7 < blen);
      bool raw_valid = (raw_soc <= 100) && (raw_soh <= 100);
      new_soc = tbSelectSocValue(new_soc_calc, (int)raw_soc, raw_valid, extended_layout);
      if (raw_valid && (extended_layout || g_soc_source_mode == 1 || abs((int)raw_soc - new_soc_calc) <= 15)) {
        new_soh = (int)raw_soh;
      } else {
        uint16_t s16 = get_u16(b, meta + 2);
        if (s16 <= 100) new_soh = (int)s16;
      }
    }

    // V2.64: Spike filter reads previous state under mutex.
    // V2.66 H1: also read last_seen so hold-last-value can check staleness.
    // V2.66.1 C2: also capture current + validity independent of the 15s spike
    //   window, so H1 hold logic does not re-read bms[addr] outside the mutex.
    float prev_volt = 0, prev_cur = 0; int prev_soc = 0;
    unsigned long prev_seen = 0;
    bool prev_valid = false;
    float prev_cur_for_hold = 0;       // V2.66.1 C2
    bool  prev_valid_for_hold = false; // V2.66.1 C2
    if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
      prev_valid = bms[addr].valid && (millis() - bms[addr].last_seen < 15000);
      if (prev_valid) {
        prev_volt = bms[addr].voltage;
        prev_cur  = bms[addr].current;
        prev_soc  = bms[addr].soc;
      }
      // Capture last_seen even if spike-filter prev_valid window expired,
      // so hold logic can use a wider 120s window.
      prev_seen = bms[addr].last_seen;
      // V2.66.1 C2: capture current + validity for H1 hold window (wider 120s)
      prev_valid_for_hold = bms[addr].valid;
      prev_cur_for_hold = bms[addr].current;
      xSemaphoreGive(dataMutex);
    }
    if (prev_valid) {
      if (fabs(volt - prev_volt) > g_spike_volt) { addToLog("Spike Filter: Voltage jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
      if (fabs(cur - prev_cur)   > g_spike_cur)  { addToLog("Spike Filter: Current jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
      if (abs(new_soc - prev_soc) > g_spike_soc) { addToLog("Spike Filter: SOC jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
    }

    // V2.66 H1: BMS current hold-last-value.
    // TopBand BMS firmware only populates the current field every ~90 seconds.
    // Between samples it reports 0.00 A regardless of actual battery activity.
    // Detect that pattern and preserve the previous reading if it's fresh enough.
    // V2.66.1 C2: use locals captured under mutex above, no cross-mutex re-reads.
    if (fabs(cur) < 0.01f) {
      // BMS reports zero. Check if we have a fresh non-zero value to hold.
      bool have_prev_nonzero = prev_valid_for_hold && (fabs(prev_cur_for_hold) > 0.01f);
      bool prev_fresh = (prev_seen > 0) && (millis() - prev_seen < 120000UL);
      if (have_prev_nonzero && prev_fresh) {
        cur = prev_cur_for_hold;                     // hold the last real reading
        g_bms_stats[addr].current_holds++;
        // Log the first hold per BMS since boot so operator knows the mechanism is active.
        static bool hold_logged[MAX_BMS] = {false};
        if (addr >= 0 && addr < MAX_BMS && !hold_logged[addr]) {
          hold_logged[addr] = true;
          addToLog("BMS" + String(addr) + ": current hold-last-value active (V2.66 H1)", false);
        }
      }
    }

    local.current = cur; local.voltage = volt; local.rem_ah = rem; local.full_ah = full;
    local.soc = new_soc;
    local.soh = new_soh;
    local.cycles = new_cycles;
  }
  local.valid = true; local.last_seen = millis();

  // V2.64: Commit snapshot to shared state atomically under dataMutex.
  // Single memcpy ~350 bytes = ~1 microsecond. Core 1 readers see consistent state.
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
    memcpy(&bms[addr], &local, sizeof(BMSData));
    xSemaphoreGive(dataMutex);
    g_bms_stats[addr].ok++;
  } else {
    addToLog("WARN: parseTopband mutex timeout BMS" + String(addr), true);
    g_bms_stats[addr].errors++;
  }
}

float calcFactor(float t, float min, float max) {
    if (t < min) return 0.0; if (t > max) return 0.0;
    float soft_zone = 5.0; 
    if (t < (min + soft_zone)) return 0.2; 
    if (t > (max - soft_zone)) return 0.5; 
    return 1.0; 
}

float getEffectiveChargeMinTemp() {
  if (g_expert_mode) return g_tc_min;
  return (g_tc_min < 5.0f) ? 5.0f : g_tc_min;
}

float getEffectiveDischargeMinTemp() {
  if (g_expert_mode) return g_td_min;
  return (g_td_min < -20.0f) ? -20.0f : g_td_min;
}
void enforceCellBasedSafetyCaps() {
  int cells = g_force_cell_count > 0 ? g_force_cell_count : g_detected_cells;
  if (cells <= 0 || cells > 32) return;

  float cap_cvl = ((float)cells) * 3.50f;
  float cap_safe = ((float)cells) * 3.55f;
  float cap_cell = 3.55f;

  if (g_cvl_voltage > cap_cvl) g_cvl_voltage = cap_cvl;
  if (g_safe_volt > cap_safe) g_safe_volt = cap_safe;
  if (g_safe_cell > cap_cell) g_safe_cell = cap_cell;
  if (g_maint_target_v > g_safe_volt) g_maint_target_v = g_safe_volt;
}

void saveAutoBalanceTimestamp(uint32_t ts) {
  preferences.begin("gateway", false);
  preferences.putULong("ab_last", ts);
  preferences.end();
}

// ==========================================
// SAFETY LOGIC & VICTRON DATA AGGREGATION (runs on Core 0 every 500ms)
// Aggregates all BMS packs into victronData struct for CAN output.
// Applies temperature throttling (soft zones), protocol-level current caps
// from BMS 0x47, overvolt/undervolt cutoffs, and alarm flag management.
// State-transition logging: safety events and throttle changes are logged
// only when they change, not on every cycle.
// Alarm flags bitmask: 0x02=overvolt, 0x08=temp stop, 0x10=undervolt,
//   0x20=cell imbalance, 0x40=BMS status alarm, 0x80=no packs online
// ==========================================
void calculateVictronData() {
  float sumI = 0, sumV = 0, sumSOC = 0, sumSOH = 0, sumT = 0, sumCap = 0, sumRem = 0;
  float proto_ccl_cap = 0, proto_dcl_cap = 0, proto_cvl_cap = 999.0f;
  bool proto_under_v_hit = false;
  int first_proto_idx = -1;
  int proto_count = 0;
  int count = 0; alarm_flags = 0; sys_error_msg = "OK";
  #ifdef MODE_SIMPLE_CABLE
    if(g_detected_cells > 0) { g_cvl_voltage = g_detected_cells * 3.5; g_safe_volt = g_detected_cells * 3.65; }
  #endif
  enforceCellBasedSafetyCaps();

  float t_check_val = 25.0; 
  for (int i = 0; i < g_bms_count; i++) {
    bool on = bms[i].valid && (millis() - bms[i].last_seen < 120000);
    if (simulation_active) on = true;
    // Detect BMS going offline
    if (g_bms_was_online[i] && !on) {
      addToLog("ALARM: BMS" + String(i) + " went offline", true);
      if (sys_error_msg == "OK") sys_error_msg = "ALARM: BMS" + String(i) + " OFFLINE";
    }
    g_bms_was_online[i] = on;
    if (on) {
      bool alarm_fresh = (g_bms_alarm_last_seen[i] > 0) && (millis() - g_bms_alarm_last_seen[i] < 60000);
      if (alarm_fresh) {
        int cells_for_filter = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        float temp_limit = (g_tc_max < g_td_max) ? g_tc_max : g_td_max;
        uint64_t crit_bits = tbFilterCriticalAlarmBits(
            g_bms_alarm_bits[i],
            bms[i].voltage,
            bms[i].maxTemp,
            cells_for_filter,
            temp_limit,
            bms[i].maxCellV,
            g_bms_sys_cell_high_v[i],
            g_bms_sys_module_high_v[i]);
        if (crit_bits != 0) {
          const uint64_t uv_alarm_bits = (1ULL << 12) | (1ULL << 13);
          if (crit_bits & uv_alarm_bits) {
            alarm_flags |= 0x10;
            if (sys_error_msg == "OK") sys_error_msg = "ALARM: PACK UNDERVOLT";
          } else {
            alarm_flags |= 0x40;
            if (sys_error_msg == "OK") sys_error_msg = "ALARM: BMS STATUS";
          }
        }
      }
      bool sp_fresh = (g_bms_sys_last_seen[i] > 0) && (millis() - g_bms_sys_last_seen[i] < 300000);
      if (sp_fresh) {
        if (first_proto_idx < 0) first_proto_idx = i;
        if (g_bms_sys_charge_max_a[i] > 0.1f) proto_ccl_cap += g_bms_sys_charge_max_a[i];
        if (g_bms_sys_discharge_max_a[i] > 0.1f) proto_dcl_cap += g_bms_sys_discharge_max_a[i];
        int cells_for_cap = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        float cvl_cap = g_bms_sys_module_high_v[i] - 0.20f;
        if (cells_for_cap > 0 && cells_for_cap <= 32) {
          float chem_cap = ((float)cells_for_cap) * 3.60f;
          if (cvl_cap > chem_cap) cvl_cap = chem_cap;
        }
        if (g_safe_volt > 40.0f && cvl_cap > (g_safe_volt - 0.20f)) cvl_cap = g_safe_volt - 0.20f;
        if (g_cvl_voltage > 40.0f && cvl_cap > g_cvl_voltage) cvl_cap = g_cvl_voltage;
        if (cvl_cap > 40.0f && cvl_cap < proto_cvl_cap) proto_cvl_cap = cvl_cap;
        int cells_for_uv = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        if (tbShouldFlagProtoUnderVolt(bms[i].voltage, g_bms_sys_module_under_v[i], cells_for_uv)) proto_under_v_hit = true;
        proto_count++;
      }
      if(bms[i].voltage > g_safe_volt) { alarm_flags |= 0x02; sys_error_msg = "ALARM: PACK OVERVOLT"; }
      if(bms[i].maxCellV > g_safe_cell) { alarm_flags |= 0x02; sys_error_msg = "ALARM: CELL OVERVOLT"; }
      if(g_temp_mode == 0) t_check_val = bms[i].maxTemp; else t_check_val = bms[i].avgTemp; 
      if((bms[i].maxCellV - bms[i].minCellV) > g_safe_drift) { alarm_flags |= 0x20; if(sys_error_msg == "OK") sys_error_msg = "WARN: CELL IMBALANCE"; }
      sumI += bms[i].current; sumV += bms[i].voltage; sumSOC += bms[i].soc; sumSOH += bms[i].soh; sumT += bms[i].avgTemp; sumCap += bms[i].full_ah; sumRem += bms[i].rem_ah; count++;
    }
  }
  
  if (count > 0) {
    if (g_easy_mode && !g_expert_mode && !g_easy_auto_applied && first_proto_idx >= 0) {
      TBSysParam sp;
      sp.cell_high_v = g_bms_sys_cell_high_v[first_proto_idx];
      sp.module_high_v = g_bms_sys_module_high_v[first_proto_idx];
      sp.charge_low_t = g_bms_sys_charge_low_t[first_proto_idx];
      sp.charge_high_t = g_bms_sys_charge_high_t[first_proto_idx];
      sp.discharge_low_t = g_bms_sys_discharge_low_t[first_proto_idx];
      sp.discharge_high_t = g_bms_sys_discharge_high_t[first_proto_idx];
      sp.charge_current_max_a = g_bms_sys_charge_max_a[first_proto_idx];
      sp.discharge_current_max_a = g_bms_sys_discharge_max_a[first_proto_idx];

      float n_safe_cell, n_safe_volt, n_tc_min, n_tc_max, n_td_min, n_td_max, n_chg_a, n_dis_a, n_cvl_v;
      tbBuildAutoCfgSuggestion(sp, 20.0f, 0.30f, 2.0f, 10.0f,
                               n_safe_cell, n_safe_volt, n_tc_min, n_tc_max, n_td_min, n_td_max, n_chg_a, n_dis_a, n_cvl_v);
      g_safe_cell = n_safe_cell;
      g_safe_volt = n_safe_volt;
      g_tc_min = n_tc_min;
      g_tc_max = n_tc_max;
      g_td_min = n_td_min;
      g_td_max = n_td_max;
      g_charge_amps = n_chg_a;
      g_discharge_amps = n_dis_a;
      g_cvl_voltage = n_cvl_v;
      g_easy_auto_applied = true;
      addToLog("Easy Mode: Auto config applied from BMS system params", false);
    }

    victronData.activePacks = count; victronData.totalCurrent = sumI; victronData.avgVoltage = sumV / count;
    victronData.totalPower = sumI * victronData.avgVoltage; victronData.avgSOC = sumSOC / count;
    victronData.avgSOH = sumSOH / count; victronData.avgTemp = sumT / count; victronData.totalCapacity = sumCap; victronData.remainCapacity = sumRem;
    
    float eff_tc_min = getEffectiveChargeMinTemp();
    float eff_td_min = getEffectiveDischargeMinTemp();
    factor_charge = calcFactor(t_check_val, eff_tc_min, g_tc_max);
    factor_discharge = calcFactor(t_check_val, eff_td_min, g_td_max);

    float safe_chg = count * g_charge_amps * factor_charge;
    float safe_dis = count * g_discharge_amps * factor_discharge;

    if (proto_count > 0) {
      if (proto_ccl_cap > 0.1f && safe_chg > proto_ccl_cap) safe_chg = proto_ccl_cap;
      if (proto_dcl_cap > 0.1f && safe_dis > proto_dcl_cap) safe_dis = proto_dcl_cap;
      if (proto_cvl_cap < 900.0f) g_runtime_cvl = proto_cvl_cap;
      else g_runtime_cvl = g_cvl_voltage;
      g_runtime_proto_ccl_cap = proto_ccl_cap;
      g_runtime_proto_dcl_cap = proto_dcl_cap;
    } else {
      g_runtime_cvl = g_cvl_voltage;
      g_runtime_proto_ccl_cap = 0.0f;
      g_runtime_proto_dcl_cap = 0.0f;
    }

    if(factor_charge == 0.0) { alarm_flags |= 0x08; sys_error_msg = "INFO: TEMP CHARGE STOP"; }
    if(factor_discharge == 0.0) { alarm_flags |= 0x08; sys_error_msg = "INFO: TEMP DISCHG STOP"; }
    g_runtime_proto_uv_hit = proto_under_v_hit;
    if(proto_under_v_hit) { alarm_flags |= 0x10; safe_dis = 0; if(sys_error_msg == "OK") sys_error_msg = "ALARM: PACK UNDERVOLT"; }
    if( (alarm_flags & 0x02) || (alarm_flags & 0x10) || (alarm_flags & 0x40) ) { safe_chg = 0; safe_dis = 0; } 
    // State-transition logging (only log on change to avoid spam)
    static uint8_t prev_alarm_flags = 0;
    static float prev_factor_charge = 1.0f, prev_factor_discharge = 1.0f;
    if ((alarm_flags & 0x02) && !(prev_alarm_flags & 0x02)) addToLog("SAFETY: Overvoltage cutoff triggered", true);
    if ((alarm_flags & 0x10) && !(prev_alarm_flags & 0x10)) addToLog("SAFETY: Undervoltage cutoff triggered", true);
    if ((alarm_flags & 0x08) && !(prev_alarm_flags & 0x08)) addToLog("SAFETY: Temperature cutoff active", true);
    if ((alarm_flags & 0x20) && !(prev_alarm_flags & 0x20)) addToLog("SAFETY: Cell imbalance detected", true);
    if ((alarm_flags & 0x40) && !(prev_alarm_flags & 0x40)) addToLog("SAFETY: BMS alarm status active", true);
    if (factor_charge < 1.0f && prev_factor_charge >= 1.0f) addToLog("THROTTLE: Charge current reduced to " + String((int)(factor_charge * 100)) + "% (temp)", false);
    if (factor_discharge < 1.0f && prev_factor_discharge >= 1.0f) addToLog("THROTTLE: Discharge current reduced to " + String((int)(factor_discharge * 100)) + "% (temp)", false);
    if (factor_charge >= 1.0f && prev_factor_charge < 1.0f) addToLog("THROTTLE: Charge current restored to 100%", false);
    if (factor_discharge >= 1.0f && prev_factor_discharge < 1.0f) addToLog("THROTTLE: Discharge current restored to 100%", false);
    prev_alarm_flags = alarm_flags; prev_factor_charge = factor_charge; prev_factor_discharge = factor_discharge; 
    if (!g_maint_charge_mode) {
      if (victronData.avgSOC >= 99) safe_chg = count * 2.0;
      if (victronData.avgSOC == 100) safe_chg = 0.0;
    } else {
      float maint_target = g_maint_target_v;
      if (maint_target < 40.0f) maint_target = 40.0f;
      if (maint_target > g_safe_volt) maint_target = g_safe_volt;
      if (victronData.avgVoltage >= maint_target && safe_chg > count * 2.0f) safe_chg = count * 2.0f;
    }

    // Optional auto-balance watchdog: if target was not reached for 30 days, keep a small charge allowance.
    g_runtime_auto_balance_due = false;
    if (g_auto_balance_enable) {
      time_t t_now;
      time(&t_now);
      if (t_now > 1700000000) {
        uint32_t now_ts = (uint32_t)t_now;
        float bal_target = g_maint_target_v;
        if (bal_target < 40.0f) bal_target = 40.0f;
        if (bal_target > g_safe_volt) bal_target = g_safe_volt;
        bool reached_now = victronData.avgVoltage >= bal_target;
        if (reached_now && !g_runtime_balance_reached_prev) {
          g_auto_balance_last_ts = now_ts;
          saveAutoBalanceTimestamp(g_auto_balance_last_ts);
        }
        g_runtime_balance_reached_prev = reached_now;

        if (!reached_now && !g_maint_charge_mode) {
          const uint32_t window_s = 30UL * 24UL * 3600UL;
          if (g_auto_balance_last_ts == 0 || (now_ts > g_auto_balance_last_ts && (now_ts - g_auto_balance_last_ts) > window_s)) {
            g_runtime_auto_balance_due = true;
            if (!(alarm_flags & 0x02) && !(alarm_flags & 0x10) && factor_charge > 0.0f) {
              float hold_chg = count * 2.0f;
              if (safe_chg < hold_chg) safe_chg = hold_chg;
              if (sys_error_msg == "OK") sys_error_msg = "INFO: AUTO BALANCE ACTIVE";
            }
          }
        }
      }
    }
    
    victronData.maxChargeCurrent = safe_chg; victronData.maxDischargeCurrent = safe_dis;
  } else { victronData.activePacks = 0; victronData.totalCurrent = 0; victronData.totalPower = 0; victronData.maxChargeCurrent = 0; victronData.maxDischargeCurrent = 0; g_runtime_proto_uv_hit = false; g_runtime_auto_balance_due = false; g_runtime_balance_reached_prev = false; alarm_flags |= 0x80; }
}

// ==========================================
// CAN BUS OUTPUT (Victron/Pylontech/SMA protocol)
// Sends battery data to inverter: 0x351=limits, 0x355=SOC/SOH,
// 0x356=voltage/current/temp, 0x35A=alarms, 0x35E=manufacturer ID
// ==========================================
// V2.65 H4: debug_can_status writes go through canStatusMutex into g_can_status[].
// V2.65 M1: g_can_tx_ok/fail/fail_streak counters exposed in /data for health tracking.
static inline void setCanStatus(const char *s) {
  if (canStatusMutex && xSemaphoreTake(canStatusMutex, pdMS_TO_TICKS(20))) {
    strncpy(g_can_status, s, sizeof(g_can_status) - 1);
    g_can_status[sizeof(g_can_status) - 1] = '\0';
    xSemaphoreGive(canStatusMutex);
  }
}

void sendCanFrame(uint32_t id, uint8_t *data) {
  twai_message_t m; m.identifier = id; m.extd = 0; m.data_length_code = 8; memcpy(m.data, data, 8);
  esp_err_t result = twai_transmit(&m, pdMS_TO_TICKS(5));
  if (result == ESP_OK) {
    can_error_flag = false;
    setCanStatus("OK - Connected");
    g_can_tx_ok++;
    g_can_tx_fail_streak = 0;
  }
  else {
    twai_status_info_t status; twai_get_status_info(&status);
    if (status.state == TWAI_STATE_BUS_OFF) {
      can_error_flag = true;
      setCanStatus("Bus off - check cable and termination (120 Ohm)");
      twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_RECOVERING) {
      can_error_flag = true;
      setCanStatus("Bus recovery in progress...");
    } else if (status.state == TWAI_STATE_STOPPED) {
      can_error_flag = true;
      setCanStatus("Bus stopped - restart required");
    } else if (result == 0x107) {
      can_error_flag = false;
      if (status.tx_error_counter > 127) {
        setCanStatus("No inverter found - check cable");
      } else {
        setCanStatus("Waiting for inverter (no device on CAN bus)");
      }
    } else if (result == 0x103) {
      can_error_flag = true;
      setCanStatus("CAN driver not started - restart needed");
    } else {
      can_error_flag = true;
      setCanStatus("TX error - check CAN connection");
    }
    g_can_tx_fail++;
    g_can_tx_fail_streak++;
    // Log a warning if we cross a failure streak threshold (rate-limited via streak value).
    if (g_can_tx_fail_streak == 50) {
      addToLog("CAN: 50 consecutive TX failures", true);
    } else if (g_can_tx_fail_streak == 500) {
      addToLog("CAN: 500 consecutive TX failures, check bus", true);
    }
  }
  stat_tx_count++;
}

void sendVictronCAN() {
  uint8_t d[8] = {0}; float cvl = (g_runtime_cvl > 0.1f) ? g_runtime_cvl : g_cvl_voltage;
  int cv = (int)(cvl * 10), ccl = (int)(victronData.maxChargeCurrent * 10), dcl = (int)(victronData.maxDischargeCurrent * 10);
  d[0] = cv & 0xFF; d[1] = cv >> 8; d[2] = ccl & 0xFF; d[3] = ccl >> 8; d[4] = dcl & 0xFF; d[5] = dcl >> 8;
  sendCanFrame(0x351, d); delay(2);
  d[0] = (int)victronData.avgSOC & 0xFF; d[1] = (int)victronData.avgSOC >> 8; d[2] = (int)victronData.avgSOH & 0xFF; d[3] = (int)victronData.avgSOH >> 8;
  int cap = (int)(victronData.totalCapacity * 10); d[4] = cap & 0xFF; d[5] = cap >> 8; d[6]=0; d[7]=0;
  sendCanFrame(0x355, d); delay(2);
  int v = (int)(victronData.avgVoltage * 100), i = (int)(victronData.totalCurrent * 10), t = (int)(victronData.avgTemp * 10);
  d[0] = v & 0xFF; d[1] = v >> 8; d[2] = i & 0xFF; d[3] = i >> 8; d[4] = t & 0xFF; d[5] = t >> 8; d[6]=0; d[7]=0;
  sendCanFrame(0x356, d); delay(2);
  memset(d, 0, 8); 
  if(alarm_flags & 0x01) d[4] |= 0x80; if(alarm_flags & 0x40) d[4] |= 0x80; if(alarm_flags & 0x02) d[4] |= 0x40; if(alarm_flags & 0x08) d[4] |= 0x20; if(alarm_flags & 0x10) d[4] |= 0x10; 
  if(g_can_protocol == 2 && victronData.maxChargeCurrent < 0.1) d[4] |= 0x02; if(g_can_protocol == 2 && victronData.maxDischargeCurrent < 0.1) d[4] |= 0x01; 
  sendCanFrame(0x35A, d); delay(2);
  if(g_can_protocol == 1) { char n[] = "PYLON   "; sendCanFrame(0x35E, (uint8_t*)n); }
  else if(g_can_protocol == 2) { char n[] = "SMA     "; sendCanFrame(0x35E, (uint8_t*)n); }
  else { char n[] = "TOPBAND "; sendCanFrame(0x35E, (uint8_t*)n); } 
}

#ifdef MODE_SMART_WIFI
// ==========================================
// HISTORY, ENERGY PERSISTENCE & EXPORT (NVS namespace "h")
// V2.65.4 architectural change: chart history is RAM-only now. See top-of-file
// changelog for rationale. NVS in this namespace holds ONLY energy counters:
//   ei/eo       = today's charge/discharge in kWh (float)
//   edi/edo     = rolling 7-day charge/discharge (float[7])
//   emi/emo     = monthly totals (float[12])
// These are written at most once per hour plus on day rollover. Total NVS
// footprint for this namespace: ~15 entries, stable across uptime.
// Chart arrays (powerHistory[], voltHistory[], socHistory[], tempHistory[]
// plus their min/max siblings) live in RAM only. After a reboot, charts
// start empty and fill over 48 hours. CSV export at /export reads those
// RAM arrays directly.
// ==========================================

// Legacy NVS keys from pre-V2.65.4 firmware. Removed on first V2.65.4 boot
// so their entries can be reclaimed by the NVS garbage collector.
static void deleteLegacyHistoryKeys() {
  static const char *legacy[] = {
    "hdat", "hvlt", "hsoc", "htmp",   // V2.65.1 and earlier: 4-blob schema
    "hidx", "hfill",                   // V2.65.1 and earlier: index metadata
    "hblob"                            // V2.65.2 / V2.65.3: consolidated blob
  };
  bool any_removed = false;
  for (size_t i = 0; i < sizeof(legacy)/sizeof(legacy[0]); i++) {
    if (histStore.isKey(legacy[i])) {
      histStore.remove(legacy[i]);
      any_removed = true;
    }
  }
  if (any_removed) {
    addToLog("NVS: legacy history keys removed (V2.65.4 cleanup)", false);
  }
}

// ==========================================
// V2.65.1 D1: NVS STATS HELPER
// Returns used/free/total entries for the default NVS partition. Called
// at boot (for a baseline log line) and on every /data response (for
// continuous dashboard monitoring). Returns false if the API errors.
// ==========================================
bool getNvsStats(uint32_t &used, uint32_t &free, uint32_t &total) {
  nvs_stats_t st;
  esp_err_t err = nvs_get_stats(NULL, &st);
  if (err != ESP_OK) { used = free = total = 0; return false; }
  used  = (uint32_t)st.used_entries;
  free  = (uint32_t)st.free_entries;
  total = (uint32_t)st.total_entries;
  return true;
}

// V2.65.4: loadHistory only restores energy counters. Chart arrays default
// to zero (global init) and fill during runtime.
void loadHistory() {
  histStore.begin("h", false);  // R/W so we can delete legacy keys

  energy_in_today  = histStore.getFloat("ei", 0);
  energy_out_today = histStore.getFloat("eo", 0);

  if (histStore.getBytesLength("edi") == sizeof(energy_days_in)) {
    histStore.getBytes("edi", energy_days_in,  sizeof(energy_days_in));
    histStore.getBytes("edo", energy_days_out, sizeof(energy_days_out));
  } else if (histStore.isKey("edi")) {
    addToLog("NVS load: edi size mismatch, kept zeros", true);
  }
  if (histStore.getBytesLength("emi") == sizeof(energy_months_in)) {
    histStore.getBytes("emi", energy_months_in,  sizeof(energy_months_in));
    histStore.getBytes("emo", energy_months_out, sizeof(energy_months_out));
  } else if (histStore.isKey("emi")) {
    addToLog("NVS load: emi size mismatch, kept zeros", true);
  }

  // R2: Always attempt to purge legacy history keys from older firmware.
  deleteLegacyHistoryKeys();

  histStore.end();

  // Chart arrays are already zero via global initialization.
  // historyIdx = 0, historyFilled = 0 - Frontend blanks charts correctly.

  // V2.65.1 D1: Boot-time NVS partition snapshot.
  uint32_t u,f,t;
  if (getNvsStats(u,f,t)) {
    addToLog(String("NVS: ") + u + " used / " + f + " free / " + t + " total entries", false);
  }
  // V2.65.4 R7: make the RAM-only design explicit in the boot log.
  addToLog(String("History: RAM-only, 48h depth at 3-min resolution, ") +
           String(HISTORY_LEN / 3) + " chart points", false);
}

// V2.65.4 R1: saveHistory writes only energy counters. No includeGraph
// parameter, no blob logic, no malloc. Called at most once per hour
// from the main loop, plus on day rollover and before OTA.
bool saveHistory() {
  histStore.begin("h", false);
  bool ok = true;

  if (histStore.putFloat("ei", energy_in_today)  == 0) { addToLog("NVS save: ei FAILED",  true); ok = false; }
  if (histStore.putFloat("eo", energy_out_today) == 0) { addToLog("NVS save: eo FAILED",  true); ok = false; }
  if (histStore.putBytes("edi", energy_days_in,  sizeof(energy_days_in))  != sizeof(energy_days_in))  { addToLog("NVS save: edi FAILED", true); ok = false; }
  if (histStore.putBytes("edo", energy_days_out, sizeof(energy_days_out)) != sizeof(energy_days_out)) { addToLog("NVS save: edo FAILED", true); ok = false; }
  if (histStore.putBytes("emi", energy_months_in,  sizeof(energy_months_in))  != sizeof(energy_months_in))  { addToLog("NVS save: emi FAILED", true); ok = false; }
  if (histStore.putBytes("emo", energy_months_out, sizeof(energy_months_out)) != sizeof(energy_months_out)) { addToLog("NVS save: emo FAILED", true); ok = false; }

  histStore.end();

  if (ok) {
    addToLog("Energy counters saved", false);
  } else {
    uint32_t u,f,t;
    if (getNvsStats(u,f,t)) {
      addToLog(String("NVS state: ") + u + " used / " + f + " free", true);
    }
  }
  return ok;
}

// ==========================================
// V2.61: ALERT RING BUFFER FUNCTIONS
// ==========================================
// Storage: "gateway" namespace, keys "a_cnt", "a_hd", "a_dat" (blob).
// Size budget: 25 entries * sizeof(AlertEntry) = ~2.7 KB in NVS.

static uint32_t alertNowEpoch() {
  time_t now_t = time(nullptr);
  return (now_t > 1700000000UL) ? (uint32_t)now_t : 0;  // 0 = "no valid NTP"
}

void addAlertSrv(uint8_t sev, const String& msg) {
  if (msg.length() == 0) return;
  uint32_t ep = alertNowEpoch();
  uint32_t up = millis() / 1000;
  // Dedup: suppress if same message raised within the last 30s (scan last 5)
  int scan = g_alert_count < 5 ? g_alert_count : 5;
  for (int i = 1; i <= scan; i++) {
    int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
    if (strncmp(g_alerts[idx].msg, msg.c_str(), ALERT_MSG_LEN - 1) == 0) {
      uint32_t prev_up = g_alerts[idx].uptime_s;
      if (up >= prev_up && (up - prev_up) < 30) return;
    }
  }
  AlertEntry& e = g_alerts[g_alert_head];
  e.ts = ep; e.uptime_s = up; e.sev = sev;
  strncpy(e.msg, msg.c_str(), ALERT_MSG_LEN - 1);
  e.msg[ALERT_MSG_LEN - 1] = 0;
  g_alert_head = (g_alert_head + 1) % ALERT_RING_SIZE;
  if (g_alert_count < ALERT_RING_SIZE) g_alert_count++;
  g_alerts_dirty = true;
}

void loadAlertsFromNvs() {
  preferences.begin("gateway", true);
  g_alert_count = preferences.getInt("a_cnt", 0);
  g_alert_head = preferences.getInt("a_hd", 0);
  if (g_alert_count < 0 || g_alert_count > ALERT_RING_SIZE) g_alert_count = 0;
  if (g_alert_head < 0 || g_alert_head >= ALERT_RING_SIZE) g_alert_head = 0;
  size_t expected = sizeof(g_alerts);
  if (g_alert_count > 0 && preferences.getBytesLength("a_dat") == expected) {
    preferences.getBytes("a_dat", g_alerts, expected);
  } else {
    memset(g_alerts, 0, sizeof(g_alerts));
    g_alert_count = 0; g_alert_head = 0;
  }
  preferences.end();
}

void saveAlertsToNvs() {
  preferences.begin("gateway", false);
  preferences.putInt("a_cnt", g_alert_count);
  preferences.putInt("a_hd", g_alert_head);
  preferences.putBytes("a_dat", g_alerts, sizeof(g_alerts));
  preferences.end();
  g_alerts_dirty = false;
  g_alerts_last_save = millis();
}

void clearAlertsAll() {
  memset(g_alerts, 0, sizeof(g_alerts));
  g_alert_count = 0; g_alert_head = 0;
  g_alerts_dirty = false;
  preferences.begin("gateway", false);
  preferences.remove("a_cnt");
  preferences.remove("a_hd");
  preferences.remove("a_dat");
  preferences.end();
  memset(&g_alert_state, 0, sizeof(g_alert_state));
  addToLog("Alerts cleared (RAM + NVS)", false);
}

// Called from main loop on Core 1. State-transition based (raises on rising edge,
// clears state flag on falling edge so alert re-raises if condition returns).
// V2.64: Snapshots victronData and bms[] under dataMutex before running detection logic.
// Runs only once per 10s on Core 1, so lock overhead is negligible.
void alertDetectTick() {
  unsigned long now = millis();
  if (now - g_alerts_last_detect < 10000) return;
  g_alerts_last_detect = now;

  // Snapshot shared state under mutex (no function calls while holding lock)
  VictronType vs;
  BMSData bs[MAX_BMS];
  int bmsCnt = g_bms_count;
  if (!dataMutex || !xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    addToLog("WARN: alertDetectTick mutex timeout, skipping tick", true);
    return;
  }
  memcpy(&vs, &victronData, sizeof(vs));
  memcpy(bs, bms, sizeof(bs));
  xSemaphoreGive(dataMutex);

  // --- System-level: no packs online ---
  bool no_packs_now = (vs.activePacks == 0);
  if (no_packs_now && !g_alert_state.no_packs) addAlertSrv(2, "No BMS packs online");
  g_alert_state.no_packs = no_packs_now;

  // --- Per-BMS checks (operate on snapshot, not live bms[]) ---
  for (int i = 0; i < bmsCnt && i < MAX_BMS; i++) {
    bool online = bs[i].valid && (now - bs[i].last_seen < 120000);

    bool offline_now = !online;
    if (offline_now && !g_alert_state.bms_offline[i]) addAlertSrv(1, "BMS #" + String(i) + " offline");
    g_alert_state.bms_offline[i] = offline_now;
    if (!online) continue;

    float maxC = bs[i].maxCellV;
    float minC = bs[i].minCellV;
    float drift = (maxC > minC) ? (maxC - minC) : 0.0f;

    bool cm_near = (maxC >= g_safe_cell - 0.02f);
    if (cm_near && !g_alert_state.cell_max_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " cell max " + String(maxC, 3) + "V near limit");
    g_alert_state.cell_max_near[i] = cm_near;

    bool cn_crit = (minC > 0 && minC <= 2.8f);
    if (cn_crit && !g_alert_state.cell_min_crit[i])
      addAlertSrv(2, "BMS #" + String(i) + " cell min " + String(minC, 3) + "V critically low");
    g_alert_state.cell_min_crit[i] = cn_crit;

    bool dr_near = (drift >= g_safe_drift - 0.03f);
    if (dr_near && !g_alert_state.drift_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " cell drift " + String(drift, 3) + "V near limit");
    g_alert_state.drift_near[i] = dr_near;

    // Temperature: any sensor within 3C of discharge min
    bool tmp_near = false;
    for (int t = 0; t < bs[i].temp_count; t++) {
      if (bs[i].temps[t] <= g_td_min + 3.0f) { tmp_near = true; break; }
    }
    if (tmp_near && !g_alert_state.temp_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " temperature approaching discharge min");
    g_alert_state.temp_near[i] = tmp_near;

    bool slow = (bs[i].soc <= 10);
    if (slow && !g_alert_state.soc_low[i])
      addAlertSrv(1, "BMS #" + String(i) + " SOC " + String(bs[i].soc) + "% critically low");
    g_alert_state.soc_low[i] = slow;
  }

  // --- System SOC ---
  bool sys_low_now = (vs.activePacks > 0 && vs.avgSOC <= 15.0f);
  if (sys_low_now && !g_alert_state.sys_soc_low)
    addAlertSrv(1, "System SOC " + String(vs.avgSOC, 0) + "% low");
  g_alert_state.sys_soc_low = sys_low_now;

  // --- CAN bus error ---
  #ifdef MODE_SMART_WIFI
  // V2.65 H4: Snapshot g_can_status under mutex into local buffer, then scan.
  char canSnap[64] = {0};
  if (canStatusMutex && xSemaphoreTake(canStatusMutex, pdMS_TO_TICKS(20))) {
    strncpy(canSnap, g_can_status, sizeof(canSnap) - 1);
    xSemaphoreGive(canStatusMutex);
  }
  bool can_err_now = (strstr(canSnap, "Err") != NULL || strstr(canSnap, "FAIL") != NULL);
  if (can_err_now && !g_alert_state.can_error_state)
    addAlertSrv(2, String("CAN bus error: ") + canSnap);
  g_alert_state.can_error_state = can_err_now;

  // --- MQTT disconnect (if enabled and persistently failing) ---
  bool mq_fail_now = (g_mqtt_enable && !mqtt.connected() && mqtt_fail_count >= 3);
  if (mq_fail_now && !g_alert_state.mqtt_fail_state)
    addAlertSrv(1, "MQTT disconnected (" + String(mqtt_fail_count) + " failures)");
  g_alert_state.mqtt_fail_state = mq_fail_now;
  #endif

  // --- Throttled save: every 60s if dirty ---
  if (g_alerts_dirty && (now - g_alerts_last_save > 60000)) {
    saveAlertsToNvs();
  }
}

void calculateEnergy(float power_w) {
  unsigned long now = millis(); if (last_energy_calc == 0) { last_energy_calc = now; return; }
  double h = (now - last_energy_calc) / 3600000.0; last_energy_calc = now;
  double kw = power_w / 1000.0;
  if (kw > 0.001) energy_in_today += (kw * h); else if (kw < -0.001) energy_out_today += (fabs(kw) * h);
  struct tm t; if (getLocalTime(&t)) {
    // Day rollover: push today's totals into rolling 7-day array
    if (current_day != -1 && current_day != t.tm_mday) {
      for (int i = 6; i > 0; i--) { energy_days_in[i] = energy_days_in[i-1]; energy_days_out[i] = energy_days_out[i-1]; }
      energy_days_in[0] = energy_in_today; energy_days_out[0] = energy_out_today;
      // Month rollover: accumulate into monthly totals
      if (current_month != -1 && current_month != t.tm_mon) {
        for (int i = 11; i > 0; i--) { energy_months_in[i] = energy_months_in[i-1]; energy_months_out[i] = energy_months_out[i-1]; }
        energy_months_in[0] = 0; energy_months_out[0] = 0;
      }
      energy_months_in[0] += energy_in_today; energy_months_out[0] += energy_out_today;
      energy_in_today = 0; energy_out_today = 0;
      saveHistory();
      addToLog("Energy: day rollover, saved history", false);
    }
    current_day = t.tm_mday;
    current_month = t.tm_mon;
  }
}
#if HAS_SD_CARD
void initSD() {
  if (!g_sd_enable) return;
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS)) { sd_ok = true; sd_error_flag = false; } else { sd_ok = false; sd_error_flag = true; addToLog("SD: No Card", true); }
}
void writeLogToSD() {
  if (!g_sd_enable || !sd_ok) return;
  File f = SD.open("/log.csv", FILE_APPEND);
  if (f) { f.printf("%s;%.1f;%.0f;%.1f\n", getTimeStr().c_str(), victronData.avgSOC, victronData.totalPower, victronData.totalCurrent); f.close(); } else { sd_error_flag = true; }
}
void handleSDDownload() { if(!server.hasArg("file")) { server.send(400, "text/plain", "Bad Request"); return; } String path = server.arg("file"); if (!sd_ok || !SD.exists(path)) { server.send(404, "text/plain", "File Not Found"); return; } File f = SD.open(path); if (f) { server.streamFile(f, "application/octet-stream"); f.close(); } }
void handleSpyDownload() { if (!sd_ok) { server.send(404, "text/plain", "No SD"); return; } File f = SD.open("/spy.txt"); if (f) { server.streamFile(f, "text/plain"); f.close(); } else server.send(404, "text/plain", "No Spy Log"); }
void handleSDList() { if(!sd_ok) { server.send(200, "application/json", "[]"); return; } String out = "["; File root = SD.open("/"); File file = root.openNextFile(); bool first = true; while(file){ if(!first) out += ","; out += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}"; first = false; file = root.openNextFile(); } out += "]"; server.send(200, "application/json", out); }
void handleSDDelete() { if(!sd_ok) { server.send(500, "text/plain", "No SD"); return; } if(server.hasArg("file")) { String f = server.arg("file"); if(SD.exists(f)) { SD.remove(f); server.send(200, "text/plain", "Deleted"); } else server.send(404, "text/plain", "Not Found"); } else if(server.hasArg("all")) { File root = SD.open("/"); File file = root.openNextFile(); while(file){ String n = file.name(); file = root.openNextFile(); SD.remove("/"+n); } server.send(200, "text/plain", "Wiped"); } else server.send(400, "text/plain", "Bad Args"); }
#else
void initSD() {}
void writeLogToSD() {}
#endif
// ==========================================
// MQTT & HOME ASSISTANT INTEGRATION
// MQTT publishes to {topic}/data every 5s with SOC, power, voltage, current,
// temperature, energy, alarm status, and optionally per-BMS cell data.
// Separate {topic}/alarm topic (retained) publishes on alarm state changes
// for HA automations. LWT: {topic}/status="offline" on unexpected disconnect.
// Reconnect uses exponential backoff: 5s -> 30s -> 60s.
// HA auto-discovery is re-sent on every MQTT connect to keep FW version
// and sensor config current after OTA updates.
// ==========================================
bool sendHaDiscovery() {
    if(!mqtt.connected()) return false;
    bool ok = true;
    String uid = g_dev_uid;
    String tp = "homeassistant/sensor/" + uid + "/";
    String btp = "homeassistant/binary_sensor/" + uid + "/";
    String dev = "\"dev\":{\"ids\":\"" + uid + "\",\"name\":\"Topband BMS (" + uid.substring(3) + ")\",\"mdl\":\"V" + String(FW_VERSION) + "\",\"mf\":\"DIY\"}";
    String stat_t = g_mqtt_topic + "/data";
    String av = "";
    String p1 = "{\"name\":\"SOC\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.soc}}\",\"unit_of_meas\":\"%\",\"dev_cla\":\"battery\",\"uniq_id\":\"" + uid + "_soc\"," + dev + "}";
    ok &= mqtt.publish((tp + "soc/config").c_str(), p1.c_str(), true);
    String p2 = "{\"name\":\"Power\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.p}}\",\"unit_of_meas\":\"W\",\"dev_cla\":\"power\",\"uniq_id\":\"" + uid + "_p\"," + dev + "}";
    ok &= mqtt.publish((tp + "p/config").c_str(), p2.c_str(), true);
    String p3 = "{\"name\":\"Voltage\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.v}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_v\"," + dev + "}";
    ok &= mqtt.publish((tp + "v/config").c_str(), p3.c_str(), true);
    String p4 = "{\"name\":\"Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.i}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_i\"," + dev + "}";
    ok &= mqtt.publish((tp + "i/config").c_str(), p4.c_str(), true);

    String p5 = "{\"name\":\"Active BMS\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.active}}\",\"unit_of_meas\":\"packs\",\"icon\":\"mdi:battery-multiple\",\"uniq_id\":\"" + uid + "_active\"," + dev + "}";
    ok &= mqtt.publish((tp + "active/config").c_str(), p5.c_str(), true);
    String p6 = "{\"name\":\"Max Charge Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.max_chg}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_max_chg\"," + dev + "}";
    ok &= mqtt.publish((tp + "max_chg/config").c_str(), p6.c_str(), true);
    String p7 = "{\"name\":\"Max Discharge Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.max_dis}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_max_dis\"," + dev + "}";
    ok &= mqtt.publish((tp + "max_dis/config").c_str(), p7.c_str(), true);
    String p8 = "{\"name\":\"Alarm\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.alarm}}\",\"icon\":\"mdi:alert\",\"uniq_id\":\"" + uid + "_alarm\"," + dev + "}";
    ok &= mqtt.publish((tp + "alarm/config").c_str(), p8.c_str(), true);
    String p9 = "{\"name\":\"Avg Temperature\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.avg_temp}}\",\"unit_of_meas\":\"\u00b0C\",\"dev_cla\":\"temperature\",\"uniq_id\":\"" + uid + "_avg_temp\"," + dev + "}";
    ok &= mqtt.publish((tp + "avg_temp/config").c_str(), p9.c_str(), true);
    String p10 = "{\"name\":\"SOH\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.soh}}\",\"unit_of_meas\":\"%\",\"icon\":\"mdi:battery-heart-variant\",\"uniq_id\":\"" + uid + "_soh\"," + dev + "}";
    ok &= mqtt.publish((tp + "soh/config").c_str(), p10.c_str(), true);
    String p11 = "{\"name\":\"Remaining Capacity\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.rem_cap}}\",\"unit_of_meas\":\"Ah\",\"icon\":\"mdi:battery-charging\",\"uniq_id\":\"" + uid + "_rem_cap\"," + dev + "}";
    ok &= mqtt.publish((tp + "rem_cap/config").c_str(), p11.c_str(), true);
    String p12 = "{\"name\":\"Total Capacity\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.total_cap}}\",\"unit_of_meas\":\"Ah\",\"icon\":\"mdi:battery\",\"uniq_id\":\"" + uid + "_total_cap\"," + dev + "}";
    ok &= mqtt.publish((tp + "total_cap/config").c_str(), p12.c_str(), true);
    String p13 = "{\"name\":\"Energy In Today\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.energy_in}}\",\"unit_of_meas\":\"kWh\",\"dev_cla\":\"energy\",\"uniq_id\":\"" + uid + "_energy_in\"," + dev + "}";
    ok &= mqtt.publish((tp + "energy_in/config").c_str(), p13.c_str(), true);
    String p14 = "{\"name\":\"Energy Out Today\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.energy_out}}\",\"unit_of_meas\":\"kWh\",\"dev_cla\":\"energy\",\"uniq_id\":\"" + uid + "_energy_out\"," + dev + "}";
    ok &= mqtt.publish((tp + "energy_out/config").c_str(), p14.c_str(), true);

    // V2.67 F1: Per-BMS discovery is now level-gated.
    //   L1: no per-BMS entities
    //   L2: 6 existing (soc, v, i, min_cell, max_cell, online) + 6 new stat entities
    //   L3: L2 entities + 16 cell + up to 4 temp entities per BMS on cells topic
    // On downgrade or disable, empty retained payloads remove stale entities.
    String cells_stat_base = g_mqtt_topic + "/cells/bms";  // used for L3 entities
    for (int i = 0; i < MAX_BMS; i++) {
      String idx = String(i);
      bool active = (g_mqtt_level >= 1) && (i < g_bms_count);

      // --- L2 core 6 entities (soc, v, i, min, max, online) ---
      String t_soc = tp + "bms" + idx + "_soc/config";
      String t_v   = tp + "bms" + idx + "_v/config";
      String t_i   = tp + "bms" + idx + "_i/config";
      String t_min = tp + "bms" + idx + "_min_cell/config";
      String t_max = tp + "bms" + idx + "_max_cell/config";
      String t_on  = btp + "bms" + idx + "_online/config";
      if (active) {
        String s_soc = "{\"name\":\"BMS " + idx + " SOC\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].soc | default(none)}}\",\"unit_of_meas\":\"%\",\"dev_cla\":\"battery\",\"uniq_id\":\"" + uid + "_bms" + idx + "_soc\"," + dev + "}";
        String s_v   = "{\"name\":\"BMS " + idx + " Voltage\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].pack_v | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_v\"," + dev + "}";
        String s_i   = "{\"name\":\"BMS " + idx + " Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].current | default(none)}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_bms" + idx + "_i\"," + dev + "}";
        String s_min = "{\"name\":\"BMS " + idx + " Min Cell\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].min_cell | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_min_cell\"," + dev + "}";
        String s_max = "{\"name\":\"BMS " + idx + " Max Cell\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].max_cell | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_max_cell\"," + dev + "}";
        String s_on  = "{\"name\":\"BMS " + idx + " Online\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{ 'ON' if (value_json.bms[" + idx + "].online | default(false)) else 'OFF' }}\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"connectivity\",\"uniq_id\":\"" + uid + "_bms" + idx + "_online\"," + dev + "}";
        ok &= mqtt.publish(t_soc.c_str(), s_soc.c_str(), true);
        ok &= mqtt.publish(t_v.c_str(),   s_v.c_str(),   true);
        ok &= mqtt.publish(t_i.c_str(),   s_i.c_str(),   true);
        ok &= mqtt.publish(t_min.c_str(), s_min.c_str(), true);
        ok &= mqtt.publish(t_max.c_str(), s_max.c_str(), true);
        ok &= mqtt.publish(t_on.c_str(),  s_on.c_str(),  true);
      } else {
        ok &= mqtt.publish(t_soc.c_str(), "", true);
        ok &= mqtt.publish(t_v.c_str(),   "", true);
        ok &= mqtt.publish(t_i.c_str(),   "", true);
        ok &= mqtt.publish(t_min.c_str(), "", true);
        ok &= mqtt.publish(t_max.c_str(), "", true);
        ok &= mqtt.publish(t_on.c_str(),  "", true);
      }

      // --- L2 NEW 6 stat entities (drift_mv, polls, timeouts, errors, spikes, current_holds) ---
      struct StatDef { const char *suffix; const char *name; const char *unit; const char *dc; const char *sc; const char *icon; };
      static const StatDef stats[] = {
        {"drift_mv",      "Cell Drift",     "mV", "",                  "measurement",      "mdi:chart-bell-curve"},
        {"polls",         "Polls",          "",   "",                  "total_increasing", "mdi:counter"},
        {"timeouts",      "Timeouts",       "",   "",                  "total_increasing", "mdi:timer-off"},
        {"errors",        "Errors",         "",   "",                  "total_increasing", "mdi:alert-circle"},
        {"spikes",        "Spike Rejects",  "",   "",                  "total_increasing", "mdi:filter-variant-remove"},
        {"current_holds", "Current Holds",  "",   "",                  "total_increasing", "mdi:pause-circle"},
      };
      const int stat_count = sizeof(stats) / sizeof(stats[0]);
      for (int s = 0; s < stat_count; s++) {
        String t_stat = tp + "bms" + idx + "_" + stats[s].suffix + "/config";
        if (active) {
          String tpl = "{{value_json.bms[" + idx + "]." + stats[s].suffix + " | default(none)}}";
          String payload = "{\"name\":\"BMS " + idx + " " + stats[s].name + "\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"" + tpl + "\",\"uniq_id\":\"" + uid + "_bms" + idx + "_" + stats[s].suffix + "\"";
          if (stats[s].unit[0]) { payload += ",\"unit_of_meas\":\""; payload += stats[s].unit; payload += "\""; }
          if (stats[s].dc[0])   { payload += ",\"dev_cla\":\"";     payload += stats[s].dc;   payload += "\""; }
          if (stats[s].sc[0])   { payload += ",\"stat_cla\":\"";    payload += stats[s].sc;   payload += "\""; }
          if (stats[s].icon[0]) { payload += ",\"icon\":\"";        payload += stats[s].icon; payload += "\""; }
          payload += "," + dev + "}";
          ok &= mqtt.publish(t_stat.c_str(), payload.c_str(), true);
        } else {
          ok &= mqtt.publish(t_stat.c_str(), "", true);
        }
      }

      // --- L3 cells + temps entities, stat_t on {base}/cells/bms{n} ---
      bool l3 = (g_mqtt_level >= 2) && (i < g_bms_count);
      String cells_stat_t = cells_stat_base + idx;
      // 16 cell entities (MAX_CELLS assumed 16 per TopBand spec).
      for (int c = 0; c < 16; c++) {
        String t_cell = tp + "bms" + idx + "_cell" + String(c + 1) + "/config";
        if (l3) {
          String payload = "{\"name\":\"BMS " + idx + " Cell " + String(c + 1) + "\",\"stat_t\":\"" + cells_stat_t + "\",\"val_tpl\":\"{{ value_json.cells[" + String(c) + "] | default(none) }}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"stat_cla\":\"measurement\",\"uniq_id\":\"" + uid + "_bms" + idx + "_cell" + String(c + 1) + "\"," + dev + "}";
          ok &= mqtt.publish(t_cell.c_str(), payload.c_str(), true);
        } else {
          ok &= mqtt.publish(t_cell.c_str(), "", true);
        }
      }
      // Up to 4 per-pack temp entities. TopBand exposes T1..T4, MOS (idx4), ENV (idx5), BAL (idx6).
      // We publish 4 generic-indexed temp entities; HA entity availability follows cells topic.
      for (int tmp = 0; tmp < 4; tmp++) {
        String t_temp = tp + "bms" + idx + "_temp" + String(tmp + 1) + "/config";
        if (l3) {
          String payload = "{\"name\":\"BMS " + idx + " Temp " + String(tmp + 1) + "\",\"stat_t\":\"" + cells_stat_t + "\",\"val_tpl\":\"{{ value_json.temps[" + String(tmp) + "] | default(none) }}\",\"unit_of_meas\":\"\u00b0C\",\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\",\"uniq_id\":\"" + uid + "_bms" + idx + "_temp" + String(tmp + 1) + "\"," + dev + "}";
          ok &= mqtt.publish(t_temp.c_str(), payload.c_str(), true);
        } else {
          ok &= mqtt.publish(t_temp.c_str(), "", true);
        }
      }
    }
    return ok;
}

bool clearHaDiscovery() {
    if(!mqtt.connected()) return false;
    bool ok = true;
    String tp = "homeassistant/sensor/" + g_dev_uid + "/";
    String btp = "homeassistant/binary_sensor/" + g_dev_uid + "/";
    ok &= mqtt.publish((tp + "soc/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "p/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "v/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "i/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "active/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "max_chg/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "max_dis/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "alarm/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "avg_temp/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "soh/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "rem_cap/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "total_cap/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "energy_in/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "energy_out/config").c_str(), "", true);
    for (int i = 0; i < MAX_BMS; i++) {
      String idx = String(i);
      ok &= mqtt.publish((tp + "bms" + idx + "_soc/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_v/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_i/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_min_cell/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_max_cell/config").c_str(), "", true);
      ok &= mqtt.publish((btp + "bms" + idx + "_online/config").c_str(), "", true);
      // V2.67 F1: Also clear new L2 stat entities and all L3 per-cell/temp entities.
      static const char* const l2_stats[] = {"drift_mv","polls","timeouts","errors","spikes","current_holds"};
      for (int s = 0; s < 6; s++) {
        ok &= mqtt.publish((tp + "bms" + idx + "_" + l2_stats[s] + "/config").c_str(), "", true);
      }
      for (int c = 0; c < 16; c++) {
        ok &= mqtt.publish((tp + "bms" + idx + "_cell" + String(c + 1) + "/config").c_str(), "", true);
      }
      for (int tmp = 0; tmp < 4; tmp++) {
        ok &= mqtt.publish((tp + "bms" + idx + "_temp" + String(tmp + 1) + "/config").c_str(), "", true);
      }
    }
    return ok;
}

void handleHaDiscoverySend() {
  if (!checkAuth()) return;
  if (!g_mqtt_enable || g_mqtt_server == "") { server.send(400, "application/json", "{\"ok\":false,\"err\":\"mqtt disabled\"}"); return; }
  mqttReconnect();
  if (!mqtt.connected()) { server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt not connected\"}"); return; }
  if (!sendHaDiscovery()) {
    addToLog("HA Discovery: publish failed", true);
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt publish failed\"}");
    return;
  }
  addToLog("HA Discovery: sent", false);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleHaDiscoveryClear() {
  if (!checkAuth()) return;
  if (!g_mqtt_enable || g_mqtt_server == "") { server.send(400, "application/json", "{\"ok\":false,\"err\":\"mqtt disabled\"}"); return; }
  mqttReconnect();
  if (!mqtt.connected()) { server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt not connected\"}"); return; }
  if (!clearHaDiscovery()) {
    addToLog("HA Discovery: clear publish failed", true);
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt publish failed\"}");
    return;
  }
  addToLog("HA Discovery: cleared", false);
  server.send(200, "application/json", "{\"ok\":true}");
}

void mqttReconnect() {
  if (!g_mqtt_enable || g_mqtt_server == "") return;
  if (mqtt.connected()) { mqtt_fail_count = 0; mqtt_reconnect_interval = 5000; return; }
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - last_mqtt_reconnect_attempt < mqtt_reconnect_interval) return;
  last_mqtt_reconnect_attempt = now;
  esp_task_wdt_reset();
  addToLog("MQTT: connecting to " + g_mqtt_server + ":" + String(g_mqtt_port) + " (attempt " + String(mqtt_fail_count + 1) + ")", false);
  String willTopic = g_mqtt_topic + "/status";
  if (mqtt.connect(g_dev_uid.c_str(), g_mqtt_user.c_str(), g_mqtt_pass.c_str(), willTopic.c_str(), 0, true, "offline")) {
    mqtt.publish((g_mqtt_topic + "/status").c_str(), "online", true);
    mqtt_fail_count = 0;
    mqtt_reconnect_interval = 5000;
    addToLog("MQTT: connected", false);
    // Auto-send HA discovery on every connect (updates FW version in HA)
    if (g_ha_enable) {
      if (sendHaDiscovery()) addToLog("HA: discovery sent (V" + String(FW_VERSION) + ")", false);
      else addToLog("HA: discovery send failed", true);
      // V2.66.3 D2: also (re)send diag discovery if both toggles are active.
      // Retained configs in HA survive HA restarts but not broker-level resets.
      if (g_mqtt_diag) sendMqttDiagDiscovery();
    }
  } else {
    mqtt_fail_count++;
    mqtt_reconnect_interval = mqtt_fail_count < 3 ? 5000 : mqtt_fail_count < 10 ? 30000 : 60000;
    addToLog("MQTT: connect failed (rc=" + String(mqtt.state()) + "), next in " + String(mqtt_reconnect_interval / 1000) + "s", true);
  }
  esp_task_wdt_reset();
}


// Build MQTT JSON under dataMutex, then release before network I/O.
// V2.67 F1: Per-BMS block gated by g_mqtt_level >= 1 (L2+). Adds 6 stat
// fields (drift_mv, polls, timeouts, errors, spikes, current_holds) per BMS.
// L3 cells topic is published separately by sendMqttCells().
void sendMqttData() {
  if (g_mqtt_enable && mqtt.connected()) {
    // Snapshot data under mutex, then release before network I/O
    String json; json.reserve(g_mqtt_level >= 1 ? 2560 : 512);
    String statusTopic = g_mqtt_topic + "/status";
    String dataTopic = g_mqtt_topic + "/data";
    // V2.66.1 C1: Check Take result. On timeout, skip this publish cycle
    // rather than building JSON unlocked and calling Give on unowned mutex.
    bool locked = (dataMutex != NULL) && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200));
    if (!locked) {
      addToLog("WARN: sendMqttData mutex timeout, skipping publish", true);
      return;
    }
    json = "{"; json += "\"soc\":" + String(victronData.avgSOC, 1) + ","; json += "\"p\":" + String(victronData.totalPower, 0) + ",";
    json += "\"i\":" + String(victronData.totalCurrent, 1) + ","; json += "\"v\":" + String(victronData.avgVoltage, 2) + ",";
    json += "\"alarm\":\"" + sys_error_msg + "\","; json += "\"active\":" + String(victronData.activePacks);
    json += ",\"avg_temp\":" + String(victronData.avgTemp, 1);
    json += ",\"soh\":" + String(victronData.avgSOH, 1);
    json += ",\"rem_cap\":" + String(victronData.remainCapacity, 0);
    json += ",\"total_cap\":" + String(victronData.totalCapacity, 0);
    json += ",\"energy_in\":" + String(energy_in_today, 2);
    json += ",\"energy_out\":" + String(energy_out_today, 2);
    json += ",\"max_chg\":" + String(victronData.maxChargeCurrent, 1) + ",\"max_dis\":" + String(victronData.maxDischargeCurrent, 1);
    json += ",\"fw\":\"" + String(FW_VERSION) + "\",\"uid\":\"" + g_dev_uid + "\",\"uptime\":" + String(millis() / 1000);
    json += ",\"level\":" + String((int)g_mqtt_level);  // V2.67 F1
    if (g_mqtt_level >= 1) {
      json += ",\"bms\":[";
      for (int i = 0; i < g_bms_count; i++) {
        if (i > 0) json += ",";
        bool on = bms[i].valid && (millis() - bms[i].last_seen < 120000);
        json += "{\"id\":" + String(i) + ",\"online\":" + String(on ? "true" : "false");
        if (on) {
          json += ",\"soc\":" + String(bms[i].soc);
          json += ",\"current\":" + String(bms[i].current, 1);
          json += ",\"pack_v\":" + String(bms[i].voltage, 2);
          json += ",\"soh\":" + String(bms[i].soh);
          json += ",\"min_cell\":" + String(bms[i].minCellV, 3);
          json += ",\"max_cell\":" + String(bms[i].maxCellV, 3);
          // V2.67 F1: drift_mv = (max_cell - min_cell) * 1000, rounded int.
          int drift_mv = (int)lround((bms[i].maxCellV - bms[i].minCellV) * 1000.0f);
          json += ",\"drift_mv\":" + String(drift_mv);
          json += ",\"cells\":[";
          for (int c = 0; c < bms[i].cell_count; c++) {
            if (c > 0) json += ",";
            json += String(bms[i].cells[c], 3);
          }
          json += "]";
        }
        // V2.67 F1: Per-BMS stat counters (always present on L2+, even if offline).
        json += ",\"polls\":" + String(g_bms_stats[i].polls);
        json += ",\"timeouts\":" + String(g_bms_stats[i].timeouts);
        json += ",\"errors\":" + String(g_bms_stats[i].errors);
        json += ",\"spikes\":" + String(g_bms_stats[i].spikes);
        json += ",\"current_holds\":" + String(g_bms_stats[i].current_holds);
        json += "}";
      }
      json += "]";
    }
    json += "}";
    xSemaphoreGive(dataMutex);  // V2.66.1 C1: locked==true confirmed above
    // Publish outside mutex - network I/O can take 100ms+
    mqtt.publish(statusTopic.c_str(), "online", true);
    mqtt.publish(dataTopic.c_str(), json.c_str());
    // Publish alarm state on separate topic (retained, for HA automations)
    static String prev_alarm_mqtt = "OK";
    String alarm_snap = sys_error_msg;
    if (alarm_snap != prev_alarm_mqtt) {
      String alarmTopic = g_mqtt_topic + "/alarm";
      String alarmJson = "{\"state\":\"" + alarm_snap + "\",\"flags\":" + String(alarm_flags) + ",\"safe_lock\":" + String(((alarm_flags & 0x02)||(alarm_flags & 0x10)||(alarm_flags & 0x40)||(alarm_flags & 0x80)) ? "true" : "false") + "}";
      mqtt.publish(alarmTopic.c_str(), alarmJson.c_str(), true);
      prev_alarm_mqtt = alarm_snap;
    }
  }
}

// V2.67 F1: Publish per-BMS cells topic retained at 20s cadence. L3 only.
// Schema v1: {"v":1,"online":bool,"cell_count":int,"cells":[floats],"temps":[floats]}.
// Offline BMS still publishes a retained frame with online:false and empty arrays
// so HA entities transition cleanly to unavailable.
//
// V2.67.1 hotfix: Snapshot pattern. Previously held dataMutex across up to 16
// synchronous mqtt.publish() TCP sends. On WiFi stalls each publish can block
// for seconds; with 16 packs worst-case exceeded the 60 s TASK_WDT window and
// rebooted the gateway (field-observed 2026-04-24 02:42 on L3).
// Fix: snapshot the ~700 bytes of per-BMS data we need into locals under the
// mutex, release immediately, then build JSON and publish outside the lock.
// rs485Task on Core 0 is no longer blocked by publish stalls. Between packs
// we WDT-reset and bail out on lost MQTT to avoid 16x TCP-timeout cascades.
void sendMqttCells() {
  if (!g_mqtt_enable || !mqtt.connected()) return;
  if (g_mqtt_level < 2) return;

  // Stackside snapshot. 16 packs x ~176 bytes = ~2.8 KB on main-loop task
  // stack (8 KB total), well under headroom.
  struct CellSnap {
    bool on;
    int cell_count;
    int temp_count;
    float cells[32];
    float temps[8];
  };
  static CellSnap snap[MAX_BMS];   // static to keep off the recursion path if
                                   // the compiler ever inlines; same storage
                                   // class used elsewhere for large locals.
  int bms_count_local;

  bool locked = (dataMutex != NULL) && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200));
  if (!locked) {
    addToLog("WARN: sendMqttCells mutex timeout, skipping", true);
    return;
  }
  bms_count_local = g_bms_count;
  if (bms_count_local > MAX_BMS) bms_count_local = MAX_BMS;
  unsigned long now_ms = millis();
  for (int i = 0; i < bms_count_local; i++) {
    snap[i].on = bms[i].valid && (now_ms - bms[i].last_seen < 120000);
    snap[i].cell_count = bms[i].cell_count;
    snap[i].temp_count = bms[i].temp_count;
    int cc = snap[i].cell_count; if (cc < 0) cc = 0; if (cc > 32) cc = 32;
    int tc = snap[i].temp_count; if (tc < 0) tc = 0; if (tc > 8)  tc = 8;
    for (int c = 0; c < cc; c++) snap[i].cells[c] = bms[i].cells[c];
    for (int t = 0; t < tc; t++) snap[i].temps[t] = bms[i].temps[t];
  }
  xSemaphoreGive(dataMutex);

  // Build + publish outside the mutex. rs485Task can poll/parse freely now.
  for (int i = 0; i < bms_count_local; i++) {
    if (!mqtt.connected()) return;   // bail cleanly on mid-batch disconnect
    String payload; payload.reserve(512);
    payload = "{\"v\":" + String(MQTT_CELLS_SCHEMA_V);
    payload += ",\"online\":" + String(snap[i].on ? "true" : "false");
    if (snap[i].on) {
      int cc = snap[i].cell_count; if (cc < 0) cc = 0; if (cc > 32) cc = 32;
      int tc = snap[i].temp_count; if (tc < 0) tc = 0; if (tc > 8)  tc = 8;
      payload += ",\"cell_count\":" + String(cc);
      payload += ",\"cells\":[";
      for (int c = 0; c < cc; c++) {
        if (c > 0) payload += ",";
        payload += String(snap[i].cells[c], 3);
      }
      payload += "],\"temps\":[";
      for (int t = 0; t < tc; t++) {
        if (t > 0) payload += ",";
        payload += String(snap[i].temps[t], 1);
      }
      payload += "]";
    } else {
      payload += ",\"cell_count\":0,\"cells\":[],\"temps\":[]";
    }
    payload += "}";
    String topic = g_mqtt_topic + "/cells/bms" + String(i);
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    esp_task_wdt_reset();   // safety net between publishes
  }
}

// V2.67 F3: Single source of truth for diag-key explanations.
// Used by (a) MQTT /diag "_help" object and (b) Web UI Diagnostics tooltips.
// Each help string is <= 80 chars, plain English, acronyms expanded on first use.
// Keys MUST match the snprintf fields in buildDiagPayloadJson and the entries[]
// table in sendMqttDiagDiscovery.
struct DiagHelp { const char *key; const char *help; };
static const DiagHelp DIAG_KEY_HELP[] = {
  {"fw",                 "Firmware version string, e.g. 2.67."},
  {"uptime",             "Seconds since last boot."},
  {"boot_reason",        "ESP32 reset cause (POWERON, SW, PANIC, TASK_WDT, BROWNOUT)."},
  {"heap_free",          "Free heap RAM in bytes right now."},
  {"heap_min",           "Lowest free-heap value seen since boot."},
  {"nvs_used",           "Non-Volatile Storage entries currently used."},
  {"nvs_free",           "Non-Volatile Storage entries still free."},
  {"can_tx_ok",          "Successful CAN-bus frames sent since boot."},
  {"can_tx_fail",        "Failed CAN-bus frame sends since boot."},
  {"can_tx_fail_streak", "Consecutive CAN TX failures; resets on first success."},
  {"can_status",         "Current CAN bus state (DISABLED, OK, BUS-OFF, etc)."},
  {"rs485_hwm",          "RS485 task stack high-water mark in bytes."},
  {"bms_polls",          "Total BMS poll attempts across all packs since boot."},
  {"bms_timeouts",       "BMS polls that got no response before timeout."},
  {"current_holds",      "Safety current-hold activations (drift or spike cutoffs)."},
  {"spike_rejects",      "Values rejected by spike filter (cell V, current, SOC)."},
  {"rl_rejects",         "HTTP 429 rate-limit responses served since boot."},
  {"mqtt_fail",          "MQTT broker connect or publish failures since boot."},
  {"session_age_d",      "Age in days of the active login session token."},
  {"stream_aborts",      "/data handler aborts from client disconnect mid-stream."},
  {"handler_max_ms",     "Longest HTTP handler runtime observed since boot."},
  {"loop_max_ms",        "Longest main-loop iteration since boot in ms."},
  {"wdt_warnings",       "Main-loop iterations over 45 s (near WDT trip)."},
  {"last_reset_ts",      "Unix epoch of last counter reset (0=never, 7-day auto)."},
};
static const int DIAG_KEY_HELP_COUNT = (int)(sizeof(DIAG_KEY_HELP) / sizeof(DIAG_KEY_HELP[0]));

// V2.67 F3: Build the diag JSON payload into buf. Adds "_help" object when
// includeHelp=true. Returns number of bytes written or -1 on overflow.
// Shared by sendMqttDiag (MQTT /diag topic) and handleDiag (HTTP /diag GET).
int buildDiagPayloadJson(char *buf, size_t bufsize, bool includeHelp) {
  uint32_t total_polls = 0, total_timeouts = 0;
  uint32_t total_holds = 0, total_spikes = 0;
  for (int i = 0; i < MAX_BMS; i++) {
    total_polls    += g_bms_stats[i].polls;
    total_timeouts += g_bms_stats[i].timeouts;
    total_holds    += g_bms_stats[i].current_holds;
    total_spikes   += g_bms_stats[i].spikes;
  }
  uint32_t nvs_u = 0, nvs_f = 0, nvs_t = 0;
  getNvsStats(nvs_u, nvs_f, nvs_t);
  uint32_t rs485_hwm = 0;
  if (rs485TaskHandle) rs485_hwm = (uint32_t)uxTaskGetStackHighWaterMark(rs485TaskHandle);
  char can_snap[64] = {0};
  if (canStatusMutex && xSemaphoreTake(canStatusMutex, pdMS_TO_TICKS(20))) {
    strncpy(can_snap, g_can_status, sizeof(can_snap) - 1);
    xSemaphoreGive(canStatusMutex);
  }
  for (int k = 0; k < 63 && can_snap[k]; k++) {
    if (can_snap[k] == '"' || can_snap[k] == '\\') can_snap[k] = ' ';
  }
  uint32_t sess_age_d = 0;
  time_t now_t; time(&now_t);
  if (g_session_ts > 1672531200UL && (uint32_t)now_t > g_session_ts) {
    sess_age_d = ((uint32_t)now_t - g_session_ts) / 86400UL;
  }

  int n = snprintf(buf, bufsize,
    "{\"fw\":\"%s\",\"uptime\":%lu,\"boot_reason\":\"%s\""
    ",\"heap_free\":%lu,\"heap_min\":%lu"
    ",\"nvs_used\":%lu,\"nvs_free\":%lu"
    ",\"can_tx_ok\":%lu,\"can_tx_fail\":%lu,\"can_tx_fail_streak\":%lu"
    ",\"can_status\":\"%s\""
    ",\"rs485_hwm\":%lu"
    ",\"bms_polls\":%lu,\"bms_timeouts\":%lu"
    ",\"current_holds\":%lu,\"spike_rejects\":%lu"
    ",\"rl_rejects\":%lu"
    ",\"mqtt_fail\":%u"
    ",\"session_age_d\":%lu"
    ",\"stream_aborts\":%lu,\"handler_max_ms\":%lu"
    ",\"loop_max_ms\":%lu,\"wdt_warnings\":%lu"
    ",\"last_reset_ts\":%lu}",
    FW_VERSION,
    (unsigned long)(millis() / 1000UL),
    g_boot_reason.c_str(),
    (unsigned long)ESP.getFreeHeap(), (unsigned long)heap_min,
    (unsigned long)nvs_u, (unsigned long)nvs_f,
    (unsigned long)g_can_tx_ok, (unsigned long)g_can_tx_fail, (unsigned long)g_can_tx_fail_streak,
    can_snap,
    (unsigned long)rs485_hwm,
    (unsigned long)total_polls, (unsigned long)total_timeouts,
    (unsigned long)total_holds, (unsigned long)total_spikes,
    (unsigned long)g_rl_rejects,
    (unsigned)mqtt_fail_count,
    (unsigned long)sess_age_d,
    (unsigned long)g_stream_aborts, (unsigned long)g_handler_max_ms,
    (unsigned long)g_loop_max_ms, (unsigned long)g_wdt_warnings,
    (unsigned long)g_last_reset_ts);
  if (n <= 0 || n >= (int)bufsize) return -1;

  if (includeHelp && buf[n - 1] == '}') {
    int pos = n - 1;
    int remain = (int)bufsize - pos;
    int w = snprintf(buf + pos, remain, ",\"_help\":{");
    if (w < 0 || w >= remain) { buf[n - 1] = '}'; return n; }
    pos += w; remain -= w;
    for (int i = 0; i < DIAG_KEY_HELP_COUNT; i++) {
      w = snprintf(buf + pos, remain, "%s\"%s\":\"%s\"",
                   i == 0 ? "" : ",",
                   DIAG_KEY_HELP[i].key, DIAG_KEY_HELP[i].help);
      if (w < 0 || w >= remain - 2) {
        buf[n - 1] = '}'; buf[n] = '\0';
        return n;
      }
      pos += w; remain -= w;
    }
    w = snprintf(buf + pos, remain, "}}");
    if (w < 0 || w >= remain) { buf[n - 1] = '}'; buf[n] = '\0'; return n; }
    pos += w;
    return pos;
  }
  return n;
}

// V2.66.2 D1: Diagnostic MQTT publish, opt-in via g_mqtt_diag. 30s cadence.
// Separate topic {base}/diag so existing /data subscribers are unaffected.
// V2.67 F3: Refactored to use buildDiagPayloadJson helper; now emits "_help".
void sendMqttDiag() {
  if (!g_mqtt_diag || !g_mqtt_enable || !mqtt.connected()) return;
  // V2.67 F3: Grown buffer from 768 to 3072 to accommodate "_help" object.
  // Stack impact bounded; sendMqttDiag runs on the main loop task (>=8 KB stack).
  char buf[3072];
  int n = buildDiagPayloadJson(buf, sizeof(buf), true);
  if (n > 0) {
    String diagTopic = g_mqtt_topic + "/diag";
    // V2.66.2 D1: retained=true so the last snapshot survives on the broker
    // after an unexpected reboot. Essential for post-mortem of TASK_WDT/WDT
    // events where the device is offline right when diagnosis is needed.
    mqtt.publish(diagTopic.c_str(), buf, true);
  }
}

// V2.67 F3: HTTP GET /diag. Returns the same JSON payload as the MQTT /diag
// topic, including "_help". Used by Web UI Diagnostics panel. Auth-required.
void handleDiag() {
  if (!checkAuth()) return;
  static char buf[3072];
  int n = buildDiagPayloadJson(buf, sizeof(buf), true);
  if (n <= 0) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"buffer overflow\"}");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

// V2.66.3 D2: Home Assistant Auto-Discovery for diag topic entities.
// Called after MQTT (re)connect when diag+ha_en are both on, and when the
// diag toggle flips on. Publishes retained discovery configs for all 21
// diag keys so HA auto-registers them as sensors. Unpublishing happens
// separately via removeMqttDiagDiscovery() on toggle-off.
//
// Topic pattern: homeassistant/sensor/<dev_uid>_diag_<key>/config
// All entities grouped under a single HA device for clean UI.
bool sendMqttDiagDiscovery() {
  if (!mqtt.connected()) return false;
  if (!g_mqtt_diag || !g_ha_enable) return false;

  String uid = g_dev_uid;
  String stat_t = g_mqtt_topic + "/diag";
  // Reuse the same HA device object as the main discovery so diag sensors
  // appear under the same "Topband BMS (xxx)" device in HA UI.
  String dev = "\"dev\":{\"ids\":\"" + uid + "\",\"name\":\"Topband BMS (" + uid.substring(3) + ")\",\"mdl\":\"V" + String(FW_VERSION) + "\",\"mf\":\"DIY\"}";

  // Table-driven publish. Columns: key (JSON field), display name, unit,
  // device_class, state_class, icon. Empty string means "omit that attribute".
  // Strings (boot_reason, can_status, fw) have no state_class, text only.
  struct DiagEntry {
    const char *key;
    const char *name;
    const char *unit;
    const char *dev_class;
    const char *state_class;
    const char *icon;
  };
  static const DiagEntry entries[] = {
    // System
    {"fw",                 "FW Version",            "",     "",             "",                  "mdi:chip"},
    {"uptime",             "Uptime",                "s",    "duration",     "total_increasing",  ""},
    {"boot_reason",        "Boot Reason",           "",     "",             "",                  "mdi:restart"},
    {"heap_free",          "Heap Free",             "B",    "data_size",    "measurement",       ""},
    {"heap_min",           "Heap Min",              "B",    "data_size",    "measurement",       ""},
    // NVS
    {"nvs_used",           "NVS Used",              "",     "",             "measurement",       "mdi:database"},
    {"nvs_free",           "NVS Free",              "",     "",             "measurement",       "mdi:database-outline"},
    // CAN
    {"can_tx_ok",          "CAN TX OK",             "",     "",             "total_increasing",  "mdi:check-network"},
    {"can_tx_fail",        "CAN TX Fail",           "",     "",             "total_increasing",  "mdi:alert-network"},
    {"can_tx_fail_streak", "CAN TX Fail Streak",    "",     "",             "measurement",       "mdi:alert-network-outline"},
    {"can_status",         "CAN Status",            "",     "",             "",                  "mdi:can-bus"},
    // RS485 / BMS
    {"rs485_hwm",          "RS485 Stack HWM",       "B",    "data_size",    "measurement",       ""},
    {"bms_polls",          "BMS Total Polls",       "",     "",             "total_increasing",  "mdi:counter"},
    {"bms_timeouts",       "BMS Total Timeouts",    "",     "",             "total_increasing",  "mdi:timer-off"},
    // Workarounds
    {"current_holds",      "Current Holds",         "",     "",             "total_increasing",  "mdi:pause-circle"},
    {"spike_rejects",      "Spike Rejects",         "",     "",             "total_increasing",  "mdi:filter-variant-remove"},
    {"rl_rejects",         "Rate-Limit Rejects",    "",     "",             "total_increasing",  "mdi:speedometer"},
    // MQTT
    {"mqtt_fail",          "MQTT Fail Count",       "",     "",             "total_increasing",  "mdi:alert"},
    // Session
    {"session_age_d",      "Session Age",           "d",    "duration",     "measurement",       "mdi:key-chain"},
    // Debug (V2.66.2)
    {"stream_aborts",      "Stream Aborts",         "",     "",             "total_increasing",  "mdi:connection"},
    {"handler_max_ms",     "Handler Max",           "ms",   "duration",     "measurement",       "mdi:clock-alert"},
    {"loop_max_ms",        "Loop Max",              "ms",   "duration",     "measurement",       "mdi:clock-alert-outline"},
    {"wdt_warnings",       "WDT Warnings",          "",     "",             "total_increasing",  "mdi:watchdog"},
    // V2.67 F4: Counter-reset epoch as plain integer sensor. No timestamp
    // device_class here because the shared val_tpl is {{value_json.<key>}}
    // with no Jinja epoch-to-ISO conversion. Users who want a wall-clock
    // render can add a template sensor in HA. 0 means "never reset".
    {"last_reset_ts",      "Last Counter Reset",    "",     "",             "",                  "mdi:counter"},
  };
  const int entry_count = sizeof(entries) / sizeof(entries[0]);

  bool ok = true;
  char topic[128];
  for (int i = 0; i < entry_count; i++) {
    const DiagEntry &e = entries[i];
    // Build config payload as String. One per entry = 23 temporary Strings
    // during publish burst. Heap impact bounded, publishes rarely.
    String cfg;
    cfg.reserve(384);
    cfg = "{\"name\":\"";
    cfg += e.name;
    cfg += "\",\"stat_t\":\"";
    cfg += stat_t;
    cfg += "\",\"val_tpl\":\"{{value_json.";
    cfg += e.key;
    cfg += "}}\",\"uniq_id\":\"";
    cfg += uid;
    cfg += "_diag_";
    cfg += e.key;
    cfg += "\"";
    if (e.unit[0])        { cfg += ",\"unit_of_meas\":\""; cfg += e.unit;        cfg += "\""; }
    if (e.dev_class[0])   { cfg += ",\"dev_cla\":\"";     cfg += e.dev_class;   cfg += "\""; }
    if (e.state_class[0]) { cfg += ",\"stat_cla\":\"";    cfg += e.state_class; cfg += "\""; }
    if (e.icon[0])        { cfg += ",\"icon\":\"";        cfg += e.icon;        cfg += "\""; }
    cfg += ",";
    cfg += dev;
    cfg += "}";

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_diag_%s/config", uid.c_str(), e.key);
    ok &= mqtt.publish(topic, cfg.c_str(), true);
  }
  if (ok) {
    addToLog("HA: diag discovery sent (" + String(entry_count) + " entities)", false);
  } else {
    addToLog("HA: diag discovery partial failure", true);
  }
  return ok;
}

// V2.66.3 D2: Remove HA Discovery configs for diag topic.
// Called when the diag toggle flips OFF so zombie sensors disappear from HA
// instead of showing "unavailable" forever. Publishes empty retained payload
// to each discovery topic, which HA treats as "entity removed".
void removeMqttDiagDiscovery() {
  if (!mqtt.connected()) return;
  String uid = g_dev_uid;
  // Key list must mirror sendMqttDiagDiscovery() exactly. Kept as a static
  // const char* array so both functions reference the same source.
  static const char* const diag_keys[] = {
    "fw", "uptime", "boot_reason", "heap_free", "heap_min",
    "nvs_used", "nvs_free",
    "can_tx_ok", "can_tx_fail", "can_tx_fail_streak", "can_status",
    "rs485_hwm", "bms_polls", "bms_timeouts",
    "current_holds", "spike_rejects", "rl_rejects",
    "mqtt_fail", "session_age_d",
    "stream_aborts", "handler_max_ms", "loop_max_ms", "wdt_warnings",
    "last_reset_ts"   // V2.67 F4
  };
  const int count = sizeof(diag_keys) / sizeof(diag_keys[0]);
  char topic[128];
  for (int i = 0; i < count; i++) {
    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_diag_%s/config", uid.c_str(), diag_keys[i]);
    mqtt.publish(topic, "", true);  // empty retained = remove entity
  }
  addToLog("HA: diag discovery removed (" + String(count) + " entities)", false);
}

bool svcParseBmsId(int &id) {
  if (!server.hasArg("id")) return false;
  id = server.arg("id").toInt();
  if (id < 0 || id >= g_bms_count || id >= MAX_BMS) return false;
  return true;
}

float svcArgFloat(const char *name, float def) {
  if (!server.hasArg(name)) return def;
  return server.arg(name).toFloat();
}

void handleServiceBmsTimeGet() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)id, 0x4D, nullptr, 0, fr, err, true)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  if (fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"BMS code 0x" + String(fr.cid2, HEX) + "\"}");
    return;
  }
  TBDate d;
  if (!tbParseDateFromFrame(fr, d)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"date parse failed\"}");
    return;
  }
  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"date\":{\"year\":" + String(d.year) + ",\"month\":" + String(d.month) + ",\"day\":" + String(d.day) + ",\"hour\":" + String(d.hour) + ",\"minute\":" + String(d.minute) + ",\"second\":" + String(d.second) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleServiceBmsTimeSet() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  TBDate d = {};
  if (server.hasArg("year")) {
    d.year = server.arg("year").toInt();
    d.month = server.arg("month").toInt();
    d.day = server.arg("day").toInt();
    d.hour = server.arg("hour").toInt();
    d.minute = server.arg("minute").toInt();
    d.second = server.arg("second").toInt();
  } else {
    struct tm t;
    if (!getLocalTime(&t)) {
      server.send(500, "application/json", "{\"ok\":false,\"err\":\"gateway time unavailable\"}");
      return;
    }
    d.year = t.tm_year + 1900;
    d.month = t.tm_mon + 1;
    d.day = t.tm_mday;
    d.hour = t.tm_hour;
    d.minute = t.tm_min;
    d.second = t.tm_sec;
  }

  uint8_t info7[7] = {
    (uint8_t)((d.year >> 8) & 0xFF), (uint8_t)(d.year & 0xFF),
    (uint8_t)d.month, (uint8_t)d.day, (uint8_t)d.hour, (uint8_t)d.minute, (uint8_t)d.second
  };

  TBFrame fr;
  String err;
  bool ok = tbRequestFrame((uint8_t)id, 0x4E, info7, 7, fr, err, true);
  if ((!ok || fr.cid2 != 0x00)) {
    // Fallback for variants requiring BMS id in payload.
    uint8_t info8[8] = {
      (uint8_t)id,
      (uint8_t)((d.year >> 8) & 0xFF), (uint8_t)(d.year & 0xFF),
      (uint8_t)d.month, (uint8_t)d.day, (uint8_t)d.hour, (uint8_t)d.minute, (uint8_t)d.second
    };
    ok = tbRequestFrame((uint8_t)id, 0x4E, info8, 8, fr, err, true);
  }

  if (!ok) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  if (fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"BMS code 0x" + String(fr.cid2, HEX) + "\"}");
    return;
  }

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"set\":{\"year\":" + String(d.year) + ",\"month\":" + String(d.month) + ",\"day\":" + String(d.day) + ",\"hour\":" + String(d.hour) + ",\"minute\":" + String(d.minute) + ",\"second\":" + String(d.second) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleServiceAutoCfgApply() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }
  float buf_cell_mv = svcArgFloat("buf_cell_mv", 20.0f);
  float buf_pack_v = svcArgFloat("buf_pack_v", 0.30f);
  float buf_temp_c = svcArgFloat("buf_temp_c", 2.0f);
  float buf_curr_pct = svcArgFloat("buf_curr_pct", 10.0f);

  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)id, 0x47, nullptr, 0, fr, err, true) || fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter read failed\"}");
    return;
  }

  TBSysParam sp;
  if (!tbParseSystemParameterFromFrame(fr, sp)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter parse failed\"}");
    return;
  }
  if (!tbSystemParamPlausible(sp)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter out of plausible range\"}");
    return;
  }

  float safe_cell, safe_volt, tc_min, tc_max, td_min, td_max, chg_a, dis_a, cvl_v;
  tbBuildAutoCfgSuggestion(sp, buf_cell_mv, buf_pack_v, buf_temp_c, buf_curr_pct,
                           safe_cell, safe_volt, tc_min, tc_max, td_min, td_max, chg_a, dis_a, cvl_v);

  g_expert_mode = true;
  g_safe_cell = safe_cell;
  g_safe_volt = safe_volt;
  g_tc_min = tc_min;
  g_tc_max = tc_max;
  g_td_min = td_min;
  g_td_max = td_max;
  g_charge_amps = chg_a;
  g_discharge_amps = dis_a;
  g_cvl_voltage = cvl_v;

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"applied\":{";
  j += "\"s_cel\":" + String(g_safe_cell, 3) + ",";
  j += "\"s_vol\":" + String(g_safe_volt, 2) + ",";
  j += "\"t_c_min\":" + String(g_tc_min, 1) + ",";
  j += "\"t_c_max\":" + String(g_tc_max, 1) + ",";
  j += "\"t_d_min\":" + String(g_td_min, 1) + ",";
  j += "\"t_d_max\":" + String(g_td_max, 1) + ",";
  j += "\"chg\":" + String(g_charge_amps, 1) + ",";
  j += "\"dis\":" + String(g_discharge_amps, 1) + ",";
  j += "\"cvl\":" + String(g_cvl_voltage, 2);
  j += "}}";
  server.send(200, "application/json", j);
}

void handleServiceBmsDiag() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  float buf_cell_mv = svcArgFloat("buf_cell_mv", 20.0f);
  float buf_pack_v = svcArgFloat("buf_pack_v", 0.30f);
  float buf_temp_c = svcArgFloat("buf_temp_c", 2.0f);
  float buf_curr_pct = svcArgFloat("buf_curr_pct", 10.0f);

  TBFrame fr_manu, fr_date, fr_alarm, fr_sys, fr_hist, fr_aval;
  String err_manu, err_date, err_alarm, err_sys, err_hist, err_aval;

  bool manu_ok = tbRequestFrame((uint8_t)id, 0x51, nullptr, 0, fr_manu, err_manu, true) && fr_manu.cid2 == 0x00;
  bool date_ok = tbRequestFrame((uint8_t)id, 0x4D, nullptr, 0, fr_date, err_date, true) && fr_date.cid2 == 0x00;
  uint8_t alarm_req[1] = {(uint8_t)id};
  bool alarm_ok = tbRequestFrame((uint8_t)id, 0x44, alarm_req, 1, fr_alarm, err_alarm, true) && fr_alarm.cid2 == 0x00;
  bool sys_ok = tbRequestFrame((uint8_t)id, 0x47, nullptr, 0, fr_sys, err_sys, true) && fr_sys.cid2 == 0x00;
  uint8_t hist_req[2] = {0x00, (uint8_t)id};  // first event
  bool hist_ok = tbRequestFrame((uint8_t)id, 0x4B, hist_req, 2, fr_hist, err_hist, true) && fr_hist.cid2 == 0x00;
  uint8_t aval_req[1] = {(uint8_t)id};
  bool aval_ok = tbRequestFrame((uint8_t)id, 0x42, aval_req, 1, fr_aval, err_aval, true) && fr_aval.cid2 == 0x00;

  String hw = "", sw = "", mid = "";
  TBDate bms_date = {};
  TBAlarmInfoParsed ai = {};
  TBSysParam sp = {};
  TBDate hist_date = {};
  int hist_event = -1;
  uint64_t hist_status = 0;
  float hist_i = 0, hist_v = 0, hist_rem = 0;
  TBAnalogDiag ad = {};

  if (manu_ok) manu_ok = tbParseManufacturerFromFrame(fr_manu, hw, sw, mid);
  if (date_ok) date_ok = tbParseDateFromFrame(fr_date, bms_date);
  if (alarm_ok) {
    alarm_ok = tbParseAlarmInfoFromFrame(fr_alarm, ai);
    if (alarm_ok) {
      g_bms_alarm_bits[id] = ai.status_bits;
      g_bms_alarm_status_count[id] = ai.status_count;
      g_bms_alarm_last_seen[id] = millis();
    }
  }
  if (sys_ok) {
    sys_ok = tbParseSystemParameterFromFrame(fr_sys, sp);
    if (sys_ok && !tbSystemParamPlausible(sp)) { sys_ok = false; err_sys = "system parameter out of plausible range"; }
  }
  if (hist_ok) hist_ok = tbParseHistoricalFirstFromFrame(fr_hist, hist_date, hist_event, hist_status, hist_i, hist_v, hist_rem);
  if (aval_ok) aval_ok = tbParseAnalogDiagFromFrame(fr_aval, ad);

  float sug_cell = 0, sug_volt = 0, sug_tcmin = 0, sug_tcmax = 0, sug_tdmin = 0, sug_tdmax = 0, sug_chg = 0, sug_dis = 0, sug_cvl = 0;
  if (sys_ok) {
    tbBuildAutoCfgSuggestion(sp, buf_cell_mv, buf_pack_v, buf_temp_c, buf_curr_pct,
                             sug_cell, sug_volt, sug_tcmin, sug_tcmax, sug_tdmin, sug_tdmax, sug_chg, sug_dis, sug_cvl);
  }

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";

  j += "\"manufacturer\":{\"ok\":" + String(manu_ok ? "true" : "false");
  if (manu_ok) {
    j += ",\"hw\":\"" + jsonEscape(hw) + "\",\"sw\":\"" + jsonEscape(sw) + "\",\"id\":\"" + jsonEscape(mid) + "\"";
  } else {
    j += ",\"err\":\"" + jsonEscape(err_manu) + "\"";
  }
  j += "},";

  j += "\"date\":{\"ok\":" + String(date_ok ? "true" : "false");
  if (date_ok) {
    j += ",\"year\":" + String(bms_date.year) + ",\"month\":" + String(bms_date.month) + ",\"day\":" + String(bms_date.day) + ",\"hour\":" + String(bms_date.hour) + ",\"minute\":" + String(bms_date.minute) + ",\"second\":" + String(bms_date.second);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_date) + "\"";
  }
  j += "},";

  j += "\"analog\":{\"ok\":" + String(aval_ok ? "true" : "false");
  if (aval_ok) {
    j += ",\"id\":" + String(ad.id);
    j += ",\"cells\":" + String(ad.cell_count);
    j += ",\"temps\":" + String(ad.temp_count);
    j += ",\"pack_v\":" + String(ad.pack_v, 2);
    j += ",\"current\":" + String(ad.current_a, 2);
    j += ",\"soc\":" + String(ad.soc);
    j += ",\"soh\":" + String(ad.soh);
    j += ",\"min_cell\":" + String(ad.min_cell_v, 3);
    j += ",\"max_cell\":" + String(ad.max_cell_v, 3);
    j += ",\"rem_ah\":" + String(ad.rem_ah, 2);
    j += ",\"full_ah\":" + String(ad.full_ah, 2);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_aval) + "\"";
  }
  j += "},";

  j += "\"alarm\":{\"ok\":" + String(alarm_ok ? "true" : "false");
  if (alarm_ok) {
    j += ",\"status_count\":" + String(ai.status_count);
    j += ",\"status_hex\":\"" + u64Hex(ai.status_bits) + "\"";
    j += ",\"critical\":" + String(tbHasCriticalAlarm(ai.status_bits) ? "true" : "false");
    j += ",\"active\":[";
    bool first = true;
    for (uint8_t b = 0; b < 64; b++) {
      if (ai.status_bits & (1ULL << b)) {
        if (!first) j += ",";
        j += "{\"bit\":" + String(b) + ",\"name\":\"" + jsonEscape(String(tbAlarmBitName(b))) + "\"}";
        first = false;
      }
    }
    j += "]";
  } else {
    j += ",\"err\":\"" + jsonEscape(err_alarm) + "\"";
  }
  j += "},";

  j += "\"system\":{\"ok\":" + String(sys_ok ? "true" : "false");
  if (sys_ok) {
    j += ",\"cell_high_v\":" + String(sp.cell_high_v, 3);
    j += ",\"cell_low_v\":" + String(sp.cell_low_v, 3);
    j += ",\"cell_under_v\":" + String(sp.cell_under_v, 3);
    j += ",\"module_high_v\":" + String(sp.module_high_v, 2);
    j += ",\"module_low_v\":" + String(sp.module_low_v, 2);
    j += ",\"module_under_v\":" + String(sp.module_under_v, 2);
    j += ",\"charge_high_t\":" + String(sp.charge_high_t, 1);
    j += ",\"charge_low_t\":" + String(sp.charge_low_t, 1);
    j += ",\"discharge_high_t\":" + String(sp.discharge_high_t, 1);
    j += ",\"discharge_low_t\":" + String(sp.discharge_low_t, 1);
    j += ",\"charge_current_max_a\":" + String(sp.charge_current_max_a, 1);
    j += ",\"discharge_current_max_a\":" + String(sp.discharge_current_max_a, 1);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_sys) + "\"";
  }
  j += "},";

  j += "\"historical\":{\"ok\":" + String(hist_ok ? "true" : "false");
  if (hist_ok) {
    j += ",\"year\":" + String(hist_date.year) + ",\"month\":" + String(hist_date.month) + ",\"day\":" + String(hist_date.day) + ",\"hour\":" + String(hist_date.hour) + ",\"minute\":" + String(hist_date.minute) + ",\"second\":" + String(hist_date.second);
    j += ",\"event\":" + String(hist_event);
    j += ",\"status_hex\":\"" + u64Hex(hist_status) + "\"";
    j += ",\"pack_v\":" + String(hist_v, 2);
    j += ",\"current\":" + String(hist_i, 2);
    j += ",\"remain_ah\":" + String(hist_rem, 2);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_hist) + "\"";
  }
  j += "},";

  j += "\"autocfg\":{\"ok\":" + String(sys_ok ? "true" : "false");
  if (sys_ok) {
    j += ",\"buffer\":{\"cell_mv\":" + String(buf_cell_mv, 1) + ",\"pack_v\":" + String(buf_pack_v, 2) + ",\"temp_c\":" + String(buf_temp_c, 1) + ",\"curr_pct\":" + String(buf_curr_pct, 1) + "}";
    j += ",\"suggest\":{\"s_cel\":" + String(sug_cell, 3) + ",\"s_vol\":" + String(sug_volt, 2) + ",\"t_c_min\":" + String(sug_tcmin, 1) + ",\"t_c_max\":" + String(sug_tcmax, 1) + ",\"t_d_min\":" + String(sug_tdmin, 1) + ",\"t_d_max\":" + String(sug_tdmax, 1) + ",\"chg\":" + String(sug_chg, 1) + ",\"dis\":" + String(sug_dis, 1) + ",\"cvl\":" + String(sug_cvl, 2) + "}";
  }
  j += "}";

  j += "}";
  server.send(200, "application/json", j);
}

// ==========================================
// WEB ENDPOINT HANDLERS
// All handlers call checkAuth() first (cookie session check, returns 401/302
// if not authenticated). Settings are saved to NVS
// and the device reboots. handleData() uses chunked HTTP streaming
// with a mutex snapshot pattern to avoid blocking Core 0.
// ==========================================
void handleSim() { if (!checkAuth()) return; if (server.hasArg("act")) simulation_active = (server.arg("act") == "1"); server.send(200); }

// ==========================================
// V2.65.1 D2: RS485 FRAME SPY
// spyCaptureFrame() is called from the top of parseTopband() whenever the
// spy is active, capturing the raw hex response into the ring buffer.
// handleSpy() toggles capture on/off via /spy?act=1 or /spy?act=0.
// Auto-off is enforced in loop() by checking g_rs485_spy_deadline.
// ==========================================
void spyCaptureFrame(const String &hex, uint8_t addr) {
  if (!g_rs485_spy_enable) return;
  if (!spyMutex) return;
  if (xSemaphoreTake(spyMutex, pdMS_TO_TICKS(20))) {
    SpyEntry &e = g_spy_ring[g_spy_head];
    e.ts = millis();
    e.addr = addr;
    size_t n = hex.length();
    if (n >= SPY_FRAME_MAX) n = SPY_FRAME_MAX - 1;
    memcpy(e.hex, hex.c_str(), n);
    e.hex[n] = '\0';
    e.len = (uint16_t)n;
    g_spy_head = (g_spy_head + 1) % SPY_RING_SIZE;
    if (g_spy_count < SPY_RING_SIZE) g_spy_count++;
    g_spy_capture_count++;
    xSemaphoreGive(spyMutex);
  }
}

void handleSpy() {
  if (!checkAuth()) return;
  if (server.hasArg("act")) {
    bool want = (server.arg("act") == "1");
    if (want) {
      g_rs485_spy_enable = true;
      g_rs485_spy_deadline = millis() + (5UL * 60UL * 1000UL);  // auto-off in 5 min
      // Clear ring on start so we capture only fresh frames
      if (spyMutex && xSemaphoreTake(spyMutex, pdMS_TO_TICKS(50))) {
        g_spy_head = 0; g_spy_count = 0; g_spy_capture_count = 0;
        xSemaphoreGive(spyMutex);
      }
      addToLog("Frame Spy: enabled (5 min window)", false);
    } else {
      g_rs485_spy_enable = false;
      g_rs485_spy_deadline = 0;
      addToLog("Frame Spy: disabled", false);
    }
  }
  // Emit current state + frames as JSON
  String j = "{\"active\":";
  j += (g_rs485_spy_enable ? "true" : "false");
  j += ",\"captured\":" + String(g_spy_capture_count);
  j += ",\"remain_ms\":" + String(g_rs485_spy_deadline > millis() ? (g_rs485_spy_deadline - millis()) : 0UL);
  j += ",\"frames\":[";
  if (spyMutex && xSemaphoreTake(spyMutex, pdMS_TO_TICKS(100))) {
    bool first = true;
    for (uint8_t i = 0; i < g_spy_count; i++) {
      // Emit newest-first
      uint8_t idx = (g_spy_head + SPY_RING_SIZE - 1 - i) % SPY_RING_SIZE;
      SpyEntry &e = g_spy_ring[idx];
      if (!first) j += ",";
      first = false;
      j += "{\"ts\":" + String(e.ts) + ",\"a\":" + String(e.addr) + ",\"hex\":\"" + String(e.hex) + "\"}";
    }
    xSemaphoreGive(spyMutex);
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleChartCfg() {
  if (!checkAuth(true)) return;
  if (server.hasArg("c1")) { g_chart1 = constrain(server.arg("c1").toInt(), 0, 3); }
  if (server.hasArg("c2")) { g_chart2 = constrain(server.arg("c2").toInt(), 0, 3); }
  preferences.begin("gateway", false);
  preferences.putInt("chart1", g_chart1); preferences.putInt("chart2", g_chart2);
  preferences.end();
  server.send(200, "application/json", "{\"ok\":true}");
}

// V2.61: GET /alerts -> JSON array of last 25 alerts (newest first)
void handleAlertsGet() {
  if (!checkAuth()) return;
  String j = "["; bool first = true;
  // Walk back from head (newest last written) to oldest still in buffer
  for (int i = 1; i <= g_alert_count; i++) {
    int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
    if (!first) j += ",";
    first = false;
    // Escape message for JSON (backslash, quote, control chars)
    String m; m.reserve(ALERT_MSG_LEN + 16);
    const char* p = g_alerts[idx].msg;
    for (int k = 0; k < ALERT_MSG_LEN && p[k]; k++) {
      char ch = p[k];
      if (ch == '"' || ch == '\\') { m += '\\'; m += ch; }
      else if (ch == '\n') m += " ";
      else if ((uint8_t)ch < 0x20) m += ' ';
      else m += ch;
    }
    j += "{\"ts\":" + String(g_alerts[idx].ts);
    j += ",\"up\":" + String(g_alerts[idx].uptime_s);
    j += ",\"sev\":" + String((int)g_alerts[idx].sev);
    j += ",\"msg\":\"" + m + "\"}";
  }
  j += "]";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// V2.61: POST/GET /alerts/clear -> wipe RAM + NVS
void handleAlertsClear() {
  if (!checkAuth()) return;
  clearAlertsAll();
  server.send(200, "application/json", "{\"ok\":true}");
}

// V2.67 F4: Zero all in-RAM counters and stamp last_reset_ts.
// Called from (a) manual reset via /svc/reset_counters and (b) 7-day rollover
// in loop(). Writes NVS once per call, not per rollover-check tick.
//
// Concurrency: the per-BMS counters (polls, timeouts, errors, spikes,
// current_holds) are written by the rs485 task on Core 0. uint32_t stores
// are word-atomic on Xtensa ESP32, so a worst-case race produces at most one
// stray leftover increment (+1) that becomes visible in the next diag
// publish. Not a correctness issue for a reset-to-zero operation and
// matches the existing cross-core read pattern the code already accepts.
//
// Preserves (per spec): heap_min, handler_max_ms, loop_max_ms, rs485_hwm,
// wdt_warnings, can_tx_fail_streak. Everything else in the F4 reset list
// goes to 0. Reboot resets counters too but does NOT call this; the
// reboot path leaves g_last_reset_ts untouched.
void resetCountersNow(uint32_t new_reset_ts) {
  // Aggregate counters (Core 1 single-writer except spike/current which are
  // rolled up from per-BMS; safe to zero from here).
  g_can_tx_ok     = 0;
  g_can_tx_fail   = 0;
  g_stream_aborts = 0;
  g_rl_rejects    = 0;
  mqtt_fail_count = 0;
  // Note: g_can_tx_fail_streak, g_handler_max_ms, g_loop_max_ms,
  // g_wdt_warnings, heap_min are intentionally preserved per F4 spec.

  // Per-BMS counters. polls and current_holds are aggregated into the
  // diag payload; reset the per-pack fields so both per-pack and aggregate
  // views go to zero.
  for (int i = 0; i < MAX_BMS; i++) {
    g_bms_stats[i].polls         = 0;
    g_bms_stats[i].ok            = 0;
    g_bms_stats[i].timeouts      = 0;
    g_bms_stats[i].errors        = 0;
    g_bms_stats[i].spikes        = 0;
    g_bms_stats[i].current_holds = 0;
  }

  g_last_reset_ts = new_reset_ts;

  // Persist just the timestamp. Counters themselves are RAM-only by design.
  preferences.begin("gateway", false);
  preferences.putULong("lrst_ts", g_last_reset_ts);
  preferences.end();

  addToLog("Counters reset (ts=" + String(g_last_reset_ts) + ")", false);
}

// V2.67 F4: POST/GET /svc/reset_counters -> manual counter reset.
// Defers to resetCountersNow with a current timestamp. If NTP is not yet
// synced, stamp with 0 so the rollover logic will re-anchor on first sync.
// Returns JSON with the stamped ts so the UI can refresh.
void handleResetCounters() {
  if (!checkAuth()) return;
  time_t now_t; time(&now_t);
  uint32_t ts = ((uint32_t)now_t > NTP_SYNC_EPOCH_MIN) ? (uint32_t)now_t : 0;
  resetCountersNow(ts);
  String j = "{\"ok\":true,\"last_reset_ts\":" + String(g_last_reset_ts) + "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void handleSave() {
  if (!checkAuth()) return;
  saveHistory(); delay(100);
  if (g_alerts_dirty) saveAlertsToNvs();   // V2.61: flush alerts before reboot
  preferences.begin("gateway", false); 
  if (server.hasArg("cnt")) g_bms_count = server.arg("cnt").toInt();
  if (server.hasArg("cells")) g_force_cell_count = server.arg("cells").toInt();
  if (server.hasArg("soc_mode")) g_soc_source_mode = server.arg("soc_mode").toInt();
  if (server.hasArg("chart1")) g_chart1 = constrain(server.arg("chart1").toInt(), 0, 3);
  if (server.hasArg("chart2")) g_chart2 = constrain(server.arg("chart2").toInt(), 0, 3);
  if (g_soc_source_mode < 0 || g_soc_source_mode > 2) g_soc_source_mode = 2;
  g_maint_charge_mode = server.hasArg("maint_en");
  if (server.hasArg("maint_v")) g_maint_target_v = server.arg("maint_v").toFloat();
  if (g_maint_target_v < 40.0f) g_maint_target_v = 40.0f;
  if (g_maint_target_v > 65.0f) g_maint_target_v = 65.0f;
  g_auto_balance_enable = server.hasArg("ab_en");
  g_easy_mode = server.hasArg("easy");
  g_expert_mode = server.hasArg("exp");
  if (g_easy_mode) g_expert_mode = false;
  g_setup_done = true;
  if(g_expert_mode) {
      if(server.hasArg("chg")) g_charge_amps = server.arg("chg").toFloat();
      if(server.hasArg("dis")) g_discharge_amps = server.arg("dis").toFloat();
      if(server.hasArg("cvl")) g_cvl_voltage = server.arg("cvl").toFloat();
      if(server.hasArg("s_vol")) g_safe_volt = server.arg("s_vol").toFloat();
      if(server.hasArg("s_cel")) g_safe_cell = server.arg("s_cel").toFloat();
      if(server.hasArg("t_c_min")) g_tc_min = server.arg("t_c_min").toFloat();
      if(server.hasArg("t_c_max")) g_tc_max = server.arg("t_c_max").toFloat();
      if(server.hasArg("t_d_min")) g_td_min = server.arg("t_d_min").toFloat();
      if(server.hasArg("t_d_max")) g_td_max = server.arg("t_d_max").toFloat();
      if(server.hasArg("t_mode")) g_temp_mode = server.arg("t_mode").toInt();
      if(server.hasArg("s_drift")) g_safe_drift = server.arg("s_drift").toFloat();
      // V2.66 M4: Spike filter thresholds (expert-only)
      if(server.hasArg("sp_v")) g_spike_volt = server.arg("sp_v").toFloat();
      if(server.hasArg("sp_a")) g_spike_cur  = server.arg("sp_a").toFloat();
      if(server.hasArg("sp_s")) g_spike_soc  = server.arg("sp_s").toInt();
      // Clamp to sane ranges so user can't disable the filter accidentally
      if (g_spike_volt < 0.5f)  g_spike_volt = 0.5f;   if (g_spike_volt > 20.0f)  g_spike_volt = 20.0f;
      if (g_spike_cur  < 10.0f) g_spike_cur  = 10.0f;  if (g_spike_cur  > 500.0f) g_spike_cur  = 500.0f;
      if (g_spike_soc  < 5)     g_spike_soc  = 5;      if (g_spike_soc  > 50)     g_spike_soc  = 50;
  }  // Defaults when expert mode is off
  if (!g_expert_mode) {
    g_easy_mode = false;
    g_charge_amps = 30.0f;
    g_discharge_amps = 30.0f;
    g_cvl_voltage = 52.5f;
    g_safe_volt = 53.25f;
    g_safe_cell = 3.55f;
    g_safe_drift = 0.20f;
    g_tc_min = 5.0f;
    g_tc_max = 50.0f;
    g_td_min = -20.0f;
    g_td_max = 60.0f;
    g_maint_target_v = 51.0f;
  }
  g_sd_enable = server.hasArg("sd_en"); g_sd_spy = server.hasArg("spy"); g_serial_debug = server.hasArg("debug");
  preferences.putBool("sd_en", g_sd_enable); preferences.putBool("debug", g_serial_debug); preferences.putBool("spy", g_sd_spy);
  if(server.hasArg("theme")) g_theme_id = server.arg("theme").toInt();
  if(server.hasArg("can_proto")) g_can_protocol = server.arg("can_proto").toInt(); 
  g_mqtt_enable = server.hasArg("mq_en"); g_victron_enable = server.hasArg("vic_en");
  g_mqtt_full = server.hasArg("mq_full"); g_ha_enable = server.hasArg("ha_en"); 
  // V2.66.3 D2: detect toggle edge so we can publish/remove HA diag discovery.
  bool prev_mq_diag = g_mqtt_diag;
  g_mqtt_diag = server.hasArg("mq_diag");  // V2.66.2 D1
  if (g_mqtt_diag != prev_mq_diag && g_ha_enable && mqtt.connected()) {
    if (g_mqtt_diag) sendMqttDiagDiscovery();
    else             removeMqttDiagDiscovery();
  }
  // V2.67 F1: Tiered MQTT level (L1/L2/L3). On change, re-publish the full
  // HA discovery set so entities appear/disappear as appropriate. Empty
  // retained payloads remove entities cleanly on downgrade.
  uint8_t prev_mq_level = g_mqtt_level;
  if (server.hasArg("mq_level")) {
    int lvl = server.arg("mq_level").toInt();
    if (lvl < 0) lvl = 0; if (lvl > 2) lvl = 2;
    g_mqtt_level = (uint8_t)lvl;
  }
  if (g_mqtt_level != prev_mq_level && g_ha_enable && mqtt.connected()) {
    sendHaDiscovery();
  }
  if (server.hasArg("mq_ip")) g_mqtt_server = server.arg("mq_ip");
  if (server.hasArg("mq_pt")) g_mqtt_port = server.arg("mq_pt").toInt();
  if (server.hasArg("mq_us")) g_mqtt_user = server.arg("mq_us");
  if (server.hasArg("mq_pw") && server.arg("mq_pw").length() > 0) g_mqtt_pass = server.arg("mq_pw");
  if (server.hasArg("mq_top") && server.arg("mq_top").length() > 0) g_mqtt_topic = server.arg("mq_top");
  if (server.hasArg("ntp_svr")) g_ntp_server = server.arg("ntp_svr");
  if (server.hasArg("tz_off")) g_timezone_offset = server.arg("tz_off").toInt();
  preferences.putInt("cnt", g_bms_count); preferences.putInt("cells", g_force_cell_count);
  preferences.putFloat("chg", g_charge_amps); preferences.putFloat("dis", g_discharge_amps);
  preferences.putBool("exp", g_expert_mode); preferences.putFloat("cvl", g_cvl_voltage);
  preferences.putInt("soc_mode", g_soc_source_mode);
  preferences.putInt("chart1", g_chart1);
  preferences.putInt("chart2", g_chart2);
  preferences.putBool("maint_en", g_maint_charge_mode);
  preferences.putFloat("maint_v", g_maint_target_v);
  preferences.putBool("ab_en", g_auto_balance_enable);
  preferences.putBool("easy", g_easy_mode); preferences.putBool("setupd", g_setup_done);
  preferences.putFloat("s_vol", g_safe_volt); preferences.putFloat("s_cel", g_safe_cell);
  preferences.putFloat("t_c_min", g_tc_min); preferences.putFloat("t_c_max", g_tc_max);
  preferences.putFloat("t_d_min", g_td_min); preferences.putFloat("t_d_max", g_td_max);
  preferences.putInt("t_mode", g_temp_mode); preferences.putFloat("s_drift", g_safe_drift);
  // V2.66 M4: Persist spike filter thresholds
  preferences.putFloat("sp_v", g_spike_volt);
  preferences.putFloat("sp_a", g_spike_cur);
  preferences.putInt("sp_s", g_spike_soc);
  preferences.putBool("mq_en", g_mqtt_enable); preferences.putBool("mq_full", g_mqtt_full); preferences.putBool("vic_en", g_victron_enable); 
  preferences.putBool("mq_diag", g_mqtt_diag);  // V2.66.2 D1
  preferences.putInt("mq_level", (int)g_mqtt_level);  // V2.67 F1
  preferences.putBool("ha_en", g_ha_enable); preferences.putInt("theme", g_theme_id); preferences.putInt("can_proto", g_can_protocol);
  preferences.putString("mq_ip", g_mqtt_server); preferences.putInt("mq_pt", g_mqtt_port);
  preferences.putString("mq_us", g_mqtt_user); preferences.putString("mq_pw", g_mqtt_pass);
  preferences.putString("mq_top", g_mqtt_topic);
  preferences.putString("ntp", g_ntp_server); preferences.putInt("tz", g_timezone_offset);
  // Board & pin settings
  if (server.hasArg("board_type")) {
    int new_board = constrain(server.arg("board_type").toInt(), 0, 2);
    g_board_type = new_board;
    preferences.putInt("board_type", g_board_type);
    if (new_board < 2) {
      applyBoardPreset(new_board);
    } else {
      // Custom: read pin values from form
      if (server.hasArg("p_rs_tx")) g_pin_rs485_tx = constrain(server.arg("p_rs_tx").toInt(), 0, 48);
      if (server.hasArg("p_rs_rx")) g_pin_rs485_rx = constrain(server.arg("p_rs_rx").toInt(), 0, 48);
      if (server.hasArg("p_rs_dir")) g_pin_rs485_dir = constrain(server.arg("p_rs_dir").toInt(), -1, 48);
      if (server.hasArg("p_can_tx")) g_pin_can_tx = constrain(server.arg("p_can_tx").toInt(), 0, 48);
      if (server.hasArg("p_can_rx")) g_pin_can_rx = constrain(server.arg("p_can_rx").toInt(), 0, 48);
      if (server.hasArg("p_led")) g_pin_led = constrain(server.arg("p_led").toInt(), -1, 48);
    }
    preferences.putInt("p_rs_tx", g_pin_rs485_tx); preferences.putInt("p_rs_rx", g_pin_rs485_rx);
    preferences.putInt("p_rs_dir", g_pin_rs485_dir); preferences.putInt("p_can_tx", g_pin_can_tx);
    preferences.putInt("p_can_rx", g_pin_can_rx); preferences.putInt("p_led", g_pin_led);
  }
  // Auth settings
  g_auth_enable = server.hasArg("auth_en");
  if (server.hasArg("auth_user") && server.arg("auth_user").length() > 0) g_auth_user = server.arg("auth_user");
  bool auth_credentials_changed = false;
  if (server.hasArg("auth_pw") && server.arg("auth_pw").length() > 0) {
    g_auth_hash = sha256Hash(server.arg("auth_pw"));
    auth_credentials_changed = true;
  }
  if (!g_auth_enable && g_auth_hash.length() > 0) {
    g_auth_hash = "";
    auth_credentials_changed = true;
  }
  preferences.putBool("auth_en", g_auth_enable);
  preferences.putString("auth_user", g_auth_user);
  preferences.putString("auth_hash", g_auth_hash);
  preferences.end();
  // V2.64: Rotate session token if auth credentials changed. Invalidates all existing sessions.
  if (auth_credentials_changed) {
    rotateSessionToken();
    addToLog("Auth: credentials changed, all sessions invalidated", false);
  }
  delay(100);
  String rp = R"RP(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rebooting</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:40px;text-align:center;max-width:400px;width:90%}
h2{font-size:16px;font-weight:500;margin-bottom:8px}
.s{color:var(--tx2);font-size:13px;margin-bottom:16px}
.bar{width:100%;height:4px;background:rgba(128,128,128,0.2);border-radius:2px;overflow:hidden}
.bar div{height:100%;background:var(--acc);border-radius:2px;width:0%;transition:width 0.5s}
.c{font-size:12px;color:var(--tx2);margin-top:12px}
</style></head><body><div class="b"><h2>Settings saved</h2><div class="s">Device is rebooting...</div><div class="bar"><div id="br"></div></div><div class="c" id="ct">Waiting for reboot...</div></div>
<script>var n=0,b=document.getElementById('br'),c=document.getElementById('ct');var iv=setInterval(function(){n++;b.style.width=Math.min(n*8,95)+'%';c.textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);b.style.width='100%';c.textContent='Loading dashboard...';location.href='/'}).catch(function(){})},3000)</script></body></html>)RP";
  server.send(200, "text/html", rp);
  g_restart_at = millis() + 500;   // V2.65 M2: deferred restart, non-blocking
}
// /data endpoint: JSON with victron, config, status, energy, history, bms arrays.
// Takes a fast snapshot (<1ms) of shared data under dataMutex, then streams
// without holding the lock. Accepts ?c1=N&c2=N for live chart type switching.
// ==========================================
// V2.66 M6: /data rate-limit (token bucket per client IP)
// Protects the endpoint against runaway clients without affecting the
// normal 2.5s dashboard poll. 4 burst tokens, refill 2/sec. Unknown IPs
// get an LRU slot. All access on Core 1, no mutex needed.
// ==========================================
#define RL_SLOTS     8
#define RL_CAPACITY  4.0f
#define RL_REFILL    2.0f      // tokens per second

struct RateLimitSlot {
  uint32_t ip;                 // IPv4 packed, 0 = free slot
  float    tokens;
  unsigned long last_refill_ms;
  unsigned long last_used_ms;
};
RateLimitSlot g_rl_slots[RL_SLOTS] = {0};
uint32_t g_rl_rejects = 0;     // lifetime 429 count (exposed in /data)

// Returns true if the request should proceed; false if rate-limited.
bool rateLimitCheck(uint32_t ip) {
  unsigned long now = millis();
  int found = -1;
  int lru = 0;
  unsigned long lru_age = 0;

  // Find existing slot for this IP, or identify LRU candidate.
  // V2.66.1 H1: Do NOT break on first empty slot - that would cause a later
  // slot holding this IP to be missed, and the client would get a fresh
  // 4-token bucket on every request. Remember the empty slot as fallback
  // and keep scanning for an existing match.
  int empty_slot = -1;
  for (int i = 0; i < RL_SLOTS; i++) {
    if (g_rl_slots[i].ip == ip && ip != 0) { found = i; break; }
    if (g_rl_slots[i].ip == 0) {
      if (empty_slot < 0) empty_slot = i;   // remember first empty, keep scanning
      continue;
    }
    unsigned long age = now - g_rl_slots[i].last_used_ms;
    if (age > lru_age) { lru = i; lru_age = age; }
  }
  // Prefer empty slot over LRU eviction when no existing match
  if (found < 0 && empty_slot >= 0) lru = empty_slot;

  RateLimitSlot &s = (found >= 0) ? g_rl_slots[found] : g_rl_slots[lru];
  if (found < 0) {
    // Fresh slot: initialize at full capacity minus one (this request)
    s.ip = ip;
    s.tokens = RL_CAPACITY;
    s.last_refill_ms = now;
  } else {
    // Refill based on elapsed time since last refill
    float elapsed_s = (now - s.last_refill_ms) / 1000.0f;
    s.tokens += elapsed_s * RL_REFILL;
    if (s.tokens > RL_CAPACITY) s.tokens = RL_CAPACITY;
    s.last_refill_ms = now;
  }
  s.last_used_ms = now;
  if (s.tokens >= 1.0f) {
    s.tokens -= 1.0f;
    return true;
  }
  g_rl_rejects++;
  return false;
}

// V2.66.1 C3: streaming health gate. Called between chunks in large responses.
// Returns false if the TCP client disconnected; caller should return early.
// Also resets the task watchdog so that long streams survive WDT_TIMEOUT even
// under disk/NVS/heap pressure elsewhere in the iteration.
// V2.66.2 C3+: sets global g_stream_abort so that inner-lambda returns can
// propagate up past the immediate caller to the outermost handler frame.
static inline bool streamAlive() {
  if (!server.client().connected()) {
    g_stream_abort = true;   // V2.66.2 C3+
    g_stream_aborts++;       // V2.66.2 D1
    return false;
  }
  esp_task_wdt_reset();
  return true;
}

void handleData() {
  if (!checkAuth(true)) return;
  // V2.66.2 C3+: reset abort flag fresh for this request
  g_stream_abort = false;
  // V2.66.2 D1: measure handler runtime for debug telemetry
  unsigned long handler_start_ms = millis();
  // V2.66 M6: rate-limit check. Uses client IP as bucket key.
  uint32_t client_ip = (uint32_t)server.client().remoteIP();
  if (!rateLimitCheck(client_ip)) {
    server.sendHeader("Retry-After", "1");
    server.send(429, "application/json", "{\"error\":\"rate_limited\"}");
    return;
  }
  // Snapshot shared data under mutex (fast, <1ms)
  VictronType vSnap;
  BMSData bSnap[MAX_BMS];
  uint8_t alarmSnap = alarm_flags;
  String alarmMsg = sys_error_msg;
  int bmsCnt = g_bms_count;
  // V2.66.1 C1: check Take result. On timeout, 503 instead of building JSON
  // with unlocked reads and Give-on-unowned-mutex.
  bool locked = (dataMutex != NULL) && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500));
  if (!locked) {
    addToLog("WARN: handleData mutex timeout, 503", true);
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }
  memcpy(&vSnap, &victronData, sizeof(VictronType));
  memcpy(bSnap, bms, sizeof(BMSData) * MAX_BMS);
  alarmSnap = alarm_flags;
  alarmMsg = sys_error_msg;
  xSemaphoreGive(dataMutex);  // V2.66.1 C1: locked==true confirmed above
  // Now stream without holding mutex
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  
  // Helper: build + send a chunk, then free
  String c;
  c.reserve(512);

  // Victron section
  c = "{ \"victron\": {";
  c += "\"active\":" + String(vSnap.activePacks) + ",\"total_amps\":" + String(vSnap.totalCurrent, 1);
  c += ",\"total_power\":" + String(vSnap.totalPower, 0) + ",\"avg_soc\":" + String(vSnap.avgSOC, 1);
  c += ",\"avg_soh\":" + String(vSnap.avgSOH, 1) + ",\"voltage\":" + String(vSnap.avgVoltage, 2);
  c += ",\"rem_cap\":" + String(vSnap.remainCapacity, 0) + ",\"total_cap\":" + String(vSnap.totalCapacity, 0);
  c += ",\"avg_temp\":" + String(vSnap.avgTemp, 1);
  c += ",\"max_chg\":" + String(vSnap.maxChargeCurrent, 1) + ",\"max_dis\":" + String(vSnap.maxDischargeCurrent, 1);
  c += ",\"dyn_cvl\":" + String(g_runtime_cvl > 0.1f ? g_runtime_cvl : g_cvl_voltage, 2);
  c += ",\"proto_ccl\":" + String(g_runtime_proto_ccl_cap, 1) + ",\"proto_dcl\":" + String(g_runtime_proto_dcl_cap, 1);
  c += ",\"proto_uv_hit\":" + String(g_runtime_proto_uv_hit ? "true" : "false");
  c += ",\"maint_en\":" + String(g_maint_charge_mode ? "true" : "false") + ",\"maint_v\":" + String(g_maint_target_v, 2);
  c += ",\"autobal_due\":" + String(g_runtime_auto_balance_due ? "true" : "false");
  float conf_chg = bmsCnt * g_charge_amps;
  float conf_dis = bmsCnt * g_discharge_amps;
  c += ",\"cfg_chg\":" + String(conf_chg, 1) + ",\"cfg_dis\":" + String(conf_dis, 1);
  c += "}, ";
  server.sendContent(c);

  // Config section
  c = "\"config\": {";
  c += "\"cnt\":" + String(bmsCnt) + ",\"cells\":" + String(g_force_cell_count);
  c += ",\"chg\":" + String(g_charge_amps) + ",\"dis\":" + String(g_discharge_amps);
  c += ",\"cvl\":" + String(g_cvl_voltage) + ",\"exp\":" + String(g_expert_mode ? "true" : "false");
  c += ",\"s_vol\":" + String(g_safe_volt) + ",\"s_cel\":" + String(g_safe_cell);
  c += ",\"t_c_min\":" + String(g_tc_min) + ",\"t_c_max\":" + String(g_tc_max);
  c += ",\"t_d_min\":" + String(g_td_min) + ",\"t_d_max\":" + String(g_td_max);
  c += ",\"t_mode\":" + String(g_temp_mode) + ",\"s_drift\":" + String(g_safe_drift);
  c += ",\"sp_v\":" + String(g_spike_volt) + ",\"sp_a\":" + String(g_spike_cur) + ",\"sp_s\":" + String(g_spike_soc);
  c += ",\"debug\":" + String(g_serial_debug ? "true" : "false") + ",\"spy\":" + String(g_sd_spy ? "true" : "false");
  c += ",\"can_proto\":" + String(g_can_protocol) + ",\"theme\":" + String(g_theme_id);
  c += ",\"ntp\":\"" + g_ntp_server + "\",\"tz\":" + String(g_timezone_offset);
  c += ",\"sd\":" + String(g_sd_enable ? "true" : "false") + ",\"vic_en\":" + String(g_victron_enable ? "true" : "false");
  c += ",\"mq_en\":" + String(g_mqtt_enable ? "true" : "false") + ",\"mq_full\":" + String(g_mqtt_full ? "true" : "false");
  c += ",\"mq_diag\":" + String(g_mqtt_diag ? "true" : "false");  // V2.66.2 D1
  c += ",\"mq_level\":" + String((int)g_mqtt_level);  // V2.67 F1
  c += ",\"ha_en\":" + String(g_ha_enable ? "true" : "false") + ",\"mq_ip\":\"" + g_mqtt_server + "\"";
  c += ",\"mq_pt\":" + String(g_mqtt_port) + ",\"mq_us\":\"" + g_mqtt_user + "\"";
  c += ",\"mq_pw_set\":" + String(g_mqtt_pass.length() > 0 ? "true" : "false");
  c += ",\"mq_top\":\"" + g_mqtt_topic + "\"";
  c += ",\"mq_connected\":" + String(mqtt.connected() ? "true" : "false");
  c += ",\"fw\":\"" + String(FW_VERSION) + "\",\"dev_uid\":\"" + g_dev_uid + "\"";
  // V2.67 F2: About-section static fields. Stamped at compile time or read once from hardware.
  c += ",\"git_sha\":\"" GIT_SHA "\"";
  c += ",\"compile_date\":\"" __DATE__ " " __TIME__ "\"";
  c += ",\"sdk\":\"" + String(ESP.getSdkVersion()) + "\"";
  c += ",\"mac\":\"" + WiFi.macAddress() + "\"";
  c += ",\"psram_free\":" + String(ESP.getFreePsram());
  c += ",\"psram_size\":" + String(ESP.getPsramSize());
  c += ",\"github\":\"" + String(GITHUB_URL) + "\"";
  c += ",\"license\":\"" + String(LICENSE_STR) + "\"";
  c += ",\"soc_mode\":" + String(g_soc_source_mode);
  c += ",\"maint_en\":" + String(g_maint_charge_mode ? "true" : "false") + ",\"maint_v\":" + String(g_maint_target_v, 2);
  c += ",\"ab_en\":" + String(g_auto_balance_enable ? "true" : "false") + ",\"ab_last\":" + String(g_auto_balance_last_ts);
  c += ",\"easy\":" + String(g_easy_mode ? "true" : "false") + ",\"setup_done\":" + String(g_setup_done ? "true" : "false");
  c += ",\"chart1\":" + String(g_chart1) + ",\"chart2\":" + String(g_chart2);
  c += ",\"auth_en\":" + String(g_auth_enable ? "true" : "false") + ",\"auth_user\":\"" + jsonEscape(g_auth_user) + "\"";
  c += ",\"board_type\":" + String(g_board_type);
  c += ",\"pins\":{\"rs_tx\":" + String(g_pin_rs485_tx) + ",\"rs_rx\":" + String(g_pin_rs485_rx);
  c += ",\"rs_dir\":" + String(g_pin_rs485_dir) + ",\"can_tx\":" + String(g_pin_can_tx);
  c += ",\"can_rx\":" + String(g_pin_can_rx) + ",\"led\":" + String(g_pin_led) + "}";
  c += ",\"dummy\":0}, ";
  server.sendContent(c);
  if (!streamAlive()) return;  // V2.66.1 C3: gate after victron+config chunks

  // Status section
  struct tm t; unsigned long ts = 0; if(getLocalTime(&t)) { time_t t_now; time(&t_now); ts = (unsigned long)t_now; }
  // V2.67 F2: Lazy-capture boot epoch once NTP has synced. Cutoff 1672531200 = 2023-01-01 UTC (pre-NTP guard).
  if (g_boot_epoch == 0 && ts > 1672531200UL) {
    uint32_t up_s = (uint32_t)(millis() / 1000);
    if ((uint32_t)ts > up_s) g_boot_epoch = (uint32_t)ts - up_s;
  }
  c = "\"ts\":" + String(ts);
  c += ",\"boot_epoch\":" + String(g_boot_epoch);
  c += ",\"wifi_ip\":\"" + WiFi.localIP().toString() + "\"";
  c += ",\"wifi_rssi\":" + String(WiFi.RSSI());
  c += ",\"wifi_host\":\"" + String(HOSTNAME) + "\"";
  c += ",\"uptime_ms\":" + String(millis());
  c += ",\"free_heap\":" + String(ESP.getFreeHeap());
  c += ",\"min_heap\":" + String(heap_min);
  // V2.65.1 D1: Continuous NVS partition monitoring.
  {
    uint32_t nvs_u=0, nvs_f=0, nvs_t=0;
    if (getNvsStats(nvs_u, nvs_f, nvs_t)) {
      c += ",\"nvs_used\":" + String(nvs_u);
      c += ",\"nvs_free\":" + String(nvs_f);
      c += ",\"nvs_total\":" + String(nvs_t);
    }
  }
  c += ",\"chip_temp\":" + String(temperatureRead(), 1);
  c += ",\"sketch_size\":" + String(ESP.getSketchSize());
  c += ",\"sketch_free\":" + String(ESP.getFreeSketchSpace());
  c += ",\"flash_size\":" + String(ESP.getFlashChipSize());
  c += ",\"chip_model\":\"" + String(ESP.getChipModel()) + "\"";
  c += ",\"chip_rev\":" + String(ESP.getChipRevision());
  c += ",\"chip_cores\":" + String(ESP.getChipCores());
  c += ",\"cpu_mhz\":" + String(ESP.getCpuFreqMHz());
  c += ",\"boot_reason\":\"" + g_boot_reason + "\"";
  c += ",\"mqtt_fails\":" + String(mqtt_fail_count);
  c += ",\"rl_rejects\":" + String(g_rl_rejects);   // V2.66 M6: total 429 responses since boot
  // V2.65 M1: CAN TX health counters
  c += ",\"can_ok\":" + String(g_can_tx_ok);
  c += ",\"can_fail\":" + String(g_can_tx_fail);
  c += ",\"can_fail_streak\":" + String(g_can_tx_fail_streak);
  // V2.65 M3: Log emitted as JSON array, newest-first.
  c += ",\"log\":[";
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50))) {
    bool first = true;
    for (uint8_t i = 0; i < g_log_count; i++) {
      uint8_t idx = (g_log_head + LOG_RING_SIZE - 1 - i) % LOG_RING_SIZE;
      LogEntry &e = g_log_ring[idx];
      if (!first) c += ",";
      first = false;
      c += "{\"t\":" + String(e.ts) + ",\"e\":" + String(e.flags & 1 ? 1 : 0) + ",\"m\":\"" + jsonEscape(String(e.msg)) + "\"}";
    }
    xSemaphoreGive(logMutex);
  }
  c += "], ";
  // V2.65 H4: Snapshot g_can_status under mutex before emission.
  if(!g_victron_enable) c += "\"can_status\":\"DISABLED\",\"can_error\":false, ";
  else {
    char canSnap[64] = {0};
    if (canStatusMutex && xSemaphoreTake(canStatusMutex, pdMS_TO_TICKS(20))) {
      strncpy(canSnap, g_can_status, sizeof(canSnap) - 1);
      xSemaphoreGive(canStatusMutex);
    }
    c += "\"can_status\":\"" + jsonEscape(String(canSnap)) + "\",\"can_error\":" + String(can_error_flag ? "true" : "false") + ", ";
  }
  #if HAS_SD_CARD
  c += "\"sd_ok\":" + String(sd_ok ? "true" : "false") + ",\"sd_err\":" + String(sd_error_flag ? "true" : "false") + ", ";
  #else
  c += "\"sd_ok\":false,\"sd_err\":false, ";
  #endif
  c += "\"safe_lock\":" + String(((alarmSnap & 0x02) || (alarmSnap & 0x10) || (alarmSnap & 0x40) || (alarmSnap & 0x80)) ? "true" : "false");
  c += ",\"alarm\":\"" + jsonEscape(alarmMsg) + "\", ";
  server.sendContent(c);

  // Energy section
  c = "\"energy\":{\"in\":" + String(energy_in_today, 2) + ",\"out\":" + String(energy_out_today, 2);
  // 7-day rolling totals
  float week_in = energy_in_today, week_out = energy_out_today;
  c += ",\"days_in\":["; for(int i=0;i<7;i++){if(i)c+=",";c+=String(energy_days_in[i],1);week_in+=energy_days_in[i];}
  c += "],\"days_out\":["; for(int i=0;i<7;i++){if(i)c+=",";c+=String(energy_days_out[i],1);week_out+=energy_days_out[i];}
  c += "],\"week_in\":" + String(week_in, 1) + ",\"week_out\":" + String(week_out, 1);
  c += ",\"month_in\":" + String(energy_months_in[0] + energy_in_today, 1);
  c += ",\"month_out\":" + String(energy_months_out[0] + energy_out_today, 1);
  c += "}, ";
  server.sendContent(c);

  // History - two configurable charts, downsampled, chunked
  // V2.65.4 R3: DS=3 triples the chart resolution from 96 points to 320.
  if (!streamAlive()) return;  // V2.66.1 C3: gate before charts (hot path)
  auto sendHist = [&](const char* name, int16_t* src) {
    server.sendContent(String("\"") + name + "\":[");
    const int DS = 3;
    int startIdx = historyIdx;
    bool hFirst = true;
    c = "";
    int cnt = 0;
    for (int i = 0; i < HISTORY_LEN; i += DS) {
      int32_t sum = 0;
      for (int k = 0; k < DS && (i + k) < HISTORY_LEN; k++) sum += src[(startIdx + i + k) % HISTORY_LEN];
      if (!hFirst) c += ",";
      c += String((int16_t)(sum / DS));
      hFirst = false;
      if (++cnt >= 32) { server.sendContent(c); c = ""; cnt = 0; if (!streamAlive()) return; }  // V2.66.1 C3
    }
    if (c.length() > 0) server.sendContent(c);
    server.sendContent("], ");
  };
  int16_t* chartSrc[] = { powerHistory, voltHistory, socHistory, tempHistory };
  int16_t* chartMin[] = { powerMin, voltMin, socMin, tempMin };
  int16_t* chartMax[] = { powerMax, voltMax, socMax, tempMax };
  int ci1 = g_chart1 < 4 ? g_chart1 : 0, ci2 = g_chart2 < 4 ? g_chart2 : 1;
  if (server.hasArg("c1")) ci1 = constrain(server.arg("c1").toInt(), 0, 3);
  if (server.hasArg("c2")) ci2 = constrain(server.arg("c2").toInt(), 0, 3);
  sendHist("hist1", chartSrc[ci1]); if (g_stream_abort) return;  // V2.66.2 C3+
  sendHist("h1mn", chartMin[ci1]);  if (g_stream_abort) return;  // V2.66.2 C3+
  sendHist("h1mx", chartMax[ci1]);  if (g_stream_abort) return;  // V2.66.2 C3+
  sendHist("hist2", chartSrc[ci2]); if (g_stream_abort) return;  // V2.66.2 C3+
  sendHist("h2mn", chartMin[ci2]);  if (g_stream_abort) return;  // V2.66.2 C3+
  sendHist("h2mx", chartMax[ci2]);  if (g_stream_abort) return;  // V2.66.2 C3+
  // hstart = index in downsampled array where real data begins.
  // Points before this are unwritten zeros from boot, client must skip them.
  // V2.65.4: HISTORY_LEN=960, DS=3 -> 320 downsampled chart points.
  {
    const int DS = 3;
    int unfilled = HISTORY_LEN - historyFilled;
    int hstart = (unfilled + DS - 1) / DS;  // round up so we never show partial-empty downsampled cells
    if (hstart < 0) hstart = 0;
    if (hstart > HISTORY_LEN/DS) hstart = HISTORY_LEN/DS;
    server.sendContent("\"hstart\":" + String(hstart) + ", ");
    // V2.65.4 R4: Emit minutes-per-chart-point so the frontend X-axis
    // labels stay accurate without hardcoded assumptions.
    int minPer = (HISTORY_INTERVAL / 60000) * DS;    // 3 min slots x 3 DS = 9 min per point
    server.sendContent("\"minPer\":" + String(minPer) + ", ");
    server.sendContent("\"histLen\":" + String(HISTORY_LEN) + ", ");
    server.sendContent("\"histFilled\":" + String(historyFilled) + ", ");
  }

  // BMS array - one BMS per chunk
  if (!streamAlive()) return;  // V2.66.1 C3: gate before BMS array
  server.sendContent("\"bms\":[");
  for (int i = 0; i < bmsCnt; i++) {
    if (i > 0) server.sendContent(",");
    bool on = (bSnap[i].valid && (millis() - bSnap[i].last_seen < 120000)) || simulation_active;
    c = "{\"id\":" + String(i) + ",\"online\":" + String(on ? "true" : "false");
    if (on) {
      c += ",\"soc\":" + String(bSnap[i].soc) + ",\"current\":" + String(bSnap[i].current, 1);
      c += ",\"pack_v\":" + String(bSnap[i].voltage, 2) + ",\"soh\":" + String(bSnap[i].soh);
      c += ",\"min_cell\":" + String(bSnap[i].minCellV, 3) + ",\"max_cell\":" + String(bSnap[i].maxCellV, 3);
      c += ",\"avg_temp\":" + String(bSnap[i].avgTemp, 1);
      c += ",\"cycles\":" + String(bSnap[i].cycles);
      c += ",\"temps\":[";
      for (int t = 0; t < bSnap[i].temp_count; t++) {
        if (t > 0) c += ",";
        String lbl = "T" + String(t + 1); if (t == 4) lbl = "MOS"; if (t == 5) lbl = "ENV"; if (t == 6) lbl = "BAL";
        c += "{\"val\":" + String(bSnap[i].temps[t], 1) + ",\"lbl\":\"" + lbl + "\"}";
      }
      c += "],\"cells\":[";
      for (int cc = 0; cc < bSnap[i].cell_count; cc++) { if (cc > 0) c += ","; c += String(bSnap[i].cells[cc], 3); }
      c += "]";
    }
    c += ",\"stats\":{\"polls\":" + String(g_bms_stats[i].polls) + ",\"ok\":" + String(g_bms_stats[i].ok);
    c += ",\"timeout\":" + String(g_bms_stats[i].timeouts) + ",\"err\":" + String(g_bms_stats[i].errors);
    c += ",\"spikes\":" + String(g_bms_stats[i].spikes) + ",\"holds\":" + String(g_bms_stats[i].current_holds) + "}";
    c += "}";
    server.sendContent(c);
    if (g_stream_abort) return;  // V2.66.2 E1: per-BMS disconnect gate
  }
  server.sendContent("], ");
  if (!streamAlive()) return;  // V2.66.1 C3: gate before alerts array
  // V2.61: embed alert ring buffer (newest first)
  server.sendContent("\"alerts\":[");
  {
    bool first = true;
    for (int i = 1; i <= g_alert_count; i++) {
      int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
      if (!first) server.sendContent(",");
      first = false;
      String m; m.reserve(ALERT_MSG_LEN + 16);
      const char* p = g_alerts[idx].msg;
      for (int k = 0; k < ALERT_MSG_LEN && p[k]; k++) {
        char ch = p[k];
        if (ch == '"' || ch == '\\') { m += '\\'; m += ch; }
        else if ((uint8_t)ch < 0x20) m += ' ';
        else m += ch;
      }
      String ae = "{\"ts\":" + String(g_alerts[idx].ts)
        + ",\"up\":" + String(g_alerts[idx].uptime_s)
        + ",\"sev\":" + String((int)g_alerts[idx].sev)
        + ",\"msg\":\"" + m + "\"}";
      server.sendContent(ae);
    }
  }
  server.sendContent("] }");
  server.sendContent("");  // signal end of chunked transfer
  // V2.66.2 D1: record handler runtime for debug telemetry
  unsigned long handler_ms = millis() - handler_start_ms;
  if (handler_ms > g_handler_max_ms) g_handler_max_ms = handler_ms;
}

// ==========================================
// OTA FIRMWARE UPDATE
// Standalone page at /update (GET) + upload handler (POST).
// V2.65.4: saveHistory() persists only energy counters. Chart history
// lives in RAM only and is intentionally discarded across OTA reboots.
// ==========================================
void handleOTAPage() {
  if (!checkAuth()) return;
  String h = R"rawliteral(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Firmware Update</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--bg3:#f3f4f6;--tx:#1f2937;--tx2:#6b7280;--tx3:#9ca3af;--bd:rgba(0,0,0,0.15);--acc:#059669;--acc2:#047857;--r:8px;--r2:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--bg3:#16213e;--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;--bd:rgba(255,255,255,0.12)}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh;padding:20px}
.box{background:var(--bg2);border:0.5px solid var(--bd);border-radius:var(--r2);padding:30px;max-width:450px;width:100%;text-align:center}
h2{font-size:16px;font-weight:500;margin-bottom:4px}
.ver{color:var(--tx3);font-size:12px;margin-bottom:20px}
.drop{border:1.5px dashed var(--bd);border-radius:var(--r);padding:40px 20px;margin:16px 0;cursor:pointer;transition:all 0.2s}
.drop:hover,.drop.over{border-color:var(--acc);background:rgba(5,150,105,0.05)}
.drop input{display:none}.drop label{cursor:pointer;color:var(--tx2);font-size:13px}
.obtn{background:var(--acc);color:#fff;border:none;padding:10px 24px;border-radius:var(--r);font-size:13px;cursor:pointer;width:100%;margin-top:12px;font-weight:500}
.obtn:hover{background:var(--acc2)}.obtn:disabled{background:var(--bg3);color:var(--tx3);cursor:not-allowed}
.prog{width:100%;height:6px;border-radius:3px;background:var(--bg3);margin:16px 0 8px;overflow:hidden;display:none}
.prog div{height:100%;background:var(--acc);border-radius:3px;width:0%;transition:width 0.3s}
.msg{font-size:12px;min-height:20px;margin-top:8px}
.msg.ok{color:var(--acc)}.msg.err{color:#ef4444}.msg.info{color:#3b82f6}
a.back{color:var(--tx3);font-size:12px;text-decoration:none;display:inline-block;margin-top:16px}a.back:hover{color:var(--tx2)}
</style></head><body>
<div class="box">
<h2>Firmware Update</h2>
<div class="ver" id="ota_ver"></div>
<div class="drop" id="drop">
<input type="file" id="file" accept=".bin">
<label for="file" id="flbl">Select .bin file or drag and drop here</label>
</div>
<div class="prog" id="prog"><div id="bar"></div></div>
<div class="msg" id="msg"></div>
<button class="obtn" id="obtn" onclick="doUpload()" disabled>Upload firmware</button>
<br><a class="back" href="/">&larr; Back to dashboard</a>
</div>
<script>
var fileData=null;
var dp=document.getElementById('drop');
var fi=document.getElementById('file');
dp.ondragover=function(ev){ev.preventDefault();dp.classList.add('over');};
dp.ondragleave=function(){dp.classList.remove('over');};
dp.ondrop=function(ev){ev.preventDefault();dp.classList.remove('over');if(ev.dataTransfer.files.length)pickFile(ev.dataTransfer.files[0]);};
fi.onchange=function(){if(fi.files.length)pickFile(fi.files[0]);};
 function pickFile(f){
  if(!f.name.endsWith('.bin')){showMsg('Only .bin files allowed','err');return;}
  fileData=f;
  document.getElementById('flbl').innerText=f.name+' ('+Math.round(f.size/1024)+' KB)';
  document.getElementById('obtn').disabled=false;
  showMsg('Ready to upload','info');
}
 function showMsg(t,c){var m=document.getElementById('msg');m.className='msg '+(c?c:'');m.innerText=t;}
 function doUpload(){
  if(!fileData)return;
  var xhr=new XMLHttpRequest();
  var fd=new FormData();
  fd.append('update',fileData);
  document.getElementById('obtn').disabled=true;
  document.getElementById('prog').style.display='block';
  showMsg('Uploading...','info');
  var uploadDone=false;
  xhr.upload.onprogress=function(ev){if(ev.lengthComputable){var p=Math.round(ev.loaded/ev.total*100);document.getElementById('bar').style.width=p+'%';showMsg('Upload: '+p+'%','info');if(p>=100)uploadDone=true;}};
  xhr.onload=function(){showMsg('Update successful! Rebooting...','ok');pollBoot();};
  xhr.onerror=function(){if(uploadDone){showMsg('Update successful! Rebooting...','ok');pollBoot();}else{showMsg('Connection error','err');document.getElementById('obtn').disabled=false;}};
  function pollBoot(){var n=0;var iv=setInterval(function(){n++;showMsg('Waiting for reboot... ('+n+'s)','info');fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);location.href='/'}).catch(function(){})},3000)}
  xhr.open('POST','/update');
  xhr.send(fd);
}
fetch('/data').then(function(r){return r.json();}).then(function(d){if(d.config&&d.config.fw)document.getElementById('ota_ver').innerText='Current version: V'+d.config.fw;});
</script></body></html>)rawliteral";
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send(200, "text/html", h);
}

void handleOTAUpload() {
  if (!checkAuth()) return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    saveHistory();
    if (g_alerts_dirty) saveAlertsToNvs();   // V2.61
    addToLog("OTA: Start " + upload.filename, false);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      addToLog("OTA: Begin failed", true);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    esp_task_wdt_reset();
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      addToLog("OTA: Write failed", true);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      addToLog("OTA: Success, rebooting", false);
    } else {
      addToLog("OTA: End failed", true);
    }
  }
}

void handleOTADone() {
  if (!checkAuth()) return;
  if (Update.hasError()) {
    server.send(500, "text/plain", "Update failed");
  } else {
    String rp = R"RP(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rebooting</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:40px;text-align:center;max-width:400px;width:90%}
h2{font-size:16px;font-weight:500;margin-bottom:8px}.s{color:var(--tx2);font-size:13px;margin-bottom:16px}
.bar{width:100%;height:4px;background:rgba(128,128,128,0.2);border-radius:2px;overflow:hidden}.bar div{height:100%;background:var(--acc);border-radius:2px;width:0%;transition:width 0.5s}
.c{font-size:12px;color:var(--tx2);margin-top:12px}
</style></head><body><div class="b"><h2>Firmware updated</h2><div class="s">Device is rebooting...</div><div class="bar"><div id="br"></div></div><div class="c" id="ct">Waiting for reboot...</div></div>
<script>var n=0,b=document.getElementById('br'),c=document.getElementById('ct');var iv=setInterval(function(){n++;b.style.width=Math.min(n*5,95)+'%';c.textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);b.style.width='100%';c.textContent='Loading dashboard...';location.href='/'}).catch(function(){})},3000)</script></body></html>)RP";
    server.send(200, "text/html", rp);
    g_restart_at = millis() + 1000;   // V2.65 M2: deferred restart, non-blocking
  }
}

// ==========================================
// SETTINGS BACKUP & RESTORE
// Backup exports all NVS settings as JSON (excludes MQTT password and auth hash).
// Restore imports JSON, writes to NVS, and reboots.
// ==========================================
void handleBackup() {
  if (!checkAuth()) return;
  preferences.begin("gateway", true);
  String j = "{\n";
  j += "  \"_info\": \"TopBand BMS Gateway Settings Backup\",\n";
  j += "  \"_fw\": \"" + String(FW_VERSION) + "\",\n";
  j += "  \"_uid\": \"" + g_dev_uid + "\",\n";
  j += "  \"cnt\": " + String(preferences.getInt("cnt", 2)) + ",\n";
  j += "  \"cells\": " + String(preferences.getInt("cells", 0)) + ",\n";
  j += "  \"chg\": " + String(preferences.getFloat("chg", 30.0), 1) + ",\n";
  j += "  \"dis\": " + String(preferences.getFloat("dis", 30.0), 1) + ",\n";
  j += "  \"cvl\": " + String(preferences.getFloat("cvl", 52.5), 2) + ",\n";
  j += "  \"exp\": " + String(preferences.getBool("exp", true) ? "true" : "false") + ",\n";
  j += "  \"s_vol\": " + String(preferences.getFloat("s_vol", 53.25), 2) + ",\n";
  j += "  \"s_cel\": " + String(preferences.getFloat("s_cel", 3.55), 3) + ",\n";
  j += "  \"s_drift\": " + String(preferences.getFloat("s_drift", 0.20), 2) + ",\n";
  j += "  \"sp_v\": " + String(preferences.getFloat("sp_v", 5.0), 2) + ",\n";
  j += "  \"sp_a\": " + String(preferences.getFloat("sp_a", 250.0), 1) + ",\n";
  j += "  \"sp_s\": " + String(preferences.getInt("sp_s", 20)) + ",\n";
  j += "  \"t_c_min\": " + String(preferences.getFloat("t_c_min", 5.0), 1) + ",\n";
  j += "  \"t_c_max\": " + String(preferences.getFloat("t_c_max", 50.0), 1) + ",\n";
  j += "  \"t_d_min\": " + String(preferences.getFloat("t_d_min", -20.0), 1) + ",\n";
  j += "  \"t_d_max\": " + String(preferences.getFloat("t_d_max", 60.0), 1) + ",\n";
  j += "  \"t_mode\": " + String(preferences.getInt("t_mode", 0)) + ",\n";
  j += "  \"soc_mode\": " + String(preferences.getInt("soc_mode", 2)) + ",\n";
  j += "  \"maint_en\": " + String(preferences.getBool("maint_en", false) ? "true" : "false") + ",\n";
  j += "  \"maint_v\": " + String(preferences.getFloat("maint_v", 51.0), 2) + ",\n";
  j += "  \"ab_en\": " + String(preferences.getBool("ab_en", true) ? "true" : "false") + ",\n";
  j += "  \"vic_en\": " + String(preferences.getBool("vic_en", true) ? "true" : "false") + ",\n";
  j += "  \"can_proto\": " + String(preferences.getInt("can_proto", 0)) + ",\n";
  j += "  \"mq_en\": " + String(preferences.getBool("mq_en", false) ? "true" : "false") + ",\n";
  j += "  \"mq_full\": " + String(preferences.getBool("mq_full", false) ? "true" : "false") + ",\n";
  j += "  \"mq_diag\": " + String(preferences.getBool("mq_diag", false) ? "true" : "false") + ",\n";  // V2.66.2 D1
  j += "  \"mq_level\": " + String(preferences.getInt("mq_level", 0)) + ",\n";  // V2.67 F1
  j += "  \"ha_en\": " + String(preferences.getBool("ha_en", false) ? "true" : "false") + ",\n";
  j += "  \"mq_ip\": \"" + jsonEscape(preferences.getString("mq_ip", "")) + "\",\n";
  j += "  \"mq_pt\": " + String(preferences.getInt("mq_pt", 1883)) + ",\n";
  j += "  \"mq_us\": \"" + jsonEscape(preferences.getString("mq_us", "")) + "\",\n";
  j += "  \"mq_top\": \"" + jsonEscape(preferences.getString("mq_top", "Topband/BMS")) + "\",\n";
  j += "  \"ntp\": \"" + jsonEscape(preferences.getString("ntp", "pool.ntp.org")) + "\",\n";
  j += "  \"tz\": " + String(preferences.getInt("tz", 1)) + ",\n";
  j += "  \"chart1\": " + String(preferences.getInt("chart1", 0)) + ",\n";
  j += "  \"chart2\": " + String(preferences.getInt("chart2", 1)) + ",\n";
  j += "  \"auth_en\": " + String(preferences.getBool("auth_en", false) ? "true" : "false") + ",\n";
  j += "  \"auth_user\": \"" + jsonEscape(preferences.getString("auth_user", "admin")) + "\",\n";
  j += "  \"board_type\": " + String(preferences.getInt("board_type", 0)) + ",\n";
  j += "  \"p_rs_tx\": " + String(preferences.getInt("p_rs_tx", 17)) + ",\n";
  j += "  \"p_rs_rx\": " + String(preferences.getInt("p_rs_rx", 18)) + ",\n";
  j += "  \"p_rs_dir\": " + String(preferences.getInt("p_rs_dir", 21)) + ",\n";
  j += "  \"p_can_tx\": " + String(preferences.getInt("p_can_tx", 15)) + ",\n";
  j += "  \"p_can_rx\": " + String(preferences.getInt("p_can_rx", 16)) + ",\n";
  j += "  \"p_led\": " + String(preferences.getInt("p_led", 38)) + "\n";
  j += "}";
  preferences.end();
  server.sendHeader("Content-Disposition", "attachment; filename=topband_backup.json");
  server.send(200, "application/json", j);
  addToLog("Settings backup downloaded", false);
}

void handleRestore() {
  if (!checkAuth()) return;
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
    return;
  }
  String body = server.arg("plain");
  if (body.length() < 10 || body.indexOf("cnt") < 0) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"invalid json\"}");
    return;
  }
  // Simple JSON value extractors
  auto getInt = [&](const char* key, int def) -> int {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p).toInt();
  };
  auto getFloat = [&](const char* key, float def) -> float {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p).toFloat();
  };
  auto getBool = [&](const char* key, bool def) -> bool {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p, p + 4) == "true";
  };
  auto getStr = [&](const char* key, String def) -> String {
    String k = String("\"") + key + "\": \"";
    int p = body.indexOf(k);
    if (p < 0) { k = String("\"") + key + "\":\""; p = body.indexOf(k); }
    if (p < 0) return def;
    p += k.length();
    int e = body.indexOf("\"", p);
    if (e < 0) return def;
    return body.substring(p, e);
  };

  preferences.begin("gateway", false);
  preferences.putInt("cnt", getInt("cnt", 2));
  preferences.putInt("cells", getInt("cells", 0));
  preferences.putFloat("chg", getFloat("chg", 30.0));
  preferences.putFloat("dis", getFloat("dis", 30.0));
  preferences.putFloat("cvl", getFloat("cvl", 52.5));
  preferences.putBool("exp", getBool("exp", true));
  preferences.putFloat("s_vol", getFloat("s_vol", 53.25));
  preferences.putFloat("s_cel", getFloat("s_cel", 3.55));
  preferences.putFloat("s_drift", getFloat("s_drift", 0.20));
  preferences.putFloat("sp_v", getFloat("sp_v", 5.0));
  preferences.putFloat("sp_a", getFloat("sp_a", 250.0));
  preferences.putInt("sp_s", getInt("sp_s", 20));
  preferences.putFloat("t_c_min", getFloat("t_c_min", 5.0));
  preferences.putFloat("t_c_max", getFloat("t_c_max", 50.0));
  preferences.putFloat("t_d_min", getFloat("t_d_min", -20.0));
  preferences.putFloat("t_d_max", getFloat("t_d_max", 60.0));
  preferences.putInt("t_mode", getInt("t_mode", 0));
  preferences.putInt("soc_mode", getInt("soc_mode", 2));
  preferences.putBool("maint_en", getBool("maint_en", false));
  preferences.putFloat("maint_v", getFloat("maint_v", 51.0));
  preferences.putBool("ab_en", getBool("ab_en", true));
  preferences.putBool("vic_en", getBool("vic_en", true));
  preferences.putInt("can_proto", getInt("can_proto", 0));
  preferences.putBool("mq_en", getBool("mq_en", false));
  preferences.putBool("mq_full", getBool("mq_full", false));
  preferences.putBool("mq_diag", getBool("mq_diag", false));  // V2.66.2 D1
  {
    int lvl = getInt("mq_level", 0);
    if (lvl < 0) lvl = 0; if (lvl > 2) lvl = 2;
    preferences.putInt("mq_level", lvl);  // V2.67 F1
  }
  preferences.putBool("ha_en", getBool("ha_en", false));
  preferences.putString("mq_ip", getStr("mq_ip", ""));
  preferences.putInt("mq_pt", getInt("mq_pt", 1883));
  preferences.putString("mq_us", getStr("mq_us", ""));
  preferences.putString("mq_top", getStr("mq_top", "Topband/BMS"));
  preferences.putString("ntp", getStr("ntp", "pool.ntp.org"));
  preferences.putInt("tz", getInt("tz", 1));
  preferences.putInt("chart1", getInt("chart1", 0));
  preferences.putInt("chart2", getInt("chart2", 1));
  preferences.putBool("auth_en", getBool("auth_en", false));
  preferences.putString("auth_user", getStr("auth_user", "admin"));
  // Pin configuration
  preferences.putInt("board_type", getInt("board_type", 0));
  preferences.putInt("p_rs_tx", getInt("p_rs_tx", 17));
  preferences.putInt("p_rs_rx", getInt("p_rs_rx", 18));
  preferences.putInt("p_rs_dir", getInt("p_rs_dir", 21));
  preferences.putInt("p_can_tx", getInt("p_can_tx", 15));
  preferences.putInt("p_can_rx", getInt("p_can_rx", 16));
  preferences.putInt("p_led", getInt("p_led", 38));
  // Note: auth_hash intentionally excluded from restore for security
  preferences.putBool("setupd", true);
  preferences.end();

  addToLog("Settings restored from backup", false);
  server.send(200, "application/json", "{\"ok\":true}");
  g_restart_at = millis() + 500;   // V2.65 M2: deferred restart, non-blocking
}

// ==========================================
// HISTORY EXPORT (CSV download of chart data)
// Exports all 4 chart types as CSV with timestamps.
// Each row = one 3-minute interval, 960 rows = 48h.
// ==========================================
void handleExport() {
  if (!checkAuth()) return;
  server.sendHeader("Content-Disposition", "attachment; filename=\"topband_history.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("index,minutes_ago,power_w,voltage_v,soc_pct,temperature_c\n");
  String buf; buf.reserve(512);
  int cnt = 0;
  for (int i = 0; i < HISTORY_LEN; i++) {
    int idx = (historyIdx + i) % HISTORY_LEN;
    int mins_ago = (HISTORY_LEN - i) * 3;   // V2.65.4: back to 3-minute interval
    float pw = (float)powerHistory[idx];
    float vt = (float)voltHistory[idx] * 0.01f;
    float sc = (float)socHistory[idx] * 0.1f;
    float tp = (float)tempHistory[idx] * 0.1f;
    buf += String(i) + "," + String(mins_ago) + "," + String(pw, 0) + "," + String(vt, 2) + "," + String(sc, 1) + "," + String(tp, 1) + "\n";
    if (++cnt >= 40) { server.sendContent(buf); buf = ""; cnt = 0; }
  }
  if (buf.length() > 0) server.sendContent(buf);
  server.sendContent("");
}

void handleServicePage() {
  if (!checkAuth()) return;
  String h = R"SVCHTML(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMS Diagnostics</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--bg3:#f3f4f6;--tx:#1f2937;--tx2:#6b7280;--tx3:#9ca3af;--bd:rgba(0,0,0,0.15);--bd2:rgba(0,0,0,0.3);--acc:#059669;--acc2:#047857;--accbg:#dcfce7;--acctx:#166534;--red:#f87171;--redbg:#fee2e2;--redtx:#991b1b;--amber:#fbbf24;--amberbg:#fef3c7;--ambertx:#92400e;--r:8px;--r2:12px;--mono:'SF Mono',Consolas,monospace}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--bg3:#16213e;--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;--bd:rgba(255,255,255,0.12);--bd2:rgba(255,255,255,0.25);--accbg:rgba(22,101,52,0.3);--acctx:#4ade80;--redbg:rgba(153,27,27,0.3);--redtx:#fca5a5;--amberbg:rgba(146,64,14,0.3);--ambertx:#fbbf24}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;padding:20px}
.gw{max-width:700px;margin:0 auto}
.top{background:var(--bg2);border:0.5px solid var(--bd);border-bottom:2px solid var(--acc);border-radius:var(--r2) var(--r2) 0 0;padding:12px 16px;display:flex;justify-content:space-between;align-items:center}
.top h1{font-size:15px;font-weight:500}.top .v{font-size:11px;color:var(--tx3)}
.ct{background:var(--bg2);border:0.5px solid var(--bd);border-top:none;border-radius:0 0 var(--r2) var(--r2);padding:16px}
.sl{font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:14px 0 8px;padding-bottom:5px;border-bottom:0.5px solid var(--bd)}
.sl:first-child{margin-top:0}
.row{display:flex;justify-content:space-between;font-size:12px;padding:4px 0;border-bottom:0.5px solid var(--bd)}
.row .k{color:var(--tx2)}.row .vl{font-weight:500;font-family:var(--mono);font-size:11px}
.row .vl.ok{color:var(--acctx)}.row .vl.er{color:var(--redtx)}.row .vl.wa{color:var(--ambertx)}
.ctr{display:flex;gap:8px;align-items:center;margin-bottom:12px;flex-wrap:wrap}
select,input{padding:7px 9px;border:0.5px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px;outline:none}
select:focus,input:focus{border-color:var(--acc)}
.bp{background:var(--acc);color:#fff;border:none;padding:8px 16px;border-radius:var(--r);font-size:12px;font-weight:500;cursor:pointer}
.bp:hover{background:var(--acc2)}.bp:disabled{background:var(--bg3);color:var(--tx3);cursor:not-allowed}
.bo{background:transparent;border:0.5px solid var(--bd2);padding:7px 14px;border-radius:var(--r);font-size:12px;cursor:pointer;color:var(--tx)}
.bo:hover{background:var(--bg3)}
.msg{font-size:12px;padding:8px 12px;border-radius:var(--r);margin:8px 0}
.msg.ok{background:var(--accbg);color:var(--acctx)}.msg.er{background:var(--redbg);color:var(--redtx)}.msg.ld{background:var(--bg3);color:var(--tx3)}
a.back{color:var(--tx3);font-size:12px;text-decoration:none;display:inline-block;margin-top:14px}a.back:hover{color:var(--tx2)}
.pill{font-size:10px;padding:2px 8px;border-radius:10px;font-weight:500}
.pill-g{background:var(--accbg);color:var(--acctx)}.pill-r{background:var(--redbg);color:var(--redtx)}.pill-y{background:var(--amberbg);color:var(--ambertx)}
.alarm-list{margin-top:6px}.alarm-list span{display:inline-block;font-size:11px;background:var(--bg3);padding:2px 8px;border-radius:4px;margin:2px 4px 2px 0}
</style></head><body>
<div class="gw">
<div class="top"><h1>BMS Diagnostics <span class="v" id="sfw"></span></h1></div>
<div class="ct">
<div class="ctr">
<select id="sid"></select>
<button class="bp" id="rbtn" onclick="rd()">Read diagnostics</button>
<button class="bo" onclick="st()">Sync time</button>
</div>
<div id="stt" class="msg ld" style="display:none"></div>
<div id="out"></div>
<a class="back" href="/">&larr; Back to dashboard</a>
</div></div>
<script>
 function $(i){return document.getElementById(i)}
 function msg(t,c){var e=$('stt');e.style.display='block';e.className='msg '+(c||'ld');e.textContent=t}
fetch('/data').then(function(r){return r.json()}).then(function(d){
  var c=d.config||{};var sel=$('sid');
  for(var i=0;i<(c.cnt||2);i++){var o=document.createElement('option');o.value=i;o.textContent='BMS #'+i;sel.appendChild(o)}
  $('sfw').textContent='V'+(c.fw||'');
});
 function rd(){
  var id=$('sid').value;msg('Reading BMS '+id+'...');$('rbtn').disabled=true;
  fetch('/svc/bms?id='+id).then(function(r){return r.json()}).then(function(d){
    $('rbtn').disabled=false;
    if(!d.ok){msg('Error: '+(d.err||'unknown'),'er');return}
    msg('BMS '+id+' data loaded','ok');
    var h='';
    if(d.manufacturer&&d.manufacturer.ok){
      h+='<div class="sl">Manufacturer</div>';
      h+='<div class="row"><span class="k">Hardware</span><span class="vl">'+esc(d.manufacturer.hw)+'</span></div>';
      h+='<div class="row"><span class="k">Software</span><span class="vl">'+esc(d.manufacturer.sw)+'</span></div>';
      h+='<div class="row"><span class="k">ID</span><span class="vl">'+esc(d.manufacturer.id)+'</span></div>';
    }
    if(d.date&&d.date.ok){
      h+='<div class="sl">BMS clock</div>';
      h+='<div class="row"><span class="k">Date/Time</span><span class="vl">'+d.date.year+'-'+pad(d.date.month)+'-'+pad(d.date.day)+' '+pad(d.date.hour)+':'+pad(d.date.minute)+':'+pad(d.date.second)+'</span></div>';
    }
    if(d.analog&&d.analog.ok){
      var a=d.analog;
      h+='<div class="sl">Analog data (0x42)</div>';
      h+='<div class="row"><span class="k">Pack voltage</span><span class="vl">'+a.pack_v.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Current</span><span class="vl">'+a.current.toFixed(2)+' A</span></div>';
      h+='<div class="row"><span class="k">SOC / SOH</span><span class="vl">'+a.soc+'% / '+a.soh+'%</span></div>';
      h+='<div class="row"><span class="k">Capacity</span><span class="vl">'+a.rem_ah.toFixed(2)+' / '+a.full_ah.toFixed(2)+' Ah</span></div>';
      h+='<div class="row"><span class="k">Cells / Temps</span><span class="vl">'+a.cells+' / '+a.temps+'</span></div>';
      h+='<div class="row"><span class="k">Cell range</span><span class="vl">'+a.min_cell.toFixed(3)+' .. '+a.max_cell.toFixed(3)+' V</span></div>';
    }
    if(d.system&&d.system.ok){
      var s=d.system;
      h+='<div class="sl">System parameters (0x47)</div>';
      h+='<div class="row"><span class="k">Cell high / low / under</span><span class="vl">'+s.cell_high_v.toFixed(3)+' / '+s.cell_low_v.toFixed(3)+' / '+s.cell_under_v.toFixed(3)+' V</span></div>';
      h+='<div class="row"><span class="k">Module high / low / under</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_low_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Charge current max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">Discharge current max</span><span class="vl">'+s.discharge_current_max_a.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">Charge temp range</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';
      h+='<div class="row"><span class="k">Discharge temp range</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>';
    }
    if(d.alarm&&d.alarm.ok){
      h+='<div class="sl">Alarm status (0x44)</div>';
      h+='<div class="row"><span class="k">Critical</span><span class="vl '+(d.alarm.critical?'er':'ok')+'">'+(d.alarm.critical?'YES':'No')+'</span></div>';
      h+='<div class="row"><span class="k">Status bits</span><span class="vl">0x'+d.alarm.status_hex.replace(/^0+/,'')+'</span></div>';
      if(d.alarm.active&&d.alarm.active.length>0){
        h+='<div class="alarm-list">';d.alarm.active.forEach(function(a){h+='<span>'+esc(a.name)+'</span>'});h+='</div>';
      }
    }
    if(d.historical&&d.historical.ok){
      var hi=d.historical;
      h+='<div class="sl">Last historical event (0x4B)</div>';
      h+='<div class="row"><span class="k">Date</span><span class="vl">'+hi.year+'-'+pad(hi.month)+'-'+pad(hi.day)+' '+pad(hi.hour)+':'+pad(hi.minute)+'</span></div>';
      h+='<div class="row"><span class="k">Event / Status</span><span class="vl">Type '+hi.event+' / 0x'+hi.status_hex.replace(/^0+/,'')+'</span></div>';
      h+='<div class="row"><span class="k">Pack / Current / Remain</span><span class="vl">'+hi.pack_v.toFixed(2)+' V / '+hi.current.toFixed(2)+' A / '+hi.remain_ah.toFixed(2)+' Ah</span></div>';
    }
    if(d.autocfg&&d.autocfg.ok&&d.autocfg.suggest){
      var sg=d.autocfg.suggest;
      h+='<div class="sl">Auto-config suggestion</div>';
      h+='<div class="row"><span class="k">Safe cell / pack</span><span class="vl">'+sg.s_cel.toFixed(3)+' V / '+sg.s_vol.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Charge / Discharge</span><span class="vl">'+sg.chg.toFixed(1)+' / '+sg.dis.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">CVL</span><span class="vl">'+sg.cvl.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Temp charge</span><span class="vl">'+sg.t_c_min.toFixed(1)+' .. '+sg.t_c_max.toFixed(1)+' C</span></div>';
      h+='<div class="row"><span class="k">Temp discharge</span><span class="vl">'+sg.t_d_min.toFixed(1)+' .. '+sg.t_d_max.toFixed(1)+' C</span></div>';
      h+='<div style="margin-top:10px"><button class="bp" onclick="acf()">Apply to gateway</button></div>';
    }
    $('out').innerHTML=h;
  }).catch(function(e){$('rbtn').disabled=false;msg('Fetch error: '+e.message,'er')});
}
 function st(){
  var id=$('sid').value;msg('Setting BMS '+id+' time...');
  fetch('/svc/bms/time/set?id='+id).then(function(r){return r.json()}).then(function(d){
    if(d.ok){msg('BMS '+id+' time synced','ok');rd()}else msg('Failed: '+(d.err||''),'er');
  }).catch(function(e){msg('Error: '+e.message,'er')});
}
 function acf(){
  var id=$('sid').value;if(!confirm('Apply auto-config from BMS '+id+' to gateway? You must Save and Reboot afterwards.'))return;
  msg('Applying auto-config from BMS '+id+'...');
  fetch('/svc/autocfg/apply?id='+id).then(function(r){return r.json()}).then(function(d){
    if(d.ok){msg('Auto-config applied. Go to Battery tab and Save and Reboot to persist.','ok')}else msg('Failed: '+(d.err||''),'er');
  }).catch(function(e){msg('Error: '+e.message,'er')});
}
 function pad(n){return String(n).padStart(2,'0')}
 function esc(s){if(!s)return'';var d=document.createElement('div');d.textContent=s;return d.innerHTML}
</script></body></html>)SVCHTML";
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send(200, "text/html", h);
}

// ==========================================
// MAIN WEB UI (single-page app served as raw literal)
// Glassmorphism sidebar layout with 5 pages:
//   Dashboard (read-only), Battery, Network, General, Alerts.
// Sidebar navigation with SVG icons, frosted glass cards.
// Light + dark mode via CSS variables + localStorage toggle.
// Polls /data every 2.5s, charts render on <canvas>.
// Form fields use same name attributes as /save POST endpoint.
// Alert tracking uses localStorage for "new since last visit" badge.
// ==========================================
void handleRoot() {
  if (!checkAuth()) return;
  String h = R"RAWUI(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>TopBand BMS Gateway</title>
<style>
/* ============================================================
   TOPBAND BMS GATEWAY V2.64 - Glassmorphism Sidebar UI
   Light + Dark mode, responsive sidebar, frosted glass cards.
   ============================================================ */

/* --- DESIGN TOKENS: LIGHT MODE --- */
:root{
--bg:#f0f2f5;--bg2:#ffffff;--bg3:#f8f9fb;
--tx:#0f172a;--tx2:#64748b;--tx3:#94a3b8;
--bd:rgba(0,0,0,0.07);--bd2:rgba(0,0,0,0.15);
--acc:#059669;--acc2:#047857;--accbg:#ecfdf5;--acctx:#065f46;
--red:#ef4444;--redbg:#fef2f2;--redtx:#991b1b;--redbd:#fca5a5;
--amber:#f59e0b;--amberbg:#fffbeb;--ambertx:#92400e;--amberbd:#fcd34d;
--blue:#3b82f6;--bluebg:#eff6ff;--bluetx:#1e40af;
--glass:rgba(255,255,255,0.75);--blur:blur(14px);
--sh:0 1px 4px rgba(0,0,0,0.05);--sh2:0 6px 20px rgba(0,0,0,0.07);
--nav:#0e1726;--navtx:rgba(255,255,255,0.55);--navhi:#fff;
--r:10px;--r2:14px;--mono:ui-monospace,'SF Mono',Consolas,monospace}

/* --- DESIGN TOKENS: DARK MODE --- */
@media(prefers-color-scheme:dark){:root:not(.lt){
--bg:#080e1a;--bg2:rgba(15,23,42,0.65);--bg3:rgba(22,33,55,0.5);
--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;
--bd:rgba(255,255,255,0.06);--bd2:rgba(255,255,255,0.12);
--glass:rgba(15,23,42,0.55);
--accbg:rgba(5,150,105,0.15);--acctx:#34d399;
--redbg:rgba(239,68,68,0.12);--redtx:#fca5a5;--redbd:rgba(239,68,68,0.35);
--amberbg:rgba(245,158,11,0.12);--ambertx:#fcd34d;--amberbd:rgba(245,158,11,0.35);
--bluebg:rgba(59,130,246,0.12);--bluetx:#93c5fd;
--sh:0 1px 4px rgba(0,0,0,0.3);--sh2:0 6px 24px rgba(0,0,0,0.3);
--nav:#060b14;--navtx:rgba(255,255,255,0.4)}}
:root.dk{
--bg:#080e1a;--bg2:rgba(15,23,42,0.65);--bg3:rgba(22,33,55,0.5);
--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;
--bd:rgba(255,255,255,0.06);--bd2:rgba(255,255,255,0.12);
--glass:rgba(15,23,42,0.55);
--accbg:rgba(5,150,105,0.15);--acctx:#34d399;
--redbg:rgba(239,68,68,0.12);--redtx:#fca5a5;--redbd:rgba(239,68,68,0.35);
--amberbg:rgba(245,158,11,0.12);--ambertx:#fcd34d;--amberbd:rgba(245,158,11,0.35);
--bluebg:rgba(59,130,246,0.12);--bluetx:#93c5fd;
--sh:0 1px 4px rgba(0,0,0,0.3);--sh2:0 6px 24px rgba(0,0,0,0.3);
--nav:#060b14;--navtx:rgba(255,255,255,0.4)}

/* --- RESET + BASE --- */
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx)}

/* --- APP SHELL: SIDEBAR + MAIN --- */
.app{display:grid;grid-template-columns:56px 1fr;min-height:100vh}
@media(max-width:768px){.app{grid-template-columns:1fr;grid-template-rows:auto 1fr}}

/* --- SIDEBAR (always dark) --- */
.nav{background:var(--nav);display:flex;flex-direction:column;align-items:center;padding:12px 0;gap:4px;border-right:1px solid rgba(255,255,255,0.06);z-index:10}
@media(max-width:768px){.nav{flex-direction:row;justify-content:space-around;order:1;padding:env(safe-area-inset-top,0) 8px 0;border-right:none;border-bottom:1px solid rgba(255,255,255,0.06);position:sticky;top:0}}
.nav .logo{width:32px;height:32px;margin-bottom:12px;opacity:0.7}
@media(max-width:768px){.nav .logo{display:none}}
.ni{display:flex;flex-direction:column;align-items:center;gap:2px;padding:10px 0;width:100%;cursor:pointer;color:var(--navtx);transition:color 0.15s;text-decoration:none;position:relative}
.ni svg{width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round}
.ni span{font-size:8px;letter-spacing:0.04em;text-transform:uppercase;font-weight:500}
.ni:hover,.ni.on{color:var(--navhi)}
.ni.on::before{content:'';position:absolute;left:0;top:8px;bottom:8px;width:2px;background:var(--acc);border-radius:0 2px 2px 0}
@media(max-width:768px){.ni.on::before{left:8px;right:8px;top:0;bottom:auto;width:auto;height:2px;border-radius:0 0 2px 2px}}
.ni .badge{position:absolute;top:6px;right:10px;background:var(--red);color:#fff;font-size:8px;font-weight:700;min-width:14px;height:14px;border-radius:7px;display:flex;align-items:center;justify-content:center;padding:0 3px}
@media(max-width:768px){.ni .badge{top:4px;right:50%;transform:translateX(14px)}}

/* --- MAIN CONTENT --- */
.main{overflow-y:auto;max-height:100vh}
@media(max-width:768px){.main{max-height:none;overflow:visible}}

/* --- TOPBAR --- */
.top{display:flex;align-items:center;justify-content:space-between;padding:10px 20px;background:var(--glass);backdrop-filter:var(--blur);border-bottom:1px solid var(--bd);position:sticky;top:0;z-index:5;flex-wrap:wrap;gap:6px}
.top h1{font-size:13px;font-weight:600;letter-spacing:-0.01em}
.top .rt{display:flex;align-items:center;gap:8px}
.clock{font-size:11px;color:var(--tx3);font-family:var(--mono)}
.thm{background:var(--bg3);border:1px solid var(--bd);border-radius:6px;padding:4px 8px;cursor:pointer;font-size:12px;color:var(--tx2);line-height:1;transition:background 0.15s}
.thm:hover{background:var(--bd)}
.pls{display:flex;gap:3px}
.pl{font-size:9px;padding:2px 7px;border-radius:8px;font-weight:600;letter-spacing:0.02em}
.pl-g{background:var(--accbg);color:var(--acctx)}
.pl-y{background:var(--amberbg);color:var(--ambertx)}
.pl-r{background:var(--redbg);color:var(--redtx)}

/* --- PAGES --- */
/* V2.67.2: max-width removed so content scales to available window width. */
.pg{display:none;padding:20px}
.pg.on{display:block}

/* --- ALERT BANNER --- */
.ab{display:none;align-items:center;gap:10px;padding:10px 14px;border-radius:var(--r);margin-bottom:14px;font-size:12px;background:var(--amberbg);color:var(--ambertx);border:1px solid var(--amberbd)}
.ab.err{background:var(--redbg);color:var(--redtx);border-color:var(--redbd)}

/* --- GLASS CARDS (metric cards, BMS cards, form panels) --- */
.mc{background:var(--glass);backdrop-filter:var(--blur);border-radius:var(--r);padding:14px 16px;border:1px solid var(--bd);box-shadow:var(--sh);transition:box-shadow 0.2s,transform 0.15s}
.mc:hover{box-shadow:var(--sh2);transform:translateY(-1px)}
.mc .l{font-size:9px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin-bottom:4px;font-weight:600}
.mc .n{font-size:22px;font-weight:600;letter-spacing:-0.02em}.mc .n.sm{font-size:15px;font-weight:500}
.mc .s{font-size:10px;color:var(--tx2);margin-top:3px}

/* --- METRIC GRIDS --- */
.mg{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-bottom:14px}
.mg5{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px;margin-bottom:14px}
@media(max-width:700px){.mg,.mg5{grid-template-columns:repeat(2,1fr)}.mc-w{grid-column:span 2}}

/* --- SECTION LABELS --- */
.sl{font-size:9px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.08em;margin:18px 0 10px;padding-bottom:6px;border-bottom:1px solid var(--bd);font-weight:600}

/* --- CHART CONTAINERS --- */
.cp{background:var(--glass);backdrop-filter:var(--blur);border-radius:var(--r);height:clamp(160px,18vw,280px);position:relative;overflow:hidden;margin:6px 0 0;border:1px solid var(--bd)}
.cp canvas{width:100%!important;height:100%!important}

/* --- CHART + BMS GRIDS (always stretch to fill) --- */
.cg{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px}
@media(max-width:700px){.cg{grid-template-columns:1fr}}
.bg{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:10px}
@media(max-width:600px){.bg{grid-template-columns:1fr}}
.bc{border:1px solid var(--bd);border-radius:var(--r);padding:14px 16px;border-left:3px solid var(--acc);background:var(--glass);backdrop-filter:var(--blur);box-shadow:var(--sh);transition:box-shadow 0.2s}
.bc:hover{box-shadow:var(--sh2)}
.bc.bcr{border-left-color:var(--red)}.bc.bcy{border-left-color:var(--amber)}.bc.bco{border-left-color:var(--tx3)}
.bc .hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.bc .hd .nm{font-weight:600;font-size:13px}
.bc .st{display:flex;gap:10px;font-size:11px;color:var(--tx2);flex-wrap:wrap}
.bc .st b{color:var(--tx);font-weight:600}
.cell-bar{display:flex;gap:2px;margin-top:8px;align-items:flex-end;height:clamp(36px,5vw,64px)}
.cell-bar .cb{flex:1;background:var(--acc);border-radius:3px 3px 0 0;min-width:0;transition:height 0.3s;opacity:0.8}
.cell-bar .cb.lo{background:var(--blue);opacity:1}.cell-bar .cb.hi{background:var(--red);opacity:1}
.cell-lbl{display:flex;justify-content:space-between;font-size:9px;color:var(--tx3);margin-top:3px}

/* --- EVENTS LIST --- */
.evl{max-height:160px;overflow-y:auto}
.evi{display:flex;justify-content:space-between;font-size:11px;padding:5px 0;border-bottom:1px solid var(--bd)}
.evi .ts{color:var(--tx3);font-family:var(--mono);white-space:nowrap;margin-left:8px;font-size:10px}

/* --- FORM ELEMENTS --- */
.fg{display:grid;grid-template-columns:1fr 1fr;gap:10px 14px}
@media(max-width:500px){.fg{grid-template-columns:1fr}}
.fg .fu{grid-column:1/-1}
.fg label{display:block;font-size:11px;color:var(--tx2);margin-bottom:4px;font-weight:500}
.fg input,.fg select{width:100%;padding:8px 10px;border:1px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px;outline:none;transition:border-color 0.15s}
.fg input:focus,.fg select:focus{border-color:var(--acc);box-shadow:0 0 0 3px var(--accbg)}
.fg input[readonly]{opacity:0.5}
.cr{display:flex;align-items:center;font-size:12px;padding:4px 0;cursor:pointer;margin-bottom:6px}
.cr input[type=checkbox]{width:16px;height:16px;accent-color:var(--acc);margin-right:10px;flex-shrink:0}

/* --- BUTTONS --- */
.bp{background:var(--acc);color:#fff;border:none;padding:10px 22px;border-radius:var(--r);font-size:12px;font-weight:600;cursor:pointer;transition:background 0.15s}
.bp:hover{background:var(--acc2)}
.bo{background:transparent;border:1px solid var(--bd2);padding:8px 16px;border-radius:var(--r);font-size:12px;cursor:pointer;color:var(--tx);transition:all 0.15s;display:inline-flex;align-items:center;justify-content:center;min-height:34px;white-space:nowrap}
.bo:hover{background:var(--bg3);border-color:var(--acc)}
/* V2.67.2: .bo.sm for compact secondary actions (Copy, Close, etc). */
.bo.sm{padding:4px 10px;font-size:11px;min-height:26px}
/* V2.67.2: .btn-row for equal-width button pairs with inline description. */
.btn-row{display:grid;grid-template-columns:180px 1fr;gap:12px;align-items:center;margin-top:10px}
.btn-row .bo{width:100%}
.btn-row .desc{font-size:11px;color:var(--tx2)}
@media(max-width:500px){.btn-row{grid-template-columns:1fr}}
/* V2.67.2: .lvl-opt for radio option with label on top line and description below. */
.lvl-opt{padding:10px 0;border-bottom:1px solid var(--bd)}
.lvl-opt:last-child{border-bottom:none}
.lvl-opt label{display:flex;align-items:center;gap:8px;cursor:pointer;font-size:12px;font-weight:500;color:var(--tx);margin-bottom:4px}
.lvl-opt label input[type=radio]{width:14px;height:14px;accent-color:var(--acc);flex-shrink:0}
.lvl-opt .desc{font-size:11px;color:var(--tx2);margin-left:22px;line-height:1.5}

/* --- INFO ROWS / NOTES --- */
.ir{display:flex;justify-content:space-between;font-size:12px;padding:6px 0;border-bottom:1px solid var(--bd)}
.ir .k{color:var(--tx2)}.ir .vl{font-weight:600}
.nt{font-size:11px;color:var(--tx3);margin-top:6px}
.ib{font-size:11px;color:var(--tx2);background:var(--bg3);border:1px solid var(--bd);border-radius:var(--r);padding:10px 14px;margin-top:8px;line-height:1.6}
.bpr{background:var(--bg3);border-radius:var(--r);padding:12px 14px;margin-top:10px;border:1px solid var(--bd)}
.bpr .row{display:flex;justify-content:space-between;font-size:12px;padding:3px 0}
.bpr .row .k{color:var(--tx2)}.bpr .row .vl{font-weight:600}

/* --- BATTERY TAB ELEMENTS --- */
.bd{border:1px solid var(--bd);border-radius:var(--r);padding:14px;margin-bottom:12px;background:var(--glass);backdrop-filter:var(--blur)}
.bd.bdy{border-left:3px solid var(--amberbd)}.bd.bdr{border-left:3px solid var(--redbd)}
.bd .bh{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.bd .bh .nm{font-weight:600;font-size:14px}
.tg{display:grid;grid-template-columns:repeat(7,1fr);gap:6px;margin-top:8px}
@media(max-width:500px){.tg{grid-template-columns:repeat(4,1fr)}}
.tc{background:var(--bg3);border-radius:var(--r);padding:8px 6px;text-align:center;border:1px solid var(--bd);transition:all 0.15s}
.tc .tv{font-size:14px;font-weight:600;margin-bottom:2px}
.tc .tl{font-size:9px;color:var(--tx3);text-transform:uppercase;font-weight:500}
.tc.tok .tv{color:var(--acctx)}
.tc.twa{border-color:var(--amberbd);background:var(--amberbg)}.tc.twa .tv{color:var(--ambertx)}
.tc.ter{border-color:var(--redbd);background:var(--redbg)}.tc.ter .tv{color:var(--redtx)}

/* --- SAVE BAR (for config pages) --- */
.foot{padding:12px 20px;text-align:right;background:var(--glass);backdrop-filter:var(--blur);border-top:1px solid var(--bd);position:sticky;bottom:0;display:none}
.foot.vis{display:block}

/* --- ALERTS PAGE --- */
.al-item{display:flex;align-items:flex-start;gap:12px;padding:12px 14px;border:1px solid var(--bd);border-radius:var(--r);margin-bottom:8px;background:var(--glass);backdrop-filter:var(--blur);border-left:3px solid var(--tx3)}
.al-item.al-err{border-left-color:var(--red)}.al-item.al-warn{border-left-color:var(--amber)}.al-item.al-info{border-left-color:var(--acc)}
.al-item.al-new{box-shadow:var(--sh2)}
.al-item .al-t{font-size:12px;font-weight:500;flex:1}.al-item .al-ts{font-size:10px;color:var(--tx3);font-family:var(--mono);white-space:nowrap}
.al-item .al-d{font-size:11px;color:var(--tx2);margin-top:2px}
.al-empty{text-align:center;color:var(--tx3);padding:40px 0;font-size:13px}

/* --- DEBUG LOG --- */
.dlog{font-family:var(--mono);font-size:11px;max-height:120px;overflow-y:auto;background:var(--bg3);padding:8px;border-radius:var(--r);margin-top:10px;display:none;border:1px solid var(--bd)}
.log-ok{color:var(--tx2);line-height:1.5}
.log-err{color:var(--redtx);line-height:1.5}
/* --- V2.67 F3: Tooltips for diag keys --- */
.tip{position:relative;display:inline-flex;align-items:center;justify-content:center;width:14px;height:14px;margin-left:6px;border-radius:7px;background:var(--bg3);color:var(--tx2);font-size:10px;font-weight:600;cursor:help;border:1px solid var(--bd);user-select:none}
.tip:hover{background:var(--accbg);color:var(--acctx);border-color:var(--acc)}
.tip:hover::after{content:attr(data-tip);position:absolute;bottom:calc(100% + 6px);left:50%;transform:translateX(-50%);background:var(--nav);color:#fff;padding:6px 10px;border-radius:var(--r);font-size:11px;font-weight:400;white-space:normal;width:260px;text-align:left;line-height:1.4;box-shadow:var(--sh2);z-index:20;pointer-events:none}
.tip:hover::before{content:'';position:absolute;bottom:calc(100% + 1px);left:50%;transform:translateX(-50%);border:5px solid transparent;border-top-color:var(--nav);z-index:20;pointer-events:none}
.dk-tbl{width:100%;font-size:12px;margin-top:8px;border-collapse:collapse}
.dk-tbl td{padding:4px 6px;border-bottom:1px solid var(--bd)}
.dk-tbl td:first-child{color:var(--tx2);font-family:var(--mono);white-space:nowrap}
.dk-tbl td:last-child{text-align:right;font-family:var(--mono);word-break:break-all}
</style>
</head>
<body>
<div class="app">

<!-- ==========================================
     SIDEBAR NAVIGATION
     5 items: Dashboard, Battery, Network, General, Alerts
     Always dark background, icons + micro-labels.
     Active indicator: left accent bar (desktop), bottom bar (mobile).
     ========================================== -->
<nav class="nav">
<svg class="logo" viewBox="0 0 32 32" fill="none" stroke="rgba(255,255,255,0.5)" stroke-width="1.5"><rect x="6" y="4" width="20" height="24" rx="3"/><line x1="12" y1="1" x2="12" y2="4"/><line x1="20" y1="1" x2="20" y2="4"/><line x1="10" y1="12" x2="22" y2="12"/><line x1="10" y1="17" x2="22" y2="17"/><line x1="10" y1="22" x2="22" y2="22"/></svg>
<a class="ni on" data-p="d" onclick="nav(this)"><svg viewBox="0 0 24 24"><rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="4" rx="1"/><rect x="3" y="14" width="7" height="4" rx="1"/><rect x="14" y="11" width="7" height="7" rx="1"/></svg><span>Dash</span></a>
<a class="ni" data-p="b" onclick="nav(this)"><svg viewBox="0 0 24 24"><rect x="3" y="6" width="18" height="14" rx="2"/><line x1="7" y1="3" x2="7" y2="6"/><line x1="17" y1="3" x2="17" y2="6"/><line x1="7" y1="11" x2="11" y2="11"/><line x1="9" y1="9" x2="9" y2="13"/><line x1="13" y1="11" x2="17" y2="11"/></svg><span>Battery</span></a>
<a class="ni" data-p="n" onclick="nav(this)"><svg viewBox="0 0 24 24"><path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><circle cx="12" cy="20" r="1"/></svg><span>Network</span></a>
<a class="ni" data-p="g" onclick="nav(this)"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/></svg><span>General</span></a>
<a class="ni" data-p="a" onclick="nav(this)"><svg viewBox="0 0 24 24"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"/><path d="M13.73 21a2 2 0 0 1-3.46 0"/></svg><span>Alerts</span><span class="badge" id="abadge" style="display:none">0</span></a>
</nav>

<!-- ==========================================
     MAIN CONTENT AREA
     ========================================== -->
<div class="main">

<!-- TOPBAR: status pills, alert indicator, user, theme, clock -->
<div class="top">
<h1>TopBand BMS Gateway</h1>
<div class="rt">
<div class="pls"><span class="pl pl-g" id="pb">BMS 0x</span><span class="pl pl-g" id="pc">CAN</span><span class="pl pl-g" id="pm">MQTT</span><span class="pl pl-g" id="pw">WiFi</span></div>
<span class="clock" id="hck">--:--</span>
<span id="hauth" style="display:none;font-size:11px;color:var(--tx2)"><span id="hauser"></span> <a href="/logout" style="color:var(--acc);text-decoration:none">Logout</a></span>
<button class="thm" id="thm" onclick="tgT()" title="Toggle theme">&#9790;</button>
</div>
</div>

<!-- FORM wraps all config pages (Battery, Network, General) -->
<form id="sf" action="/save" method="POST"><input type="hidden" name="exp" value="1"><input type="hidden" name="theme" value="0">

<!-- ==========================================
     PAGE: DASHBOARD (read-only live data)
     ========================================== -->
<div class="pg on" id="p-d">
<div class="ab" id="dab"><span>&#9888;</span><span id="dam"></span></div>
<div class="mg">
<div class="mc mc-w"><div class="l">SOC</div><div class="n" id="ds" style="color:var(--acc)">--%</div><div class="s" id="dss">-- / -- Ah</div></div>
<div class="mc"><div class="l">Power</div><div class="n" id="dp">-- W</div><div class="s" id="dst">Idle</div></div>
<div class="mc"><div class="l">Current</div><div class="n" id="dc">-- A</div><div class="s" id="dcs">Limit: --</div></div>
<div class="mc mc-w"><div class="l">Voltage</div><div class="n" id="dv">-- V</div><div class="s" id="dvs">CVL: --</div></div>
<div class="mc mc-w"><div class="l">Energy today</div><div class="n sm" id="dei" style="color:var(--acc)">-- kWh</div><div class="s" id="deo">Out: -- kWh</div></div>
</div>
<div class="mg5">
<div class="mc"><div class="l">Cell min</div><div class="n sm" id="dcn">-- V</div><div class="s" id="dcni">--</div></div>
<div class="mc"><div class="l">Cell max</div><div class="n sm" id="dcx">-- V</div><div class="s" id="dcxi">--</div></div>
<div class="mc mc-w"><div class="l">Cell drift</div><div class="n sm" id="dcd">-- V</div><div class="s" id="dcds">Limit: --</div></div>
<div class="mc"><div class="l">Temperature</div><div class="n sm" id="dtp">-- C</div><div class="s" id="dtps">SOH: --%</div></div>
<div class="mc"><div class="l">Runtime est.</div><div class="n sm" id="drt">--</div><div class="s" id="drts">&nbsp;</div></div>
</div>
<div class="cg">
<div><div style="margin-bottom:4px"><span style="font-size:11px;color:var(--tx3)" id="ch1lbl">Power (48h)</span></div><div class="cp"><canvas id="ch1"></canvas></div></div>
<div><div style="margin-bottom:4px"><span style="font-size:11px;color:var(--tx3)" id="ch2lbl">Voltage (48h)</span></div><div class="cp"><canvas id="ch2"></canvas></div></div>
</div>
<!-- V2.65.4 R6: info banner shown only while charts are less than half full -->
<div id="histInfo" style="display:none;margin-top:6px;padding:8px 12px;background:var(--bg3);border:1px solid var(--bd);border-radius:var(--r);font-size:11px;color:var(--tx2);text-align:center">
Chart data collects since last boot. 48-hour window fills over time. <span id="histInfoPct"></span>
</div>
<div class="sl">BMS packs</div>
<div class="bg" id="bcon"></div>
</div>

<!-- ==========================================
     PAGE: BATTERY (config + diagnostics)
     ========================================== -->
<div class="pg" id="p-b">
<div class="sl">Battery configuration</div>
<div class="fg"><div><label>BMS count</label><input type="number" name="cnt" id="i_cnt" min="1" max="16"></div><div><label>Cells per BMS (0 = auto)</label><input type="number" name="cells" id="i_cells" min="0" max="32"></div><div><label>SOC source</label><select name="soc_mode" id="i_soc_mode"><option value="2">Hybrid (recommended)</option><option value="1">Raw BMS</option><option value="0">Calculated</option></select></div></div>
<div class="sl" style="margin-top:16px">Temperature sensors</div>
<div id="btmp"><div style="color:var(--tx3);font-size:12px">Waiting for data...</div></div>
<div class="sl" style="margin-top:16px">BMS reported limits (read-only)</div>
<div id="bpro" class="bpr"><div style="color:var(--tx3);font-size:12px">Loading...</div></div>
<div class="nt">Hardware limits from BMS protocol 0x47.</div>
<div class="btn-row">
<button type="button" class="bo" onclick="$('svcSec').style.display=$('svcSec').style.display==='none'?'block':'none'">BMS diagnostics</button>
<span class="desc">Open service tools to read manufacturer info, alarms, system parameters, and event history from a selected pack.</span>
</div>
<div id="svcSec" style="display:none;margin-top:12px;border:1px solid var(--bd);border-radius:var(--r);padding:16px;background:var(--glass);backdrop-filter:var(--blur)">
<div style="font-size:13px;font-weight:500;margin-bottom:8px">BMS Diagnostics</div>
<div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:10px">
<select id="svcId" style="padding:7px 9px;border:1px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px"></select>
<button type="button" class="bp" id="svcRdBtn" onclick="svcRd()">Read diagnostics</button>
<button type="button" class="bo" onclick="svcTm()">Sync time</button>
</div>
<div id="svcMsg" class="nt" style="display:none"></div>
<div id="svcOut"></div>
</div>
<div class="sl" style="margin-top:16px">Gateway limits</div>
<div class="btn-row">
<button type="button" class="bo" id="acbtn" onclick="acBatt()">Auto-configure</button>
<span id="acst" class="desc">Read limits from connected BMS packs and pre-fill the fields below. Values are not saved until you press Save.</span>
</div>
<div class="fg">
<div><label>Charge current (A per pack)</label><input type="number" step="0.1" name="chg" id="i_chg"></div>
<div><label>Discharge current (A per pack)</label><input type="number" step="0.1" name="dis" id="i_dis"></div>
<div><label>Charge voltage CVL (V)</label><input type="number" step="0.01" name="cvl" id="i_cvl"></div>
<div><label>Max pack voltage (V)</label><input type="number" step="0.01" name="s_vol" id="i_svol"></div>
<div><label>Max cell voltage (V)</label><input type="number" step="0.01" name="s_cel" id="i_scel"></div>
<div><label>Max cell drift (V)</label><input type="number" step="0.01" name="s_drift" id="i_sdrift"></div>
</div>
<div class="sl" style="margin-top:16px">Temperature cutoff (C)</div>
<div class="fg">
<div><label>Charge min</label><input type="number" step="0.1" name="t_c_min" id="i_tcmin"></div>
<div><label>Charge max</label><input type="number" step="0.1" name="t_c_max" id="i_tcmax"></div>
<div><label>Discharge min</label><input type="number" step="0.1" name="t_d_min" id="i_tdmin"></div>
<div><label>Discharge max</label><input type="number" step="0.1" name="t_d_max" id="i_tdmax"></div>
<div><label>Evaluation mode</label><select name="t_mode" id="i_tmode"><option value="0">Hottest sensor (safe)</option><option value="1">Average</option></select></div>
</div>
<!-- V2.66 M4: Configurable spike filter thresholds -->
<div class="sl" style="margin-top:16px">Spike filter</div>
<div class="ib" style="grid-column:1/-1;font-size:11px;color:var(--tx2);margin-bottom:8px">Rejects implausible poll-to-poll jumps in BMS readings. Increase if your packs legitimately swing faster; decrease for noisy RS485 buses.</div>
<div class="fg">
<div><label>Max voltage jump (V)</label><input type="number" step="0.1" min="0.5" max="20" name="sp_v" id="i_spv"></div>
<div><label>Max current jump (A)</label><input type="number" step="1" min="10" max="500" name="sp_a" id="i_spa"></div>
<div><label>Max SOC jump (%)</label><input type="number" step="1" min="5" max="50" name="sp_s" id="i_sps"></div>
</div>
<div class="sl" style="margin-top:16px">Charge management</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="maint_en" id="i_maint_en"><span>Maintenance charge mode</span></label></div>
<div><label>Maintenance target (V)</label><input type="number" step="0.1" name="maint_v" id="i_maint_v"></div>
<div class="fu"><label class="cr"><input type="checkbox" name="ab_en" id="i_ab_en"><span>Auto balance watchdog (30 days)</span></label></div>
</div>
<div class="sl" style="margin-top:16px">Energy</div>
<div class="mg5">
<div class="mc"><div class="l">Today in</div><div class="n sm" id="bet_i" style="color:var(--acc)">--</div><div class="s" id="bet_o">Out: --</div></div>
<div class="mc"><div class="l">Week (7d) in</div><div class="n sm" id="bew_i">--</div><div class="s" id="bew_o">Out: --</div></div>
<div class="mc"><div class="l">Month in</div><div class="n sm" id="bem_i">--</div><div class="s" id="bem_o">Out: --</div></div>
</div>
<div style="font-size:10px;color:var(--tx3);margin-top:2px" id="bedays"></div>
</div>

<!-- ==========================================
     PAGE: NETWORK (WiFi, CAN, MQTT, HA)
     ========================================== -->
<div class="pg" id="p-n">
<div class="sl">WiFi</div>
<div class="fg" style="grid-template-columns:1fr 1fr 1fr"><div><label>IP address</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nip">--</div></div><div><label>Signal</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nrs">--</div></div><div><label>Hostname</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nhn">--</div></div></div>
<div class="sl" style="margin-top:16px">CAN-Bus</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="vic_en" id="i_vic_en"><span>Enable CAN-Bus</span></label></div><div><label>Protocol</label><select name="can_proto" id="i_can_proto"><option value="0">Standard (Victron)</option><option value="1">Pylontech Strict</option><option value="2">SMA Sunny Island</option></select></div></div>
<div class="sl" style="margin-top:16px">MQTT</div>
<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:8px 0 8px;font-weight:600">Broker</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="mq_en" id="i_mq_en"><span>Enable MQTT</span></label></div>
<div><label>Broker IP</label><input type="text" name="mq_ip" id="i_mq_ip" placeholder="192.168.1.x"></div>
<div><label>Port</label><input type="number" name="mq_pt" id="i_mq_pt"></div>
<div><label>User</label><input type="text" name="mq_us" id="i_mq_us"></div>
<div><label>Password</label><input type="password" name="mq_pw" id="i_mq_pw" placeholder="(unchanged)"></div>
<div><label>Topic</label><input type="text" name="mq_top" id="i_mq_top" placeholder="Topband/BMS"></div>
<div><label>Device ID</label><input type="text" id="nuid" readonly></div>
</div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:16px 0 8px;font-weight:600">Home Assistant</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="ha_en" id="i_ha_en"><span>Auto-discovery</span></label></div></div>
<div class="btn-row">
<button type="button" class="bo" onclick="haS()">Send discovery</button>
<span class="desc">Publishes retained discovery configs for all active entities. Home Assistant picks them up automatically.</span>
</div>
<div class="btn-row">
<button type="button" class="bo" onclick="haC()">Clear discovery</button>
<span class="desc">Removes all retained discovery configs. Entities disappear from Home Assistant.</span>
</div>
<div id="has" class="nt"></div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:16px 0 8px;font-weight:600">Battery data (always on)</div>
<div class="nt" style="margin-top:0">Pack-level values: voltage, current, state of charge, minimum and maximum cell voltage, alarms, energy totals. Always published while MQTT is enabled.</div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:16px 0 8px;font-weight:600">Extended battery monitoring (optional)</div>
<div class="lvl-opt">
<label><input type="radio" name="mq_level" value="0" id="i_mq_lv0"><span>Off</span></label>
<div class="desc">Nothing beyond the core data above.</div>
</div>
<div class="lvl-opt">
<label><input type="radio" name="mq_level" value="1" id="i_mq_lv1"><span>Per-pack statistics</span></label>
<div class="desc">One topic per pack with cell voltage spread, communication counters, timeouts, errors, rejected spikes, held current readings. For diagnosing a single pack.</div>
</div>
<div class="lvl-opt">
<label><input type="radio" name="mq_level" value="2" id="i_mq_lv2"><span>Additional per-cell voltages</span></label>
<div class="desc">Per-pack statistics plus a retained topic every 20 seconds with every cell voltage and pack temperature. Heavy traffic.</div>
</div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:16px 0 8px;font-weight:600">Gateway self-monitoring (optional, separate)</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="mq_diag" id="i_mq_diag"><span>Publish gateway diagnostics</span></label></div></div>
<div class="nt" style="margin-top:0">Separate topic every 30 seconds with the gateway's own health: memory, CAN errors, uptime, reset counters. About the ESP32 itself, not the batteries.</div>
</div>

<!-- ==========================================
     PAGE: GENERAL (preferences, security, device info, OTA)
     ========================================== -->
<div class="pg" id="p-g">
<div class="sl">Hardware</div>
<div class="fg">
<div><label>Board</label><select name="board_type" id="i_board" onchange="tgPins()"><option value="0">Waveshare ESP32-S3 RS485/CAN Controller</option><option value="1">LilyGo T-CAN485</option><option value="2">Custom (set pins manually)</option></select></div>
</div>
<div id="pinFields" style="display:none;margin-top:8px">
<div class="fg" style="grid-template-columns:repeat(3,1fr)">
<div><label>RS485 TX</label><input type="number" name="p_rs_tx" id="i_p_rs_tx" min="0" max="48"></div>
<div><label>RS485 RX</label><input type="number" name="p_rs_rx" id="i_p_rs_rx" min="0" max="48"></div>
<div><label>RS485 DIR (-1=none)</label><input type="number" name="p_rs_dir" id="i_p_rs_dir" min="-1" max="48"></div>
<div><label>CAN TX</label><input type="number" name="p_can_tx" id="i_p_can_tx" min="0" max="48"></div>
<div><label>CAN RX</label><input type="number" name="p_can_rx" id="i_p_can_rx" min="0" max="48"></div>
<div><label>LED (-1=none)</label><input type="number" name="p_led" id="i_p_led" min="-1" max="48"></div>
</div>
</div>
<div id="pinInfo" class="ib" style="margin-top:8px"></div>
<div class="sl" style="margin-top:16px">Preferences</div>
<div class="fg"><div class="fu" style="display:flex;gap:14px"><div style="flex:1"><label>Chart left</label><select name="chart1" id="i_ch1" onchange="svCh();chDirty=true;upd()"><option value="0">Power</option><option value="1">Voltage</option><option value="2">SOC</option><option value="3">Temperature</option></select></div><div style="flex:1"><label>Chart right</label><select name="chart2" id="i_ch2" onchange="svCh();chDirty=true;upd()"><option value="0">Power</option><option value="1">Voltage</option><option value="2">SOC</option><option value="3">Temperature</option></select></div></div>
<div class="ib" style="grid-column:1/-1">The dashboard shows two charts covering the last 48 hours. A new data point is recorded every 3 minutes, capturing the average, minimum, and maximum values over that interval. The shaded area behind the line shows the min/max range. Chart data is saved to flash memory every 15 minutes and survives reboots and firmware updates. Chart selection is saved instantly and applies to all devices.</div>
<div class="fu" style="display:flex;gap:14px"><div style="flex:1"><label>NTP server</label><input type="text" name="ntp_svr" id="i_ntp"></div><div style="flex:1;max-width:140px"><label>Timezone (GMT+)</label><input type="number" name="tz_off" id="i_tz"></div></div></div>
<div class="sl" style="margin-top:16px">Security</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="auth_en" id="i_auth_en" onchange="tgAuth()"><span>Enable web authentication</span></label></div>
<div id="authFields" style="display:none;grid-column:1/-1"><div class="fg"><div><label>Username</label><input type="text" name="auth_user" id="i_auth_user" maxlength="32" autocomplete="username"></div><div><label>New password</label><input type="password" name="auth_pw" id="i_auth_pw" placeholder="(unchanged)" autocomplete="new-password"></div></div></div>
</div>
<div class="sl" style="margin-top:16px">Diagnostics</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="debug" id="i_debug"><span>Serial debug output</span></label></div></div>
<!-- V2.67.2: Live log with header and aligned Copy button -->
<div style="display:flex;justify-content:space-between;align-items:center;margin-top:10px;margin-bottom:6px">
<div class="sl" style="margin:0;border:none;padding:0">Log (live)</div>
<button type="button" class="bo sm" onclick="copyLog()">Copy log</button>
</div>
<div class="dlog" id="dlog"></div>
<!-- V2.67.2: Tools grid with equal-width buttons and inline descriptions -->
<div class="sl" style="margin-top:16px">Tools</div>
<div class="btn-row">
<button type="button" class="bo" id="spyBtn" onclick="spyToggle()">Frame spy</button>
<span id="spySt" class="desc">Capture RS485 responses from BMS for 5 minutes. Raw hex plus decoded current and voltage per frame.</span>
</div>
<div class="btn-row">
<button type="button" class="bo" id="dkBtn" onclick="dkToggle()">Runtime counters</button>
<span class="desc">Live communication counters, rejected spikes, held current readings, and worst-case loop times. Hover the ? icons for explanations.</span>
</div>
<!-- Frame spy output panel -->
<div id="spyPanel" class="dlog" style="display:none;max-height:280px;margin-top:10px"></div>
<div id="spyActions" style="display:none;gap:8px;margin-top:8px">
<button type="button" class="bo sm" id="spyCopyBtn" onclick="copySpy()">Copy hex</button>
<button type="button" class="bo sm" id="spyCloseBtn" onclick="spyClose()">Close</button>
</div>
<!-- Runtime counters panel -->
<div id="dkPanel" style="display:none;margin-top:10px;border:1px solid var(--bd);border-radius:var(--r);padding:14px;background:var(--glass)">
<div class="sl" style="margin:0 0 10px 0">Runtime counters</div>
<table class="dk-tbl"><tbody id="dkBody"><tr><td colspan="2" style="color:var(--tx3);text-align:center">Loading...</td></tr></tbody></table>
<div style="margin-top:12px;display:flex;align-items:center;gap:12px;flex-wrap:wrap;padding-top:10px;border-top:1px solid var(--bd)">
<button type="button" class="bo sm" id="rcBtn" onclick="rcReset()">Reset counters</button>
<span class="nt" id="rcSt" style="font-size:11px;color:var(--tx3);flex:1;margin:0">Zeros communication counters, error counters, rejected spikes, held current readings. Heap min, worst-case loop times, RS485 stack watermark, watchdog warnings, and CAN failure streak are preserved. Auto-rolls every 7 days.</span>
</div>
</div>
<div class="sl" style="margin-top:16px">About</div>
<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:8px 0 6px;font-weight:600">Build info</div>
<div class="ir"><span class="k">Firmware</span><span class="vl" id="gfw">--</span></div>
<div class="ir"><span class="k">Build</span><span class="vl" id="gbuild">--</span></div>
<div class="ir"><span class="k">Compiled</span><span class="vl" id="gcompile">--</span></div>
<div class="ir"><span class="k">ESP32 Arduino core</span><span class="vl" id="gsdk">--</span></div>
<div class="ir"><span class="k">License</span><span class="vl" id="glic">--</span></div>
<div class="ir"><span class="k">Project</span><span class="vl"><a id="ggh" href="#" target="_blank" rel="noopener noreferrer" style="color:var(--acc);text-decoration:none">GitHub</a></span></div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:14px 0 6px;font-weight:600">Runtime state</div>
<div class="ir"><span class="k">Uptime</span><span class="vl" id="gup">--</span></div>
<div class="ir"><span class="k">Last boot time</span><span class="vl" id="gboottime">--</span></div>
<div class="ir"><span class="k">Last boot reason</span><span class="vl" id="gboot">--</span></div>
<div class="ir"><span class="k">Free heap</span><span class="vl" id="ghp">--</span></div>
<div class="ir" id="gpsram_row" style="display:none"><span class="k">Free PSRAM</span><span class="vl" id="gpsram">--</span></div>
<div class="ir"><span class="k">Flash usage</span><span class="vl" id="gflash">--</span></div>
<div class="ir"><span class="k">NVS usage</span><span class="vl" id="gnvs">--</span></div>

<div style="font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:14px 0 6px;font-weight:600">Hardware</div>
<div class="ir"><span class="k">Device ID</span><span class="vl" id="guid">--</span></div>
<div class="ir"><span class="k">MAC address</span><span class="vl" id="gmac">--</span></div>
<div class="ir"><span class="k">Chip</span><span class="vl" id="gchip">--</span></div>
<div class="ir"><span class="k">Chip temperature</span><span class="vl" id="gctemp">--</span></div>
<div class="ir"><span class="k">GPIO pins</span><span class="vl" id="gpins">--</span></div>
<div class="sl" style="margin-top:16px">Maintenance</div>
<div class="btn-row">
<button type="button" class="bo" onclick="$('otaSec').style.display=$('otaSec').style.display==='none'?'block':'none'">Firmware update</button>
<span class="desc">Upload a new firmware binary. Gateway reboots automatically, settings are preserved.</span>
</div>
<div class="btn-row">
<a href="/backup" class="bo" style="text-decoration:none">Download backup</a>
<span class="desc">Export all settings as a JSON file: pin config, limits, MQTT credentials, auth hash.</span>
</div>
<div class="btn-row">
<button type="button" class="bo" id="rbtn" onclick="$('rfile').click()">Restore backup</button>
<span class="desc">Load previously exported settings from a JSON file. Triggers a reboot.</span>
</div>
<input type="file" id="rfile" accept=".json" style="display:none" onchange="doRestore(this)">
<div class="btn-row">
<a href="/export" class="bo" style="text-decoration:none">Export history</a>
<span class="desc">Download the last 48 hours of chart history as a CSV file.</span>
</div>
<div id="rmsg" class="nt"></div>
<div id="otaSec" style="display:none;margin-top:12px;border:1px solid var(--bd);border-radius:var(--r);padding:16px;background:var(--glass);backdrop-filter:var(--blur)">
<div style="font-size:13px;font-weight:500;margin-bottom:8px">OTA Firmware Update</div>
<div id="otaDrop" style="border:1.5px dashed var(--bd2);border-radius:var(--r);padding:24px;text-align:center;cursor:pointer;color:var(--tx2);font-size:12px;transition:border-color 0.2s">
<input type="file" id="otaFile" accept=".bin" style="display:none"><label for="otaFile" style="cursor:pointer" id="otaLbl">Select .bin file or drag and drop</label></div>
<div id="otaProg" style="display:none;margin-top:10px"><div style="width:100%;height:4px;background:var(--bg3);border-radius:2px;overflow:hidden"><div id="otaBar" style="height:100%;background:var(--acc);width:0%;transition:width 0.3s;border-radius:2px"></div></div></div>
<div id="otaMsg" class="nt" style="margin-top:6px"></div>
<button type="button" class="bp" id="otaBtn" disabled style="margin-top:10px;opacity:0.5" onclick="otaUp()">Upload firmware</button>
<div class="nt">ESP32 reboots automatically. Settings are preserved.</div>
</div>
</div>

<!-- ==========================================
     PAGE: ALERTS (event history with "new" tracking)
     ========================================== -->
<div class="pg" id="p-a">
<div style="display:flex;justify-content:space-between;align-items:center"><div class="sl" style="margin:0;border:none;padding:0">System alerts</div><div style="display:flex;gap:8px;align-items:center"><span class="al-cnt" id="acnt" style="font-size:10px;color:var(--tx3)"></span><button type="button" class="bo sm" onclick="clearAlertsSrv()">Clear</button></div></div>
<div id="alertList" style="margin-top:10px"><div class="al-empty">No alerts</div></div>
</div>

</form>
<!-- Save bar: visible on Battery, Network, General pages -->
<div class="foot" id="sbar"><button type="submit" form="sf" class="bp">Save and reboot</button></div>
</div>
</div>

<!-- ==========================================
     JAVASCRIPT
     Navigation, theme, data polling, charts, alerts.
     All element IDs match backend /data and /save endpoints.
     ========================================== -->
<script>
/* --- GLOBALS --- */
var D=null,I=false,prL=false,chDirty=true,lastPwr=0;
var reqC1=0,reqC2=1;  // Track what chart types were actually requested (matches data)
var lastBmsRender=0;  // Throttle BMS card rebuilds (ms timestamp)
var alertLog=[];  // V2.61: populated from server /data D.alerts (NVS-backed ring buffer of last 25)
var lastAlertTs=parseInt(localStorage.getItem('tbLastAlert')||'0');
/* V2.61: POST /alerts/clear to wipe RAM + NVS on the gateway */
 function clearAlertsSrv(){
if(!confirm('Clear all alerts? This will wipe the stored alert history on the device.'))return;
fetch('/alerts/clear',{method:'POST'}).then(function(r){return r.json()}).then(function(d){
if(d&&d.ok){alertLog=[];updAlertPage();updAlertBadge()}}).catch(function(){
/* fallback to GET if POST blocked */
fetch('/alerts/clear').then(function(){alertLog=[];updAlertPage();updAlertBadge()})})}

 function $(i){return document.getElementById(i)}
 function fV(v){return v!=null?v.toFixed(2):'--'}
 function fV3(v){return v!=null?v.toFixed(3):'--'}
 function fA(v){return v!=null?v.toFixed(1):'--'}
 function tS(ts){if(!ts)return'--';var d=new Date(ts*1000);return d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0')}
 function uT(ms){var s=Math.floor(ms/1000),m=Math.floor(s/60),h=Math.floor(m/60),d=Math.floor(h/24);if(d>0)return d+'d '+h%24+'h';if(h>0)return h+'h '+m%60+'m';return m+'m'}
 function tCl(v,mn,mx){if(v==null)return'tok';if(v>=mx-3||v<=mn+2)return'ter';if(v>=mx-8||v<=mn+5)return'twa';return'tok'}
 function pd(n){return String(n).padStart(2,'0')}
 function esc(s){if(!s)return'';var d=document.createElement('div');d.textContent=s;return d.innerHTML}

/* --- SIDEBAR NAVIGATION --- */
 function nav(el){
document.querySelectorAll('.ni').forEach(function(x){x.classList.remove('on')});
document.querySelectorAll('.pg').forEach(function(x){x.classList.remove('on')});
el.classList.add('on');
var pg=$('p-'+el.dataset.p);if(pg)pg.classList.add('on');
// Show save bar only on config pages
var isConfig=el.dataset.p!=='d'&&el.dataset.p!=='a';
$('sbar').classList.toggle('vis',isConfig);
// Mark alerts as seen when visiting alerts page
if(el.dataset.p==='a'){lastAlertTs=Math.floor(Date.now()/1000);localStorage.setItem('tbLastAlert',String(lastAlertTs));updAlertBadge()}
// Redraw charts when switching to dashboard
if(el.dataset.p==='d'){chDirty=true;drAll()}
}

/* --- THEME TOGGLE --- */
setInterval(function(){var d=new Date();$('hck').textContent=d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0')},5000);
var dk=localStorage.getItem('theme')==='dark'||(localStorage.getItem('theme')===null&&matchMedia('(prefers-color-scheme:dark)').matches);
 function apT(){var r=document.documentElement;if(dk){r.classList.add('dk');r.classList.remove('lt')}else{r.classList.add('lt');r.classList.remove('dk')}$('thm').textContent=dk?'\u2600':'\u263E'}
 function tgT(){dk=!dk;localStorage.setItem('theme',dk?'dark':'light');apT()}apT();
 function tgAuth(){var on=$('i_auth_en').checked;$('authFields').style.display=on?'block':'none'}
 function tgPins(){
var bt=parseInt($('i_board').value);
$('pinFields').style.display=bt===2?'block':'none';
var presets={0:{tx:17,rx:18,dir:21,ctx:15,crx:16,led:38,info:'Waveshare ESP32-S3 RS485/CAN Controller: RS485 via DIR pin (GPIO 21)'},1:{tx:22,rx:21,dir:-1,ctx:27,crx:26,led:4,info:'LilyGo T-CAN485: RS485 via EN/CB pins, SD card slot'},2:{tx:0,rx:0,dir:-1,ctx:0,crx:0,led:-1,info:'Set GPIO pins for your board. DIR=-1 if your RS485 transceiver has auto-direction. LED=-1 to disable.'}};
var p=presets[bt]||presets[0];
if(bt<2){$('i_p_rs_tx').value=p.tx;$('i_p_rs_rx').value=p.rx;$('i_p_rs_dir').value=p.dir;$('i_p_can_tx').value=p.ctx;$('i_p_can_rx').value=p.crx;$('i_p_led').value=p.led}
$('pinInfo').textContent=p.info}
 function svCh(){fetch('/chartcfg?c1='+$('i_ch1').value+'&c2='+$('i_ch2').value)}
window.onresize=function(){chDirty=true;drAll()};

/* --- ALERT TRACKING (V2.61: alertLog populated from server NVS ring buffer) --- */
/* Legacy no-op kept in case any extension script still calls it */
 function addAlert(sev,msg,ts){return}
 function updAlertBadge(){
var cnt=0;for(var i=0;i<alertLog.length;i++){if(alertLog[i].ts>lastAlertTs)cnt++}
var b=$('abadge');if(cnt>0){b.textContent=cnt;b.style.display='flex'}else{b.style.display='none'}
}
 function updAlertPage(){
var c=$('alertList');if(!c)return;
var ac=$('acnt');if(ac){var ec=0,wc=0,ic=0;alertLog.forEach(function(a){if(a.sev==='err')ec++;else if(a.sev==='warn')wc++;else ic++});ac.textContent=alertLog.length>0?(ec+' errors, '+wc+' warnings, '+ic+' info'):''}
if(alertLog.length===0){c.innerHTML='<div class="al-empty">No alerts</div>';return}
var h='';alertLog.forEach(function(a){
var cls='al-item';if(a.sev==='err')cls+=' al-err';else if(a.sev==='warn')cls+=' al-warn';else cls+=' al-info';
if(a.ts>lastAlertTs)cls+=' al-new';
var tstr;
if(a.ts&&a.ts>1700000000){var d=new Date(a.ts*1000);tstr=pd(d.getHours())+':'+pd(d.getMinutes())+':'+pd(d.getSeconds())}
else if(a.up){/* V2.61: no NTP when raised, show as "Xh Ym uptime" */
var u=a.up;var hh=Math.floor(u/3600),mm=Math.floor((u%3600)/60);tstr=(hh>0?hh+'h '+mm+'m':mm+'m')+' up'}
else tstr='--';
h+='<div class="'+cls+'"><div style="flex:1"><div class="al-t">'+esc(a.msg)+'</div></div><span class="al-ts">'+tstr+'</span></div>'
});c.innerHTML=h
}

/* --- CHART RENDERING (V2.61: uses hstart instead of zero-filter heuristic) --- */
var CM=['Power','Voltage','SOC','Temperature'];var CU=['W','V','%','C'];var CC=['#059669','#3b82f6','#8b5cf6','#f59e0b'];
var CSC=[1,0.01,0.1,0.1];
 function drAll(){if(!D||!chDirty)return;chDirty=false;
var c1=reqC1,c2=reqC2;
$('ch1lbl').textContent=CM[c1]+' (48h)';$('ch2lbl').textContent=CM[c2]+' (48h)';
var hs=D.hstart||0;
drLine('ch1',D.hist1,c1,D.h1mn,D.h1mx,hs);
drLine('ch2',D.hist2,c2,D.h2mn,D.h2mx,hs)}
 function drLine(id,data,typ,dmin,dmax,skip){var c=$(id);if(!c||!data||data.length<2)return;
var w=c.parentElement.clientWidth,h=c.parentElement.clientHeight||180;if(w<10||h<10)return;
c.width=w;c.height=h;var ctx=c.getContext('2d');
var sc=CSC[typ]||1,col=CC[typ]||'#059669',unit=CU[typ]||'';
var pad=40,bt=24,tp=8;
skip=skip||0;if(skip>=data.length-1)skip=0;
/* V2.61: trim leading unfilled slots (reported by server via hstart) */
var src=skip>0?data.slice(skip):data;
var srcMn=(skip>0&&dmin)?dmin.slice(skip):dmin;
var srcMx=(skip>0&&dmax)?dmax.slice(skip):dmax;
if(src.length<2)return;
var pv=[],pmn=[],pmx=[];
for(var i=0;i<src.length;i++){pv.push(src[i]*sc);pmn.push(srcMn?srcMn[i]*sc:src[i]*sc);pmx.push(srcMx?srcMx[i]*sc:src[i]*sc)}
/* Y-axis range from all real points (no more zero-filter hack) */
var mn=Infinity,mx=-Infinity,hasData=false;
for(i=0;i<pv.length;i++){
var lo=pmn[i]<pv[i]?pmn[i]:pv[i];var hi2=pmx[i]>pv[i]?pmx[i]:pv[i];
if(lo<mn)mn=lo;if(hi2>mx)mx=hi2;hasData=true}
if(!hasData)return;
if(mn===mx){if(typ===2){mn=0;mx=100}else if(typ===1){mn=mn-2;mx=mx+2}else{mn=mn-50;mx=mx+50}}
var rng=mx-mn;if(rng<0.5)rng=0.5;mn-=rng*0.08;mx+=rng*0.08;rng=mx-mn;
var dh=h-bt-tp;
 function gy(v){return tp+dh-((v-mn)/rng*dh)}
ctx.strokeStyle='rgba(128,128,128,0.08)';ctx.lineWidth=0.5;
for(var s=0;s<=3;s++){var v2=mn+(rng*(s/3)),y=gy(v2);ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w,y);ctx.stroke();
ctx.fillStyle='var(--tx3)';ctx.font='10px sans-serif';ctx.textAlign='right';
var lbl=(typ===1||typ===3)?v2.toFixed(1):Math.round(v2);ctx.fillText(lbl,pad-4,y+3)}
if(typ===0&&mn<0&&mx>0){var y0=gy(0);ctx.setLineDash([3,3]);ctx.strokeStyle='rgba(128,128,128,0.2)';ctx.beginPath();ctx.moveTo(pad,y0);ctx.lineTo(w,y0);ctx.stroke();ctx.setLineDash([])}
var sx=(w-pad)/(pv.length-1);
/* Min/max band */
if(srcMn&&srcMx){var cr2=parseInt(col.slice(1,3),16),cg=parseInt(col.slice(3,5),16),cb=parseInt(col.slice(5,7),16);
ctx.beginPath();for(i=0;i<pv.length;i++){var x=pad+i*sx;if(i===0)ctx.moveTo(x,gy(pmx[i]));else ctx.lineTo(x,gy(pmx[i]))}
for(i=pv.length-1;i>=0;i--){ctx.lineTo(pad+i*sx,gy(pmn[i]))}ctx.closePath();
ctx.fillStyle='rgba('+cr2+','+cg+','+cb+',0.06)';ctx.fill()}
/* Area fill */
var grd=ctx.createLinearGradient(0,tp,0,h-bt);
if(typ===0){grd.addColorStop(0,'rgba(5,150,105,0.08)');grd.addColorStop(0.5,'rgba(5,150,105,0.01)');grd.addColorStop(0.5,'rgba(217,119,6,0.01)');grd.addColorStop(1,'rgba(217,119,6,0.08)')}
else{var r2=parseInt(col.slice(1,3),16),g3=parseInt(col.slice(3,5),16),b3=parseInt(col.slice(5,7),16);grd.addColorStop(0,'rgba('+r2+','+g3+','+b3+',0.06)');grd.addColorStop(1,'rgba('+r2+','+g3+','+b3+',0.01)')}
var baseY=typ===0&&mn<0?gy(0):h-bt;
ctx.beginPath();ctx.moveTo(pad,baseY);ctx.lineTo(pad,gy(pv[0]));
for(i=1;i<pv.length;i++){ctx.lineTo(pad+i*sx,gy(pv[i]))}
ctx.lineTo(pad+(pv.length-1)*sx,baseY);ctx.fillStyle=grd;ctx.fill();
/* Main line */
ctx.beginPath();ctx.moveTo(pad,gy(pv[0]));for(i=1;i<pv.length;i++){ctx.lineTo(pad+i*sx,gy(pv[i]))}
ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.stroke();
/* Last point marker */
var lastIdx=pv.length-1;
var lx=pad+lastIdx*sx,ly=gy(pv[lastIdx]);
ctx.beginPath();ctx.arc(lx,ly,3,0,Math.PI*2);ctx.fillStyle=col;ctx.fill();
ctx.beginPath();ctx.arc(lx,ly,5,0,Math.PI*2);ctx.strokeStyle=col;ctx.lineWidth=1;ctx.globalAlpha=0.3;ctx.stroke();ctx.globalAlpha=1;
ctx.fillStyle=col;ctx.font='10px sans-serif';ctx.textAlign='right';
var nv=pv[lastIdx];ctx.fillText((typ===1||typ===3?nv.toFixed(1):Math.round(nv))+' '+unit,lx-8,ly-8);
/* Time axis labels (adjusted for trimmed length).
   V2.65.4 R4: minPer comes from backend so labels match current resolution. */
ctx.fillStyle='rgba(128,128,128,0.4)';ctx.font='9px sans-serif';ctx.textAlign='center';
var ts2=D.ts||0;var minPer=(D&&D.minPer)?D.minPer:15;
for(i=0;i<pv.length;i+=Math.max(1,Math.floor(pv.length/6))){var t=new Date((ts2-((pv.length-1-i)*minPer*60))*1000);ctx.fillText(String(t.getHours()).padStart(2,'0')+':00',pad+i*sx,h-4)}}

/* V2.61: ResizeObserver + delayed redraws fix mobile rendering race */
var _chartObs=null;
 function initChartObserver(){if(_chartObs||!window.ResizeObserver)return;
_chartObs=new ResizeObserver(function(){chDirty=true;drAll()});
var cps=document.querySelectorAll('.cp');for(var i=0;i<cps.length;i++){_chartObs.observe(cps[i])}}

/* --- HELPER FUNCTIONS (HA, AutoCfg, Restore, OTA) --- */
 function haS(){var s=$('has');s.textContent='Sending...';fetch('/ha/discovery/send').then(function(r){return r.json()}).then(function(d){s.textContent=d.ok?'Discovery sent':'Failed: '+(d.err||'');s.style.color=d.ok?'var(--acctx)':'var(--redtx)'}).catch(function(e){s.textContent='Error';s.style.color='var(--redtx)'})}
 function haC(){var s=$('has');s.textContent='Clearing...';fetch('/ha/discovery/clear').then(function(r){return r.json()}).then(function(d){s.textContent=d.ok?'Cleared':'Failed: '+(d.err||'');s.style.color=d.ok?'var(--acctx)':'var(--redtx)'}).catch(function(e){s.textContent='Error';s.style.color='var(--redtx)'})}
 function acBatt(){var s=$('acst');s.textContent='Reading BMS...';s.style.color='var(--tx2)';$('acbtn').disabled=true;
fetch('/svc/autocfg/apply?id=0').then(function(r){return r.json()}).then(function(d){$('acbtn').disabled=false;
if(!d.ok||!d.applied){s.textContent='Failed: '+(d.err||'');s.style.color='var(--redtx)';return}
var a=d.applied;
if(a.s_vol!=null)$('i_svol').value=a.s_vol;if(a.s_cel!=null)$('i_scel').value=a.s_cel;
if(a.chg!=null)$('i_chg').value=a.chg;if(a.dis!=null)$('i_dis').value=a.dis;
if(a.cvl!=null)$('i_cvl').value=a.cvl;
if(a.t_c_min!=null)$('i_tcmin').value=a.t_c_min;if(a.t_c_max!=null)$('i_tcmax').value=a.t_c_max;
if(a.t_d_min!=null)$('i_tdmin').value=a.t_d_min;if(a.t_d_max!=null)$('i_tdmax').value=a.t_d_max;
s.textContent='Applied. Save and Reboot to persist.';s.style.color='var(--acctx)'}).catch(function(e){$('acbtn').disabled=false;s.textContent='Error';s.style.color='var(--redtx)'})}
 function doRestore(inp){if(!inp.files||!inp.files[0])return;var f=inp.files[0];var rm=$('rmsg');rm.textContent='Reading '+f.name+'...';rm.style.color='var(--tx2)';
var rd=new FileReader();rd.onload=function(){try{var j=JSON.parse(rd.result);if(!j.cnt){rm.textContent='Invalid backup file';rm.style.color='var(--redtx)';return}
if(!confirm('Restore settings from this backup? Device will reboot.'))return;rm.textContent='Restoring...';
fetch('/restore',{method:'POST',headers:{'Content-Type':'application/json'},body:rd.result}).then(function(r){return r.json()}).then(function(d){if(d.ok){rm.textContent='Restored! Rebooting...';rm.style.color='var(--acctx)';setTimeout(function(){location.reload()},5000)}else{rm.textContent='Failed: '+(d.err||'');rm.style.color='var(--redtx)'}}).catch(function(e){rm.textContent='Rebooting...';rm.style.color='var(--acctx)';setTimeout(function(){location.reload()},5000)})}catch(e){rm.textContent='Invalid JSON file';rm.style.color='var(--redtx)'}};rd.readAsText(f);inp.value=''}
var otaData=null;
(function(){var dp=$('otaDrop'),fi=$('otaFile');if(!dp||!fi)return;dp.ondragover=function(e){e.preventDefault();dp.style.borderColor='var(--acc)'};dp.ondragleave=function(){dp.style.borderColor='var(--bd2)'};dp.ondrop=function(e){e.preventDefault();dp.style.borderColor='var(--bd2)';if(e.dataTransfer.files.length)otaPick(e.dataTransfer.files[0])};fi.onchange=function(){if(fi.files.length)otaPick(fi.files[0])}})();
 function otaPick(f){if(!f.name.endsWith('.bin')){$('otaMsg').textContent='Only .bin files';$('otaMsg').style.color='var(--redtx)';return}otaData=f;$('otaLbl').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';$('otaBtn').disabled=false;$('otaBtn').style.opacity='1';$('otaMsg').textContent='Ready';$('otaMsg').style.color='var(--acctx)'}
 function otaUp(){if(!otaData)return;var xhr=new XMLHttpRequest();var fd=new FormData();fd.append('update',otaData);$('otaBtn').disabled=true;$('otaProg').style.display='block';$('otaMsg').textContent='Uploading...';$('otaMsg').style.color='var(--tx2)';var done=false;xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);$('otaBar').style.width=p+'%';$('otaMsg').textContent='Upload: '+p+'%';if(p>=100)done=true}};xhr.onload=function(){$('otaMsg').textContent='Update successful! Rebooting...';$('otaMsg').style.color='var(--acctx)';pollReboot()};xhr.onerror=function(){if(done){$('otaMsg').textContent='Update successful! Rebooting...';$('otaMsg').style.color='var(--acctx)';pollReboot()}else{$('otaMsg').textContent='Connection error';$('otaMsg').style.color='var(--redtx)';$('otaBtn').disabled=false}};xhr.open('POST','/update');xhr.send(fd)}
 function pollReboot(){var n=0;var iv=setInterval(function(){n++;$('otaMsg').textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);location.href='/'}).catch(function(){})},3000)}

/* --- V2.65.1 D2: RS485 FRAME SPY ---
   Toggle capture via /spy?act=1|0, poll /spy every 2s while active to
   fetch ring buffer. TopBand 0x42 frame layout:
     ... D0 7C [flags] [cells] [cell_v x cells*2B] [t_cnt] [temps x t_cnt*2B]
     [cur int16 signed *0.01A] [volt uint16 *0.01V] ...
*/
var spyTimer=null,spyActive=false;
 function spyDecodeFrame(hex){
  // Parse hex string into byte array
  if(!hex||hex.length<4)return{ok:false,err:'empty'};
  var b=[];for(var i=0;i+1<hex.length;i+=2){var v=parseInt(hex.substr(i,2),16);if(isNaN(v))return{ok:false,err:'bad hex at '+i};b.push(v)}
  // Locate D0 7C marker
  var idx=-1;for(var i=0;i<b.length-1;i++){if(b[i]==0xD0&&b[i+1]==0x7C){idx=i;break}}
  if(idx<0)return{ok:false,err:'no D0 7C marker'};
  var p=idx+3; // skip D0 7C [flags]
  if(p>=b.length)return{ok:false,err:'truncated at cells header'};
  var cells=b[p++];if(cells>32)cells=32;
  if(p+cells*2>b.length)return{ok:false,err:'truncated at cell voltages ('+cells+' cells)'};
  p+=cells*2;
  if(p>=b.length)return{ok:false,err:'truncated at temp count'};
  var tc=b[p++];if(tc>8)tc=8;
  if(p+tc*2>b.length)return{ok:false,err:'truncated at temps ('+tc+' temps)'};
  p+=tc*2;
  if(p+3>b.length)return{ok:false,err:'truncated before current field (need 4B @ '+p+')'};
  var curRaw=(b[p]<<8)|b[p+1];if(curRaw>32767)curRaw-=65536;
  var cur=curRaw*0.01;
  var voltRaw=(b[p+2]<<8)|b[p+3];
  var volt=voltRaw*0.01;
  return{ok:true,cells:cells,t_cnt:tc,cur:cur,volt:volt,bytes:b.length};
}
 function spySetBtn(active,remainMs){
  var b=$('spyBtn'),s=$('spySt');
  if(active){b.textContent='Stop capture';b.style.background='var(--redtx)';b.style.color='#fff';
    var secs=Math.max(0,Math.round((remainMs||0)/1000));s.textContent='Capturing, '+secs+'s remaining';s.style.color='var(--acctx)'}
  else{b.textContent='Frame spy';b.style.background='';b.style.color='';
    s.textContent='Capture RS485 responses from BMS for 5 minutes. Raw hex plus decoded current and voltage per frame.';s.style.color='var(--tx2)'}
}
 function spyRender(j){
  var p=$('spyPanel');
  if(!j.active&&(!j.frames||!j.frames.length)){p.style.display='none';return}
  p.style.display='block';
  var h='<div style="font-size:10px;color:var(--tx2);margin-bottom:6px">Captured '+(j.captured||0)+' frames total, showing last '+(j.frames?j.frames.length:0)+' (newest first)</div>';
  if(j.frames)for(var i=0;i<j.frames.length;i++){
    var fr=j.frames[i];
    var d=spyDecodeFrame(fr.hex);
    var uptime=Math.round(fr.ts/1000);
    var head='<span style="color:var(--tx2)">['+uptime+'s BMS'+fr.a+']</span> ';
    var body;
    if(d.ok){
      var curCls=Math.abs(d.cur)<0.01?'color:var(--redtx);font-weight:600':'color:var(--acctx)';
      body='<span style="'+curCls+'">'+d.cur.toFixed(2)+' A</span> '+d.volt.toFixed(2)+' V '+
           '<span style="color:var(--tx3);font-size:10px">('+d.cells+'c/'+d.t_cnt+'t/'+d.bytes+'B)</span>';
    }else{body='<span style="color:var(--redtx)">decode: '+d.err+'</span>'}
    var hexEsc=fr.hex.replace(/</g,'&lt;');
    h+='<div style="margin-bottom:4px;line-height:1.5">'+head+body+'<br><span style="color:var(--tx3);font-size:10px;font-family:var(--mono);word-break:break-all">'+hexEsc+'</span></div>';
  }
  p.innerHTML=h;
}
 function spyPoll(){
  fetch('/spy').then(function(r){return r.json()}).then(function(j){
    spyActive=!!j.active;spySetBtn(spyActive,j.remain_ms);spyRender(j);
    if(!spyActive&&spyTimer){clearInterval(spyTimer);spyTimer=null}
  }).catch(function(){})
}
 function spyToggle(){
  var want=spyActive?0:1;
  fetch('/spy?act='+want).then(function(r){return r.json()}).then(function(j){
    spyActive=!!j.active;spySetBtn(spyActive,j.remain_ms);spyRender(j);
    if(spyActive&&!spyTimer){spyTimer=setInterval(spyPoll,2000)}
    else if(!spyActive&&spyTimer){clearInterval(spyTimer);spyTimer=null}
  })
}
// On page load, check if spy is already running (e.g. after tab switch)
setTimeout(function(){if($('spyBtn'))spyPoll()},500);

/* --- V2.65.3 U1/U2/U3: Copy-to-clipboard + spy close ---
   Dual-path copy: navigator.clipboard (modern, requires HTTPS) with
   execCommand fallback for plain HTTP which is the usual case here.
*/
 function copyToClipboard(text,msgEl){
  var ok=false;
  try{
    if(navigator.clipboard&&window.isSecureContext){
      navigator.clipboard.writeText(text).then(function(){
        if(msgEl){var p=msgEl.textContent;msgEl.textContent='Copied';setTimeout(function(){msgEl.textContent=p},1500)}
      });return
    }
  }catch(e){}
  // Fallback for HTTP contexts
  var ta=document.createElement('textarea');ta.value=text;ta.style.position='fixed';ta.style.top='-1000px';
  document.body.appendChild(ta);ta.focus();ta.select();
  try{ok=document.execCommand('copy')}catch(e){ok=false}
  document.body.removeChild(ta);
  if(msgEl){var p2=msgEl.textContent;msgEl.textContent=ok?'Copied':'Copy failed';setTimeout(function(){msgEl.textContent=p2},1500)}
}
 function copyLog(){
  var dl=$('dlog');if(!dl)return;
  // Serialize log rows to "HH:MM:SS message" plain text, oldest-first
  var rows=dl.querySelectorAll('div');var lines=[];
  for(var i=rows.length-1;i>=0;i--){lines.push(rows[i].textContent||'')}
  copyToClipboard(lines.join('\n'),null);
}
 function copySpy(){
  // Refetch current spy state and build tab-separated block:
  // ts_ms  bms_addr  cur_A  volt_V  hex
  fetch('/spy').then(function(r){return r.json()}).then(function(j){
    if(!j.frames||!j.frames.length){return}
    var lines=['ts_ms\tbms\tcur_A\tvolt_V\thex'];
    // Emit oldest-first for easier analysis
    for(var i=j.frames.length-1;i>=0;i--){
      var f=j.frames[i];var d=spyDecodeFrame(f.hex);
      var cur=d.ok?d.cur.toFixed(2):'';var volt=d.ok?d.volt.toFixed(2):'';
      lines.push(f.ts+'\t'+f.a+'\t'+cur+'\t'+volt+'\t'+f.hex);
    }
    copyToClipboard(lines.join('\n'),$('spySt'));
  })
}
 function spyClose(){
  // Hide panel but leave server-side ring buffer intact. User can re-open
  // by pressing Start (which will either resume existing capture or start fresh).
  $('spyPanel').style.display='none';
  $('spyActions').style.display='none';
}
/* Patch spyRender to reveal the Copy/Close chrome whenever content exists */
var _origSpyRender=spyRender;
spyRender=function(j){
  _origSpyRender(j);
  var haveFrames=(j.frames&&j.frames.length>0);
  var panelVisible=$('spyPanel').style.display!=='none';
  $('spyActions').style.display=(haveFrames&&panelVisible)?'flex':'none';
};

/* --- V2.67 F3: Diag counters panel (live values + hover tooltips) --- */
/* V2.67 F4: DIAG_KEYS extended with last_reset_ts. dkFmt formats epoch to local wall clock. */
/* V2.67.2: Removed fw, uptime, boot_reason, heap_free, heap_min, nvs_used, */
/* nvs_free, session_age_d. These live in the About section only; showing them */
/* twice on the same page was redundant. */
var DIAG_KEYS=['can_tx_ok','can_tx_fail','can_tx_fail_streak','can_status','rs485_hwm','bms_polls','bms_timeouts','current_holds','spike_rejects','rl_rejects','mqtt_fail','stream_aborts','handler_max_ms','loop_max_ms','wdt_warnings','last_reset_ts'];
var dkTimer=null;
 function dkToggle(){var p=$('dkPanel');if(p.style.display==='none'){p.style.display='block';$('dkBtn').textContent='Hide runtime counters';dkFetch();dkTimer=setInterval(dkFetch,15000)}else{p.style.display='none';$('dkBtn').textContent='Runtime counters';if(dkTimer){clearInterval(dkTimer);dkTimer=null}}}
 function dkFmt(k,v){if(v==null)return'--';if(k==='uptime'){var s=parseInt(v)||0;var d=Math.floor(s/86400);var h=Math.floor((s%86400)/3600);var m=Math.floor((s%3600)/60);return(d?d+'d ':'')+h+'h '+m+'m'}if(k==='heap_free'||k==='heap_min'||k==='rs485_hwm')return Math.round(v/1024)+' KB';if(k==='last_reset_ts'){var ts=parseInt(v)||0;if(ts===0)return'never';try{return new Date(ts*1000).toLocaleString()}catch(e){return String(v)}}return String(v)}
 function dkFetch(){fetch('/diag').then(function(r){if(r.status===401){location.href='/login';return null}return r.json()}).then(function(d){if(!d)return;var help=d._help||{};var rows='';for(var i=0;i<DIAG_KEYS.length;i++){var k=DIAG_KEYS[i];var v=d[k];var h=help[k]||'';var esc=function(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')};rows+='<tr><td>'+k+'<span class="tip" data-tip="'+esc(h)+'">?</span></td><td>'+esc(dkFmt(k,v))+'</td></tr>'}$('dkBody').innerHTML=rows}).catch(function(e){$('dkBody').innerHTML='<tr><td colspan="2" style="color:var(--redtx);text-align:center">Error: '+e.message+'</td></tr>'})}
/* V2.67 F4: Manual counter reset with confirm dialog. POSTs /svc/reset_counters, then refreshes panel. */
 function rcReset(){if(!confirm('Reset all diag counters to zero?\n\nThis zeroes bms_polls, can_tx_ok/fail, stream_aborts, current_holds, spike_rejects, rl_rejects, mqtt_fail, plus per-BMS polls/timeouts/errors/spikes.\n\nPreserved: heap_min, handler_max_ms, loop_max_ms, rs485_hwm, wdt_warnings, can_tx_fail_streak.\n\nThis cannot be undone.'))return;var b=$('rcBtn');var s=$('rcSt');b.disabled=true;s.textContent='Resetting...';s.style.color='var(--tx2)';fetch('/svc/reset_counters',{method:'POST'}).then(function(r){if(r.status===401){location.href='/login';return null}return r.json()}).then(function(j){b.disabled=false;if(!j||!j.ok){s.textContent='Reset failed';s.style.color='var(--redtx)';return}s.textContent='Counters reset at '+(j.last_reset_ts?new Date(j.last_reset_ts*1000).toLocaleString():'(NTP not synced)');s.style.color='var(--acctx)';dkFetch()}).catch(function(e){b.disabled=false;s.textContent='Error: '+e.message;s.style.color='var(--redtx)'})}


/* --- BMS DIAGNOSTICS --- */
 function svcMsg(t,c){var e=$('svcMsg');e.style.display='block';e.textContent=t;e.style.color=c||'var(--tx2)'}
 function svcRd(){var id=$('svcId').value;svcMsg('Reading BMS '+id+'...');$('svcRdBtn').disabled=true;
fetch('/svc/bms?id='+id).then(function(r){return r.json()}).then(function(d){$('svcRdBtn').disabled=false;if(!d.ok){svcMsg('Error: '+(d.err||''),'var(--redtx)');return}svcMsg('BMS '+id+' loaded','var(--acctx)');
var h='';
if(d.manufacturer&&d.manufacturer.ok){h+='<div class="sl">Manufacturer</div>';h+='<div class="ir"><span class="k">Hardware</span><span class="vl">'+esc(d.manufacturer.hw)+'</span></div>';h+='<div class="ir"><span class="k">Software</span><span class="vl">'+esc(d.manufacturer.sw)+'</span></div>';h+='<div class="ir"><span class="k">ID</span><span class="vl">'+esc(d.manufacturer.id)+'</span></div>'}
if(d.date&&d.date.ok){h+='<div class="sl">BMS clock</div>';h+='<div class="ir"><span class="k">Date/Time</span><span class="vl">'+d.date.year+'-'+pd(d.date.month)+'-'+pd(d.date.day)+' '+pd(d.date.hour)+':'+pd(d.date.minute)+':'+pd(d.date.second)+'</span></div>'}
if(d.analog&&d.analog.ok){var a=d.analog;h+='<div class="sl">Analog (0x42)</div>';h+='<div class="ir"><span class="k">Pack / Current</span><span class="vl">'+a.pack_v.toFixed(2)+' V / '+a.current.toFixed(2)+' A</span></div>';h+='<div class="ir"><span class="k">SOC / SOH</span><span class="vl">'+a.soc+'% / '+a.soh+'%</span></div>';h+='<div class="ir"><span class="k">Capacity</span><span class="vl">'+a.rem_ah.toFixed(2)+' / '+a.full_ah.toFixed(2)+' Ah</span></div>';h+='<div class="ir"><span class="k">Cell range</span><span class="vl">'+a.min_cell.toFixed(3)+' .. '+a.max_cell.toFixed(3)+' V</span></div>'}
if(d.system&&d.system.ok){var s=d.system;h+='<div class="sl">System params (0x47)</div>';h+='<div class="ir"><span class="k">Cell H/L/U</span><span class="vl">'+s.cell_high_v.toFixed(3)+' / '+s.cell_low_v.toFixed(3)+' / '+s.cell_under_v.toFixed(3)+' V</span></div>';h+='<div class="ir"><span class="k">Module H/L/U</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_low_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';h+='<div class="ir"><span class="k">Charge/Disch max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' / '+s.discharge_current_max_a.toFixed(1)+' A</span></div>';h+='<div class="ir"><span class="k">Charge temp</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';h+='<div class="ir"><span class="k">Discharge temp</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>'}
if(d.alarm&&d.alarm.ok){h+='<div class="sl">Alarms (0x44)</div>';h+='<div class="ir"><span class="k">Critical</span><span class="vl" style="color:'+(d.alarm.critical?'var(--redtx)':'var(--acctx)')+'">'+(d.alarm.critical?'YES':'No')+'</span></div>';if(d.alarm.active&&d.alarm.active.length>0){h+='<div style="margin-top:6px;display:flex;flex-wrap:wrap;gap:4px">';d.alarm.active.forEach(function(a){h+='<span style="font-size:11px;background:var(--bg3);padding:2px 8px;border-radius:4px">'+esc(a.name)+'</span>'});h+='</div>'}}
if(d.autocfg&&d.autocfg.ok&&d.autocfg.suggest){var sg=d.autocfg.suggest;h+='<div class="sl">Auto-config suggestion</div>';h+='<div class="ir"><span class="k">Safe cell/pack</span><span class="vl">'+sg.s_cel.toFixed(3)+' / '+sg.s_vol.toFixed(2)+' V</span></div>';h+='<div class="ir"><span class="k">Chg/Dis</span><span class="vl">'+sg.chg.toFixed(1)+' / '+sg.dis.toFixed(1)+' A, CVL '+sg.cvl.toFixed(2)+' V</span></div>';h+='<div style="margin-top:8px"><button type="button" class="bp" onclick="svcAcf()">Apply to gateway</button></div>'}
$('svcOut').innerHTML=h}).catch(function(e){$('svcRdBtn').disabled=false;svcMsg('Error: '+e.message,'var(--redtx)')})}
 function svcTm(){var id=$('svcId').value;svcMsg('Syncing BMS '+id+' time...');fetch('/svc/bms/time/set?id='+id).then(function(r){return r.json()}).then(function(d){if(d.ok){svcMsg('Time synced','var(--acctx)');svcRd()}else svcMsg('Failed','var(--redtx)')}).catch(function(e){svcMsg('Error','var(--redtx)')})}
 function svcAcf(){var id=$('svcId').value;if(!confirm('Apply auto-config from BMS '+id+'?'))return;svcMsg('Applying...');fetch('/svc/autocfg/apply?id='+id).then(function(r){return r.json()}).then(function(d){if(d.ok&&d.applied){var a=d.applied;if(a.s_vol!=null)$('i_svol').value=a.s_vol;if(a.s_cel!=null)$('i_scel').value=a.s_cel;if(a.chg!=null)$('i_chg').value=a.chg;if(a.dis!=null)$('i_dis').value=a.dis;if(a.cvl!=null)$('i_cvl').value=a.cvl;if(a.t_c_min!=null)$('i_tcmin').value=a.t_c_min;if(a.t_c_max!=null)$('i_tcmax').value=a.t_c_max;if(a.t_d_min!=null)$('i_tdmin').value=a.t_d_min;if(a.t_d_max!=null)$('i_tdmax').value=a.t_d_max;svcMsg('Applied. Save and Reboot to persist.','var(--acctx)')}else svcMsg('Failed','var(--redtx)')}).catch(function(e){svcMsg('Error','var(--redtx)')})}

/* --- MAIN DATA UPDATE (polls /data every 2.5s) --- */
 function upd(){var s1=$('i_ch1'),s2=$('i_ch2');var q='';var rc1=0,rc2=1;if(I&&s1&&s2){rc1=parseInt(s1.value)||0;rc2=parseInt(s2.value)||1;q='?c1='+rc1+'&c2='+rc2}fetch('/data'+q).then(function(r){if(r.status===401){location.href='/login';return null}return r.json()}).then(function(d){
if(I){reqC1=rc1;reqC2=rc2}else if(d&&d.config){reqC1=d.config.chart1!=null?d.config.chart1:0;reqC2=d.config.chart2!=null?d.config.chart2:1}
if(!d)return;
D=d;var v=d.victron||{},c=d.config||{},bs=d.bms||[];
$('pb').textContent='BMS '+(v.active||0)+'x';$('pb').className='pl '+(v.active>0?'pl-g':'pl-r');
$('pc').className='pl '+(c.vic_en?(d.can_error?'pl-r':'pl-g'):'pl-y');$('pc').textContent=c.vic_en?(d.can_error?'CAN!':'CAN'):'CAN off';
var mqf=d.mqtt_fails||0;$('pm').className='pl '+(c.mq_en?(c.mq_connected?'pl-g':(mqf>=5?'pl-r':'pl-y')):'pl-y');$('pm').textContent=c.mq_en?(c.mq_connected?'MQTT':(mqf>0?'MQTT x'+mqf:'MQTT...')):'MQTT off';
var wf=d.wifi_rssi;$('pw').className='pl '+(wf!=null?(wf>-70?'pl-g':(wf>-80?'pl-y':'pl-r')):'pl-y');$('pw').textContent='WiFi';
/* --- Alert banner + alert log --- */
var ab=$('dab'),am='',ae=false;
if(d.safe_lock){am=d.alarm&&d.alarm!=='OK'?d.alarm:'Safety cutoff active';ae=true}
else if(d.alarm&&d.alarm!=='OK'&&d.alarm.indexOf('INFO')<0){am=d.alarm;if(d.alarm.indexOf('ALARM')>=0)ae=true}
else{bs.forEach(function(bm){if(!bm.online||!bm.temps)return;bm.temps.forEach(function(t){if(t.val>=(c.t_c_max||50)-5&&!am)am='BMS'+bm.id+' / '+t.lbl+': '+t.val.toFixed(1)+' C approaching charge max ('+(c.t_c_max||50)+' C)'})})}
if(am){ab.style.display='flex';ab.className=ae?'ab err':'ab';$('dam').innerHTML=am}else{ab.style.display='none'}
/* V2.61: alert log now authoritative from server NVS ring buffer (D.alerts, newest first).
   Server-side detection runs on Core 1 (alertDetectTick), rising-edge based to avoid spam. */
if(d.alerts&&d.alerts.length!==undefined){
alertLog=d.alerts.map(function(a){var s=a.sev===2?'err':(a.sev===1?'warn':'info');
return{sev:s,msg:a.msg,ts:a.ts||0,up:a.up||0}});
updAlertBadge();updAlertPage()}
/* --- Dashboard metrics --- */
$('ds').textContent=(v.avg_soc!=null?v.avg_soc.toFixed(1):'--')+'%';
$('dss').textContent=Math.round(v.rem_cap||0)+' / '+Math.round(v.total_cap||0)+' Ah';
var pw=v.total_power||0;$('dp').textContent=Math.round(pw)+' W';
if(Math.abs(pw-lastPwr)>5){chDirty=true;lastPwr=pw}
var stT='Idle',stC='var(--tx2)';if(pw>10){stT='Charging';stC='var(--acc)'}else if(pw<-10){stT='Discharging';stC='var(--ambertx)'}
$('dst').textContent=stT;$('dst').style.color=stC;
$('dc').textContent=fA(v.total_amps)+' A';$('dcs').textContent='Limit: '+fA(v.max_chg)+' A';
$('dv').textContent=fV(v.voltage)+' V';$('dvs').textContent='CVL: '+fV(v.dyn_cvl||c.cvl||0)+' V';
if(d.energy){var ei=d.energy.in||0,eo=d.energy.out||0;$('dei').textContent=(ei<10?ei.toFixed(2):ei.toFixed(1))+' kWh';$('deo').textContent='Out: '+(eo<10?eo.toFixed(2):eo.toFixed(1))+' kWh';
/* --- Energy details (Battery tab) --- */
$('bet_i').textContent=ei.toFixed(2)+' kWh';$('bet_o').textContent='Out: '+eo.toFixed(2)+' kWh';
if(d.energy.week_in!=null){$('bew_i').textContent=d.energy.week_in.toFixed(1)+' kWh';$('bew_o').textContent='Out: '+d.energy.week_out.toFixed(1)+' kWh'}
if(d.energy.month_in!=null){$('bem_i').textContent=d.energy.month_in.toFixed(1)+' kWh';$('bem_o').textContent='Out: '+d.energy.month_out.toFixed(1)+' kWh'}
if(d.energy.days_in){var dh='Last 7 days (in): ';d.energy.days_in.forEach(function(v,i){if(i)dh+=' | ';dh+=v.toFixed(1)});$('bedays').textContent=dh}
}
/* --- Cell extremes --- */
var mnV=Infinity,mxV=-Infinity,mnI='',mxI='',drft=0;
bs.forEach(function(bm){if(!bm.online)return;if(bm.min_cell<mnV){mnV=bm.min_cell;mnI='BMS'+bm.id}if(bm.max_cell>mxV){mxV=bm.max_cell;mxI='BMS'+bm.id}});
drft=mxV-mnV;
$('dcn').textContent=mnV<99?fV3(mnV):'--';$('dcni').textContent=mnI||'--';
$('dcx').textContent=mxV>-99?fV3(mxV):'--';$('dcxi').textContent=mxI||'--';
$('dcd').textContent=drft>0?fV3(drft):'--';$('dcds').textContent='Limit: '+fV3(c.s_drift);
$('dtp').textContent=fA(v.avg_temp)+' C';$('dtps').textContent='SOH: '+fA(v.avg_soh)+'%';
/* --- Runtime estimate --- */
var rt='--';if(v.total_cap>0&&v.total_amps){var hrs=0;if(v.total_amps<-0.2)hrs=v.rem_cap/Math.abs(v.total_amps);else if(v.total_amps>0.2)hrs=(v.total_cap-v.rem_cap)/v.total_amps;if(hrs>0&&hrs<999){rt=hrs>=1?Math.floor(hrs)+'h '+Math.round((hrs%1)*60)+'m':Math.round(hrs*60)+'m';$('drts').textContent=v.total_amps<-0.2?'until empty':'until full'}else $('drts').innerHTML='&nbsp;'}$('drt').textContent=rt;
/* --- BMS cards (throttled to every 15s to reduce visual noise) --- */
var bmsNow=Date.now();
if(bmsNow-lastBmsRender>=15000||lastBmsRender===0){lastBmsRender=bmsNow;
var con=$('bcon');var bH='';
bs.forEach(function(bm,bi){
var on=bm.online;var pc2='pl-g',brd='',st2='Idle';
if(!on){pc2='pl-r';brd=' bco';st2='Offline'}else if(bm.current>0.15){pc2='pl-g';st2='Charging'}else if(bm.current<-0.15){pc2='pl-y';brd=' bcy';st2='Discharging'}
bH+='<div class="bc'+brd+'">';
bH+='<div class="hd"><span class="nm">BMS #'+bm.id+'</span><span class="pl '+pc2+'">'+st2+'</span></div>';
if(on){
bH+='<div class="st"><span><b>'+bm.soc+'%</b></span><span>'+bm.pack_v.toFixed(2)+' V</span><span>'+bm.current.toFixed(1)+' A</span><span>'+Math.round(bm.pack_v*bm.current)+' W</span><span style="opacity:0.6">SOH '+bm.soh+'%</span></div>';
if(bm.stats){var sr=bm.stats.polls>0?Math.round(bm.stats.ok/bm.stats.polls*100):0;var sc=sr>=98?'var(--acctx)':sr>=90?'var(--ambertx)':'var(--redtx)';bH+='<div style="font-size:9px;color:var(--tx3);margin-top:4px">RS485: '+sr+'% ok <span style="opacity:0.6">('+bm.stats.ok+'/'+bm.stats.polls+')</span>'+(bm.stats.timeout>0?' <span style="color:var(--ambertx)">'+bm.stats.timeout+' tmout</span>':'')+(bm.stats.spikes>0?' <span style="color:var(--redtx)">'+bm.stats.spikes+' spike</span>':'')+'</div>'}
if(bm.cells&&bm.cells.length>1){var cl=bm.cells,ba=99,ta=-99,li=-1,xi=-1;for(var ci=0;ci<cl.length;ci++){if(cl[ci]<ba){ba=cl[ci];li=ci}if(cl[ci]>ta){ta=cl[ci];xi=ci}}var sp=ta-ba;if(sp<0.001)sp=0.001;
bH+='<div class="cell-bar">';for(ci=0;ci<cl.length;ci++){var pt=Math.max(8,Math.min(100,((cl[ci]-ba)/sp)*100));var cs='';if(ci===li)cs=' lo';else if(ci===xi)cs=' hi';bH+='<div class="cb'+cs+'" style="height:'+pt+'%"></div>'}
bH+='</div><div class="cell-lbl"><span>'+fV3(ba)+' V</span><span>'+cl.length+' cells</span><span>'+fV3(ta)+' V</span></div>'}
}bH+='</div>'});con.innerHTML=bH;
}
/* --- Events list --- */
/* Events moved to Alerts page */
/* --- Temperature grid (Battery tab) --- */
var tC=$('btmp');if(tC){var tH='';
bs.forEach(function(bm){if(!bm.online||!bm.temps)return;
var hw2=false;bm.temps.forEach(function(t){if(tCl(t.val,c.t_c_min||5,c.t_c_max||50)!=='tok')hw2=true});
tH+='<div class="bd'+(hw2?' bdy':'')+'"><div class="bh"><span class="nm">BMS #'+bm.id+'</span><span style="font-size:11px;color:var(--tx3);margin:0 8px">'+(bm.cycles||0)+' cycles</span><span class="pl '+(hw2?'pl-y':'pl-g')+'">'+(hw2?'Warning':'All OK')+'</span></div><div class="tg">';
bm.temps.forEach(function(t){tH+='<div class="tc '+tCl(t.val,c.t_c_min||5,c.t_c_max||50)+'"><div class="tv">'+t.val.toFixed(1)+'</div><div class="tl">'+t.lbl+'</div></div>'});
tH+='</div><div style="display:flex;justify-content:space-between;font-size:11px;color:var(--tx3);margin-top:6px"><span>Charge: '+(c.t_c_min||5)+' .. '+(c.t_c_max||50)+' C</span><span>Discharge: '+(c.t_d_min||-20)+' .. '+(c.t_d_max||60)+' C</span></div></div>'});
if(!tH)tH='<div style="color:var(--tx3);font-size:12px">Waiting for data...</div>';tC.innerHTML=tH}
/* --- About info (General tab, V2.67 F2) --- */
$('gfw').textContent='V'+(c.fw||'--');$('guid').textContent=c.dev_uid||'--';
if(c.git_sha)$('gbuild').textContent=c.git_sha;
if(c.compile_date)$('gcompile').textContent=c.compile_date;
if(c.mac)$('gmac').textContent=c.mac;
if(c.sdk)$('gsdk').textContent=c.sdk;
if(c.github){$('ggh').href=c.github;$('ggh').textContent=c.github.replace(/^https?:\/\//,'')}
if(c.license)$('glic').textContent=c.license;
var um=d.uptime_ms||0;$('gup').textContent=uT(um);
if(d.boot_epoch&&d.boot_epoch>1672531200){var bd=new Date(d.boot_epoch*1000);var pad2=function(n){return('0'+n).slice(-2)};$('gboottime').textContent=bd.getFullYear()+'-'+pad2(bd.getMonth()+1)+'-'+pad2(bd.getDate())+' '+pad2(bd.getHours())+':'+pad2(bd.getMinutes())+':'+pad2(bd.getSeconds())}else{$('gboottime').textContent='(waiting for NTP)'}
if(d.boot_reason){var br=d.boot_reason;$('gboot').textContent=br;$('gboot').style.color=(br==='PANIC'||br==='INT_WDT'||br==='TASK_WDT'||br==='BROWNOUT')?'var(--redtx)':'var(--acctx)'}
if(c.pins){var p=c.pins;var bn=['Waveshare','LilyGo','Custom'];$('gpins').textContent=(bn[c.board_type]||'?')+': RS485 '+p.rs_tx+'/'+p.rs_rx+(p.rs_dir>=0?' DIR '+p.rs_dir:'')+', CAN '+p.can_tx+'/'+p.can_rx+(p.led>=0?', LED '+p.led:'')}
if(d.free_heap)$('ghp').textContent=Math.round(d.free_heap/1024)+' KB (min: '+Math.round((d.min_heap||0)/1024)+' KB)';
if(c.psram_size&&c.psram_size>0){$('gpsram_row').style.display='flex';$('gpsram').textContent=Math.round((c.psram_free||0)/1024)+' / '+Math.round(c.psram_size/1024)+' KB'}
if(d.chip_temp!=null){var ct=d.chip_temp;$('gctemp').textContent=ct.toFixed(1)+' C';$('gctemp').style.color=ct>75?'var(--redtx)':ct>55?'var(--ambertx)':'var(--acctx)'}
if(d.chip_model){$('gchip').textContent=d.chip_model+' Rev '+d.chip_rev+' / '+(d.chip_cores||2)+' cores / '+(d.cpu_mhz||240)+' MHz'}
if(d.sketch_size&&d.flash_size){var pct=Math.round(d.sketch_size/d.flash_size*100);$('gflash').textContent=Math.round(d.sketch_size/1024)+' / '+Math.round(d.flash_size/1024)+' KB ('+pct+'%)'}
$('nuid').value=c.dev_uid||'--';
if(d.wifi_ip)$('nip').textContent=d.wifi_ip;
if(d.wifi_rssi!=null){var rs=d.wifi_rssi;var ql=rs>-50?'excellent':rs>-65?'good':rs>-75?'fair':'weak';$('nrs').textContent=rs+' dBm ('+ql+')';$('nrs').style.color=rs>-65?'var(--acctx)':rs>-75?'var(--ambertx)':'var(--redtx)'}
if(d.wifi_host)$('nhn').textContent=d.wifi_host;
if(d.log&&d.log.length){var dl=$('dlog');dl.style.display='block';var h='';var esc=function(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')};for(var i=0;i<d.log.length;i++){var L=d.log[i];var ts;if(L.t>1672531200){var dt=new Date(L.t*1000);ts=('0'+dt.getHours()).slice(-2)+':'+('0'+dt.getMinutes()).slice(-2)+':'+('0'+dt.getSeconds()).slice(-2)}else{ts=L.t+'s'}h+='<div class="'+(L.e?'log-err':'log-ok')+'">['+ts+'] '+esc(L.m)+'</div>'}dl.innerHTML=h}
/* V2.65.4 R6: chart fill-progress banner, shown only while <50% of the 48h window has data */
if(d.histLen&&d.histFilled!=null){var hi=$('histInfo');if(hi){var pct=Math.round(d.histFilled*100/d.histLen);if(pct<50){hi.style.display='block';$('histInfoPct').textContent='('+pct+'% filled)'}else{hi.style.display='none'}}}
/* V2.65.1 D1: NVS partition usage row */
if(d.nvs_total){var nu=d.nvs_used||0,nf=d.nvs_free||0,nt=d.nvs_total||1;var pct=Math.round(nu*100/nt);var col=pct>85?'var(--redtx)':pct>70?'var(--ambertx)':'var(--tx2)';$('gnvs').textContent=nu+' / '+nt+' entries ('+pct+'% used, '+nf+' free)';$('gnvs').style.color=col}
/* --- Populate form fields (once) --- */
if(!I&&c){
$('i_cnt').value=c.cnt;$('i_cells').value=c.cells;$('i_chg').value=c.chg;$('i_dis').value=c.dis;$('i_cvl').value=c.cvl;
$('i_svol').value=c.s_vol;$('i_scel').value=c.s_cel;$('i_sdrift').value=c.s_drift;
if(c.sp_v!=null)$('i_spv').value=c.sp_v;if(c.sp_a!=null)$('i_spa').value=c.sp_a;if(c.sp_s!=null)$('i_sps').value=c.sp_s;
$('i_tcmin').value=c.t_c_min;$('i_tcmax').value=c.t_c_max;$('i_tdmin').value=c.t_d_min;$('i_tdmax').value=c.t_d_max;$('i_tmode').value=c.t_mode;
$('i_maint_en').checked=!!c.maint_en;$('i_maint_v').value=c.maint_v;$('i_ab_en').checked=c.ab_en!=null?!!c.ab_en:true;
$('i_vic_en').checked=!!c.vic_en;$('i_can_proto').value=c.can_proto;
$('i_mq_en').checked=!!c.mq_en;$('i_mq_diag').checked=!!c.mq_diag;$('i_mq_ip').value=c.mq_ip||'';$('i_mq_pt').value=c.mq_pt;
var mqlv=(c.mq_level!=null?c.mq_level:0);$('i_mq_lv0').checked=(mqlv===0);$('i_mq_lv1').checked=(mqlv===1);$('i_mq_lv2').checked=(mqlv===2);
$('i_mq_us').value=c.mq_us||'';$('i_mq_top').value=c.mq_top||'Topband/BMS';$('i_ha_en').checked=!!c.ha_en;
$('i_soc_mode').value=c.soc_mode!=null?String(c.soc_mode):'2';$('i_ntp').value=c.ntp||'pool.ntp.org';$('i_tz').value=c.tz;$('i_debug').checked=!!c.debug;
$('i_board').value=c.board_type!=null?String(c.board_type):'0';
if(c.pins){$('i_p_rs_tx').value=c.pins.rs_tx;$('i_p_rs_rx').value=c.pins.rs_rx;$('i_p_rs_dir').value=c.pins.rs_dir;$('i_p_can_tx').value=c.pins.can_tx;$('i_p_can_rx').value=c.pins.can_rx;$('i_p_led').value=c.pins.led}
tgPins();
$('i_ch1').value=c.chart1!=null?String(c.chart1):'0';$('i_ch2').value=c.chart2!=null?String(c.chart2):'1';
$('i_auth_en').checked=!!c.auth_en;$('i_auth_user').value=c.auth_user||'admin';tgAuth();
if(c.auth_en){$('hauth').style.display='inline';$('hauser').textContent=c.auth_user||'admin'}else{$('hauth').style.display='none'}
var sel=$('svcId');if(sel&&sel.options.length===0){for(var si=0;si<c.cnt;si++){var o=document.createElement('option');o.value=si;o.textContent='BMS #'+si;sel.appendChild(o)}}
I=true;
if(!prL){prL=true;fetch('/svc/bms?id=0').then(function(r){return r.json()}).then(function(sd){
var bp=$('bpro');if(!sd.ok||!sd.system||!sd.system.ok){bp.innerHTML='<div style="color:var(--tx3);font-size:12px">Could not read BMS parameters</div>';return}
var s=sd.system;var h='';
h+='<div class="row"><span class="k">Module high / under voltage</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';
h+='<div class="row"><span class="k">Cell high voltage</span><span class="vl">'+s.cell_high_v.toFixed(3)+' V</span></div>';
h+='<div class="row"><span class="k">Charge / discharge max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' / '+s.discharge_current_max_a.toFixed(1)+' A</span></div>';
h+='<div class="row"><span class="k">Charge temp range</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';
h+='<div class="row"><span class="k">Discharge temp range</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>';
bp.innerHTML=h}).catch(function(){$('bpro').innerHTML='<div style="color:var(--tx3);font-size:12px">No data yet</div>'})}}
/* --- Sync chart dropdowns from server config (cross-device sync) --- */
if(c){var sCh1=c.chart1!=null?String(c.chart1):'0',sCh2=c.chart2!=null?String(c.chart2):'1';
if($('i_ch1').value!==sCh1||$('i_ch2').value!==sCh2){$('i_ch1').value=sCh1;$('i_ch2').value=sCh2;reqC1=parseInt(sCh1);reqC2=parseInt(sCh2);chDirty=true}}
drAll()}).catch(function(e){console.error('Fetch:',e)})}
setInterval(upd,2500);upd();
/* V2.61: ensure charts render on mobile even if initial layout hasn't settled */
initChartObserver();
setTimeout(function(){chDirty=true;drAll()},300);
setTimeout(function(){chDirty=true;drAll()},1500);
</script>
</body>
</html>
)RAWUI";
  // V2.66.2 E2: Allow short-term browser caching of the main UI shell.
  // Previously "no-store" forced every page-load to re-serialize the full
  // ~900-line HTML on the gateway, which on a slow client or flaky WiFi
  // could stall the synchronous server.send() long enough to contribute
  // to TASK_WDT. Live data still updates via /data polls every 2.5s.
  // ETag/If-None-Match would be ideal but needs hashing the literal; for
  // now a short max-age is a pragmatic tradeoff.
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send(200, "text/html", h);
}


#endif  // END OF MODE_SMART_WIFI

// ==========================================
// RS485 + CAN TASK (runs on Core 0)
// Handles all time-critical BMS communication:
//   - Data polling: every 3s, all BMS packs sequentially
//   - Alarm polling: every 15s, round-robin one BMS per cycle
//   - System param polling: every 30s, round-robin one BMS per cycle
//   - CAN output: every 500ms, calculateVictronData + sendVictronCAN
// Stack: 16KB. WDT subscribed. Uses rs485Mutex for bus access,
// dataMutex for shared data writes.
// ==========================================
void rs485Task(void *param) {
  esp_task_wdt_add(NULL);
  unsigned long task_poll_time = 0;
  unsigned long task_can_time = 0;
  unsigned long task_alarm_time = 0;
  unsigned long task_sysparam_time = 0;
  int task_alarm_bms = 0;
  int task_sysparam_bms = 0;

  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();

    // BMS data polling
    if (!simulation_active && (now - task_poll_time > POLL_INTERVAL)) {
      task_poll_time = now;
      for (int i = 0; i < g_bms_count; i++) {
        esp_task_wdt_reset();
        if (xSemaphoreTake(rs485Mutex, pdMS_TO_TICKS(2000))) {
          rs485SetTX();
          RS485.flush(); while (RS485.available()) RS485.read();
          RS485.print(POLL_CMDS[i]); RS485.flush();
          rs485SetRX();
          g_bms_stats[i].polls++;
          unsigned long start = millis(); String resp = ""; resp.reserve(512);
          while (millis() - start < RS485_TIMEOUT) {
            if (RS485.available()) { char c = RS485.read(); if (c == '\r') break; if (c != '~') resp += c; }
            else vTaskDelay(1);
          }
          xSemaphoreGive(rs485Mutex);
          if (resp.length() > 0) parseTopband(resp, i);
          else g_bms_stats[i].timeouts++;
        }
        vTaskDelay(pdMS_TO_TICKS(BUS_GUARD_TIME));
      }
    }

    // Alarm polling (round-robin, one BMS per cycle)
    if (!simulation_active && (now - task_alarm_time > ALARM_POLL_INTERVAL)) {
      task_alarm_time = now;
      pollBmsAlarmStatus(task_alarm_bms);
      task_alarm_bms++;
      if (task_alarm_bms >= g_bms_count) task_alarm_bms = 0;
    }

    // System parameter polling (round-robin)
    if (!simulation_active && (now - task_sysparam_time > SYSPARAM_POLL_INTERVAL)) {
      task_sysparam_time = now;
      pollBmsSystemParameter(task_sysparam_bms);
      task_sysparam_bms++;
      if (task_sysparam_bms >= g_bms_count) task_sysparam_bms = 0;
    }

    // Calculate derived data + send CAN
    if (now - task_can_time > 500) {
      task_can_time = now;
      // V2.66.1 C1: check Take result. On timeout, skip calculation but still
      // call sendVictronCAN so the inverter keeps receiving the previous frame
      // instead of going dark (preserves V2.66 behavior on timeout edge).
      bool locked = (dataMutex != NULL) && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200));
      if (locked) {
        calculateVictronData();
        xSemaphoreGive(dataMutex);
      } else {
        addToLog("WARN: rs485Task calculateVictron mutex timeout", true);
      }
      // V2.65 M7: also gate CAN send on simulation_active, matching poll gates above
      if (g_victron_enable && !simulation_active) { sendVictronCAN(); }
    }

    // Stack watermark check every 5 min
    static unsigned long last_stack_check = 0;
    if (now - last_stack_check > 300000) {
      last_stack_check = now;
      UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
      if (hwm < 1024) addToLog("WARN: RS485 task stack low: " + String(hwm) + " bytes free", true);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==========================================
// SETUP
// Boot sequence: Reboot reason logging -> Hardware init -> Auth reset check
// (5x power-cycle) -> WiFi (fixed SSID or WiFiManager captive portal) ->
// Load NVS settings -> Init services (history, SD, MQTT) -> Register web
// routes -> Start CAN + mDNS -> Create rs485Task on Core 0.
// ==========================================
void setup() {
  Serial.begin(115200); 
  
  // V2.65: Create all mutexes FIRST so addToLog(), setCanStatus(), and
  // snapshot reads are thread-safe from the very first use.
  rs485Mutex     = xSemaphoreCreateMutex();
  dataMutex      = xSemaphoreCreateMutex();
  logMutex       = xSemaphoreCreateMutex();
  canStatusMutex = xSemaphoreCreateMutex();
  spyMutex       = xSemaphoreCreateMutex();   // V2.65.1 D2
  
  // Log reboot reason for diagnostics
  esp_reset_reason_t rst_reason = esp_reset_reason();
  const char* rst_names[] = {"UNKNOWN","POWERON","EXT","SW","PANIC","INT_WDT","TASK_WDT","WDT","DEEPSLEEP","BROWNOUT","SDIO"};
  g_boot_reason = (rst_reason < 11) ? rst_names[rst_reason] : "OTHER";
  Serial.println("Boot reason: " + g_boot_reason);
  
  // Load pin configuration from NVS (must happen before hardware init)
  {
    Preferences pinCfg;
    pinCfg.begin("gateway", true);
    g_board_type = pinCfg.getInt("board_type", 0);
    applyBoardPreset(g_board_type);
    // Custom overrides (only applied if board_type=2 or if values exist in NVS)
    if (g_board_type == 2 || pinCfg.isKey("p_rs_tx")) {
      g_pin_rs485_tx = pinCfg.getInt("p_rs_tx", g_pin_rs485_tx);
      g_pin_rs485_rx = pinCfg.getInt("p_rs_rx", g_pin_rs485_rx);
      g_pin_rs485_dir = pinCfg.getInt("p_rs_dir", g_pin_rs485_dir);
      g_pin_can_tx = pinCfg.getInt("p_can_tx", g_pin_can_tx);
      g_pin_can_rx = pinCfg.getInt("p_can_rx", g_pin_can_rx);
      g_pin_led = pinCfg.getInt("p_led", g_pin_led);
    }
    pinCfg.end();
    Serial.printf("Board type %d: RS485 TX=%d RX=%d DIR=%d, CAN TX=%d RX=%d, LED=%d\n",
                  g_board_type, g_pin_rs485_tx, g_pin_rs485_rx, g_pin_rs485_dir,
                  g_pin_can_tx, g_pin_can_rx, g_pin_led);
  }

  // Hardware init with runtime pin config
  if (g_board_type == 1) {
    // LilyGo: 3 RS485 control pins
    pinMode(g_pin_lilygo_pwr, OUTPUT); digitalWrite(g_pin_lilygo_pwr, HIGH);
    pinMode(g_pin_lilygo_en, OUTPUT); digitalWrite(g_pin_lilygo_en, HIGH);
    pinMode(g_pin_lilygo_cb, OUTPUT); digitalWrite(g_pin_lilygo_cb, HIGH);
  } else {
    // Waveshare / Custom: single direction pin
    if (g_pin_rs485_dir >= 0) { pinMode(g_pin_rs485_dir, OUTPUT); digitalWrite(g_pin_rs485_dir, LOW); }
  }

  pixels.setPin(g_pin_led);
  pixels.begin(); pixels.setBrightness(30); 
  setLed(0,0,255); // Blue = boot
  delay(500); 

  // Rapid power-cycle auth reset: 5x power cycle within 15s
  {
    Preferences rst;
    rst.begin("gateway", false);
    uint8_t cnt = rst.getUChar("rst_cnt", 0);
    cnt++;
    if (cnt >= 5) {
      // Reset triggered - clear auth credentials AND invalidate session token (V2.64)
      rst.putBool("auth_en", false);
      rst.putString("auth_hash", "");
      rst.putString("sess_tok", "");   // V2.64: force fresh token on next loadOrCreateSessionToken()
      rst.putUInt("sess_ts", 0);       // V2.66 H2: force fresh timestamp too
      rst.putUChar("rst_cnt", 0);
      rst.end();
      Serial.println("AUTH RESET: 5x power cycle detected, credentials and sessions cleared");
      for (int i = 0; i < 6; i++) { setLed(255, 0, 0); delay(150); setLed(0, 0, 0); delay(150); }
      setLed(0, 0, 255);
    } else {
      rst.putUChar("rst_cnt", cnt);
      rst.end();
    }
  }

  #ifdef MODE_SMART_WIFI
    // 1. WiFi Manager
    WiFiManager wm;
    wm.setAPCallback(configModeCallback); 
    wm.setConnectTimeout(15);              // 15s statt 30s - gespeicherte Credentials brauchen max 10s
    wm.setConfigPortalTimeout(120);        // 2 min AP-Portal statt 3 min

    // Set hostname before WiFi connect
    WiFi.setHostname(HOSTNAME); 

    bool fixed_wifi_ok = false;
    if (String(FIXED_SSID) != "") {
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.persistent(false);
        WiFi.disconnect(true, true);
        delay(150);
        WiFi.begin(FIXED_SSID, FIXED_PASS);
        int tr = 0;
        while (WiFi.status() != WL_CONNECTED && tr < 80) {
            delay(500);
            tr++;
        }
        fixed_wifi_ok = (WiFi.status() == WL_CONNECTED);
    }

    setLed(150, 0, 150); // Purple = WiFi connecting

    if (!fixed_wifi_ok && !wm.autoConnect("Victron-Gateway-Setup")) {
        Serial.println("AP Timeout");
        setLed(255, 0, 0); // Red = AP timeout
    } else {
        if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
        configTime(g_timezone_offset * 3600, 0, g_ntp_server.c_str());
        setLed(0, 255, 255); // Teal = WiFi connected
    }

    // Generate device UID from MAC address
    {
      uint8_t mac[6];
      WiFi.macAddress(mac);
      char uid[16];
      snprintf(uid, sizeof(uid), "tb_%02x%02x%02x", mac[3], mac[4], mac[5]);
      g_dev_uid = String(uid);
      addToLog("Device UID: " + g_dev_uid, false);
    }

    // V2.64: Load persisted session token (stays valid across reboots).
    // Users no longer get logged out on every OTA / save-and-reboot.
    loadOrCreateSessionToken();

    // 2. Load settings
    preferences.begin("gateway", false);
    g_bms_count = preferences.getInt("cnt", 2); 
    g_force_cell_count = preferences.getInt("cells", 0);
    g_charge_amps = preferences.getFloat("chg", 30.0);
    g_discharge_amps = preferences.getFloat("dis", 30.0);
    g_expert_mode = preferences.getBool("exp", true);
    if(g_expert_mode) g_cvl_voltage = preferences.getFloat("cvl", 52.5); else g_cvl_voltage = 52.5;
    g_safe_volt = preferences.getFloat("s_vol", 53.25);
    g_safe_cell = preferences.getFloat("s_cel", 3.55);
    // Default charge min temp is 5.0 C
    g_tc_min = preferences.getFloat("t_c_min", 5.0);
    g_tc_max = preferences.getFloat("t_c_max", 50.0);
    g_td_min = preferences.getFloat("t_d_min", -20.0);
    g_td_max = preferences.getFloat("t_d_max", 60.0);
    g_temp_mode = preferences.getInt("t_mode", 0);
    g_safe_drift = preferences.getFloat("s_drift", 0.20);
    // V2.66 M4: Spike filter thresholds (default = legacy hardcoded values)
    g_spike_volt = preferences.getFloat("sp_v", 5.0);
    g_spike_cur  = preferences.getFloat("sp_a", 250.0);
    g_spike_soc  = preferences.getInt("sp_s", 20);
    // Config Flags
    g_sd_enable = preferences.getBool("sd_en", false); 
    g_serial_debug = preferences.getBool("debug", false); 
    g_sd_spy = preferences.getBool("spy", false); 
    g_mqtt_enable = preferences.getBool("mq_en", false);
    g_mqtt_full = preferences.getBool("mq_full", false); 
    g_mqtt_diag = preferences.getBool("mq_diag", false);  // V2.66.2 D1
    // V2.67 F1: Tiered MQTT level. Default 0 (L1) for all users on first V2.67
    // boot. mq_full key is retained untouched for rollback safety but ignored
    // by V2.67+ publish logic. User re-enables cell data via L2+.
    {
      int lvl = preferences.getInt("mq_level", 0);
      if (lvl < 0) lvl = 0; if (lvl > 2) lvl = 2;
      g_mqtt_level = (uint8_t)lvl;
    }
    g_ha_enable = preferences.getBool("ha_en", false); 
    g_theme_id = preferences.getInt("theme", 0);
    g_can_protocol = preferences.getInt("can_proto", 0); 
    g_victron_enable = preferences.getBool("vic_en", true); 
    g_soc_source_mode = preferences.getInt("soc_mode", 2);
    g_chart1 = preferences.getInt("chart1", 0);
    g_chart2 = preferences.getInt("chart2", 1);
    if (g_soc_source_mode < 0 || g_soc_source_mode > 2) g_soc_source_mode = 2;
    g_maint_charge_mode = preferences.getBool("maint_en", false);
    g_maint_target_v = preferences.getFloat("maint_v", 51.00f);
    if (g_maint_target_v < 40.0f) g_maint_target_v = 40.0f;
    if (g_maint_target_v > 65.0f) g_maint_target_v = 65.0f;
    g_auto_balance_enable = preferences.getBool("ab_en", true);
    g_auto_balance_last_ts = preferences.getULong("ab_last", 0);
    g_last_reset_ts = preferences.getULong("lrst_ts", 0);   // V2.67 F4: 0=never reset yet (first NTP sync will stamp)
    g_easy_mode = preferences.getBool("easy", false);
    g_setup_done = preferences.getBool("setupd", true);
    if (!g_expert_mode) {
    g_easy_mode = false;
    g_charge_amps = 30.0f;
    g_discharge_amps = 30.0f;
    g_cvl_voltage = 52.5f;
    g_safe_volt = 53.25f;
    g_safe_cell = 3.55f;
    g_safe_drift = 0.20f;
    g_tc_min = 5.0f;
    g_tc_max = 50.0f;
    g_td_min = -20.0f;
    g_td_max = 60.0f;
    g_maint_target_v = 51.0f;
  }
    g_mqtt_server = preferences.getString("mq_ip", ""); g_mqtt_port = preferences.getInt("mq_pt", 1883);
    g_mqtt_user = preferences.getString("mq_us", "");
    g_mqtt_pass = preferences.getString("mq_pw", "");
    g_mqtt_topic = preferences.getString("mq_top", "Topband/BMS");
    g_ntp_server = preferences.getString("ntp", "pool.ntp.org"); g_timezone_offset = preferences.getInt("tz", 1);
    g_auth_enable = preferences.getBool("auth_en", false);
    g_auth_user = preferences.getString("auth_user", "admin");
    g_auth_hash = preferences.getString("auth_hash", "");
    preferences.end();
    
    // 3. Init services
    addToLog("System start (boot: " + g_boot_reason + "), waiting for BMS...", false); 
    loadHistory(); 
    loadAlertsFromNvs();   // V2.61: restore alert ring buffer
    addToLog("Alerts loaded: " + String(g_alert_count) + " entries", false);
    initSD();
    if (g_mqtt_enable && g_mqtt_server != "") { mqtt.setServer(g_mqtt_server.c_str(), g_mqtt_port); mqtt.setBufferSize(4096); mqtt.setSocketTimeout(3); }

    // 4. Server routes
    const char* authHeaders[] = {"Cookie"};
    server.collectHeaders(authHeaders, 1);
    server.on("/login", handleLogin);
    server.on("/logout", handleLogout);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/sim", handleSim); server.on("/save", handleSave); server.on("/chartcfg", handleChartCfg);
    server.on("/spy", handleSpy);  // V2.65.1 D2: RS485 Frame Spy toggle + data
    server.on("/svc/bms", HTTP_GET, handleServiceBmsDiag);
    server.on("/svc/bms/time", HTTP_GET, handleServiceBmsTimeGet);
    server.on("/svc/bms/time/set", HTTP_GET, handleServiceBmsTimeSet);
    server.on("/svc/autocfg/apply", HTTP_GET, handleServiceAutoCfgApply);
    server.on("/ha/discovery/send", HTTP_GET, handleHaDiscoverySend);
    server.on("/ha/discovery/clear", HTTP_GET, handleHaDiscoveryClear);
    server.on("/alerts", HTTP_GET, handleAlertsGet);
    server.on("/alerts/clear", HTTP_POST, handleAlertsClear);
    server.on("/diag", HTTP_GET, handleDiag);  // V2.67 F3: diag JSON for Web UI tooltips
    server.on("/alerts/clear", HTTP_GET, handleAlertsClear);  // GET fallback for simple clients
    server.on("/svc/reset_counters", HTTP_POST, handleResetCounters);  // V2.67 F4: manual counter reset
    server.on("/svc/reset_counters", HTTP_GET,  handleResetCounters);  // V2.67 F4: GET fallback
    #if HAS_SD_CARD
    server.on("/sd/download", HTTP_GET, handleSDDownload); 
    server.on("/sd/spy", HTTP_GET, handleSpyDownload); 
    server.on("/sd/clear", HTTP_GET, handleSDDelete); 
    server.on("/sd/list", HTTP_GET, handleSDList); 
    #endif
    server.on("/update", HTTP_GET, handleOTAPage);
    server.on("/service", HTTP_GET, handleServicePage);
    server.on("/backup", HTTP_GET, handleBackup);
    server.on("/export", HTTP_GET, handleExport);
    server.on("/restore", HTTP_POST, handleRestore);
    server.on("/update", HTTP_POST, handleOTADone, handleOTAUpload);
    server.begin();
    // mDNS: allows access via http://topband-gateway.local
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      addToLog("mDNS: " + String(HOSTNAME) + ".local", false);
    }
    setLed(0,0,0); // LED off, enter loop
  #else
    // Cable mode (no WiFi)
    g_bms_count = 2; 
    g_can_protocol = 0;
  #endif

  RS485.begin(RS485_BAUD, SERIAL_8N1, g_pin_rs485_rx, g_pin_rs485_tx);
  #if ARDUINO_ESP32_MAJOR >= 2
  RS485.setRxBufferSize(1024);
  #endif
  
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)g_pin_can_tx, (gpio_num_t)g_pin_can_rx, TWAI_MODE_NORMAL);
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t can_inst = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
  esp_err_t can_start = twai_start();
  if (can_inst != ESP_OK || can_start != ESP_OK) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Init failed - check hardware (GPIO %d/%d)", g_pin_can_tx, g_pin_can_rx);
    setCanStatus(buf);
    can_error_flag = true; addToLog("CAN Init Failed", true);
  } else {
    setCanStatus("Ready - waiting for inverter");
    addToLog("CAN bus initialized", false);
  }
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_config_t wdt_cfg = { .timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << 0) | (1 << 1), .trigger_panic = true };
  esp_task_wdt_init(&wdt_cfg);
  #else
  esp_task_wdt_init(WDT_TIMEOUT, true);
  #endif
  esp_task_wdt_add(NULL);

  // Mutexes were created at the top of setup() (V2.65) for early-boot safety.

  // Launch RS485 + CAN task on Core 0 (WiFi core has idle time during polling waits)
  xTaskCreatePinnedToCore(rs485Task, "rs485", 16384, NULL, 1, &rs485TaskHandle, 0);
  addToLog("Dual-core: RS485 on Core 0, Web on Core 1", false);
  addToLog("Heap: " + String(ESP.getFreeHeap()) + " bytes, stack RS485: 16KB", false);
  addToLog("Chip: " + String(ESP.getChipModel()) + " " + String(ESP.getCpuFreqMHz()) + "MHz", false);
}

// ==========================================
// MAIN LOOP (runs on Core 1)
// Handles: WebServer, MQTT reconnect/publish, WiFi monitoring,
// NTP resync (6h), energy calculation, history accumulation (3-min),
// periodic NVS save (15-min), LED status, heap monitoring.
// Auth reset counter is cleared after 15s of stable operation.
// ==========================================
void loop() {
  esp_task_wdt_reset(); 
  unsigned long now = millis();
  // V2.66.2 D1: measure loop iteration runtime for debug telemetry.
  // Keep a rolling max and count near-WDT iterations (>45s, i.e. 75% of WDT_TIMEOUT).
  static unsigned long loop_iter_start = 0;
  if (loop_iter_start > 0) {
    unsigned long iter_ms = now - loop_iter_start;
    if (iter_ms > g_loop_max_ms) g_loop_max_ms = iter_ms;
    if (iter_ms > 45000UL) g_wdt_warnings++;
  }
  loop_iter_start = now;

  // V2.65 M2: Deferred restart. Handlers set g_restart_at instead of calling
  // delay()+ESP.restart() inline, which lets TCP response flush cleanly.
  if (g_restart_at && now >= g_restart_at) {
    Serial.println("Deferred restart triggered");
    ESP.restart();
  }

  // V2.65.1 D2: Auto-disable RS485 Frame Spy after its 5-minute window expires.
  if (g_rs485_spy_enable && g_rs485_spy_deadline && now >= g_rs485_spy_deadline) {
    g_rs485_spy_enable = false;
    g_rs485_spy_deadline = 0;
    addToLog("Frame Spy: auto-disabled (5 min window elapsed)", false);
  }
  
  uint32_t h_now = ESP.getFreeHeap();
  if (h_now < heap_min) heap_min = h_now;
  static unsigned long last_heap_warn = 0;
  if (h_now < 20000 && now - last_heap_warn > 60000) { last_heap_warn = now; addToLog("WARN: Low heap " + String(h_now) + " bytes (min: " + String(heap_min) + ")", true); }

  // Clear rapid power-cycle counter after 15s stable operation
  static bool rst_cleared = false;
  if (!rst_cleared && now > 15000) {
    rst_cleared = true;
    Preferences rst; rst.begin("gateway", false);
    rst.putUChar("rst_cnt", 0);
    rst.end();
  }

  static unsigned long last_led = 0;
  if(now - last_led > 50) { last_led = now; handleLedStatus(); }

  #ifdef MODE_SMART_WIFI
    server.handleClient();
    esp_task_wdt_reset();   // V2.66 W1: after HTTP handling (can stream large /data payloads)
    // WiFi reconnect monitoring
    static bool wifi_was_connected = true;
    static unsigned long last_wifi_check = 0;
    static unsigned long last_ntp_sync = 0;
    if (now - last_wifi_check > 10000) {
      last_wifi_check = now;
      bool wifi_ok = (WiFi.status() == WL_CONNECTED);
      if (wifi_was_connected && !wifi_ok) { addToLog("WiFi: connection lost, reconnecting...", true); WiFi.reconnect(); }
      else if (!wifi_was_connected && wifi_ok) { addToLog("WiFi: reconnected, IP=" + WiFi.localIP().toString(), false); }
      wifi_was_connected = wifi_ok;
    }
    // NTP resync every 6 hours
    if (now - last_ntp_sync > 21600000UL) {
      last_ntp_sync = now;
      configTime(g_timezone_offset * 3600, 0, g_ntp_server.c_str());
    }
    unsigned long now_hist = millis();
    // Snapshot power under mutex for energy + history (written on Core 0)
    float snap_power = 0, snap_voltage = 0, snap_soc = 0, snap_temp = 0;
    if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
      snap_power = victronData.totalPower;
      snap_voltage = victronData.avgVoltage;
      snap_soc = victronData.avgSOC;
      snap_temp = victronData.avgTemp;
      xSemaphoreGive(dataMutex);
    }
    // Accumulate min/max/avg between history intervals
    float sv[4] = { snap_power, snap_voltage * 100.0f, snap_soc * 10.0f, snap_temp * 10.0f };
    if (acc_reset) { for (int k=0;k<4;k++){acc_min[k]=sv[k];acc_max[k]=sv[k];acc_sum[k]=0;} acc_count=0; acc_reset=false; }
    for (int k=0;k<4;k++) { if(sv[k]<acc_min[k])acc_min[k]=sv[k]; if(sv[k]>acc_max[k])acc_max[k]=sv[k]; acc_sum[k]+=sv[k]; }
    acc_count++;
    if (now_hist - last_hist_time > HISTORY_INTERVAL) {
      last_hist_time = now_hist;
      int idx = historyIdx;
      int16_t *avg_arr[] = { powerHistory, voltHistory, socHistory, tempHistory };
      int16_t *min_arr[] = { powerMin, voltMin, socMin, tempMin };
      int16_t *max_arr[] = { powerMax, voltMax, socMax, tempMax };
      for (int k=0;k<4;k++) {
        int16_t a = (acc_count>0) ? (int16_t)(acc_sum[k]/acc_count) : (int16_t)sv[k];
        avg_arr[k][idx] = a;
        min_arr[k][idx] = (int16_t)acc_min[k];
        max_arr[k][idx] = (int16_t)acc_max[k];
      }
      historyIdx++; if (historyIdx >= HISTORY_LEN) historyIdx = 0;
      if (historyFilled < HISTORY_LEN) historyFilled++;
      acc_reset = true;
    }
    calculateEnergy(snap_power);
    // Periodic save: graph + energy every 15 min
    // V2.65.4 R5: Energy counters only, saved hourly (was 15 min).
    static unsigned long last_graph_save = 0;
    if (now - last_graph_save > 3600000UL) { last_graph_save = now; saveHistory(); esp_task_wdt_reset(); }  // V2.66 W1: after NVS write
    if (g_sd_enable && (now_hist - last_sd_time > SD_LOG_INTERVAL)) { last_sd_time = now_hist; writeLogToSD(); }
    mqttReconnect();
    esp_task_wdt_reset();   // V2.66 W1: mqttReconnect() can block on TCP timeout
    if (g_mqtt_enable && WiFi.status() == WL_CONNECTED && mqtt.connected()) { mqtt.loop(); if (now_hist - last_mqtt_time > MQTT_INTERVAL) { last_mqtt_time = now_hist; sendMqttData(); esp_task_wdt_reset(); } }  // V2.66 W1: after MQTT publish
    // V2.66.2 D1: Diagnostic publish (30s cadence, opt-in). Zero cost when disabled.
    if (g_mqtt_diag && g_mqtt_enable && WiFi.status() == WL_CONNECTED && mqtt.connected() && (now_hist - last_mqtt_diag > MQTT_DIAG_INTERVAL)) {
      last_mqtt_diag = now_hist;
      sendMqttDiag();
      esp_task_wdt_reset();
    }
    // V2.67 F1: L3 per-BMS cells topic (20s cadence). Zero cost on L1/L2.
    if (g_mqtt_level >= 2 && g_mqtt_enable && WiFi.status() == WL_CONNECTED && mqtt.connected() && (now_hist - last_mqtt_cells > MQTT_CELLS_INTERVAL)) {
      last_mqtt_cells = now_hist;
      sendMqttCells();
      esp_task_wdt_reset();
    }
    // V2.67 F4: Counter reset rollover check. Runs every 60 s, independent
    // of diag publish cadence. Two responsibilities:
    //   1. First-sync stamp: if g_last_reset_ts is 0 (fresh flash or NTP
    //      was never synced at any previous reset), anchor the window on
    //      the first NTP-synced check we see and persist once.
    //   2. 7-day rollover: when (now - g_last_reset_ts) >= 604800 s AND
    //      NTP is synced, zero the counter set and re-anchor. If NTP is
    //      not synced, defer; we must not reset against a bogus clock.
    if (now - g_last_rollover_check > RESET_CHECK_INTERVAL) {
      g_last_rollover_check = now;
      time_t t_now; time(&t_now);
      uint32_t ep = (uint32_t)t_now;
      bool ntp_ok = (ep > NTP_SYNC_EPOCH_MIN);
      if (ntp_ok) {
        if (g_last_reset_ts == 0) {
          // First-ever anchor. Counters stay as they are; we just stamp.
          g_last_reset_ts = ep;
          preferences.begin("gateway", false);
          preferences.putULong("lrst_ts", g_last_reset_ts);
          preferences.end();
          addToLog("Counter window anchored (ts=" + String(g_last_reset_ts) + ")", false);
          esp_task_wdt_reset();
        } else if ((ep - g_last_reset_ts) >= RESET_ROLLOVER_SEC) {
          // 7 days elapsed since last reset. Roll over.
          resetCountersNow(ep);
          esp_task_wdt_reset();
        }
      }
      // If NTP is not synced, fall through silently. The next 60 s tick
      // will re-evaluate. RAM counters continue to accumulate; acceptable.
    }
    alertDetectTick();   // V2.61: server-side alert generation (throttled to 10s internally)
  #endif
}




