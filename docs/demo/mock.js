/**
 * TopBand BMS Gateway — Demo mock backend
 *
 * Intercepts all window.fetch calls to /api/... and answers with realistic
 * canned data for a healthy 3-pack system. Write actions show a friendly
 * "Demo mode" notice instead of executing.
 *
 * Load order: must come BEFORE app.js in the HTML.
 */
(function () {
  'use strict';

  /* ── Default to light theme on first visit ───────────────────────────────── */
  if (!localStorage.getItem('tbms_theme')) {
    localStorage.setItem('tbms_theme', 'light');
  }

  /* ── Epoch anchor (used for timestamps + chart x-axis) ──────────────────── */
  const NOW_EPOCH = Math.floor(Date.now() / 1000);
  const UPTIME_BASE = 3 * 86400 + 2 * 3600 + 1800; // 3d 2h 30m

  /* ── Mutable alerts array (DELETE /api/alerts wipes it) ─────────────────── */
  let DEMO_ALERTS = [
    {
      severity: 'INFO', severity_n: 0,
      message: 'System started: TopBand BMS Gateway v3.3.0',
      source: 'boot',
      ts_epoch: NOW_EPOCH - UPTIME_BASE,
      uptime_s: 0,
    },
    {
      severity: 'WARN', severity_n: 1,
      message: 'BMS 3: RS485 timeout (pack offline for 4.2 s)',
      source: 'poller',
      ts_epoch: NOW_EPOCH - UPTIME_BASE + 200,
      uptime_s: 200,
    },
    {
      severity: 'INFO', severity_n: 0,
      message: 'BMS 3: pack came back online (was offline 4.2 s)',
      source: 'poller',
      ts_epoch: NOW_EPOCH - UPTIME_BASE + 205,
      uptime_s: 205,
    },
  ];

  /* ── Static config ───────────────────────────────────────────────────────── */
  const DEMO_CONFIG = {
    schema_version: 11,
    bms_count: 3,
    force_cell_count: 0,
    battery_config_mode: 1,           // Auto+Margin
    setup_mode: 2,                    // Manual (already configured)
    auto_from_bms_applied: true,
    charge_amps_per_pack: 50.0,
    discharge_amps_per_pack: 100.0,
    cvl_voltage: 54.6,
    safe_pack_volt: 55.0,
    safe_cell_volt: 3.65,
    safe_cell_drift: 0.100,
    charge_temp_min: 5.0,
    charge_temp_max: 45.0,
    discharge_temp_min: -10.0,
    discharge_temp_max: 55.0,
    temp_soft_zone: 5.0,
    temp_mode: 0,
    spike_volt_max: 60.0,
    spike_curr_max: 350.0,
    spike_soc_max: 95,
    can_enabled: true,
    can_protocol: 0,                   // Victron
    board_preset: 0,                   // Waveshare
    rs485_enabled: true,
    pins: { rs485_tx: 17, rs485_rx: 18, rs485_dir: 21, can_tx: 15, can_rx: 16, led: 38 },
    maint_charge_enabled: false,
    maint_target_voltage: 54.0,
    auto_balance_enabled: true,
    auto_balance_last_ts: NOW_EPOCH - 86400,
    mqtt_enabled: true,
    mqtt_host: '192.168.1.100',
    mqtt_port: 1883,
    mqtt_user: 'homeassistant',
    mqtt_pass_obf: '',
    mqtt_base_topic: 'topband-bms',
    mqtt_level: 3,
    mqtt_diag_enabled: true,
    ha_discovery_enabled: true,
    mqtt_full_publish: true,
    mqtt_solar_passthrough_topic: 'opendtu/solar/passthrough/state',
    ntp_server: 'pool.ntp.org',
    timezone_offset_h: 1,
    auth_enabled: false,               // no login prompt in demo
    auth_user: 'admin',
    auth_hash: '',
    theme_id: 0,
    chart_series_a: 0,
    chart_series_b: 1,
    ui_poll_live_ms: 1500,
    ui_poll_diag_ms: 5000,
    ui_poll_alerts_ms: 30000,
    last_reset_ts: NOW_EPOCH - UPTIME_BASE,
    notify_telegram_enabled: true,
    notify_telegram_chat_id: '123456789',
    notify_sender_name: 'BMS Gateway Demo',
    notify_telegram_token: '',
    notify_telegram_last_ok_ts: NOW_EPOCH - 3600,
    notify_telegram_verified: true,
    notify_poll_interval_s: 60,
    notify_cooldown_s: 120,
    notify_debounce_s: 30,
    notify_alert_flags: 0x3FF,
    // BLE sources (V3.1/V3.2) — SmartShunt + MPPT both paired for the demo.
    ble_shunt_enabled: true,
    ble_mppt_enabled: true,
    ble_shunt_mac: 'aa:bb:cc:dd:ee:01',
    ble_mppt_mac: 'aa:bb:cc:dd:ee:02',
    ble_shunt_key: '',                 // SECRET — always redacted
    ble_mppt_key: '',                  // SECRET — always redacted
    // Preferred AP pin (V3.2) — pinned to the strongest of the demo scan list.
    wifi_bssid: 'aa:bb:cc:11:22:33',
    wifi_rssi_threshold: -80,
    // Battery Value Sources (V3.2/schema v11) — Auto: shunt leads when fresh.
    battery_source_policy: 0,          // Auto
    voltage_source: 0,                 // Manual-mode only, ignored in Auto
    current_source: 0,
    soc_source: 0,
  };

  /* ── Notify static data ──────────────────────────────────────────────────── */
  const DEMO_NOTIFY_STATUS = {
    token_stored: true,
    chat_id_stored: true,
    verified: true,
    last_ok_ts: NOW_EPOCH - 3600,
  };

  const DEMO_ALERT_TYPES = [
    { id: 0, name: 'Pack offline',                   group: 'system' },
    { id: 1, name: 'Pack came back online',           group: 'system' },
    { id: 2, name: 'Cell overvoltage',                group: 'voltage' },
    { id: 3, name: 'Cell undervoltage',               group: 'voltage' },
    { id: 4, name: 'Pack overvoltage',                group: 'voltage' },
    { id: 5, name: 'Pack undervoltage',               group: 'voltage' },
    { id: 6, name: 'Cell imbalance (drift)',          group: 'cell' },
    { id: 7, name: 'Overtemperature (charge stop)',   group: 'temperature' },
    { id: 8, name: 'Undertemperature (charge stop)',  group: 'temperature' },
    { id: 9, name: 'BMS alarm bits set',              group: 'system' },
  ];

  /* ── Live data generator ─────────────────────────────────────────────────── */
  let g_tick = 0;

  // Base cell voltages for each pack (15 cells, LiFePO4 at ~80% SOC)
  const CELL_BASE = [
    [3.326, 3.322, 3.329, 3.320, 3.325, 3.323, 3.328, 3.321,
     3.324, 3.322, 3.327, 3.329, 3.320, 3.326, 3.323],
    [3.324, 3.321, 3.328, 3.319, 3.325, 3.322, 3.327, 3.320,
     3.323, 3.321, 3.326, 3.328, 3.319, 3.324, 3.322],
    [3.325, 3.322, 3.327, 3.320, 3.324, 3.322, 3.328, 3.321,
     3.323, 3.322, 3.326, 3.327, 3.319, 3.325, 3.323],
  ];
  const SOC_BASE   = [85, 82, 78];
  const SOC_PHASE  = [0, 1.1, 2.3];
  const TEMP_BASE  = [[22.1, 21.8], [21.5, 21.2], [22.8, 22.4]];

  function f2(v) { return Math.round(v * 100) / 100; }
  function f3(v) { return Math.round(v * 1000) / 1000; }
  function f1(v) { return Math.round(v * 10) / 10; }

  function makePack(i, t) {
    const socV = SOC_BASE[i] + Math.sin(t * 0.025 + SOC_PHASE[i]) * 1.5;
    const cur  = -3.5 + Math.sin(t * 0.07 + i * 0.8) * 2.0; // mild discharge
    const volt = 50.8 + i * 0.15 + Math.sin(t * 0.05 + i) * 0.25;

    const cells = CELL_BASE[i].map((base, ci) =>
      f3(base + Math.sin(t * 0.02 + ci * 0.31 + i * 1.73) * 0.0015)
    );
    const cellMin    = Math.min.apply(null, cells);
    const cellMax    = Math.max.apply(null, cells);
    const cellMinIdx = cells.indexOf(cellMin);
    const cellMaxIdx = cells.indexOf(cellMax);
    const drift      = f3(cellMax - cellMin);

    const temps = TEMP_BASE[i].map((b, ti) =>
      f1(b + Math.sin(t * 0.04 + i + ti * 0.5) * 1.2)
    );

    return {
      bms_id:        i,
      online:        true,
      pack_voltage:  f2(volt),
      pack_current:  f2(cur),
      soc:           Math.round(socV),
      soh:           100,
      cycles:        42 + i * 7,
      temp_max_c:    Math.max.apply(null, temps),
      temp_avg_c:    f1(temps.reduce(function(a,b){return a+b;}, 0) / temps.length),
      cell_count:    15,
      rem_ah:        f1(socV * 0.01 * 100),
      full_ah:       100.0,
      cell_min_v:    cellMin,
      cell_max_v:    cellMax,
      cell_min_idx:  cellMinIdx,
      cell_max_idx:  cellMaxIdx,
      cell_drift_v:  drift,
      current_held:  false,
      cells:         cells,
      _temps:        temps,   // kept for detail page
    };
  }

  // SmartShunt whole-bank reading — slightly different from the BMS pack
  // average (that's the point of the shunt: a truer whole-bank number).
  function makeShuntReading(t, voltAvg, curTotal) {
    return {
      voltage_v:   f2(voltAvg + 0.03 + Math.sin(t * 0.03) * 0.02),
      current_a:   f2(curTotal - 0.15 + Math.sin(t * 0.11) * 0.08),
      soc_pct:     f1(83 + Math.sin(t * 0.02) * 1.2),
      soc_valid:   true,
      consumed_ah_valid: true,
      consumed_ah: f1(-42.3 - t * 0.0006),
    };
  }

  // Victron MPPT solar reading — a gentle bell curve keyed off wall-clock hour
  // so the demo looks like a real solar day regardless of when it's viewed.
  function mpptPowerForHour(hourFrac) {
    // Sun roughly 06:00-20:00, peak ~700 W at 13:00.
    if (hourFrac < 6 || hourFrac > 20) return 0;
    var x = (hourFrac - 13) / 7; // -1..1 across the daylight window
    var shape = Math.max(0, Math.cos(x * Math.PI / 2));
    return 700 * Math.pow(shape, 1.4);
  }

  function makeLiveData() {
    g_tick++;
    var t = g_tick;
    var packs = [0, 1, 2].map(function(i) { return makePack(i, t); });

    var curTotal  = packs.reduce(function(s,p){return s+p.pack_current;}, 0);
    var voltAvg   = packs.reduce(function(s,p){return s+p.pack_voltage;}, 0) / 3;
    var socAvg    = packs.reduce(function(s,p){return s+p.soc;}, 0) / 3;
    var tempAvg   = packs.reduce(function(s,p){return s+p.temp_avg_c;}, 0) / 3;

    // ── Battery Value Sources fusion (Auto policy: shunt leads when fresh) ──
    var shuntEnabled = DEMO_CONFIG.ble_shunt_enabled;
    var shunt = makeShuntReading(t, voltAvg, curTotal);
    var shuntFresh = shuntEnabled; // always fresh in the demo (fake live poll)
    var socDisplay  = shuntFresh ? shunt.soc_pct   : f1(socAvg);
    var voltDisplay = shuntFresh ? shunt.voltage_v : f2(voltAvg);
    var curDisplay  = shuntFresh ? shunt.current_a : f2(curTotal);
    var srcId = shuntFresh ? 'shunt' : 'bms';

    // ── MPPT solar reading — driven by wall-clock hour for a realistic curve ──
    var mpptEnabled = DEMO_CONFIG.ble_mppt_enabled;
    var nowDate  = new Date();
    var hourFrac = nowDate.getHours() + nowDate.getMinutes() / 60;
    var pvPowerW = f1(mpptPowerForHour(hourFrac) + Math.sin(t * 0.2) * 4);
    var charging = pvPowerW > 5;
    var battV    = charging ? f2(voltAvg + 0.4) : null;
    var battA    = charging ? f2(pvPowerW / (voltAvg + 0.4)) : 0;

    return {
      uptime_s:              UPTIME_BASE + t * 2,
      bms_count_configured:  3,
      bms_count_online:      3,
      snapshot: {
        cycle_id:    1000 + t,
        produced_ms: (UPTIME_BASE + t * 2) * 1000 - 90,
        packs:       packs,
      },
      safety: {
        cvl_volts:           54.6,
        ccl_amps:            150.0,
        dcl_amps:            300.0,
        soc_avg:             f1(socAvg),
        soh_avg:             100.0,
        temp_avg:            f1(tempAvg),
        pack_voltage_avg:    f2(voltAvg),
        pack_current_total:  f2(curTotal),
        soc_display:         socDisplay,
        voltage_display:     voltDisplay,
        current_display:     curDisplay,
        alarm_flags:         0,
        lockout_flags:       0,
        temp_alarm:          0,
        sys_message:         'OK',
        packs_online:        3,
        packs_configured:    3,
        factor_charge:       1.0,
        factor_discharge:    1.0,
      },
      sources: {
        battery_voltage_src: srcId,
        battery_current_src: srcId,
        battery_soc_src:     srcId,
        mppt: {
          enabled:          mpptEnabled,
          seen:             mpptEnabled,
          ms_since_last_seen: mpptEnabled ? 1200 : 0,
          pv_power_valid:   mpptEnabled,
          pv_power_w:       pvPowerW,
          pv_v_valid:       mpptEnabled && charging,
          pv_voltage_v:     charging ? f2(72 + Math.sin(t * 0.1) * 2) : 0,
          pv_i_valid:       mpptEnabled && charging,
          pv_current_a:     charging ? f2(pvPowerW / 73) : 0,
          yield_valid:      mpptEnabled,
          yield_today_wh:   f1(Math.max(0, (hourFrac - 6)) * 320 + t * 0.5),
          batt_v_valid:     mpptEnabled && charging,
          batt_voltage_v:   battV,
          batt_i_valid:     mpptEnabled && charging,
          batt_current_a:   battA,
          charge_state:     charging ? 4 : 0,   // 4=Absorption, 0=Off
        },
        shunt: {
          seen:               shuntEnabled,
          ms_since_last_seen: shuntEnabled ? 900 : 0,
        },
        solar_passthrough: {
          received:      !!DEMO_CONFIG.mqtt_solar_passthrough_topic,
          ms_since_last: 4200,
          state:         charging,
        },
      },
      stats: {
        poller: {
          cycles_completed:    5000 + t,
          analog_polls_ok:     14990 + t * 3,
          analog_polls_timeout: 10,
          cycle_avg_ms:        106,
          cycle_max_ms:        312,
        },
        can: {
          tx_ok:          4800 + t * 2,
          tx_fail:        0,
          heartbeats:     2400 + t,
          express_sends:  0,
          bus_off_count:  0,
        },
        bus: {
          publishes: 5000 + t,
          reads:     50000 + t * 10,
          retries:   3,
        },
      },
      energy: {
        today_in_kwh:  f2(1.23 + t * 0.0001),
        today_out_kwh: f2(0.87 + t * 0.00005),
        week_in_kwh:   8.42,
        week_out_kwh:  6.15,
        total_in_kwh:  124.6,
        total_out_kwh: 98.3,
      },
      runtime_est_min:   420 + Math.round(Math.sin(t * 0.1) * 15),
      runtime_est_state: 'until_empty',
      now_ts_s:          NOW_EPOCH + t * 2,
      ntp_synced:        true,
    };
  }

  /* ── Latest live snapshot (for /api/bms/:id) ─────────────────────────────── */
  var g_last_live = makeLiveData();

  /* ── Health ──────────────────────────────────────────────────────────────── */
  function makeHealthData() {
    return {
      version:      '3.3.0',
      build:        'main-bce8993 2026-07-31',
      ui_version:   'h3b-1',
      uptime_s:     UPTIME_BASE + g_tick * 2,
      free_heap_b:  142336,
      free_psram_b: 6291456,
      wifi: { connected: true, rssi: -58, ssid: 'HomeNetwork', ip: '192.168.1.42' },
      mqtt: { enabled: true, state: 'connected' },
      ntp_synced:   true,
      now_ts_s:     NOW_EPOCH + g_tick * 2,
      auth_enabled: false,
    };
  }

  /* ── Solar day chart (/api/solar-day) ────────────────────────────────────── */
  // 5-minute resolution, midnight to now, matching the real gateway's day-ring.
  var SOLAR_RES = 300;
  function todayMidnightEpoch() {
    var d = new Date(NOW_EPOCH * 1000);
    d.setHours(0, 0, 0, 0);
    return Math.floor(d.getTime() / 1000);
  }
  function makeSolarDayData() {
    var midnight = todayMidnightEpoch();
    var nowTs    = NOW_EPOCH + g_tick * 2;
    var count    = Math.max(1, Math.floor((nowTs - midnight) / SOLAR_RES));
    var pts = [];
    for (var i = 0; i < count; i++) {
      var ts = midnight + i * SOLAR_RES;
      var hourFrac = (ts - midnight) / 3600;
      var w = mpptPowerForHour(hourFrac);
      pts.push(w > 0 ? Math.round(w + Math.sin(i * 0.7) * 6) : (hourFrac < 6 || hourFrac > 20 ? null : 0));
    }
    return {
      points:         pts,
      resolution_s:   SOLAR_RES,
      t0_epoch:       midnight,
      midnight_epoch: midnight,
      now_ts_s:       nowTs,
    };
  }

  /* ── Battery Drift Details (/api/drift) ──────────────────────────────────── */
  // 5-day per-cell band history for each pack, matching a healthy, well-balanced
  // 15S LiFePO4 bank. Cell 4 in pack 0 is nudged to show the "fills first" /
  // outlier-highlight behaviour so the demo isn't just fifteen identical bars.
  function makeDriftData() {
    var packs = [0, 1, 2].map(function (pid) {
      var cells = [];
      for (var ci = 0; ci < 15; ci++) {
        var baseMv = 3320 + (ci % 4) * 3 - (pid * 2);
        var isHot  = (pid === 0 && ci === 3); // this pack/cell fills first
        var nowMv  = baseMv + (isHot ? 9 : 0) + Math.round(Math.sin((g_tick + ci) * 0.05) * 2);
        cells.push({
          now:   nowMv,
          d5min: baseMv - 4 + (isHot ? 6 : 0),
          d5max: baseMv + 5 + (isHot ? 9 : 0),
          evMin: baseMv - 2 + (isHot ? 7 : 0),
          evMax: baseMv + 3 + (isHot ? 9 : 0),
        });
      }
      var spreadNow = Math.max.apply(null, cells.map(function(c){return c.now;}))
                    - Math.min.apply(null, cells.map(function(c){return c.now;}));
      return {
        id:             pid,
        name:           'Pack ' + (pid + 1),
        cell_count:     15,
        spread_now:     spreadNow,
        has_history:    true,
        has_toc:        true,
        toc_spread:     pid === 0 ? 18 : 9,
        first_full_mv:  pid === 0 ? 3334 : 0,
        first_full_idx: pid === 0 ? 3 : 0,
        ff_mode_idx:    pid === 0 ? 3 : 0,
        ff_days_won:    pid === 0 ? 4 : 1,
        ff_days_total:  5,
        n_toc_days:     5,
        drift_rate:     pid === 0 ? 0.6 : 0.1,
        has_bod:        true,
        bod_spread:     6,
        first_empty_mv: 3298,
        first_empty_idx: 7,
        fe_mode_idx:    7,
        fe_days_won:    2,
        fe_days_total:  5,
        cells:          cells,
      };
    });
    return { packs: packs };
  }

  /* ── History data generator ──────────────────────────────────────────────── */
  // Generates 2 h of data at 60 s resolution (120 points)
  var HIST_POINTS = 120;
  var HIST_RES    = 60;
  var HIST_T0     = NOW_EPOCH - HIST_POINTS * HIST_RES;

  function generateHistory(series) {
    var pts = [];
    for (var i = 0; i < HIST_POINTS; i++) {
      var frac = i / HIST_POINTS;
      var v;
      switch (series) {
        case 'power':
          // Mild discharge (-400 to -600 W), smoothly varying
          v = -(480 + Math.sin(frac * Math.PI * 4 + 0.5) * 90 +
               Math.sin(frac * Math.PI * 11) * 30);
          v = Math.round(v);
          break;
        case 'soc':
          // Slowly declining from 84% to 82%
          v = 84 - frac * 2 + Math.sin(frac * Math.PI * 6) * 0.3;
          v = Math.round(v * 10) / 10;
          break;
        case 'voltage':
          // ~50.8 V, gentle oscillation
          v = 50.82 - frac * 0.15 + Math.sin(frac * Math.PI * 5) * 0.12;
          v = Math.round(v * 10) / 10;
          break;
        case 'temp':
          // 21 -> 23 °C over the session
          v = 21.5 + frac * 1.5 + Math.sin(frac * Math.PI * 3) * 0.4;
          v = Math.round(v * 10) / 10;
          break;
        case 'drift':
          // 6-10 mV, subtle variation
          v = 7 + Math.sin(frac * Math.PI * 7) * 2;
          v = Math.round(v * 10) / 10;
          break;
        default:
          v = 0;
      }
      pts.push(v);
    }
    return pts;
  }

  function makeHistoryResponse(search) {
    var params = {};
    (search || '').replace(/^\?/, '').split('&').forEach(function(kv) {
      var p = kv.split('=');
      if (p[0]) params[decodeURIComponent(p[0])] = decodeURIComponent(p[1] || '');
    });
    var seriesKey = params['series'] || 'power';
    return {
      series: [{
        t0_epoch:     HIST_T0,
        resolution_s: HIST_RES,
        points:       generateHistory(seriesKey),
      }],
    };
  }

  /* ── Alerts ──────────────────────────────────────────────────────────────── */
  function makeAlertsResponse(search) {
    var params = {};
    (search || '').replace(/^\?/, '').split('&').forEach(function(kv) {
      var p = kv.split('=');
      if (p[0]) params[decodeURIComponent(p[0])] = decodeURIComponent(p[1] || '');
    });
    var minSev = parseInt(params['min_severity'] || '0', 10);
    var skip   = parseInt(params['skip']         || '0', 10);
    var limit  = parseInt(params['limit']        || '50', 10);
    var filtered = DEMO_ALERTS.filter(function(a) { return (a.severity_n || 0) >= minSev; });
    var page     = filtered.slice(skip, skip + limit);
    return { total: filtered.length, alerts: page };
  }

  /* ── Per-pack detail (/api/bms/:id) ─────────────────────────────────────── */
  function makeBmsDetail(path) {
    var id = parseInt((path.match(/\/api\/bms\/(\d+)/) || [])[1], 10);
    if (isNaN(id) || id < 0 || id > 2) {
      return { __status: 404, error: 'Pack not found' };
    }
    var p = g_last_live.snapshot.packs[id];
    var temps = p._temps || [22.0, 21.7];
    return {
      id:               id,
      online:           true,
      last_seen_age_ms: 85,
      soc:              p.soc,
      soh:              100,
      cycles:           p.cycles,
      pack_v:           p.pack_voltage,
      current:          p.pack_current,
      power:            f1(p.pack_voltage * p.pack_current),
      rem_ah:           p.rem_ah,
      full_ah:          p.full_ah,
      cell_count:       15,
      cell_min_v:       p.cell_min_v,
      cell_max_v:       p.cell_max_v,
      cell_min_idx:     p.cell_min_idx,
      cell_max_idx:     p.cell_max_idx,
      drift_mv:         Math.round(p.cell_drift_v * 1000),
      cells:            p.cells,
      temps:            temps.map(function(v, ti) {
        return { lbl: 'T' + (ti + 1), val: v };
      }),
      alarm_bits:       '0x0000000000000000',
      sysparam: {
        valid:              true,
        age_s:              42,
        cell_high_v:        3.650,
        cell_low_v:         2.800,
        module_high_v:      54.75,
        module_low_v:       42.0,
        module_under_v:     40.0,
        charge_t_min:       5.0,
        charge_t_max:       45.0,
        discharge_t_min:    -10.0,
        discharge_t_max:    55.0,
        charge_max_a:       50.0,
        discharge_max_a:    100.0,
      },
      rs485: {
        polls:       5000 + id * 300,
        ok:          4997 + id * 300,
        timeouts:    2,
        errors:      1,
        success_pct: 99,
      },
    };
  }

  /* ── WiFi status & scan ──────────────────────────────────────────────────── */
  function makeWifiStatus() {
    return {
      connected:       true,
      ssid:            'HomeNetwork',
      bssid:           DEMO_CONFIG.wifi_bssid,
      bssid_pin_active: true,
      rssi:            -58,
      ip:              '192.168.1.42',
      gateway:         '192.168.1.1',
      netmask:         '255.255.255.0',
      dns:             '192.168.1.1',
      mdns_hostname:   'topband-bms',
      connected_for_s: UPTIME_BASE + g_tick * 2 - 12,
    };
  }

  var DEMO_WIFI_SCAN = [
    { ssid: 'HomeNetwork',     bssid: 'aa:bb:cc:11:22:33', rssi: -58, secure: true  },
    { ssid: 'HomeNetwork',     bssid: 'aa:bb:cc:11:22:44', rssi: -69, secure: true  },
    { ssid: 'FRITZ!Box 7590',  bssid: '10:20:30:40:50:60', rssi: -71, secure: true  },
    { ssid: 'IoT-Network',     bssid: '10:20:30:40:50:61', rssi: -74, secure: true  },
    { ssid: 'GuestNet',        bssid: '10:20:30:40:50:62', rssi: -82, secure: false },
  ];

  /* ── Diagnostics ─────────────────────────────────────────────────────────── */
  function makeDiagData() {
    var t = g_tick;
    return {
      system: {
        fw:                  '3.3.0',
        build:               'main-bce8993 2026-07-31',
        uptime_s:            UPTIME_BASE + t * 2,
        reset_reason:        'Power on',
        free_heap:           142336,
        dram_free:           98304,
        dram_min:            84992,
        dram_largest_block:  65536,
        psram_free:          6291456,
        psram_largest_block: 4194304,
        cpu_temp_c:          null,
      },
      packs: g_last_live.snapshot.packs.map(function (p) {
        return {
          id:               p.bms_id,
          bms_id:           p.bms_id,
          online:           p.online,
          last_seen_age_ms: 85,
          polls:            5200 + p.bms_id * 300,
          ok:               5195 + p.bms_id * 300,
          timeouts:         2,
          errors:           1,
          success_pct:      99,
          soc:              p.soc,
          soh:              p.soh,
          cell_min_v:       p.cell_min_v,
          cell_min_idx:     p.cell_min_idx,
          cell_max_v:       p.cell_max_v,
          cell_max_idx:     p.cell_max_idx,
          drift_mv:         Math.round(p.cell_drift_v * 1000),
          sysparam_valid:   true,
          sys_charge_max_a:    50.0,
          sys_discharge_max_a: 100.0,
          sys_cell_high_v:     3.650,
          alarm_bits:       '0x0000000000000000',
        };
      }),
      battery: {
        has_data:      true,
        packs_online:  3,
        cvl_volts:     54.6,
        ccl_amps:      150.0,
        dcl_amps:      300.0,
        dvl_volts:     45.0,
        alarm_flags:   0,
        lockout_flags: 0,
        temp_alarm:    0,
        sys_message:   'OK',
      },
      tasks: [
        { name: 'safetyLoop',  stack_hwm: 876,  core: 0, prio: 5  },
        { name: 'canTx',       stack_hwm: 944,  core: 1, prio: 4  },
        { name: 'mqttPub',     stack_hwm: 2048, core: 0, prio: 3  },
        { name: 'bmsPoll',     stack_hwm: 2304, core: 1, prio: 5  },
        { name: 'httpd',       stack_hwm: 3200, core: 0, prio: 4  },
        { name: 'main',        stack_hwm: 3640, core: 0, prio: 1  },
        { name: 'timerTask',   stack_hwm: 1836, core: 0, prio: 22 },
        { name: 'ipc0',        stack_hwm: 536,  core: 0, prio: 24 },
        { name: 'ipc1',        stack_hwm: 536,  core: 1, prio: 24 },
      ],
      poller: {
        cycles_completed:  5200 + t,
        cycle_avg_ms:      106,
        cycle_max_ms:      312,
        rs485_ok:          15587 + t * 3,
        rs485_timeouts:    8,
        rs485_parse_err:   5,
        alarm_polls_ok:    15587 + t * 3,
        alarm_polls_err:   0,
        sysparam_polls_ok: 520 + t,
        sysparam_polls_err: 0,
        wrong_addr:        0,
      },
      can: {
        protocol:           'victron',
        tx_ok:              4800 + t * 2,
        tx_fail:            0,
        tx_fail_streak_max: 0,
        heartbeats:         2400 + t,
        express_sends:      12,
        bus_off_count:      0,
        driver_restarts:    0,
      },
      snapshot_bus: {
        publishes: 5200 + t,
        reads:     52000 + t * 10,
        retries:   3,
      },
      mqtt: {
        enabled:       true,
        state:         'connected',
        effective_base: DEMO_CONFIG.mqtt_base_topic,
        publish_ok:    4800 + t * 2,
        publish_fail:  0,
        publish_drops: 0,
        publish_max_ms: 12,
      },
      ntp: { synced: true, server: 'pool.ntp.org', now_ts_s: NOW_EPOCH + t * 2 },
      littlefs: {
        total_b: 1507328,
        used_b:  204800 + t * 2,
        free_b:  1302528 - t * 2,
      },
      energy: {
        today_in_kwh:  f2(1.23 + t * 0.0001),
        today_out_kwh: f2(0.87 + t * 0.00005),
        total_in_kwh:  124.6,
        total_out_kwh: 98.3,
      },
      history: {
        fine_samples:   120,
        coarse_samples: 48,
      },
      alerts_count: DEMO_ALERTS.length,
      ble_status: {
        ble_active:       true,
        stack:            'NimBLE',
        ble_gap_events:   184320 + t * 4,
        ble_victron_advs: 92150 + t * 2,
        ble_mppt_advs:    46080 + t,
        ble_debug: {
          configured_mac:      DEMO_CONFIG.ble_mppt_mac,
          mppt_mac_valid:      true,
          mppt_key_valid:      true,
          victron_total:       92150 + t * 2,
          mppt_type_match:     46200 + t,
          mppt_mac_match:      46150 + t,
          mppt_decrypt_ok:     46080 + t,
          configured_shunt_mac: DEMO_CONFIG.ble_shunt_mac,
          shunt_mac_valid:     true,
          shunt_key_valid:     true,
          shunt_type_match:    46000 + t,
          shunt_mac_match:     45950 + t,
          shunt_decrypt_ok:    45900 + t,
          shunt_last_mfg_len:  27,
          shunt_last_new_fmt:  true,
        },
        wifi_state:                      'connected',
        wifi_outage_duration_s:          0,
        wifi_reconnect_attempts:         0,
        wifi_backoff_ms:                 0,
        wifi_reconnect_attempts_total:   2,
        wifi_last_outage_duration_s:     45,
        wifi_ssid:                       'HomeNetwork',
        wifi_ip:                         '192.168.1.42',
        wifi_hostname:                   'topband-bms',
        wifi_connected_for_s:            UPTIME_BASE + t * 2 - 12,
        wifi_disconnects:                1,
        wifi_bssid:                      DEMO_CONFIG.wifi_bssid,
        wifi_rssi:                       -58,
        wifi_bssid_pin_active:           true,
        mppt: {
          enabled:        DEMO_CONFIG.ble_mppt_enabled,
          seen:            g_last_live.sources.mppt.seen,
          last_seen_s:     1,
          pv_power_w:      g_last_live.sources.mppt.pv_power_w,
          batt_voltage_v:  g_last_live.sources.mppt.batt_voltage_v || 0,
          batt_current_a:  g_last_live.sources.mppt.batt_current_a || 0,
          charge_state:    g_last_live.sources.mppt.charge_state,
          yield_today_wh:  g_last_live.sources.mppt.yield_today_wh,
        },
        shunt: {
          enabled:            DEMO_CONFIG.ble_shunt_enabled,
          last_seen_s:        1,
          current_a:          -3.5,
          voltage_v:           g_last_live.safety.pack_voltage_avg + 0.03,
          soc_valid:            true,
          soc_pct:              83.0,
          consumed_ah_valid:    true,
          consumed_ah:          -42.3,
        },
        bms_current_a:  -3.5,
        handler_last_ms: 3,
        handler_max_ms:  18,
      },
      log_ring: [
        '[    0.000] I boot: TopBand BMS Gateway v3.3.0 starting',
        '[    0.012] I storage: NVS config loaded, schema v6',
        '[    0.034] I wifi: connecting to SSID "HomeNetwork"',
        '[    1.203] I wifi: connected — IP 192.168.1.42, GW 192.168.1.1',
        '[    1.245] I ntp: sync OK — offset +0.002 s',
        '[    1.301] I mqtt: connecting to 192.168.1.100:1883 as "homeassistant"',
        '[    1.512] I mqtt: connected',
        '[    1.520] I mqtt: HA discovery sent (42 entities)',
        '[    2.001] I bms: poller started, 3 packs configured',
        '[    2.105] I bms[0]: first response OK — 15S, SOC=85%, V=50.95 V',
        '[    2.218] I bms[1]: first response OK — 15S, SOC=82%, V=50.80 V',
        '[    2.331] I bms[2]: first response OK — 15S, SOC=78%, V=50.72 V',
        '[    2.340] I safety: all 3 packs online — CCL=150 A DCL=300 A CVL=54.6 V',
        '[    2.345] I can: Victron protocol active, TX started',
        '[  205.001] W bms[2]: RS485 timeout (attempt 1/3)',
        '[  205.100] I bms[2]: RS485 recovered on retry 1',
      ],
      coredump: { present: false },
    };
  }

  /* ── Mock fetch dispatcher ───────────────────────────────────────────────── */

  function mockResponse(data, status) {
    var st   = status || 200;
    var body = JSON.stringify(data);
    return Promise.resolve(new Response(body, {
      status:  st,
      headers: { 'Content-Type': 'application/json' },
    }));
  }

  // Write actions that should show the demo notice.
  // Returns true if the request was intercepted (caller should also return).
  var WRITE_DEMO_PATHS = [
    '/api/restart',
    '/api/factory_reset',
    '/api/ota/upload',
    '/api/wifi/configure',
    '/api/svc/ha/discovery/send',
    '/api/restore',
    '/api/auth/set_password',
    '/api/auth/logout',
  ];

  function demoToast() {
    if (typeof showToast === 'function') {
      showToast('Demo mode — this action is disabled in the online demo', 'warn');
    }
  }

  var _realFetch = window.fetch.bind(window);

  window.fetch = function mockFetch(input, init) {
    var url    = (typeof input === 'string') ? input : (input && input.url) || '';
    var qmark  = url.indexOf('?');
    var path   = (qmark >= 0 ? url.slice(0, qmark) : url)
                   .replace(/^https?:\/\/[^/]+/, '');
    var search = (qmark >= 0) ? url.slice(qmark) : '';
    var method = ((init && init.method) || 'GET').toUpperCase();

    /* ── GET routes ──────────────────────────────────────────────────────── */
    if (method === 'GET' || method === 'HEAD') {
      if (path === '/api/live') {
        g_last_live = makeLiveData();
        return mockResponse(g_last_live);
      }
      if (path === '/api/health') {
        return mockResponse(makeHealthData());
      }
      if (path === '/api/config') {
        return mockResponse(DEMO_CONFIG);
      }
      if (path === '/api/diag') {
        return mockResponse(makeDiagData());
      }
      if (path === '/api/history') {
        return mockResponse(makeHistoryResponse(search));
      }
      if (path === '/api/alerts') {
        return mockResponse(makeAlertsResponse(search));
      }
      if (/^\/api\/bms\/\d+$/.test(path)) {
        var detail = makeBmsDetail(path);
        var dstatus = detail.__status || 200;
        delete detail.__status;
        return mockResponse(detail, dstatus);
      }
      if (path === '/api/wifi/status') {
        return mockResponse(makeWifiStatus());
      }
      if (path === '/api/wifi/scan') {
        // Simulate a brief scan delay
        return new Promise(function(resolve) {
          setTimeout(function() {
            resolve(new Response(JSON.stringify(DEMO_WIFI_SCAN), {
              status:  200,
              headers: { 'Content-Type': 'application/json' },
            }));
          }, 800);
        });
      }
      if (path === '/api/notify/status') {
        return mockResponse(DEMO_NOTIFY_STATUS);
      }
      if (path === '/api/notify/alert-types') {
        return mockResponse(DEMO_ALERT_TYPES);
      }
      if (path === '/api/solar-day') {
        return mockResponse(makeSolarDayData());
      }
      if (path === '/api/drift') {
        return mockResponse(makeDriftData());
      }
      if (path === '/api/net/self-test') {
        // Always answer "already finished, all green" — the demo has no real
        // TLS/DNS/TCP path to test.
        return mockResponse({
          running: false,
          current_stage: -1,
          stages: [
            { label: 'WiFi / link',         run: true, pass: true, duration_ms: 4 },
            { label: 'DNS resolution',      run: true, pass: true, duration_ms: 22 },
            { label: 'TCP connect',         run: true, pass: true, duration_ms: 38 },
            { label: 'TLS handshake',       run: true, pass: true, duration_ms: 210 },
            { label: 'Time / cert sanity',  run: true, pass: true, duration_ms: 3 },
          ],
        });
      }
      if (path === '/api/mqtt/test') {
        return mockResponse({ status: 'idle' });
      }
      if (path === '/api/notify/telegram/test') {
        return mockResponse({ status: 'idle' });
      }
      if (path === '/api/ota/status') {
        return mockResponse({ running_state: 'valid', self_test: { in_progress: false } });
      }
      if (path === '/api/backup') {
        // downloadBackup() is patched below; this path is a fallback
        return mockResponse({ error: 'Demo mode' }, 400);
      }
      // Unknown GET — 404
      return mockResponse({ error: 'Not found' }, 404);
    }

    /* ── POST / DELETE routes ────────────────────────────────────────────── */

    if (method === 'DELETE' && path === '/api/alerts') {
      DEMO_ALERTS = [];
      return mockResponse({ ok: true });
    }

    if (method === 'POST') {
      // Config save: let it "work" silently + show a toast so the visitor knows
      if (path === '/api/config') {
        setTimeout(demoToast, 80);
        return mockResponse(DEMO_CONFIG);
      }

      // MQTT connection test — return immediate "demo" failure so the UI shows
      // something in the result box rather than spinning forever
      if (path === '/api/mqtt/test') {
        setTimeout(demoToast, 80);
        return mockResponse({ status: 'failed', stage: 'tcp',
          message: 'Demo mode — connection test not available' });
      }

      // Telegram test — same pattern
      if (path === '/api/notify/telegram/test') {
        setTimeout(demoToast, 80);
        return mockResponse({ status: 'failed',
          message: 'Demo mode — notification test not available' });
      }

      // Network self-test — actually "starts" so the UI polls and shows the
      // canned all-green result from the GET handler above, rather than an
      // error (there's no real TLS/DNS/TCP path to test in the demo).
      if (path === '/api/net/self-test') {
        return mockResponse({ ok: true });
      }

      // All other write actions: show demo notice and return an error so the
      // UI stays on the current page
      demoToast();
      return mockResponse({ error: 'Demo mode — this action is disabled' }, 400);
    }

    // Fallback
    return mockResponse({ error: 'Not found' }, 404);
  };

  /* ── Post-load function patches ──────────────────────────────────────────── */
  // Patch functions that bypass fetch (XHR-based OTA, location.href backup,
  // the restart/wifi overlay that fires unconditionally after POST).
  document.addEventListener('DOMContentLoaded', function () {

    // Override updateLiveUI: app.js checks window.location.pathname === '/'
    // which fails when opened via file:// or a Pages subpath. Use DOM-element
    // presence instead so updates work regardless of URL structure.
    window.updateLiveUI = function () {
      if (typeof updateStatusBar   === 'function') updateStatusBar();
      if (typeof updateAuthBanner  === 'function') updateAuthBanner();
      if (document.getElementById('metrics-grid')) {
        if (typeof updateDashboardCards === 'function') updateDashboardCards();
        if (typeof updatePackCards      === 'function') updatePackCards();
        if (typeof updateChartBadges    === 'function') updateChartBadges();
      } else if (document.getElementById('battery-overview-grid')) {
        if (typeof updateBatteryOverviewCards === 'function') updateBatteryOverviewCards();
        if (typeof updateDriftNow             === 'function') updateDriftNow();
      } else if (document.getElementById('solar-page-root')) {
        if (typeof updateSolarValues === 'function') updateSolarValues();
      }
    };

    // Force an immediate dashboard render after the first fetch resolves.
    // setTimeout(0) pushes this past the microtask queue so g_live is already set.
    setTimeout(function () {
      if (document.getElementById('metrics-grid')) {
        if (typeof updateDashboardCards === 'function') updateDashboardCards();
        if (typeof updatePackCards      === 'function') updatePackCards();
      }
    }, 0);

    // confirmRestart calls showPageOverlay unconditionally after POST
    window.confirmRestart = function () {
      if (!confirm('Restart the gateway?\n\n(Demo mode — no action will be taken)')) return;
      demoToast();
    };

    // OTA upload uses XMLHttpRequest, not fetch
    window.startOtaUpload = function () {
      demoToast();
    };

    // downloadBackup sets window.location.href directly
    window.downloadBackup = function () {
      demoToast();
    };

    // doWifiConnect calls showPageOverlay unconditionally after POST
    window.doWifiConnect = function () {
      demoToast();
    };

    // confirmFactoryReset shows a confirm then calls doFactoryReset
    window.confirmFactoryReset = function () {
      if (!confirm('Factory reset?\n\n(Demo mode — no action will be taken)')) return;
      demoToast();
    };

    // startRestore shows a modal then calls doRestore
    window.startRestore = function () {
      demoToast();
    };
  });

})();
