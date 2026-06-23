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
      message: 'System started: TopBand BMS Gateway v3.0.0',
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
    schema_version: 6,
    bms_count: 3,
    force_cell_count: 0,
    battery_config_mode: 1,           // Auto+Margin
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
    can_enabled: true,
    can_protocol: 0,                   // Victron
    board_preset: 0,                   // Waveshare
    rs485_enabled: true,
    pins: { rs485_tx: 17, rs485_rx: 18, rs485_dir: 21, can_tx: 15, can_rx: 16, led: 38 },
    mqtt_enabled: true,
    mqtt_host: '192.168.1.100',
    mqtt_port: 1883,
    mqtt_user: 'homeassistant',
    mqtt_pass_obf: '',
    mqtt_base_topic: 'topband-bms',
    mqtt_level: 3,
    mqtt_diag_enabled: true,
    ha_discovery_enabled: true,
    ntp_server: 'pool.ntp.org',
    timezone_offset_h: 1,
    auth_enabled: false,               // no login prompt in demo
    auth_hash: '',
    notify_telegram_enabled: true,
    notify_telegram_chat_id: '123456789',
    notify_sender_name: 'BMS Gateway Demo',
    notify_telegram_token: '',
    notify_poll_interval_s: 60,
    notify_cooldown_s: 120,
    notify_debounce_s: 30,
    notify_alert_flags: 0x3FF,
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

  function makeLiveData() {
    g_tick++;
    var t = g_tick;
    var packs = [0, 1, 2].map(function(i) { return makePack(i, t); });

    var curTotal  = packs.reduce(function(s,p){return s+p.pack_current;}, 0);
    var voltAvg   = packs.reduce(function(s,p){return s+p.pack_voltage;}, 0) / 3;
    var socAvg    = packs.reduce(function(s,p){return s+p.soc;}, 0) / 3;
    var tempAvg   = packs.reduce(function(s,p){return s+p.temp_avg_c;}, 0) / 3;

    return {
      uptime_s:              UPTIME_BASE + t * 2,
      bms_count_configured:  3,
      bms_count_online:      3,
      snapshot: {
        cycle_id:    1000 + t,
        produced_ms: Date.now(),
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
        alarm_flags:         0,
        sys_message:         'OK',
        packs_online:        3,
        packs_configured:    3,
        factor_charge:       1.0,
        factor_discharge:    1.0,
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
      version:      '3.0.0',
      build:        'develop-c307a01 2026-06-16',
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
    { ssid: 'HomeNetwork',     rssi: -58, secure: true  },
    { ssid: 'FRITZ!Box 7590',  rssi: -71, secure: true  },
    { ssid: 'IoT-Network',     rssi: -74, secure: true  },
    { ssid: 'GuestNet',        rssi: -82, secure: false },
  ];

  /* ── Diagnostics ─────────────────────────────────────────────────────────── */
  function makeDiagData() {
    var t = g_tick;
    return {
      system: {
        fw:                  '3.0.0',
        build:               'develop-c307a01 2026-06-16',
        uptime_s:            UPTIME_BASE + t * 2,
        reset_reason:        'Power on',
        free_heap:           142336,
        dram_free:           98304,
        dram_min:            84992,
        dram_largest_block:  65536,
        psram_free:          6291456,
        psram_largest_block: 4194304,
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
        cycles_completed: 5200 + t,
        cycle_avg_ms:     106,
        cycle_max_ms:     312,
        rs485_polls:      15600 + t * 3,
        rs485_ok:         15587 + t * 3,
        rs485_timeouts:   8,
        rs485_parse_err:  5,
        alarm_polls_ok:   15587 + t * 3,
        alarm_polls_err:  0,
      },
      can: {
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
        state:         'connected',
        publish_ok:    4800 + t * 2,
        publish_fail:  0,
        publish_drops: 0,
      },
      ntp: { synced: true, server: 'pool.ntp.org' },
      littlefs: {
        total_b: 1507328,
        used_b:  204800 + t * 2,
        free_b:  1302528 - t * 2,
      },
      energy: {
        today_in_kwh:  f2(1.23 + t * 0.0001),
        today_out_kwh: f2(0.87 + t * 0.00005),
      },
      history: {
        fine_samples:   120,
        coarse_samples: 48,
      },
      alerts_count: DEMO_ALERTS.length,
      log_ring: [
        '[    0.000] I boot: TopBand BMS Gateway v3.0.0 starting',
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
      } else if (document.getElementById('battery-overview-grid')) {
        if (typeof updateBatteryOverviewCards === 'function') updateBatteryOverviewCards();
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
