/* TopBand BMS Gateway — Phase G dashboard JS */
'use strict';

/* ── Auth / CSRF ────────────────────────────────────────────────────────────── */
// CSRF token from meta tag (injected by handlers_static at serve-time).
// Fallback: sessionStorage written by login.html on successful login.
const CSRF_TOKEN = (function() {
  const meta = document.querySelector('meta[name="csrf-token"]');
  if (meta && meta.content && meta.content !== '{{CSRF_TOKEN}}') return meta.content;
  return sessionStorage.getItem('csrf') || '';
})();

// Authenticated fetch wrapper.
// Adds X-CSRF-Token for mutating methods; redirects to /login.html on 401.
async function apiFetch(url, options) {
  options = options || {};
  const method = (options.method || 'GET').toUpperCase();
  if (['POST', 'PUT', 'DELETE', 'PATCH'].includes(method)) {
    options.headers = Object.assign({}, options.headers, {'X-CSRF-Token': CSRF_TOKEN});
  }
  let r;
  try {
    r = await fetch(url, options);
  } catch (e) {
    throw e;
  }
  if (r.status === 401) {
    window.location.href = '/login.html?next=' + encodeURIComponent(window.location.pathname);
    return null;
  }
  return r;
}

/* ── Theme ─────────────────────────────────────────────────────────────────── */
const THEME_KEY = 'tbms_theme';

function applyTheme(theme) {
  if (theme === 'system') {
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    document.documentElement.setAttribute('data-theme', prefersDark ? 'dark' : 'light');
  } else {
    document.documentElement.setAttribute('data-theme', theme);
  }
  const isDark = document.documentElement.getAttribute('data-theme') === 'dark';
  document.getElementById('icon-sun').style.display  = isDark ? 'none'  : 'block';
  document.getElementById('icon-moon').style.display = isDark ? 'block' : 'none';
}

function cycleTheme() {
  const cur = localStorage.getItem(THEME_KEY) || 'dark';
  const next = cur === 'dark' ? 'light' : cur === 'light' ? 'system' : 'dark';
  localStorage.setItem(THEME_KEY, next);
  applyTheme(next);
}

/* ── Clock ─────────────────────────────────────────────────────────────────── */
function updateClock() {
  const now = new Date();
  const h = String(now.getHours()).padStart(2, '0');
  const m = String(now.getMinutes()).padStart(2, '0');
  const el = document.getElementById('topbar-time');
  if (el) el.textContent = h + ':' + m;
}

/* ── Routing ───────────────────────────────────────────────────────────────── */
const routes = {
  '/':         renderDashboard,
  '/dashboard':renderDashboard,
  '/battery':  renderBattery,
  '/general':  renderSettings,
  '/alerts':   renderAlerts,
  '/diag':     renderDiag,
};

function navigate(path) {
  // Stop diag auto-refresh when leaving the diag page.
  if (path !== '/diag' && g_diag_timer) {
    clearInterval(g_diag_timer);
    g_diag_timer = null;
  }
  history.pushState({}, '', path);
  renderPage(path);
  // Update active sidebar item.
  document.querySelectorAll('.sidebar-item').forEach(el => {
    el.classList.remove('active');
    const href = el.getAttribute('href');
    if (href === path || (path === '/' && href === '/')) el.classList.add('active');
  });
}

function renderPage(path) {
  const fn = routes[path] || renderDashboard;
  fn();
}

/* ── Helpers ───────────────────────────────────────────────────────────────── */
function fmt(v, dec) {
  if (v === null || v === undefined || isNaN(v)) return '—';
  return Number(v).toFixed(dec !== undefined ? dec : 1);
}
function fmtA(v) {
  if (v === null || v === undefined) return '—';
  const a = Number(v);
  return (a > 0 ? '+' : '') + a.toFixed(1);
}
function el(tag, cls, html) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (html !== undefined) e.innerHTML = html;
  return e;
}
function pill(text, cls) {
  return `<span class="charging-pill ${cls}">${text}</span>`;
}
function chargePill(current) {
  const a = Number(current);
  if (a > 0.5)  return pill('Charging',    'pill-charging');
  if (a < -0.5) return pill('Discharging', 'pill-discharging');
  return pill('Idle', 'pill-idle');
}

function formatRuntime(min) {
  if (min === undefined || min < 0) return '—';
  const h = Math.floor(min / 60);
  const m = min % 60;
  return h > 0 ? `${h}h ${m}m` : `${m}m`;
}

function formatUptime(s) {
  if (!s) return '—';
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (d > 0) return `${d}d ${h}h ${m}m`;
  if (h > 0) return `${h}h ${m}m`;
  return `${m}m`;
}

function chartEmptyMsg() {
  if (!g_live || !g_live.ntp_synced) return 'Waiting for NTP time sync…';
  if (!g_live.bms_count_online) return 'BMS offline — history will record when packs connect';
  return 'Collecting first sample…';
}

/* ── Live data state ────────────────────────────────────────────────────────── */
let g_live = null;
let g_poll_interval = null;
let g_chart_a = null;
let g_chart_b = null;

/* ── Config (loaded once at boot for alarm thresholds) ──────────────────────── */
async function fetchConfigOnce() {
  try {
    const r = await apiFetch('/api/config');
    if (r && r.ok) g_config = await r.json();
  } catch (e) { /* thresholds unavailable until config loads */ }
}

/* ── Alarm threshold helpers ────────────────────────────────────────────────── */
function alarmVolt(v) {
  if (!g_config || v === null) return false;
  const s = g_config.safe_pack_volt;
  return v > s || v < (s - 12.0);
}
function alarmCellMax(v)  { return g_config && v !== null && v > g_config.safe_cell_volt; }
function alarmCellMin(v)  { return g_config && v !== null && v < (g_config.safe_cell_volt - 1.0); }
function alarmDrift(v)    { return g_config && v !== null && v > g_config.safe_cell_drift; }
function alarmTemp(v) {
  if (!g_config || v === null) return false;
  return v > (g_config.charge_temp_max - 5) || v < (g_config.charge_temp_min + 5);
}
function alarmSoc(v)      { return v !== null && v < 10; }

async function fetchLive() {
  try {
    const r = await apiFetch('/api/live');
    if (!r || !r.ok) return;
    g_live = await r.json();
    updateLiveUI();
  } catch (e) {
    /* network error — keep showing last data */
  }
}

function startPolling(ms) {
  if (g_poll_interval) clearInterval(g_poll_interval);
  fetchLive();  // immediate first fetch
  g_poll_interval = setInterval(fetchLive, ms);
}

/* ── Top-bar status update ─────────────────────────────────────────────────── */
function updateStatusBar() {
  if (!g_live) return;
  const snap   = g_live.snapshot || {};
  const safety = g_live.safety   || {};
  const stats  = g_live.stats    || {};
  const can    = (stats.can)     || {};

  const bmsEl = document.getElementById('status-bms');
  if (bmsEl) {
    const online = g_live.bms_count_online || 0;
    const total  = g_live.bms_count_configured || 0;
    bmsEl.textContent = `BMS ${online}/${total}`;
  }

  const canEl = document.getElementById('status-can');
  if (canEl) {
    const hasAlarm = (safety.alarm_flags || 0) !== 0;
    const txFail   = (can.tx_fail || 0) > 0;
    canEl.textContent = hasAlarm ? 'CAN alarm' : (txFail ? 'CAN fail' : 'CAN ok');
    canEl.className = 'status-pill pill-can' + (hasAlarm || txFail ? ' alarm' : '');
  }
}

let g_health = null;

async function fetchHealth() {
  try {
    const r = await fetch('/api/health', { cache: 'no-store' });
    if (r && r.ok) {
      g_health = await r.json();
      updateMqttIndicator();
    }
  } catch (_) {}
}

function updateMqttIndicator() {
  const mqttEl = document.getElementById('status-mqtt');
  if (!mqttEl) return;
  const h = g_health;
  if (!h || !h.mqtt || !h.mqtt.enabled) {
    mqttEl.style.display = 'none';
    return;
  }
  const state = h.mqtt.state || 'unknown';
  mqttEl.style.display = 'inline-flex';
  const labels = { connected: 'MQTT ok', connecting: 'MQTT…', disconnected: 'MQTT off', failed: 'MQTT err' };
  mqttEl.textContent = labels[state] || 'MQTT';
  mqttEl.className = 'status-pill pill-mqtt' + (state === 'connected' ? '' : ' alarm');
}

function updateLiveUI() {
  updateStatusBar();
  if (window.location.pathname === '/' || window.location.pathname === '/dashboard') {
    updateDashboardCards();
    updatePackCards();
  }
}

/* ── Dashboard ─────────────────────────────────────────────────────────────── */
function renderDashboard() {
  const root = document.getElementById('page-root');
  root.innerHTML = `
    <div class="metrics-grid" id="metrics-grid"></div>
    <div class="charts-row">
      <div class="card chart-card">
        <div class="chart-title" id="chart-a-title">Power / SOC — last 2 h</div>
        <div id="chart-a-plot" class="chart-plot"></div>
      </div>
      <div class="card chart-card">
        <div class="chart-title" id="chart-b-title">Voltage / Temp — last 2 h</div>
        <div id="chart-b-plot" class="chart-plot"></div>
      </div>
    </div>
    <p class="section-header">Battery Packs</p>
    <div class="packs-grid" id="packs-grid"></div>
  `;
  updateDashboardCards();
  updatePackCards();
  loadCharts();
}

function updateDashboardCards() {
  const grid = document.getElementById('metrics-grid');
  if (!grid) return;

  const snap   = (g_live && g_live.snapshot) || {};
  const safety = (g_live && g_live.safety)   || {};
  const packs  = snap.packs || [];

  // Aggregate from safety (already computed by firmware).
  const soc    = safety.soc_avg         !== undefined ? safety.soc_avg         : null;
  const soh    = safety.soh_avg         !== undefined ? safety.soh_avg         : null;
  const cvl    = safety.cvl_volts       !== undefined ? safety.cvl_volts       : null;
  const ccl    = safety.ccl_amps        !== undefined ? safety.ccl_amps        : null;
  const dcl    = safety.dcl_amps        !== undefined ? safety.dcl_amps        : null;
  const cur    = safety.pack_current_total !== undefined ? safety.pack_current_total : null;
  const volt   = safety.pack_voltage_avg !== undefined ? safety.pack_voltage_avg : null;
  const temp   = safety.temp_avg        !== undefined ? safety.temp_avg        : null;

  // Per-cell min/max/drift: find across all online packs.
  let cellMin = null, cellMax = null, cellDrift = null;
  packs.forEach(p => {
    if (!p.online) return;
    if (cellMin === null || p.cell_min_v < cellMin) cellMin = p.cell_min_v;
    if (cellMax === null || p.cell_max_v > cellMax) cellMax = p.cell_max_v;
    const d = p.cell_drift_v;
    if (cellDrift === null || d > cellDrift) cellDrift = d;
  });

  const power = (cur !== null && volt !== null) ? cur * volt : null;
  const alarmFlags = safety.alarm_flags || 0;
  const sysMsg = safety.sys_message || 'OK';

  const energy  = (g_live && g_live.energy)           || {};
  const rtMin   = g_live && g_live.runtime_est_min;
  const rtState = (g_live && g_live.runtime_est_state) || 'idle';
  const rtLabels = { until_empty: 'Until empty', until_full: 'Until full', idle: 'Idle' };

  const cards = [
    {
      label: 'State of Charge',
      value: soc !== null ? fmt(soc, 0) : '—',
      unit: '%',
      sub: soh !== null ? `SOH ${fmt(soh, 0)}%` : '',
      color: 'var(--purple)',
      alarm: alarmSoc(soc),
    },
    {
      label: 'Power',
      value: power !== null ? fmt(Math.abs(power), 0) : '—',
      unit: 'W',
      sub: chargePill(cur),
      color: cur > 0 ? 'var(--accent)' : (cur < 0 ? 'var(--amber)' : 'var(--fg-muted)'),
      alarm: false,
    },
    {
      label: 'Current (total)',
      value: cur !== null ? fmtA(cur) : '—',
      unit: 'A',
      sub: `CCL ${fmt(ccl,0)} / DCL ${fmt(dcl,0)} A`,
      color: 'var(--fg)',
      alarm: false,
    },
    {
      label: 'Pack Voltage',
      value: volt !== null ? fmt(volt, 2) : '—',
      unit: 'V',
      sub: `CVL ${fmt(cvl, 2)} V`,
      color: 'var(--blue)',
      alarm: alarmVolt(volt),
    },
    {
      label: 'Cell Min',
      value: cellMin !== null ? fmt(cellMin, 3) : '—',
      unit: 'V',
      sub: '',
      color: 'var(--blue)',
      alarm: alarmCellMin(cellMin),
    },
    {
      label: 'Cell Max',
      value: cellMax !== null ? fmt(cellMax, 3) : '—',
      unit: 'V',
      sub: '',
      color: 'var(--fg)',
      alarm: alarmCellMax(cellMax),
    },
    {
      label: 'Cell Drift',
      value: cellDrift !== null ? fmt(cellDrift * 1000, 0) : '—',
      unit: 'mV',
      sub: alarmFlags & 0x20 ? '<span style="color:var(--amber)">⚠ Imbalance</span>' : 'Normal',
      color: cellDrift > 0.05 ? 'var(--amber)' : 'var(--accent)',
      alarm: alarmDrift(cellDrift),
    },
    {
      label: 'Temperature',
      value: temp !== null ? fmt(temp, 1) : '—',
      unit: '°C',
      sub: alarmFlags & 0x08 ? '<span style="color:var(--red)">Temp stop</span>' : 'Normal',
      color: temp > 40 ? 'var(--amber)' : 'var(--fg)',
      alarm: alarmTemp(temp),
    },
    {
      label: 'Energy Today',
      value: energy.today_in_kwh !== undefined ? fmt(energy.today_in_kwh, 2) : '—',
      unit: 'kWh in',
      sub: energy.today_out_kwh !== undefined ? `Out: ${fmt(energy.today_out_kwh, 2)} kWh` : '',
      color: 'var(--brand-teal)',
      alarm: false,
    },
    {
      label: 'Runtime Est.',
      value: rtMin !== undefined && rtMin >= 0 ? formatRuntime(rtMin) : '—',
      unit: '',
      sub: rtLabels[rtState] || 'Idle',
      color: 'var(--fg)',
      alarm: false,
    },
  ];

  grid.innerHTML = '';
  cards.forEach(c => {
    const card = el('div', 'card metric-card' + (c.alarm ? ' alarm' : ''));
    // Omit inline color when in alarm state so the CSS .alarm rule can control the value color.
    const valueStyle = c.alarm ? '' : `style="color:${c.color}"`;
    card.innerHTML = `
      <div class="metric-label">${c.label}</div>
      <div class="metric-value" ${valueStyle}>${c.value}<span class="metric-unit">${c.unit}</span></div>
      <div class="metric-sub">${c.sub}</div>
    `;
    grid.appendChild(card);
  });
}

function updatePackCards() {
  const grid = document.getElementById('packs-grid');
  if (!grid) return;

  const snap = (g_live && g_live.snapshot) || {};
  const packs = snap.packs || [];

  if (packs.length === 0) {
    grid.innerHTML = '<p style="color:var(--fg-muted);font-size:13px">Waiting for BMS data…</p>';
    return;
  }

  // Only recreate cards if pack count changed (avoid unnecessary DOM churn).
  const existing = grid.querySelectorAll('.pack-card');
  if (existing.length !== packs.length) {
    grid.innerHTML = '';
    packs.forEach((_, i) => {
      const card = el('div', 'card pack-card');
      card.id = `pack-card-${i}`;
      grid.appendChild(card);
    });
  }

  packs.forEach((p, i) => {
    const card = document.getElementById(`pack-card-${i}`);
    if (!card) return;
    renderPackCard(card, p);
  });
}

function renderPackCard(card, p) {
  const online  = p.online;
  const cells   = p.cells || [];
  const count   = p.cell_count || cells.length;
  const minIdx  = p.cell_min_idx;
  const maxIdx  = p.cell_max_idx;
  const driftMv = ((p.cell_drift_v || 0) * 1000).toFixed(0);

  // Success rate from snapshot stats (not available here yet — show polls from global stats).
  const rs485 = p.rs485_ok_count !== undefined
    ? `${p.rs485_ok_count}/${p.rs485_total_count}`
    : '—';

  // Build cell bars.
  let barsHtml = '';
  if (online && count > 0) {
    const vMin = Math.min(...cells.filter((_, i) => i < count));
    const vMax = Math.max(...cells.filter((_, i) => i < count));
    const vRange = vMax - vMin || 0.001;
    cells.slice(0, count).forEach((v, i) => {
      const pct = Math.max(5, Math.min(100, ((v - 2.5) / (4.2 - 2.5)) * 100));
      let cls = 'cell-ok';
      if (i === minIdx) cls = 'cell-min';
      else if (i === maxIdx) cls = 'cell-max';
      barsHtml += `<div class="cell-bar ${cls}" style="height:${pct.toFixed(0)}%">
        <div class="cell-tooltip">C${i + 1}: ${v.toFixed(3)}V</div></div>`;
    });
  } else {
    for (let i = 0; i < (count || 15); i++) {
      barsHtml += `<div class="cell-bar cell-offline" style="height:30%"></div>`;
    }
  }

  card.innerHTML = `
    <div class="pack-header">
      <span class="pack-id">BMS ${p.bms_id + 1}</span>
      <div class="pack-status-dot ${online ? 'online' : 'offline'}"></div>
      <span style="font-size:12px;color:var(--fg-muted)">${online ? 'Online' : 'Offline'}</span>
    </div>
    <div class="pack-metrics">
      <div><div class="pack-metric-label">SOC</div><div class="pack-metric-value" style="color:var(--purple)">${p.soc !== undefined ? p.soc + '%' : '—'}</div></div>
      <div><div class="pack-metric-label">Voltage</div><div class="pack-metric-value">${fmt(p.pack_voltage, 2)} V</div></div>
      <div><div class="pack-metric-label">Current</div><div class="pack-metric-value">${fmtA(p.pack_current)} A</div></div>
      <div><div class="pack-metric-label">Power</div><div class="pack-metric-value">${p.pack_voltage && p.pack_current !== undefined ? fmt(p.pack_voltage * p.pack_current, 0) : '—'} W</div></div>
      <div><div class="pack-metric-label">SOH</div><div class="pack-metric-value">${p.soh !== undefined ? p.soh + '%' : '—'}</div></div>
      <div><div class="pack-metric-label">Drift</div><div class="pack-metric-value" style="color:${driftMv > 50 ? 'var(--amber)' : 'var(--fg)'}">${driftMv} mV</div></div>
      <div><div class="pack-metric-label">Temp</div><div class="pack-metric-value">${fmt(p.temp_avg_c, 1)} °C</div></div>
      <div><div class="pack-metric-label">Cells</div><div class="pack-metric-value">${count}S</div></div>
      <div><div class="pack-metric-label">Ah rem</div><div class="pack-metric-value">${fmt(p.rem_ah, 0)}/${fmt(p.full_ah, 0)}</div></div>
    </div>
    <div class="cell-graph">${barsHtml}</div>
  `;
}

/* ── Energy + Runtime cards ─────────────────────────────────────────────────── */
function updateEnergyRuntimeCards() {
  const live   = g_live || {};
  const energy = live.energy || {};

  const inEl = document.getElementById('energy-in');
  if (!inEl) return;  // not on dashboard

  inEl.textContent = energy.today_in_kwh !== undefined ? energy.today_in_kwh.toFixed(2) : '—';
  const outEl = document.getElementById('energy-out');
  if (outEl) outEl.textContent = energy.today_out_kwh !== undefined ? energy.today_out_kwh.toFixed(2) : '—';

  const weekEl = document.getElementById('energy-week');
  if (weekEl) {
    const wIn  = energy.week_in_kwh  !== undefined ? energy.week_in_kwh.toFixed(1)  : '—';
    const wOut = energy.week_out_kwh !== undefined ? energy.week_out_kwh.toFixed(1) : '—';
    weekEl.textContent = `Week: ${wIn} / ${wOut} kWh`;
  }

  const rtMin   = live.runtime_est_min;
  const rtState = live.runtime_est_state || 'idle';
  const rtEl  = document.getElementById('runtime-val');
  const rtSub = document.getElementById('runtime-sub');
  if (rtEl) rtEl.textContent = rtMin !== undefined && rtMin >= 0 ? formatRuntime(rtMin) : '—';
  if (rtSub) {
    const labels = { until_empty: 'Until empty', until_full: 'Until full', idle: 'Idle' };
    rtSub.textContent = labels[rtState] || 'Idle';
  }
}

/* ── History charts (uPlot) ─────────────────────────────────────────────────── */

const SERIES_DEFS = {
  power:   { label: 'Power',   color: '#76D2D9', dec: 0, unit: 'W' },
  soc:     { label: 'SOC',     color: '#9B6FD4', dec: 1, unit: '%', scaleRange: { range: [0, 100] } },
  voltage: { label: 'Voltage', color: '#E25548', dec: 1, unit: 'V' },
  temp:    { label: 'Temp',    color: '#E89C5C', dec: 1, unit: '°C' },
};

function getChartConfig() {
  return {
    a: localStorage.getItem('chart_a') || 'power',
    b: localStorage.getItem('chart_b') || 'voltage',
  };
}

function saveChartConfig() {
  const aEl = document.getElementById('cfg-chart-a');
  const bEl = document.getElementById('cfg-chart-b');
  if (aEl) localStorage.setItem('chart_a', aEl.value);
  if (bEl) localStorage.setItem('chart_b', bEl.value);
  loadCharts();
  const el = document.getElementById('chart-cfg-feedback');
  if (el) { el.textContent = 'Applied.'; el.className = 'feedback-msg ok'; }
}

function buildUplotData(ser) {
  if (!ser || !ser.points || ser.points.length === 0 || ser.t0_epoch <= 0) return null;
  const n = ser.points.length;
  const xs = [];
  for (let i = 0; i < n; i++) xs.push(ser.t0_epoch + i * ser.resolution_s);
  return [xs, ser.points];
}

function makeChartOpts(width, def) {
  const cs = getComputedStyle(document.documentElement);
  const fgMuted   = cs.getPropertyValue('--text-muted').trim()   || '#8A7E69';
  const gridColor = cs.getPropertyValue('--border-subtle').trim() || '#D4CDB9';
  return {
    width,
    height: 180,
    padding: [4, 0, 0, 0],
    series: [
      {},
      {
        label: def.label,
        stroke: def.color,
        width: 2,
        spanGaps: false,
        value: (u, v) => v != null ? v.toFixed(def.dec) + ' ' + def.unit : '—',
      },
    ],
    axes: [
      {
        stroke: fgMuted,
        grid:  { stroke: gridColor, width: 0.5 },
        ticks: { stroke: gridColor, width: 0.5 },
        values: (u, ts) => ts.map(t => {
          const d = new Date(t * 1000);
          return String(d.getHours()).padStart(2, '0') + ':' + String(d.getMinutes()).padStart(2, '0');
        }),
      },
      {
        scale: 'y',
        stroke: def.color,
        grid:  { stroke: gridColor, width: 0.5 },
        ticks: { stroke: gridColor, width: 0.5 },
        size: 52,
        gap: 4,
      },
    ],
    scales: {
      x: { time: true },
      y: def.scaleRange || { auto: true },
    },
    cursor: { drag: { x: false, y: false } },
    legend: { show: true },
  };
}

async function loadCharts() {
  if (typeof uPlot === 'undefined') return;
  if (!document.getElementById('chart-a-plot')) return;

  if (g_chart_a) { try { g_chart_a.destroy(); } catch (_) {} g_chart_a = null; }
  if (g_chart_b) { try { g_chart_b.destroy(); } catch (_) {} g_chart_b = null; }

  const cfg    = getChartConfig();
  const needed = [...new Set([cfg.a, cfg.b])];

  try {
    const fetched = {};
    // Sequential fetches — avoid concurrent heap pressure on the ESP32.
    for (const s of needed) {
      const r = await apiFetch(`/api/history?series=${s}&tier=fine`);
      if (!r || !r.ok) continue;
      const d = await r.json();
      fetched[s] = (d && d.series && d.series[0]) || null;
    }

    const renderChart = (plotId, titleId, key) => {
      const el = document.getElementById(plotId);
      if (!el) return null;
      const def  = SERIES_DEFS[key];
      const data = buildUplotData(fetched[key] || null);
      const titleEl = document.getElementById(titleId);
      if (titleEl) titleEl.textContent = `${def.label} — last 2 h`;
      if (data && el.offsetWidth > 0) {
        return new uPlot(makeChartOpts(el.offsetWidth, def), data, el);
      }
      if (!data) el.innerHTML = `<p class="chart-empty">${chartEmptyMsg()}</p>`;
      return null;
    };

    g_chart_a = renderChart('chart-a-plot', 'chart-a-title', cfg.a);
    g_chart_b = renderChart('chart-b-plot', 'chart-b-title', cfg.b);
  } catch (_) { /* charts unavailable — silently ignore */ }
}

/* ── Battery page (placeholder) ─────────────────────────────────────────────── */
function renderBattery() {
  document.getElementById('page-root').innerHTML = `
    <div class="placeholder-page">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" style="width:48px;height:48px;color:var(--fg-muted)">
        <rect x="2" y="7" width="18" height="10" rx="2"/><path d="M20 11h2v2h-2"/>
      </svg>
      <h2>Battery Pack Detail</h2>
      <p>Per-pack deep detail (cell temperatures, alarm breakdown, RS485 stats) coming in Phase H.</p>
    </div>
  `;
}

/* ── Settings page ─────────────────────────────────────────────────────────── */
let g_config = null;

const EYE_SVG = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>`;

function pwField(id, label, autocomplete, placeholder) {
  return `
    <div class="form-group">
      <label>${label}</label>
      <div class="pw-wrap">
        <input type="password" id="${id}" autocomplete="${autocomplete}" placeholder="${placeholder}">
        <button type="button" class="pw-toggle" onclick="togglePw('${id}')" title="Show/hide">${EYE_SVG}</button>
      </div>
    </div>`;
}

async function renderSettings() {
  const root = document.getElementById('page-root');
  root.innerHTML = '<div class="placeholder-page"><div class="spinner"></div></div>';

  try {
    const r = await apiFetch('/api/config');
    if (!r) return;
    g_config = await r.json();
  } catch (e) {
    root.innerHTML = '<p style="color:var(--red);padding:20px">Failed to load config.</p>';
    return;
  }

  const c = g_config;
  const nowTs     = (g_live && g_live.now_ts_s) || (g_health && g_health.now_ts_s) || 0;
  const ntpSynced = !!(g_live && g_live.ntp_synced) || !!(g_health && g_health.ntp_synced);
  const deviceTimeStr = nowTs > 1000000
    ? new Date(nowTs * 1000).toLocaleString()
    : 'Unknown';
  root.innerHTML = `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">Battery</div>
        <div class="form-row">
          <div class="form-group">
            <label>BMS Pack Count</label>
            <input type="number" id="cfg-bms_count" value="${c.bms_count}" min="1" max="16">
            <div class="help">Number of TopBand packs (1–16)</div>
          </div>
          <div class="form-group">
            <label>Force Cell Count</label>
            <input type="number" id="cfg-force_cell_count" value="${c.force_cell_count}" min="0" max="16">
            <div class="help">0 = auto-detect from BMS</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Charge Amps per Pack</label>
            <input type="number" id="cfg-charge_amps_per_pack" value="${c.charge_amps_per_pack}" step="0.1">
            <div class="help">Max charge current per pack (A)</div>
          </div>
          <div class="form-group">
            <label>Discharge Amps per Pack</label>
            <input type="number" id="cfg-discharge_amps_per_pack" value="${c.discharge_amps_per_pack}" step="0.1">
            <div class="help">Max discharge current per pack (A)</div>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Voltage Limits</div>
        <div class="form-row">
          <div class="form-group">
            <label>CVL (Charge Voltage Limit)</label>
            <input type="number" id="cfg-cvl_voltage" value="${c.cvl_voltage}" step="0.01">
            <div class="help">Max pack charge voltage (V)</div>
          </div>
          <div class="form-group">
            <label>Safe Pack Voltage</label>
            <input type="number" id="cfg-safe_pack_volt" value="${c.safe_pack_volt}" step="0.01">
            <div class="help">Under-voltage alarm threshold (V)</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Safe Cell Voltage</label>
            <input type="number" id="cfg-safe_cell_volt" value="${c.safe_cell_volt}" step="0.001">
            <div class="help">Cell OVP threshold (V)</div>
          </div>
          <div class="form-group">
            <label>Max Cell Drift</label>
            <input type="number" id="cfg-safe_cell_drift" value="${c.safe_cell_drift}" step="0.001">
            <div class="help">Imbalance warning level (V)</div>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Temperature Limits</div>
        <div class="form-row">
          <div class="form-group">
            <label>Charge Temp Min (°C)</label>
            <input type="number" id="cfg-charge_temp_min" value="${c.charge_temp_min}" step="0.5">
          </div>
          <div class="form-group">
            <label>Charge Temp Max (°C)</label>
            <input type="number" id="cfg-charge_temp_max" value="${c.charge_temp_max}" step="0.5">
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Discharge Temp Min (°C)</label>
            <input type="number" id="cfg-discharge_temp_min" value="${c.discharge_temp_min}" step="0.5">
          </div>
          <div class="form-group">
            <label>Discharge Temp Max (°C)</label>
            <input type="number" id="cfg-discharge_temp_max" value="${c.discharge_temp_max}" step="0.5">
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Soft Zone Width (°C)</label>
            <input type="number" id="cfg-temp_soft_zone" value="${c.temp_soft_zone}" step="0.5">
            <div class="help">Hysteresis band for throttle steps</div>
          </div>
          <div class="form-group">
            <label>Temp Mode</label>
            <select id="cfg-temp_mode">
              <option value="0" ${c.temp_mode === 0 ? 'selected' : ''}>Hottest cell</option>
              <option value="1" ${c.temp_mode === 1 ? 'selected' : ''}>Average</option>
            </select>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Spike Filters</div>
        <div class="form-row">
          <div class="form-group">
            <label>Spike Voltage Max (V)</label>
            <input type="number" id="cfg-spike_volt_max" value="${c.spike_volt_max}" step="0.01">
            <div class="help">Reject readings above this pack voltage</div>
          </div>
          <div class="form-group">
            <label>Spike Current Max (A)</label>
            <input type="number" id="cfg-spike_curr_max" value="${c.spike_curr_max}" step="0.5">
            <div class="help">Reject current readings above this</div>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Network</div>
        <div class="form-group">
          <label>WiFi SSID</label>
          <input type="text" id="cfg-wifi_ssid" value="${c.wifi_ssid || ''}">
          <div class="help">Only the SSID is shown; password is stored separately</div>
        </div>
        <div class="form-group">
          <label>NTP Server</label>
          <input type="text" id="cfg-ntp_server" value="${c.ntp_server || ''}">
        </div>
        <div class="form-group">
          <label>Timezone Offset (hours)</label>
          <input type="number" id="cfg-timezone_offset_h" value="${c.timezone_offset_h}" min="-12" max="14">
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Time</div>
        <div class="form-row">
          <div class="form-group">
            <label>Device Time</label>
            <div style="font-size:15px;font-weight:600;color:var(--text-primary);padding:6px 0">${deviceTimeStr}</div>
            <div class="help">Snapshot from last live data poll</div>
          </div>
          <div class="form-group">
            <label>NTP Status</label>
            <div style="padding:6px 0">
              <span class="charging-pill ${ntpSynced ? 'pill-charging' : 'pill-idle'}">${ntpSynced ? 'Synced' : 'Not synced'}</span>
            </div>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">CAN / Inverter</div>
        <div class="form-row">
          <div class="form-group">
            <label>Protocol</label>
            <select id="cfg-can_protocol">
              <option value="0" ${c.can_protocol === 0 ? 'selected' : ''}>Victron</option>
              <option value="1" ${c.can_protocol === 1 ? 'selected' : ''}>Pylontech</option>
              <option value="2" ${c.can_protocol === 2 ? 'selected' : ''}>SMA</option>
            </select>
          </div>
          <div class="form-group" style="display:flex;align-items:center;gap:8px;padding-top:20px">
            <input type="checkbox" id="cfg-can_enabled" ${c.can_enabled ? 'checked' : ''} style="width:auto">
            <label for="cfg-can_enabled" style="margin:0">CAN TX enabled</label>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">MQTT</div>
        <div class="form-group" style="display:flex;align-items:center;gap:8px">
          <input type="checkbox" id="cfg-mqtt_enabled" ${c.mqtt_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-mqtt_enabled" style="margin:0">MQTT enabled</label>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Broker Host / IP</label>
            <input type="text" id="cfg-mqtt_host" value="${c.mqtt_host || ''}" placeholder="192.168.1.x">
          </div>
          <div class="form-group">
            <label>Broker Port</label>
            <input type="number" id="cfg-mqtt_port" value="${c.mqtt_port || 1883}" min="1" max="65535">
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Username</label>
            <input type="text" id="cfg-mqtt_user" value="${c.mqtt_user || ''}" autocomplete="off">
          </div>
          <div class="form-group">
            <label>Password</label>
            <input type="password" id="cfg-mqtt_pass" value="" autocomplete="new-password" placeholder="Leave blank to keep current">
            <div class="help">Leave blank to keep existing password</div>
          </div>
        </div>
        <div class="form-group">
          <label>Base Topic</label>
          <input type="text" id="cfg-mqtt_base_topic" value="${c.mqtt_base_topic || 'topband-bms'}" placeholder="topband-bms">
          <div class="help">Gateway appends -XXXX (last 4 MAC hex) automatically</div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Publish Level</label>
            <select id="cfg-mqtt_level">
              <option value="0" ${c.mqtt_level === 0 ? 'selected' : ''}>Disabled</option>
              <option value="1" ${c.mqtt_level === 1 ? 'selected' : ''}>Status only</option>
              <option value="2" ${c.mqtt_level === 2 ? 'selected' : ''}>Data (system)</option>
              <option value="3" ${c.mqtt_level === 3 ? 'selected' : ''}>Per-cell</option>
            </select>
          </div>
          <div class="form-group" style="display:flex;align-items:center;gap:8px;padding-top:20px">
            <input type="checkbox" id="cfg-mqtt_diag_enabled" ${c.mqtt_diag_enabled ? 'checked' : ''} style="width:auto">
            <label for="cfg-mqtt_diag_enabled" style="margin:0">Publish diagnostics (every 30 s)</label>
          </div>
        </div>
        <div class="form-group" style="display:flex;align-items:center;gap:8px">
          <input type="checkbox" id="cfg-ha_discovery_enabled" ${c.ha_discovery_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-ha_discovery_enabled" style="margin:0">Home Assistant auto-discovery</label>
        </div>
        <div class="btn-row" style="margin-top:12px">
          <button class="btn btn-secondary" onclick="sendHaDiscovery()">Re-send HA Discovery</button>
        </div>
      </div>

      <div id="feedback-msg" class="feedback-msg"></div>

      <div class="btn-row">
        <button class="btn btn-primary" onclick="saveConfig()">Save Settings</button>
        <button class="btn btn-secondary" onclick="downloadBackup()">Download Backup</button>
        <button class="btn btn-secondary" onclick="confirmRestart()">Restart Gateway</button>
      </div>

      <div class="settings-section" style="margin-top:32px">
        <div class="settings-section-title">Charts</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Choose which series to display on each dashboard chart card.
          Each card has a left axis and a right axis — pick any combination.
        </p>
        ${(cc => `
        <div class="form-row">
          <div class="form-group">
            <label>Chart A</label>
            <select id="cfg-chart-a">
              ${Object.entries(SERIES_DEFS).map(([k,d]) =>
                `<option value="${k}" ${cc.a === k ? 'selected' : ''}>${d.label} (${d.unit})</option>`
              ).join('')}
            </select>
          </div>
          <div class="form-group">
            <label>Chart B</label>
            <select id="cfg-chart-b">
              ${Object.entries(SERIES_DEFS).map(([k,d]) =>
                `<option value="${k}" ${cc.b === k ? 'selected' : ''}>${d.label} (${d.unit})</option>`
              ).join('')}
            </select>
          </div>
        </div>
        `)(getChartConfig())}
        <div class="btn-row" style="margin-top:8px">
          <button class="btn btn-primary" onclick="saveChartConfig()">Apply</button>
        </div>
        <div id="chart-cfg-feedback" class="feedback-msg"></div>
      </div>

      <div class="settings-section" style="margin-top:32px">
        <div class="settings-section-title">Account</div>
        ${c.auth_enabled
          ? `<p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">Change admin password</p>
             ${pwField('pw-current', 'Current password', 'current-password', 'Current password')}`
          : `<p style="font-size:13px;color:var(--color-warning);margin-bottom:14px">
               No password set — set one below to enable authentication.
             </p>`}
        ${pwField('pw-new',     'New password',     'new-password',     'New password')}
        ${pwField('pw-confirm', 'Confirm new password', 'new-password', 'Repeat new password')}
        <div id="pw-feedback" class="feedback-msg"></div>
        <button class="btn btn-primary" style="margin-top:8px" onclick="changePassword()">Save password</button>
      </div>

      <div class="settings-section" style="margin-top:32px">
        <div class="settings-section-title">System</div>
        <div class="form-row">
          <div class="form-group">
            <label>Firmware</label>
            <div class="settings-info-val">${g_health ? g_health.version : '—'}</div>
            <div class="help">${g_health ? g_health.build : ''}</div>
          </div>
          <div class="form-group">
            <label>UI</label>
            <div class="settings-info-val">h2-mvp-2</div>
            <div class="help">Served from LittleFS</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Uptime</label>
            <div class="settings-info-val">${g_health ? formatUptime(g_health.uptime_s) : '—'}</div>
          </div>
          <div class="form-group">
            <label>Free heap</label>
            <div class="settings-info-val">${g_health && g_health.free_heap_b
              ? (g_health.free_heap_b / 1024).toFixed(0) + ' KB'
              : '—'}</div>
          </div>
        </div>
      </div>

      <div class="settings-section settings-section-danger" style="margin-top:24px">
        <div class="settings-section-title settings-section-title-danger">Reset</div>
        <p style="font-size:13px;font-weight:600;color:var(--text-primary);margin-bottom:4px">Factory reset</p>
        <p style="font-size:12px;color:var(--text-muted);margin-bottom:14px">
          Wipes WiFi credentials and admin password. The gateway will reboot into captive portal mode and need to be set up again from scratch.
        </p>
        <div id="reset-feedback" class="feedback-msg"></div>
        <button class="btn btn-danger" onclick="confirmFactoryReset()">Factory Reset</button>
      </div>
    </div>
  `;
}

function readFormConfig() {
  const c = Object.assign({}, g_config);
  function num(id)  { const v = document.getElementById(id); return v ? Number(v.value) : undefined; }
  function str(id)  { const v = document.getElementById(id); return v ? v.value : undefined; }
  function bool(id) { const v = document.getElementById(id); return v ? v.checked : undefined; }

  c.bms_count             = num('cfg-bms_count');
  c.force_cell_count      = num('cfg-force_cell_count');
  c.charge_amps_per_pack  = num('cfg-charge_amps_per_pack');
  c.discharge_amps_per_pack = num('cfg-discharge_amps_per_pack');
  c.cvl_voltage           = num('cfg-cvl_voltage');
  c.safe_pack_volt        = num('cfg-safe_pack_volt');
  c.safe_cell_volt        = num('cfg-safe_cell_volt');
  c.safe_cell_drift       = num('cfg-safe_cell_drift');
  c.charge_temp_min       = num('cfg-charge_temp_min');
  c.charge_temp_max       = num('cfg-charge_temp_max');
  c.discharge_temp_min    = num('cfg-discharge_temp_min');
  c.discharge_temp_max    = num('cfg-discharge_temp_max');
  c.temp_soft_zone        = num('cfg-temp_soft_zone');
  c.temp_mode             = num('cfg-temp_mode');
  c.spike_volt_max        = num('cfg-spike_volt_max');
  c.spike_curr_max        = num('cfg-spike_curr_max');
  c.wifi_ssid             = str('cfg-wifi_ssid');
  c.ntp_server            = str('cfg-ntp_server');
  c.timezone_offset_h     = num('cfg-timezone_offset_h');
  c.can_protocol          = num('cfg-can_protocol');
  c.can_enabled           = bool('cfg-can_enabled');
  c.mqtt_enabled          = bool('cfg-mqtt_enabled');
  c.mqtt_host             = str('cfg-mqtt_host');
  c.mqtt_port             = num('cfg-mqtt_port');
  c.mqtt_user             = str('cfg-mqtt_user');
  c.mqtt_base_topic       = str('cfg-mqtt_base_topic');
  c.mqtt_level            = num('cfg-mqtt_level');
  c.mqtt_diag_enabled     = bool('cfg-mqtt_diag_enabled');
  c.ha_discovery_enabled  = bool('cfg-ha_discovery_enabled');
  // auth_hash: never send back (shown as "" from server).
  c.auth_hash = '';
  // mqtt_pass_obf: only send if user typed a new password; empty = keep existing.
  const mqttPass = str('cfg-mqtt_pass');
  c.mqtt_pass_obf = (mqttPass && mqttPass.length > 0) ? mqttPass : '';
  return c;
}

async function saveConfig() {
  const msg = document.getElementById('feedback-msg');
  msg.className = 'feedback-msg';
  msg.textContent = '';

  const cfg = readFormConfig();
  try {
    const r = await apiFetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg),
    });
    if (!r) return;
    const data = await r.json();
    if (r.ok) {
      g_config = data;
      msg.className = 'feedback-msg ok';
      msg.textContent = 'Settings saved.';
    } else {
      msg.className = 'feedback-msg err';
      msg.textContent = data.error || 'Save failed.';
    }
  } catch (e) {
    msg.className = 'feedback-msg err';
    msg.textContent = 'Network error: ' + e.message;
  }
  msg.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
}

function downloadBackup() {
  window.location.href = '/api/backup';
}

async function confirmRestart() {
  if (!confirm('Restart the gateway now?')) return;
  try {
    await apiFetch('/api/restart', { method: 'POST' });
  } catch (e) { /* ignore — device is rebooting */ }
  showPageOverlay('Restarting gateway…',
    'This page will reconnect automatically when the gateway comes back online.');
  // Wait 3 s (device restarts in 2 s) then poll until it responds with low uptime.
  setTimeout(pollUntilOnline, 3000);
}

function showPageOverlay(title, body) {
  let overlay = document.getElementById('full-overlay');
  if (!overlay) {
    overlay = document.createElement('div');
    overlay.id = 'full-overlay';
    overlay.className = 'full-overlay';
    document.body.appendChild(overlay);
  }
  overlay.innerHTML = `
    <div class="spinner" style="width:36px;height:36px;border-width:3px"></div>
    <h2>${title}</h2>
    <p>${body}</p>
  `;
  overlay.style.display = 'flex';
}

async function pollUntilOnline() {
  let attempts = 0;
  const poll = async () => {
    try {
      const r = await fetch('/api/health', { cache: 'no-store' });
      if (r.ok) {
        const data = await r.json();
        if (data.uptime_s < 30) {
          window.location.href = '/login.html';
          return;
        }
      }
    } catch (_) { /* not up yet */ }
    attempts++;
    if (attempts < 90) setTimeout(poll, 2000);  // max 3 min
  };
  await poll();
}

/* ── Alerts page ─────────────────────────────────────────────────────────────── */

const ALERTS_ACK_KEY = 'tbms_alerts_last_ack';
let g_alerts_filter  = 'ALL';
let g_alerts_skip    = 0;
let g_alerts_data    = [];
let g_alerts_total   = 0;
let g_diag_timer     = null;

const SEV_NAMES = ['INFO', 'WARN', 'ERROR', 'CRITICAL'];
const SEV_CLASSES = ['sev-info', 'sev-warn', 'sev-error', 'sev-critical'];
const SEV_MIN = { 'ALL': 0, 'INFO': 0, 'WARN': 1, 'ERROR': 2, 'CRITICAL': 3 };

function fmtRelTime(epochS) {
  if (!epochS) return '—';
  const diff = Math.floor(Date.now() / 1000) - epochS;
  if (diff < 60)   return diff + 's ago';
  if (diff < 3600) return Math.floor(diff / 60) + 'm ago';
  if (diff < 86400) return Math.floor(diff / 3600) + 'h ago';
  return Math.floor(diff / 86400) + 'd ago';
}

function fmtAbsTime(epochS) {
  if (!epochS) return '—';
  return new Date(epochS * 1000).toLocaleString();
}

async function fetchAlerts(reset) {
  if (reset) { g_alerts_skip = 0; g_alerts_data = []; }
  const minSev = SEV_MIN[g_alerts_filter] || 0;
  const url = `/api/alerts?limit=50&skip=${g_alerts_skip}&min_severity=${minSev}`;
  try {
    const r = await apiFetch(url);
    if (!r || !r.ok) return;
    const d = await r.json();
    g_alerts_total = d.total || 0;
    if (reset) g_alerts_data = d.alerts || [];
    else       g_alerts_data = g_alerts_data.concat(d.alerts || []);
    g_alerts_skip += (d.alerts || []).length;
    renderAlertsList();
  } catch (_) {}
}

function renderAlertRow(a) {
  const sevIdx = Math.min(a.severity_n || 0, 3);
  const sevCls = SEV_CLASSES[sevIdx];
  const rel    = fmtRelTime(a.ts_epoch);
  const abs    = fmtAbsTime(a.ts_epoch);
  return `
    <div class="alert-row" onclick="showAlertDetail(${JSON.stringify(JSON.stringify(a))})" title="${abs}">
      <span class="sev-dot ${sevCls}"></span>
      <div>
        <div style="font-size:13px;color:var(--text-primary)">${escHtml(a.message)}</div>
        <div class="alert-meta">
          <span>${rel}</span>
          <span class="source-pill">${escHtml(a.source)}</span>
        </div>
      </div>
      <span style="font-size:10px;font-weight:700;color:var(--text-muted)">${a.severity}</span>
    </div>`;
}

function escHtml(s) {
  return String(s || '').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

function renderAlertsList() {
  const list = document.getElementById('alerts-list');
  if (!list) return;
  if (g_alerts_data.length === 0) {
    list.innerHTML = '<div style="padding:24px;text-align:center;color:var(--text-muted)">No alerts match the current filter.</div>';
  } else {
    list.innerHTML = g_alerts_data.map(renderAlertRow).join('');
  }
  const more = document.getElementById('alerts-load-more');
  if (more) more.style.display = (g_alerts_skip < g_alerts_total) ? 'block' : 'none';
  const cnt = document.getElementById('alerts-count');
  if (cnt) cnt.textContent = `${g_alerts_total} total`;
}

function showAlertDetail(jsonStr) {
  const a = JSON.parse(jsonStr);
  const overlay = document.getElementById('modal-overlay');
  document.getElementById('modal-title').textContent = `${a.severity} — ${a.source}`;
  document.getElementById('modal-body').innerHTML =
    `<table style="width:100%;font-size:13px;border-collapse:collapse">
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0">Severity</td><td><strong>${escHtml(a.severity)}</strong></td></tr>
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0">Source</td><td><span class="source-pill">${escHtml(a.source)}</span></td></tr>
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0">Time</td><td>${fmtAbsTime(a.ts_epoch)}</td></tr>
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0">Uptime</td><td>${formatUptime(a.uptime_s)}</td></tr>
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0;vertical-align:top">Message</td><td style="word-break:break-word">${escHtml(a.message)}</td></tr>
    </table>`;
  document.getElementById('modal-confirm').style.display = 'none';
  overlay.style.display = 'flex';
}

async function clearAllAlerts() {
  const overlay = document.getElementById('modal-overlay');
  document.getElementById('modal-title').textContent = 'Clear All Alerts';
  document.getElementById('modal-body').textContent = 'Delete all stored alerts? This cannot be undone.';
  document.getElementById('modal-confirm').style.display = '';
  document.getElementById('modal-confirm').textContent = 'Clear All';
  overlay.style.display = 'flex';
  document.getElementById('modal-confirm').onclick = async () => {
    overlay.style.display = 'none';
    try {
      await apiFetch('/api/alerts', { method: 'DELETE' });
      fetchAlerts(true);
      updateAlertBadge();
    } catch (_) {}
  };
}

async function updateAlertBadge() {
  try {
    const r = await apiFetch('/api/alerts?limit=200&skip=0&min_severity=2');
    if (!r || !r.ok) return;
    const d = await r.json();
    const lastAck = parseInt(localStorage.getItem(ALERTS_ACK_KEY) || '0', 10);
    const unread = (d.alerts || []).filter(a => (a.ts_epoch || 0) > lastAck);
    const badge = document.getElementById('alert-badge');
    if (!badge) return;
    if (unread.length > 0) {
      badge.textContent = unread.length > 9 ? '9+' : String(unread.length);
      badge.style.display = 'flex';
    } else {
      badge.style.display = 'none';
    }
  } catch (_) {}
}

async function renderAlerts() {
  // Acknowledge: set last-ack timestamp to now.
  localStorage.setItem(ALERTS_ACK_KEY, String(Math.floor(Date.now() / 1000)));
  updateAlertBadge();

  const root = document.getElementById('page-root');
  root.innerHTML = `
    <div style="padding:24px">
      <div class="alerts-header">
        <h2 style="margin:0;font-size:20px">Alerts</h2>
        <div class="alerts-filter-pills" id="filter-pills">
          ${['ALL','CRITICAL','ERROR','WARN','INFO'].map(f =>
            `<button class="filter-pill${f===g_alerts_filter?' active':''}" onclick="setAlertFilter('${f}')">${f}</button>`
          ).join('')}
        </div>
        <span id="alerts-count" style="font-size:12px;color:var(--text-muted);margin-left:auto"></span>
        <button class="btn btn-danger" style="font-size:12px;padding:4px 12px" onclick="clearAllAlerts()">Clear all</button>
      </div>
      <div id="alerts-list" style="border:1px solid var(--border-subtle);border-radius:var(--radius)">
        <div style="padding:24px;text-align:center;color:var(--text-muted)">Loading…</div>
      </div>
      <div id="alerts-load-more" style="display:none;text-align:center;margin-top:12px">
        <button class="btn btn-secondary" onclick="loadMoreAlerts()">Load more</button>
      </div>
    </div>`;
  fetchAlerts(true);
}

function setAlertFilter(f) {
  g_alerts_filter = f;
  document.querySelectorAll('.filter-pill').forEach(p => {
    p.classList.toggle('active', p.textContent === f);
  });
  fetchAlerts(true);
}

function loadMoreAlerts() {
  fetchAlerts(false);
}

/* ── Diag page ──────────────────────────────────────────────────────────────── */

function kvRow(k, v) {
  return `<div class="diag-kv"><span>${escHtml(String(k))}</span><span>${escHtml(String(v))}</span></div>`;
}

function renderDiagData(d) {
  const root = document.getElementById('diag-root');
  if (!root) return;

  const sys = d.system || {};
  const pol = d.poller || {};
  const can = d.can    || {};
  const sn  = d.snapshot_bus || {};
  const mq  = d.mqtt   || {};
  const ntp = d.ntp    || {};
  const lfs = d.littlefs || {};
  const ene = d.energy || {};
  const his = d.history || {};

  const tasks = (d.tasks || []).slice().sort((a, b) => (b.stack_hwm||0) - (a.stack_hwm||0));

  root.innerHTML = `
    <div class="diag-section">
      <h3>System</h3>
      <div class="diag-kv-grid">
        ${kvRow('Firmware', sys.fw || '—')}
        ${kvRow('Uptime', formatUptime(sys.uptime_s))}
        ${kvRow('Reset reason', sys.reset_reason || '—')}
        ${kvRow('Free heap', (sys.free_heap||0).toLocaleString() + ' B')}
        ${kvRow('Min heap', (sys.min_heap||0).toLocaleString() + ' B')}
        ${kvRow('Free PSRAM', (sys.free_psram||0).toLocaleString() + ' B')}
        ${kvRow('Build', sys.build || '—')}
      </div>
    </div>

    <div class="diag-section">
      <h3>Tasks (sorted by stack HWM)</h3>
      <table class="diag-tasks-table">
        <thead><tr><th>Name</th><th>Stack HWM</th><th>Core</th><th>Priority</th></tr></thead>
        <tbody>
          ${tasks.map(t => `<tr>
            <td>${escHtml(t.name||'?')}</td>
            <td>${(t.stack_hwm||0).toLocaleString()} B</td>
            <td>${t.core >= 0 ? t.core : 'any'}</td>
            <td>${t.prio}</td>
          </tr>`).join('')}
        </tbody>
      </table>
    </div>

    <div class="diag-section">
      <h3>Poller</h3>
      <div class="diag-kv-grid">
        ${kvRow('Cycles', pol.cycles_completed||0)}
        ${kvRow('Cycle avg', (pol.cycle_avg_ms||0) + ' ms')}
        ${kvRow('Cycle max', (pol.cycle_max_ms||0) + ' ms')}
        ${kvRow('RS485 polls', pol.rs485_polls||0)}
        ${kvRow('RS485 ok', pol.rs485_ok||0)}
        ${kvRow('RS485 timeouts', pol.rs485_timeouts||0)}
        ${kvRow('RS485 parse err', pol.rs485_parse_err||0)}
        ${kvRow('Alarm polls ok', pol.alarm_polls_ok||0)}
        ${kvRow('Alarm polls err', pol.alarm_polls_err||0)}
      </div>
    </div>

    <div class="diag-section">
      <h3>CAN</h3>
      <div class="diag-kv-grid">
        ${kvRow('TX ok', can.tx_ok||0)}
        ${kvRow('TX fail', can.tx_fail||0)}
        ${kvRow('TX fail streak max', can.tx_fail_streak_max||0)}
        ${kvRow('Heartbeats', can.heartbeats||0)}
        ${kvRow('Express sends', can.express_sends||0)}
        ${kvRow('Bus-off count', can.bus_off_count||0)}
        ${kvRow('Driver restarts', can.driver_restarts||0)}
      </div>
    </div>

    <div class="diag-section">
      <h3>Snapshot Bus / MQTT / NTP</h3>
      <div class="diag-kv-grid">
        ${kvRow('SB publishes', sn.publishes||0)}
        ${kvRow('SB reads', sn.reads||0)}
        ${kvRow('SB retries', sn.retries||0)}
        ${kvRow('MQTT state', mq.state||'—')}
        ${kvRow('MQTT publish ok', mq.publish_ok||0)}
        ${kvRow('MQTT publish fail', mq.publish_fail||0)}
        ${kvRow('MQTT drops', mq.publish_drops||0)}
        ${kvRow('NTP synced', ntp.synced ? 'yes' : 'no')}
        ${kvRow('NTP server', ntp.server||'—')}
      </div>
    </div>

    <div class="diag-section">
      <h3>LittleFS / History / Energy</h3>
      <div class="diag-kv-grid">
        ${kvRow('LFS total', (lfs.total_b||0).toLocaleString() + ' B')}
        ${kvRow('LFS used', (lfs.used_b||0).toLocaleString() + ' B')}
        ${kvRow('LFS free', (lfs.free_b||0).toLocaleString() + ' B')}
        ${kvRow('Fine samples', his.fine_samples||0)}
        ${kvRow('Coarse samples', his.coarse_samples||0)}
        ${kvRow('Today in', (ene.today_in_kwh||0).toFixed(2) + ' kWh')}
        ${kvRow('Today out', (ene.today_out_kwh||0).toFixed(2) + ' kWh')}
        ${kvRow('Stored alerts', d.alerts_count||0)}
      </div>
    </div>

    <div class="diag-section">
      <h3>Log (last ${(d.log_ring||[]).length} lines)</h3>
      <div class="diag-log-box" id="diag-log">
        ${(d.log_ring||[]).map(l => escHtml(l)).join('\n')}
      </div>
    </div>`;

  // Scroll log to bottom.
  const logBox = document.getElementById('diag-log');
  if (logBox) logBox.scrollTop = logBox.scrollHeight;
}

async function renderDiag() {
  // Stop any existing refresh timer when leaving this page.
  if (g_diag_timer) { clearInterval(g_diag_timer); g_diag_timer = null; }

  const root = document.getElementById('page-root');
  root.innerHTML = `
    <div style="padding:24px">
      <div style="display:flex;align-items:center;gap:12px;margin-bottom:16px">
        <h2 style="margin:0;font-size:20px">Diagnostics</h2>
        <span style="font-size:11px;color:var(--text-muted)">Refreshes every 5 s</span>
      </div>
      <div id="diag-root"><div style="text-align:center;padding:40px;color:var(--text-muted)">Loading…</div></div>
    </div>`;

  const doFetch = async () => {
    // Stop if user navigated away.
    if (!document.getElementById('diag-root')) {
      if (g_diag_timer) clearInterval(g_diag_timer);
      g_diag_timer = null;
      return;
    }
    try {
      const r = await apiFetch('/api/diag');
      if (!r || !r.ok) return;
      const d = await r.json();
      renderDiagData(d);
    } catch (_) {}
  };

  await doFetch();
  g_diag_timer = setInterval(doFetch, 5000);
}

/* ── Router ─────────────────────────────────────────────────────────────────── */
window.addEventListener('popstate', () => renderPage(location.pathname));

/* Intercept sidebar link clicks for SPA navigation. */
document.addEventListener('click', e => {
  const a = e.target.closest('a.sidebar-item');
  if (!a) return;
  e.preventDefault();
  navigate(a.getAttribute('href'));
});

/* ── Logout ─────────────────────────────────────────────────────────────────── */
async function doLogout() {
  try {
    await apiFetch('/api/auth/logout', {method: 'POST'});
  } catch (_) {}
  sessionStorage.removeItem('csrf');
  window.location.href = '/login.html';
}

/* ── Password show/hide toggle ──────────────────────────────────────────────── */
function togglePw(id) {
  const inp = document.getElementById(id);
  if (inp) inp.type = inp.type === 'password' ? 'text' : 'password';
}

/* ── Change password ────────────────────────────────────────────────────────── */
async function changePassword() {
  const authEnabled = g_config && g_config.auth_enabled;
  const curEl = document.getElementById('pw-current');
  const cur   = curEl ? curEl.value : '';
  const nw    = document.getElementById('pw-new').value;
  const cfm   = document.getElementById('pw-confirm').value;
  const msg   = document.getElementById('pw-feedback');
  msg.className = 'feedback-msg';
  msg.textContent = '';

  if ((authEnabled && !cur) || !nw || !cfm) {
    msg.className = 'feedback-msg err';
    msg.textContent = authEnabled
      ? 'All three fields are required.'
      : 'New password and confirmation are required.';
    return;
  }
  if (nw !== cfm) {
    msg.className = 'feedback-msg err';
    msg.textContent = 'New password and confirmation do not match.';
    document.getElementById('pw-new').value = '';
    document.getElementById('pw-confirm').value = '';
    return;
  }

  try {
    const r = await apiFetch('/api/auth/set_password', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({current: cur, new: nw}),
    });
    if (!r) return;  // 401 → apiFetch redirected to login

    if (r.ok) {
      const data = await r.json().catch(() => ({}));
      if (data.csrf) sessionStorage.setItem('csrf', data.csrf);
      if (curEl) curEl.value = '';
      document.getElementById('pw-new').value = '';
      document.getElementById('pw-confirm').value = '';
      msg.className = 'feedback-msg ok';
      msg.textContent = 'Password updated.';
    } else if (r.status === 403) {
      msg.className = 'feedback-msg err';
      msg.textContent = 'Current password is incorrect.';
      if (curEl) curEl.value = '';
    } else {
      const data = await r.json().catch(() => ({}));
      msg.className = 'feedback-msg err';
      msg.textContent = data.error || 'Failed to update password.';
    }
  } catch (e) {
    msg.className = 'feedback-msg err';
    msg.textContent = 'Network error: ' + e.message;
  }
}

/* ── HA Discovery ───────────────────────────────────────────────────────────── */
async function sendHaDiscovery() {
  const msg = document.getElementById('feedback-msg');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = 'Sending HA discovery…'; }
  try {
    const r = await apiFetch('/api/svc/ha/discovery/send', { method: 'POST' });
    if (!r) return;
    if (r.ok) {
      if (msg) { msg.className = 'feedback-msg ok'; msg.textContent = 'HA discovery sent.'; }
    } else {
      const data = await r.json().catch(() => ({}));
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = data.error || 'Failed to send HA discovery.'; }
    }
  } catch (e) {
    if (msg) { msg.className = 'feedback-msg err'; msg.textContent = 'Network error: ' + e.message; }
  }
}

/* ── Factory reset ──────────────────────────────────────────────────────────── */
function confirmFactoryReset() {
  const overlay = document.getElementById('modal-overlay');
  const confirmBtn = document.getElementById('modal-confirm');
  document.getElementById('modal-title').textContent = 'Factory Reset';
  document.getElementById('modal-body').textContent =
    'This will wipe WiFi credentials and admin password. The gateway will reboot into setup mode. Continue?';
  overlay.style.display = 'flex';
  // Replace onclick each time to avoid double-binding.
  confirmBtn.onclick = () => {
    overlay.style.display = 'none';
    doFactoryReset();
  };
}

async function doFactoryReset() {
  const msg = document.getElementById('reset-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }

  try {
    const r = await apiFetch('/api/factory_reset', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({confirm: true}),
    });
    if (!r) return;
    if (r.ok) {
      showPageOverlay('Resetting device…',
        "Wipes complete. You'll need to reconnect to the setup network (TopBand-Setup-XXXX) in about 10 seconds.");
    } else {
      const data = await r.json().catch(() => ({}));
      if (msg) {
        msg.className = 'feedback-msg err';
        msg.textContent = data.error || 'Factory reset failed.';
      }
    }
  } catch (e) {
    if (msg) {
      msg.className = 'feedback-msg err';
      msg.textContent = 'Network error: ' + e.message;
    }
  }
}

/* ── Auth-disabled banner ────────────────────────────────────────────────────── */
async function checkAuthState() {
  try {
    const r = await apiFetch('/api/config');
    if (!r || !r.ok) return;
    const cfg = await r.json();
    const banner  = document.getElementById('auth-banner');
    const logoutBtn = document.getElementById('logout-btn');
    if (banner)    banner.style.display    = cfg.auth_enabled === false ? 'flex' : 'none';
    if (logoutBtn) logoutBtn.style.display = cfg.auth_enabled !== false ? 'inline-flex' : 'none';
  } catch (_) {}
}

/* ── Boot ───────────────────────────────────────────────────────────────────── */
(function init() {
  // Inject modal for factory-reset confirmation.
  const modalEl = document.createElement('div');
  modalEl.id = 'modal-overlay';
  modalEl.className = 'modal-overlay';
  modalEl.style.display = 'none';
  modalEl.innerHTML = `
    <div class="modal-box">
      <div class="modal-title" id="modal-title"></div>
      <div class="modal-body" id="modal-body"></div>
      <div class="modal-footer">
        <button class="btn btn-secondary" id="modal-cancel">Cancel</button>
        <button class="btn btn-danger" id="modal-confirm">Reset Device</button>
      </div>
    </div>`;
  document.body.appendChild(modalEl);
  document.getElementById('modal-cancel').addEventListener('click', () => {
    modalEl.style.display = 'none';
  });

  // Apply stored theme.
  applyTheme(localStorage.getItem(THEME_KEY) || 'dark');
  document.getElementById('theme-btn').addEventListener('click', cycleTheme);

  // Logout button.
  const logoutBtn = document.getElementById('logout-btn');
  if (logoutBtn) logoutBtn.addEventListener('click', doLogout);

  // Clock.
  updateClock();
  setInterval(updateClock, 10000);

  // Load config once for alarm threshold checks on dashboard cards.
  fetchConfigOnce();
  // Check auth state for banner and logout-button visibility.
  checkAuthState();

  // Render current page.
  renderPage(location.pathname);

  // Start live polling at 2 s.
  startPolling(2000);

  // Poll /api/health every 10 s for MQTT indicator.
  fetchHealth();
  setInterval(fetchHealth, 10000);

  // Refresh history charts every 60 s (only when dashboard is active).
  setInterval(() => {
    if (window.location.pathname === '/' || window.location.pathname === '/dashboard') {
      loadCharts();
    }
  }, 60000);

  // Poll alert badge every 60 s.
  updateAlertBadge();
  setInterval(updateAlertBadge, 60000);
})();
