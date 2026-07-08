/* TopBand BMS Gateway — Phase H3b dashboard JS */
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
  '/':          renderDashboard,
  '/dashboard': renderDashboard,
  '/battery':   renderBattery,
  '/solar':     renderSolar,
  '/shunt':     () => navigate('/battery'),
  '/general':   renderSettings,   // backward-compat alias
  '/settings':  renderSettings,
  // Network moved into Settings; rewrite old /network links/bookmarks to the
  // Network section and render it in one pass (renderSettings reads location.hash).
  '/network':   () => { history.replaceState({}, '', '/settings#network'); renderSettings(); },
  '/alerts':    renderAlerts,
  '/diag':      renderDiag,
};

// Timer for per-pack detail page auto-refresh.
let g_battery_detail_timer = null;

function stopBatteryDetailPoll() {
  if (g_battery_detail_timer) {
    clearInterval(g_battery_detail_timer);
    g_battery_detail_timer = null;
  }
}

function navigate(path) {
  // Stop diag auto-refresh when leaving the diag page.
  if (path !== '/diag' && g_diag_timer) {
    clearInterval(g_diag_timer);
    g_diag_timer = null;
  }
  // Stop network status polling when leaving Settings (the Network section lives
  // there now). Switching between Settings sections is handled in renderSettingsSection.
  if (path !== '/settings' && path !== '/general' && g_network_timer) {
    clearInterval(g_network_timer);
    g_network_timer = null;
  }
  // Stop battery detail polling when navigating away.
  if (!path.startsWith('/battery/')) stopBatteryDetailPoll();
  // Cancel drift refresh timer when leaving the battery overview page.
  if (path !== '/battery' && g_drift_timer) {
    clearInterval(g_drift_timer);
    g_drift_timer = null;
  }
  // Destroy solar chart and cancel its refresh timer when leaving the solar page.
  if (path !== '/solar') {
    if (g_solar_chart) { try { g_solar_chart.destroy(); } catch (_) {} g_solar_chart = null; }
    if (g_solar_chart_timer) { clearInterval(g_solar_chart_timer); g_solar_chart_timer = null; }
    g_solar_page_state = null;
  }
  // Drop dashboard chart references when leaving the dashboard (their DOM is about to be replaced).
  if (path !== '/' && path !== '/dashboard') {
    if (g_chart_a) { try { g_chart_a.destroy(); } catch (_) {} g_chart_a = null; g_chart_a_key = null; }
    if (g_chart_b) { try { g_chart_b.destroy(); } catch (_) {} g_chart_b = null; g_chart_b_key = null; }
  }
  history.pushState({}, '', path);
  renderPage(path);
  updateSidebarActive(path);
}

function updateSidebarActive(path) {
  document.querySelectorAll('.sidebar-item').forEach(el => {
    el.classList.remove('active');
    const href = el.getAttribute('href');
    if (!href) return;
    if (href === path) { el.classList.add('active'); return; }
    if (path === '/' && href === '/') { el.classList.add('active'); return; }
    if (path.startsWith('/battery') && href === '/battery') { el.classList.add('active'); return; }
    if (path === '/solar'  && href === '/solar')  { el.classList.add('active'); return; }
    if ((path === '/settings' || path === '/general') && href === '/settings') { el.classList.add('active'); return; }
  });
}

function renderPage(path) {
  // Handle /battery/:n detail routes.
  const detailMatch = path.match(/^\/battery\/(\d+)$/);
  if (detailMatch) {
    renderBatteryDetail(parseInt(detailMatch[1], 10));
    return;
  }
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
  if (a >= 0.5)  return pill('Charging',    'pill-charging');
  if (a <= -0.5) return pill('Discharging', 'pill-discharging');
  return pill('Idle', 'pill-idle');
}

const CS_LABELS = {0:'Off',1:'Low power',2:'Fault',3:'Bulk',4:'Absorption',5:'Float',6:'Storage',7:'Equalize',252:'ESS',255:'Unavailable'};
function csLabel(cs)     { return CS_LABELS[cs] !== undefined ? CS_LABELS[cs] : `State ${cs}`; }
function csPillClass(cs) {
  return (cs === 3 || cs === 4) ? 'pill-charging'
       : cs === 2 ? 'pill-discharging' : 'pill-idle';
}
function csPill(cs, extraStyle) {
  const st = extraStyle ? ` style="${extraStyle}"` : '';
  return `<span class="charging-pill ${csPillClass(cs)}"${st}>${csLabel(cs)}</span>`;
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
let g_chart_a_key = null;   // series key last used to build g_chart_a
let g_chart_b_key = null;   // series key last used to build g_chart_b
let g_solar_chart = null;
let g_solar_chart_timer = null;     // 5-min refresh timer for solar day chart
let g_solar_now_ts    = 0;          // "now" marker timestamp — read by uPlot draw hook
let g_solar_peak_w    = 0;          // peak PV watt value — read by uPlot draw hook
let g_solar_peak_ts   = 0;          // timestamp of peak — read by uPlot draw hook
let g_solar_page_state = null;      // 'disabled' | 'enabled' — what's currently rendered
let g_solar_pt_configured = false;  // ptConfigured state of the currently-rendered page
let g_drift_data   = null;          // latest /api/drift response
let g_drift_timer  = null;          // 30-s refresh timer for drift section
const g_drift_open = new Set();     // pack ids (numeric) currently expanded

/* ── Config (loaded once at boot for alarm thresholds) ──────────────────────── */
async function fetchConfigOnce() {
  try {
    const r = await apiFetch('/api/config');
    if (r && r.ok) {
      g_config = await r.json();
      updateSolarNavVisibility();
      // Solar page may have rendered before config loaded (g_config was null → early return).
      // Re-render now so loadSolarChart() fires on entry, not on the next poll tick.
      if (window.location.pathname === '/solar' && g_solar_page_state !== 'enabled') {
        renderSolar();
      }
    }
  } catch (e) { /* thresholds unavailable until config loads */ }
}

function updateSolarNavVisibility() {
  const navSolar = document.getElementById('nav-solar');
  if (!navSolar) return;
  navSolar.style.display = (g_config && g_config.ble_mppt_enabled) ? '' : 'none';
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

// Human reason for a direction-specific protection lockout. Mirrors
// src/bms/poller.cpp lockout_reason() and runSafety's block_charge/block_discharge.
// dir: 0x01 = charge disabled, 0x02 = discharge disabled.
function lockoutReason(dir, flags, tempAlarm) {
  flags = flags || 0; tempAlarm = tempAlarm || 0;
  if (flags & 0x80) return 'no packs online';
  if (dir === 0x01) {
    if (flags & 0x02) return 'cell/pack over-voltage';
    if (flags & 0x08) return (tempAlarm & 0x01) ? 'temperature too low to charge'
                                                : 'temperature out of charge range';
    if (flags & 0x40) return 'BMS critical alarm';
  } else {
    if (flags & 0x10) return 'cell/pack under-voltage';
    if (flags & 0x08) return 'temperature out of discharge range';
    if (flags & 0x40) return 'BMS critical alarm';
  }
  return 'protection active';
}

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
    bmsEl.className = 'status-pill pill-bms' + (total > 0 && online < total ? ' alarm' : '');
  }

  const canEl = document.getElementById('status-can');
  if (canEl) {
    canEl.textContent = 'CAN';
    if (!g_config || !g_config.can_enabled) {
      canEl.className = 'status-pill pill-can off';
    } else {
      const hasAlarm = (safety.alarm_flags || 0) !== 0;
      const txFail   = (can.tx_fail || 0) > 0 || (can.bus_off_count || 0) > 0;
      canEl.className = 'status-pill pill-can' + (hasAlarm || txFail ? ' alarm' : '');
    }
  }

  // MPPT pill — shown only when MPPT is enabled in config.
  const mpptEl = document.getElementById('status-mppt');
  if (mpptEl && g_config) {
    if (!g_config.ble_mppt_enabled) {
      mpptEl.style.display = 'none';
    } else {
      mpptEl.style.display = '';
      const src = (g_live && g_live.sources && g_live.sources.mppt) || {};
      const seen = src.seen;
      const stale = seen && src.ms_since_last_seen > 30000;
      mpptEl.className = 'status-pill pill-mppt' + (seen && !stale ? '' : ' off');
      mpptEl.textContent = 'MPPT';
    }
  }

  // SHUNT pill — shown only when SmartShunt is enabled in config.
  const shuntEl = document.getElementById('status-shunt');
  if (shuntEl && g_config) {
    if (!g_config.ble_shunt_enabled) {
      shuntEl.style.display = 'none';
    } else {
      shuntEl.style.display = '';
      const src = (g_live && g_live.sources && g_live.sources.shunt) || {};
      const seen = src.seen;
      const stale = seen && src.ms_since_last_seen > 30000;
      shuntEl.className = 'status-pill pill-shunt' + (seen && !stale ? '' : ' off');
      shuntEl.textContent = 'SHUNT';
    }
  }
}

let g_health = null;

async function fetchHealth() {
  try {
    const r = await fetch('/api/health', { cache: 'no-store' });
    if (r && r.ok) {
      g_health = await r.json();
      updateWifiIndicator();
      updateMqttIndicator();
      updateAuthBanner();
    }
  } catch (_) {}
}

function updateWifiIndicator() {
  const el = document.getElementById('status-wifi');
  if (!el) return;
  const wifi = g_health && g_health.wifi;
  if (!wifi || !wifi.connected) {
    el.className = 'status-pill pill-wifi off';
    el.textContent = 'WiFi';
    return;
  }
  const rssi = wifi.rssi;
  let mod = '';
  if      (rssi < -75) mod = ' weak';
  else if (rssi < -60) mod = ' fair';
  el.className = 'status-pill pill-wifi' + mod;
  el.textContent = 'WiFi';
}

function updateMqttIndicator() {
  const mqttEl = document.getElementById('status-mqtt');
  if (!mqttEl) return;
  const h = g_health;
  if (!h || !h.mqtt || !h.mqtt.enabled) {
    mqttEl.className = 'status-pill pill-mqtt off';
    mqttEl.textContent = 'MQTT';
    return;
  }
  const state = h.mqtt.state || 'unknown';
  const labels = { connected: 'MQTT', connecting: 'MQTT…', disconnected: 'MQTT off', failed: 'MQTT err' };
  mqttEl.textContent = labels[state] || 'MQTT';
  mqttEl.className = 'status-pill pill-mqtt' + (state === 'connected' ? '' : ' alarm');
}

function updateLiveUI() {
  updateStatusBar();
  updateAuthBanner();
  const p = window.location.pathname;
  if (p === '/' || p === '/dashboard') {
    updateDashboardCards();
    updatePackCards();
  } else if (p === '/battery') {
    updateBatteryOverviewCards();
    updateDriftNow();
  } else if (p === '/solar') {
    updateSolarValues();
  }
}

/* ── Dashboard value-color helpers ─────────────────────────────────────────── */
function socColor(soc) {
  if (soc === null || soc === undefined) return 'var(--text-primary)';
  if (soc < 10)  return 'var(--red)';
  if (soc < 20)  return 'var(--amber)';
  return 'var(--accent)';
}
function driftColor(driftV) {
  if (driftV === null || driftV === undefined) return 'var(--text-primary)';
  const mv = driftV * 1000;
  if (mv >= 175) return 'var(--red)';
  if (mv >= 100) return 'var(--amber)';
  return 'var(--accent)';
}
// Cell bar height for a LiFePO4 cell voltage. The old 2.5-4.2 V mapping was
// a Li-ion range: LiFePO4 cells live between ~3.0 and 3.45 V, so all bars
// rendered in a compressed upper band and looked identical (review U6).
// 2.95-3.65 V spreads the working range across the full bar height.
const CELL_BAR_MIN_V = 2.95;
const CELL_BAR_MAX_V = 3.65;
function cellBarPct(v) {
  return Math.max(5, Math.min(100, ((v - CELL_BAR_MIN_V) / (CELL_BAR_MAX_V - CELL_BAR_MIN_V)) * 100));
}

function cellVColor(v) {
  if (v === null || v === undefined) return 'var(--text-primary)';
  if (v < 2.80 || v > 3.55)                               return 'var(--red)';
  if ((v >= 2.80 && v < 3.00) || (v > 3.45 && v <= 3.55)) return 'var(--amber)';
  return 'var(--accent)';
}

/* ── Dashboard ─────────────────────────────────────────────────────────────── */
function renderDashboard() {
  const root = document.getElementById('page-root');
  root.innerHTML = `
    <div class="metrics-grid" id="metrics-grid"></div>
    <div class="charts-row">
      <div class="card chart-card">
        <div class="chart-title"><span id="chart-a-title">Power / SOC — last 2 h</span><span class="card-src" id="chart-a-src" style="display:none"></span></div>
        <div id="chart-a-plot" class="chart-plot"></div>
      </div>
      <div class="card chart-card">
        <div class="chart-title"><span id="chart-b-title">Voltage / Temp — last 2 h</span><span class="card-src" id="chart-b-src" style="display:none"></span></div>
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
  // soc/volt/cur are the Battery Value Sources fused values (shunt leads per the
  // Auto/Manual policy when fresh, else BMS — see sources.battery_*_src for
  // which one is active per metric). soh/temp are BMS-only; the backend sends
  // JSON null for any of these when the source has no usable data right now, so
  // they come through here as null and render as "—", never a misleading 0.
  const soc    = safety.soc_display     !== undefined ? safety.soc_display     : null;
  const soh    = safety.soh_avg         !== undefined ? safety.soh_avg         : null;
  const cvl    = safety.cvl_volts       !== undefined ? safety.cvl_volts       : null;
  const ccl    = safety.ccl_amps        !== undefined ? safety.ccl_amps        : null;
  const dcl    = safety.dcl_amps        !== undefined ? safety.dcl_amps        : null;
  const cur    = safety.current_display !== undefined ? safety.current_display : null;
  const volt   = safety.voltage_display !== undefined ? safety.voltage_display : null;
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

  // Direction-aware protection lockout (safety.lockout_flags): 0x01 charge,
  // 0x02 discharge. Drives the Runtime Estimate banner + a lighter charge note.
  const lockout    = safety.lockout_flags || 0;
  const tempAlarm  = safety.temp_alarm || 0;
  const dischgOff  = !!(lockout & 0x02);
  const chargeOff  = !!(lockout & 0x01);
  const rtSub = dischgOff
    ? `<span style="color:var(--red)">Discharge disabled: ${lockoutReason(0x02, alarmFlags, tempAlarm)}</span>`
    : (rtLabels[rtState] || 'Idle');

  // Sources section (MPPT / Shunt data from firmware aggregator).
  const sources   = (g_live && g_live.sources) || {};
  const mpptSrc   = sources.mppt || {};
  const shuntSrc  = sources.shunt || {};
  const curSrcId  = sources.battery_current_src || 'bms';
  const curBadge  = `<span class="source-badge badge-${curSrcId}">${curSrcId.toUpperCase()}</span>`;
  const socSrcId  = sources.battery_soc_src || 'bms';
  const voltSrcId = sources.battery_voltage_src || 'bms';
  // Battery Power (Total) is derived from current x voltage; its badge follows
  // Current's source since power tracks current's sign/magnitude most directly.
  const powSrcId  = curSrcId;

  // MPPT tiles — only shown when MPPT enabled and seen.
  const mpptEnabled = mpptSrc.enabled && mpptSrc.seen;
  const pvPowerW    = mpptEnabled && mpptSrc.pv_power_valid  ? mpptSrc.pv_power_w    : null;
  const pvVoltV     = mpptEnabled && mpptSrc.pv_v_valid      ? mpptSrc.pv_voltage_v  : null;
  const pvCurrA     = mpptEnabled && mpptSrc.pv_i_valid      ? mpptSrc.pv_current_a  : null;
  const yieldWh     = mpptEnabled && mpptSrc.yield_valid     ? mpptSrc.yield_today_wh : null;

  // Solar Passthrough — Active indicator on the Solar Power tile (Active-only, 15 s staleness).
  const ptSrcDash = sources.solar_passthrough || {};
  const ptActiveDash = ptSrcDash.received &&
                       (ptSrcDash.ms_since_last || 0) <= 15000 &&
                       ptSrcDash.state;
  const ptPill = ptActiveDash
    ? ' <span class="charging-pill" style="font-size:10px;padding:2px 7px;background:var(--brand-teal);color:var(--brand-aubergine)">Passthrough</span>'
    : '';

  // Layout: 5 columns, 2 rows.
  // Row 1: [Solar/CellMin] [Battery Power (Total)] [SOC] [Cell Drift] [Temperature]
  // Row 2: [Yield/CellMax] [Current (total)]       [Pack Voltage] [Energy Today] [Runtime]
  const col1top = mpptSrc.enabled ? {
    label: 'Solar Power',
    value: pvPowerW !== null ? fmt(pvPowerW, 0) : '—',
    unit: 'W',
    sub: mpptEnabled ? (csPill(mpptSrc.charge_state) + ptPill) : '—',
    color: 'var(--text-primary)',
    alarm: false,
    src: 'mppt',
  } : {
    label: 'Cell Min',
    value: cellMin !== null ? fmt(cellMin, 3) : '—',
    unit: 'V',
    sub: '',
    color: cellVColor(cellMin),
    alarm: alarmCellMin(cellMin),
    src: 'bms',
  };

  const col1bot = mpptSrc.enabled ? {
    label: 'Solar yield today',
    value: yieldWh !== null ? fmt(yieldWh / 1000, 2) : '—',
    unit: 'kWh',
    sub: pvVoltV !== null ? `${fmt(pvVoltV, 1)} V / ${pvCurrA !== null ? fmt(pvCurrA, 1) : '—'} A` : '',
    color: 'var(--text-primary)',
    alarm: false,
    src: 'mppt',
  } : {
    label: 'Cell Max',
    value: cellMax !== null ? fmt(cellMax, 3) : '—',
    unit: 'V',
    sub: '',
    color: cellVColor(cellMax),
    alarm: alarmCellMax(cellMax),
    src: 'bms',
  };

  const cards = [
    // Row 1
    col1top,
    { label: 'Battery Power (Total)', value: power !== null ? fmt(power, 0) : '—', unit: 'W', sub: (cur !== null ? chargePill(cur) : '') + (chargeOff ? ` <span style="color:var(--amber);font-size:10px">Charge disabled: ${lockoutReason(0x01, alarmFlags, tempAlarm)}</span>` : ''), color: 'var(--text-primary)', alarm: false,            src: powSrcId },
    { label: 'State of Charge',       value: soc !== null ? fmt(soc, 0) : '—',     unit: '%',  sub: soh !== null ? `SOH ${fmt(soh, 0)}%` : '', color: socColor(soc),         alarm: alarmSoc(soc),    src: socSrcId },
    { label: 'Cell Drift',            value: cellDrift !== null ? fmt(cellDrift * 1000, 0) : '—', unit: 'mV', sub: alarmFlags & 0x20 ? '<span style="color:var(--amber)">Imbalance</span>' : 'Normal', color: driftColor(cellDrift), alarm: alarmDrift(cellDrift), src: 'bms' },
    { label: 'Temperature',           value: temp !== null ? fmt(temp, 1) : '—',   unit: '°C', sub: alarmFlags & 0x08 ? '<span style="color:var(--red)">Temp stop</span>' : 'Normal', color: 'var(--text-primary)', alarm: alarmTemp(temp), src: 'bms' },
    // Row 2
    col1bot,
    { label: 'Current (total)',  value: cur !== null ? fmtA(cur) : '—',   unit: 'A', sub: `CCL ${fmt(ccl,0)} / DCL ${fmt(dcl,0)} A`, color: 'var(--text-primary)', alarm: false,           src: curSrcId },
    { label: 'Pack Voltage',     value: volt !== null ? fmt(volt, 2) : '—', unit: 'V', sub: `CVL ${fmt(cvl, 2)} V`,                   color: 'var(--text-primary)', alarm: alarmVolt(volt), src: voltSrcId },
    // Energy integration (poller.cpp) already accumulates from the same fused
    // current/voltage as the Combined Current tile, so its badge follows
    // curSrcId too rather than being hardwired to BMS.
    { label: 'Energy Today',     value: energy.today_in_kwh !== undefined ? fmt(energy.today_in_kwh, 2) : '—', unit: 'kWh in', sub: energy.today_out_kwh !== undefined ? `Out: ${fmt(energy.today_out_kwh, 2)} kWh` : '', color: 'var(--text-primary)', alarm: false, src: curSrcId },
    { label: 'Runtime Est.',     value: rtMin !== undefined && rtMin >= 0 ? formatRuntime(rtMin) : '—', unit: '', sub: rtSub, color: 'var(--text-primary)', alarm: dischgOff, src: 'bms' },
  ];

  // Build card structure once (or when count / col-1 label changes — MPPT toggled).
  // Then update values in-place on subsequent polls to avoid metric-tile flicker.
  const existing = grid.querySelectorAll('.metric-card');
  const col1Label = cards[0].label;
  const needsBuild = existing.length !== cards.length
    || (existing[0] && existing[0].querySelector('.metric-label').textContent !== col1Label);

  if (needsBuild) {
    grid.innerHTML = '';
    cards.forEach((c, i) => {
      const card = el('div', 'card metric-card' + (c.alarm ? ' alarm' : ''));
      card.id = `mc-${i}`;
      const valueStyle = c.alarm ? '' : `style="color:${c.color}"`;
      const srcBadge = c.src ? `<span class="card-src card-src-${c.src}" id="mc-${i}-src">${c.src.toUpperCase()}</span>` : '';
      card.innerHTML = `
        <div class="metric-label">${c.label}</div>
        <div class="metric-value" id="mc-${i}-val" ${valueStyle}>${c.value}<span class="metric-unit">${c.unit}</span></div>
        <div class="metric-footer">
          <span class="metric-sub" id="mc-${i}-sub">${c.sub}</span>
          ${srcBadge}
        </div>
      `;
      grid.appendChild(card);
    });
    return;
  }

  // In-place update — only touch text/style, no DOM node creation.
  cards.forEach((c, i) => {
    const card = document.getElementById(`mc-${i}`);
    if (!card) return;
    card.className = 'card metric-card' + (c.alarm ? ' alarm' : '');
    const valEl = document.getElementById(`mc-${i}-val`);
    if (valEl) {
      valEl.style.color = c.alarm ? '' : c.color;
      valEl.innerHTML = c.value + `<span class="metric-unit">${c.unit}</span>`;
    }
    const subEl = document.getElementById(`mc-${i}-sub`);
    if (subEl) subEl.innerHTML = c.sub;
  });
}

/* ── Solar detail page ───────────────────────────────────────────────────────── */
// renderSolar() builds the page structure ONCE with stable element IDs.
// updateSolarValues() handles all 2-second live updates without touching the structure.
function renderSolar() {
  const root = document.getElementById('page-root');

  if (!g_config || !g_config.ble_mppt_enabled) {
    root.innerHTML = `
      <div style="max-width:520px;margin:0 auto">
        <div class="card" style="padding:18px">
          <div style="display:flex;align-items:center;gap:14px">
            <div class="pack-status-dot offline" style="flex-shrink:0"></div>
            <div>
              <div style="font-weight:600;font-size:14px;margin-bottom:4px">Not enabled</div>
              <div style="font-size:12px;color:var(--text-muted)">MPPT BLE source is disabled.
                Enable it in <a href="/settings" style="color:var(--accent)" onclick="navigate('/settings');return false;">General &rarr; BLE</a>.</div>
            </div>
          </div>
        </div>
      </div>`;
    g_solar_page_state = 'disabled';
    return;
  }

  const ptConfigured = !!(g_config && g_config.mqtt_solar_passthrough_topic);
  g_solar_page_state = 'enabled';
  g_solar_pt_configured = ptConfigured;

  root.innerHTML = `
    <div style="max-width:680px;margin:0 auto" id="solar-page-root">

      <!-- Status strip — values updated in-place by updateSolarValues() -->
      <div class="card" style="margin-bottom:12px;padding:13px 18px;display:flex;align-items:center;gap:12px;flex-wrap:wrap">
        <div class="pack-status-dot offline" id="solar-status-dot" style="flex-shrink:0"></div>
        <span style="font-weight:600;font-size:14px" id="solar-status-text">—</span>
        <span id="solar-charge-pill"></span>
        <span style="margin-left:auto;font-size:12px;color:var(--text-muted)" id="solar-last-seen">—</span>
      </div>

      <!-- PV Input + Yield Today -->
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px">
        <div class="card" style="display:flex;flex-direction:column;justify-content:center;padding:16px 20px">
          <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:10px">PV Input</div>
          <div>
            <span id="solar-pv-power" style="font-size:38px;font-weight:700;line-height:1;color:var(--brand-teal)">—</span>
            <span id="solar-pv-unit" style="font-size:15px;font-weight:400;color:var(--text-muted);margin-left:3px;display:none">W</span>
          </div>
        </div>
        <div class="card" style="display:flex;flex-direction:column;justify-content:center;padding:16px 20px">
          <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:10px">Yield today</div>
          <div>
            <span id="solar-yield-kwh" style="font-size:38px;font-weight:700;line-height:1;color:var(--brand-teal)">—</span>
            <span id="solar-yield-unit" style="font-size:15px;font-weight:400;color:var(--text-muted);margin-left:3px;display:none">kWh</span>
          </div>
        </div>
      </div>

      <!-- Solar Today chart (built once, data refreshed every 5 min via setData) -->
      <div class="card" style="padding:14px 16px 10px;margin-bottom:12px">
        <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:10px" id="solar-chart-title">Solar Today</div>
        <div id="solar-chart-plot"></div>
      </div>

      <!-- MPPT Charger Output (always in DOM, shown when batt data is valid) -->
      <div id="solar-output-section" class="card" style="display:none;padding-top:14px;padding-bottom:14px;margin-bottom:12px">
        <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);padding:0 16px 8px">MPPT Charger Output</div>
        <div style="display:flex;align-items:center">
          <div style="flex:1;min-width:0;padding:14px 10px;text-align:center">
            <div style="font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">Power</div>
            <div id="solar-output-power"><span style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</span></div>
          </div>
          <div style="width:1px;background:var(--border);margin:10px 0;align-self:stretch;flex-shrink:0"></div>
          <div style="flex:1;min-width:0;padding:14px 10px;text-align:center">
            <div style="font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">Voltage</div>
            <div id="solar-output-volt"><span style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</span></div>
          </div>
          <div style="width:1px;background:var(--border);margin:10px 0;align-self:stretch;flex-shrink:0"></div>
          <div style="flex:1;min-width:0;padding:14px 10px;text-align:center">
            <div style="font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">Current</div>
            <div id="solar-output-curr"><span style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</span></div>
          </div>
        </div>
      </div>
      <!-- Solar Passthrough -->
      <div style="margin-bottom:8px">
        <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted)">Solar Passthrough</div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:3px;opacity:.75">Only relevant for setups controlled by OpenDTU-onBattery.</div>
      </div>
      ${ptConfigured ? `
      <div class="card" style="padding:16px 18px" id="solar-pt-card">
        <div style="display:flex;align-items:center;gap:12px;margin-bottom:14px" id="solar-pt-header">
          <span id="solar-pt-pill"></span>
          <span id="solar-pt-desc" style="font-size:12px;color:var(--text-muted);line-height:1.45;display:none">Solar energy is fed directly into the system, bypassing the battery.</span>
        </div>
        <div class="net-kv-grid">
          <div class="net-kv-row"><span>MQTT Topic</span><span style="font-family:monospace;font-size:12px">${escHtml(g_config.mqtt_solar_passthrough_topic || '')}</span></div>
          <div class="net-kv-row" id="solar-pt-age-row" style="display:none"><span>Last update</span><span id="solar-pt-age">—</span></div>
        </div>
      </div>` : `
      <div class="card" style="padding:18px">
        <div style="display:flex;align-items:center;gap:14px">
          <div class="pack-status-dot offline" style="flex-shrink:0"></div>
          <div>
            <div style="font-weight:600;font-size:14px;margin-bottom:4px">Not configured</div>
            <div style="font-size:12px;color:var(--text-muted)">No passthrough topic set.
              Configure it in <a href="/settings" style="color:var(--accent)" onclick="navigate('/settings');return false;">General &rarr; MQTT</a>.</div>
          </div>
        </div>
      </div>`}

    </div>
  `;

  updateSolarValues();
  loadSolarChart();
}

/* ── Solar in-place value updater (called on every 2-second live poll) ────────── */
function updateSolarValues() {
  const sources = (g_live && g_live.sources) || {};
  const m  = sources.mppt || {};
  const pt = sources.solar_passthrough || {};

  const mpptEnabled  = !!(g_config && g_config.ble_mppt_enabled);
  const ptConfigured = !!(g_config && g_config.mqtt_solar_passthrough_topic);

  // Rebuild DOM if structural state changed (MPPT toggled, passthrough configured/not, or fresh nav).
  if (!mpptEnabled || g_solar_page_state !== 'enabled' || ptConfigured !== g_solar_pt_configured
      || !document.getElementById('solar-page-root')) {
    renderSolar();
    return;
  }

  // ── Status strip ─────────────────────────────────────────────────────────────
  const cs      = m.charge_state;
  const seen    = !!m.seen;
  const staleMs = m.ms_since_last_seen || 0;
  const stale   = seen && staleMs > 30000;

  const dotEl = document.getElementById('solar-status-dot');
  if (dotEl) dotEl.className = 'pack-status-dot ' + (seen && !stale ? 'online' : 'offline');

  const textEl = document.getElementById('solar-status-text');
  if (textEl) textEl.textContent = seen && !stale ? 'Receiving data' : seen ? 'Stale' : 'No data';

  const pillEl = document.getElementById('solar-charge-pill');
  if (pillEl) pillEl.innerHTML = seen ? csPill(cs, 'margin:0') : '';

  const lastSeenEl = document.getElementById('solar-last-seen');
  if (lastSeenEl) lastSeenEl.textContent = 'Last seen: ' + (seen ? Math.round(staleMs / 1000) + ' s ago' : '—');

  // ── PV Input + Yield ─────────────────────────────────────────────────────────
  const pvPower  = m.pv_power_valid ? fmt(m.pv_power_w, 0) : null;
  const yieldKwh = m.yield_valid    ? fmt((m.yield_today_wh || 0) / 1000, 2) : null;

  const pvPowerEl = document.getElementById('solar-pv-power');
  const pvUnitEl  = document.getElementById('solar-pv-unit');
  if (pvPowerEl) pvPowerEl.textContent = pvPower !== null ? pvPower : '—';
  if (pvUnitEl)  pvUnitEl.style.display = pvPower !== null ? '' : 'none';

  const yieldEl  = document.getElementById('solar-yield-kwh');
  const yieldUEl = document.getElementById('solar-yield-unit');
  if (yieldEl)  yieldEl.textContent = yieldKwh !== null ? yieldKwh : '—';
  if (yieldUEl) yieldUEl.style.display = yieldKwh !== null ? '' : 'none';

  // ── MPPT Charger Output ───────────────────────────────────────────────────────
  const showOutput = !!(m.batt_v_valid || m.batt_i_valid);
  const outSection = document.getElementById('solar-output-section');
  if (outSection) outSection.style.display = showOutput ? '' : 'none';

  if (showOutput) {
    const battV     = m.batt_v_valid ? fmt(m.batt_voltage_v, 2) : null;
    const battA     = m.batt_i_valid ? fmt(m.batt_current_a, 1) : null;
    const battPower = (m.batt_v_valid && m.batt_i_valid) ? fmt(m.batt_voltage_v * m.batt_current_a, 0) : null;

    function setSolarOutCell(id, value, unit, accent) {
      const cel = document.getElementById(id);
      if (!cel) return;
      if (value !== null) {
        cel.innerHTML = `<span style="font-size:28px;font-weight:700;line-height:1;color:${accent || 'var(--text-primary)'}">` +
          value + `</span><span style="font-size:13px;font-weight:400;color:var(--text-muted);margin-left:2px">${unit}</span>`;
      } else {
        cel.innerHTML = '<span style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</span>';
      }
    }
    setSolarOutCell('solar-output-power', battPower, 'W', 'var(--brand-teal)');
    setSolarOutCell('solar-output-volt',  battV,     'V', null);
    setSolarOutCell('solar-output-curr',  battA,     'A', null);
  }

  // ── Passthrough note ─────────────────────────────────────────────────────────
  const PASSTHROUGH_STALE_MS = 15000;
  const ptAgeMs  = pt.ms_since_last || 0;
  const ptStale  = pt.received && ptAgeMs > PASSTHROUGH_STALE_MS;
  const ptActive = pt.received && !ptStale && pt.state;

  // ── Passthrough card values (only when ptConfigured, i.e. element IDs exist) ─
  if (ptConfigured) {
    const ptPillEl = document.getElementById('solar-pt-pill');
    const ptDescEl = document.getElementById('solar-pt-desc');
    const ptAgeEl  = document.getElementById('solar-pt-age');
    const ptAgeRow = document.getElementById('solar-pt-age-row');

    const pillHtml = ptActive
      ? '<span class="charging-pill pill-charging" style="font-size:11px;padding:2px 8px">ACTIVE</span>'
      : (!pt.received || ptStale)
        ? '<span class="charging-pill pill-idle" style="font-size:11px;padding:2px 8px;opacity:.7">UNKNOWN</span>'
        : '<span class="charging-pill pill-idle" style="font-size:11px;padding:2px 8px">INACTIVE</span>';

    if (ptPillEl) ptPillEl.innerHTML = pillHtml;
    if (ptDescEl) ptDescEl.style.display = ptActive ? '' : 'none';
    if (ptAgeEl)  ptAgeEl.textContent = pt.received ? Math.round(ptAgeMs / 1000) + ' s ago' : '—';
    if (ptAgeRow) ptAgeRow.style.display = pt.received ? '' : 'none';
  }
}

async function loadSolarChart() {
  if (typeof uPlot === 'undefined') return;
  const plotEl = document.getElementById('solar-chart-plot');
  if (!plotEl) return;

  if (g_solar_chart) { try { g_solar_chart.destroy(); } catch (_) {} g_solar_chart = null; }
  if (g_solar_chart_timer) { clearInterval(g_solar_chart_timer); g_solar_chart_timer = null; }

  let d = null;
  try {
    const r = await apiFetch('/api/solar-day');
    if (r && r.ok) d = await r.json();
  } catch (_) {}

  const parsed = d ? buildSolarChartData(d) : null;

  if (!parsed) {
    const nowTs = (d && d.now_ts_s) || 0;
    const msg = nowTs === 0 ? 'Waiting for NTP time sync…' : 'No solar data yet — first point in ~5 min';
    plotEl.innerHTML = `<p class="chart-empty">${msg}</p>`;
    g_solar_chart_timer = setInterval(refreshSolarChartData, 5 * 60 * 1000);
    return;
  }

  g_solar_now_ts  = parsed.nowTs;
  g_solar_peak_w  = parsed.peakW;
  g_solar_peak_ts = parsed.peakTs;

  const titleEl = document.getElementById('solar-chart-title');
  if (titleEl) titleEl.textContent = 'SOLAR TODAY';

  // buildOrUpdateChart is the same function dashboard charts use; solar passes a
  // customizeOpts callback for the taller canvas, y-min=0, no legend, and draw hooks.
  const result = buildOrUpdateChart(plotEl, 'solar', parsed.data, null, null, (opts) => {
    opts.height      = 200;
    opts.padding     = [10, 0, 0, 0];
    opts.legend      = { show: false };
    opts.axes[1].size = 56;
    opts.axes[1].gap  = 4;
    opts.scales.x    = { time: true, range: [parsed.midnight, parsed.midnight + 86400] };
    opts.scales.y    = { range: (u, dataMin, dataMax) => [0, (dataMax == null ? 1 : dataMax) * 1.1] };
    opts.hooks = {
      draw: [(u) => {
        const ctx = u.ctx, bbox = u.bbox;
        const nowPx = u.valToPos(g_solar_now_ts, 'x', true);
        if (nowPx >= bbox.left && nowPx <= bbox.left + bbox.width) {
          ctx.save();
          ctx.strokeStyle = 'rgba(255,255,255,0.35)';
          ctx.lineWidth   = 1;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.moveTo(nowPx, bbox.top);
          ctx.lineTo(nowPx, bbox.top + bbox.height);
          ctx.stroke();
          ctx.restore();
        }
        if (g_solar_peak_w > 0) {
          const peakPx = u.valToPos(g_solar_peak_ts, 'x', true);
          const peakPy = u.valToPos(g_solar_peak_w,  'y', true);
          if (peakPx >= bbox.left && peakPx <= bbox.left + bbox.width) {
            ctx.save();
            ctx.font         = '11px sans-serif';
            ctx.fillStyle    = SERIES_DEFS.solar.color;
            ctx.textAlign    = 'center';
            ctx.textBaseline = 'bottom';
            ctx.fillText(`${g_solar_peak_w} W`, peakPx, Math.max(peakPy - 4, bbox.top + 14));
            ctx.restore();
          }
        }
      }],
    };
  });

  g_solar_chart = result.chart;
  g_solar_chart_timer = setInterval(refreshSolarChartData, 5 * 60 * 1000);
}

async function refreshSolarChartData() {
  const plotEl = document.getElementById('solar-chart-plot');
  if (!plotEl) return;
  if (!g_solar_chart) { await loadSolarChart(); return; }

  let d;
  try {
    const r = await apiFetch('/api/solar-day');
    if (!r || !r.ok) return;
    d = await r.json();
  } catch (_) { return; }

  const parsed = buildSolarChartData(d);
  if (!parsed) return;

  g_solar_now_ts  = parsed.nowTs;
  g_solar_peak_w  = parsed.peakW;
  g_solar_peak_ts = parsed.peakTs;

  g_solar_chart.setData(parsed.data);
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

// Shared renderer for the dashboard and battery-overview pack cards
// (review U4: the two copies had already drifted apart in rounding and the
// Ah metric). Variants: detailBtn adds the Details chevron; ahRem appends
// the remaining-capacity metric on the dashboard card.
function renderPackCardInner(card, p, opts) {
  const online  = p.online;
  const cells   = p.cells || [];
  const count   = p.cell_count || cells.length;
  const minIdx  = p.cell_min_idx;
  const maxIdx  = p.cell_max_idx;
  const driftMv = Math.round((p.cell_drift_v || 0) * 1000);
  const power   = (p.pack_voltage && p.pack_current !== undefined)
                  ? fmt(p.pack_voltage * p.pack_current, 0) : '—';

  let barsHtml = '';
  if (online && count > 0) {
    cells.slice(0, count).forEach((v, i) => {
      const pct = cellBarPct(v);
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
      ${opts.detailBtn ? '<span class="pack-detail-btn">Details ›</span>' : ''}
    </div>
    <div class="pack-metrics">
      <div><div class="pack-metric-label">SOC</div><div class="pack-metric-value" style="color:var(--purple)">${p.soc !== undefined ? p.soc + '%' : '—'}</div></div>
      <div><div class="pack-metric-label">Voltage</div><div class="pack-metric-value">${fmt(p.pack_voltage, 2)} V</div></div>
      <div><div class="pack-metric-label">Current</div><div class="pack-metric-value">${fmtA(p.pack_current)} A</div></div>
      <div><div class="pack-metric-label">Power</div><div class="pack-metric-value">${power} W</div></div>
      <div><div class="pack-metric-label">SOH</div><div class="pack-metric-value">${p.soh !== undefined ? p.soh + '%' : '—'}</div></div>
      <div><div class="pack-metric-label">Drift</div><div class="pack-metric-value" style="color:${driftMv > 50 ? 'var(--amber)' : 'var(--fg)'}">${driftMv} mV</div></div>
      <div><div class="pack-metric-label">Temp</div><div class="pack-metric-value">${fmt(p.temp_avg_c, 1)} °C</div></div>
      <div><div class="pack-metric-label">Cells</div><div class="pack-metric-value">${count}S</div></div>
      ${opts.ahRem ? `<div><div class="pack-metric-label">Ah rem</div><div class="pack-metric-value">${fmt(p.rem_ah, 0)}/${fmt(p.full_ah, 0)}</div></div>` : ''}
    </div>
    <div class="cell-graph">${barsHtml}</div>`;
}

function renderPackCard(card, p) {
  renderPackCardInner(card, p, { ahRem: true });
}

/* ── History charts (uPlot) ─────────────────────────────────────────────────── */

// chartSrc: which source badge (if any) to show next to the chart title.
// - drift is always BMS: the shunt is bank-level only and can't inform
//   per-cell drift, so this is a hard fact, not a live-updating label.
// - voltage's history ring (history_task.cpp make_fine_point()) is a raw
//   average of BmsSystemSnapshot.pack[].pack_voltage — it does NOT (yet)
//   follow the Battery Value Sources fusion the live "Pack Voltage"
//   dashboard tile uses, so its badge is a static, honest "BMS" label
//   rather than a live indicator. Making it follow the active source would
//   need history_task.cpp to read SafetyState.voltage_display, which it
//   currently has no path to (deferred; flagged as a follow-up).
// power/soc/temp/solar have no chart-title badge (unset = hidden).
const SERIES_DEFS = {
  power:   { label: 'Power',      color: '#76D2D9', dec: 0, unit: 'W' },
  soc:     { label: 'SOC',        color: '#9B6FD4', dec: 1, unit: '%', scaleRange: { range: [0, 100] } },
  voltage: { label: 'Voltage',    color: '#E25548', dec: 1, unit: 'V', chartSrc: 'bms' },
  temp:    { label: 'Temp',       color: '#E89C5C', dec: 1, unit: '°C' },
  drift:   { label: 'Cell Drift', color: '#5DC264', dec: 0, unit: 'mV', chartSrc: 'bms' },
  solar:   { label: 'PV Power',   color: '#3B82F6', fill: 'rgba(59, 130, 246, 0.18)', dec: 0, unit: 'W' },
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

// Transform /api/solar-day JSON into the [xs, ys] format shared with buildUplotData.
// Returns {data, nowTs, peakW, peakTs, count, res} or null when the ring is empty.
function buildSolarChartData(d) {
  const pts     = d.points || [];
  const count   = pts.length;
  const res     = d.resolution_s || 300;
  const t0      = d.t0_epoch || 0;
  const midnight = d.midnight_epoch || t0;
  const nowTs   = d.now_ts_s || Math.round(Date.now() / 1000);
  if (count === 0 || t0 === 0) return null;
  const xs = [], ys = [];
  if (midnight < t0) { xs.push(midnight); ys.push(null); }
  let peakW = 0, peakTs = 0;
  for (let i = 0; i < count; i++) {
    const ts = t0 + i * res;
    const v  = pts[i];
    xs.push(ts);
    ys.push((v !== null && v !== undefined) ? v : null);
    if (v != null && v > peakW) { peakW = v; peakTs = ts; }
  }
  if (nowTs > xs[xs.length - 1]) { xs.push(nowTs); ys.push(null); }
  return { data: [xs, ys], nowTs, midnight, peakW, peakTs, count, res };
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
        ...(def.fill ? {fill: def.fill} : {}),
        width: 2,
        spanGaps: false,
        points: { show: false },
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

// Shared uPlot chart builder — dashboard charts AND solar chart both call this.
// data: pre-built [xs, ys] (from buildUplotData or buildSolarChartData.data), or null.
// chart/chartKey: existing uPlot instance and the series key it was built with, or null/''.
// customizeOpts: optional function(opts) called before construction for series-specific overrides.
// Returns { chart, key }.
function buildOrUpdateChart(plotEl, key, data, chart, chartKey, customizeOpts) {
  if (data && plotEl.offsetWidth > 0) {
    if (chart && chartKey === key) {
      chart.setData(data);
      return { chart, key };
    }
    if (chart) { try { chart.destroy(); } catch (_) {} }
    const opts = makeChartOpts(plotEl.offsetWidth, SERIES_DEFS[key]);
    if (customizeOpts) customizeOpts(opts);
    return { chart: new uPlot(opts, data, plotEl), key };
  }
  if (!data) plotEl.innerHTML = `<p class="chart-empty">${chartEmptyMsg()}</p>`;
  if (chart) { try { chart.destroy(); } catch (_) {} }
  return { chart: null, key: null };
}

async function loadCharts() {
  if (typeof uPlot === 'undefined') return;
  if (!document.getElementById('chart-a-plot')) return;

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

    const doUpdate = (plotId, titleId, badgeId, key, existing, existingKey) => {
      const plotEl = document.getElementById(plotId);
      if (!plotEl) return { chart: null, key: null };
      const titleEl = document.getElementById(titleId);
      if (titleEl) titleEl.textContent = `${SERIES_DEFS[key].label} — last 2 h`;
      const badgeEl = document.getElementById(badgeId);
      if (badgeEl) {
        const src = SERIES_DEFS[key].chartSrc;
        if (src) {
          badgeEl.className = `card-src card-src-${src}`;
          badgeEl.textContent = src.toUpperCase();
          badgeEl.style.display = '';
        } else {
          badgeEl.style.display = 'none';
        }
      }
      return buildOrUpdateChart(plotEl, key, buildUplotData(fetched[key] || null), existing, existingKey, null);
    };

    const ra = doUpdate('chart-a-plot', 'chart-a-title', 'chart-a-src', cfg.a, g_chart_a, g_chart_a_key);
    g_chart_a = ra.chart; g_chart_a_key = ra.key;

    const rb = doUpdate('chart-b-plot', 'chart-b-title', 'chart-b-src', cfg.b, g_chart_b, g_chart_b_key);
    g_chart_b = rb.chart; g_chart_b_key = rb.key;
  } catch (_) { /* charts unavailable — silently ignore */ }
}

/* ── Battery overview page ──────────────────────────────────────────────────── */

function batteryMBox(label, id, withBadge) {
  const badge = withBadge
    ? `<div style="margin-top:4px"><span class="card-src" id="${id}-src">—</span></div>` : '';
  return `
    <div style="flex:1;min-width:0;padding:14px 10px;text-align:center">
      <div style="font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">${label}</div>
      <div id="${id}" style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</div>
      ${badge}
    </div>`;
}

function batteryVDiv() {
  return `<div style="width:1px;background:var(--border);margin:10px 0;align-self:stretch;flex-shrink:0"></div>`;
}

function renderBattery() {
  stopBatteryDetailPoll();
  const root = document.getElementById('page-root');

  root.innerHTML = `
    <div style="max-width:860px;margin:0 auto">
      <!-- Status strip -->
      <div class="card" style="margin-bottom:12px;padding:13px 18px;display:flex;align-items:center;gap:12px;flex-wrap:wrap">
        <div class="pack-status-dot offline" id="batt-strip-dot" style="flex-shrink:0"></div>
        <span style="font-weight:600;font-size:14px" id="batt-strip-text">—</span>
        <span id="batt-strip-pill"></span>
      </div>
      <!-- Aggregate metrics -->
      <div class="card" style="padding-top:10px;padding-bottom:14px;margin-bottom:16px">
        <div style="padding:0 18px 10px;display:flex;align-items:center;gap:8px;border-bottom:1px solid var(--border);margin-bottom:4px">
          <div class="pack-status-dot offline" id="combined-fresh-dot" style="flex-shrink:0;width:8px;height:8px"></div>
          <span style="font-size:11px;color:var(--text-muted)" id="combined-fresh-text">—</span>
        </div>
        <div style="display:flex;align-items:center">
          <div style="flex:1;min-width:0;padding:14px 10px;text-align:center">
            <div style="font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">Combined SOC</div>
            <div id="bagg-soc" style="font-size:28px;font-weight:700;line-height:1;color:var(--text-muted)">—</div>
            <div style="margin-top:4px"><span class="card-src" id="bagg-soc-src">—</span></div>
          </div>
          ${batteryVDiv()}
          ${batteryMBox('Pack Voltage', 'bagg-volt', true)}
          ${batteryVDiv()}
          ${batteryMBox('Combined Current', 'bagg-curr', true)}
          ${batteryVDiv()}
          ${batteryMBox('Combined Power', 'bagg-pow', true)}
        </div>
        <div style="border-top:1px solid var(--border);padding:10px 18px 0;font-size:12px;color:var(--text-muted)">
          <span id="combined-source-sentence">—</span>
          Configure sources in <a href="/settings" style="color:var(--accent)" onclick="navigate('/settings');showSettingsSection('battery');return false;">Settings &rarr; Battery &rarr; Battery Value Sources</a>.
        </div>
      </div>
      <!-- Pack grid -->
      <div style="font-size:11px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:var(--text-muted);margin-bottom:8px">Battery Packs</div>
      <div class="packs-grid" id="battery-overview-grid">
        <div style="padding:24px;text-align:center;color:var(--text-muted)">Waiting for BMS data…</div>
      </div>
      <!-- Drift Details section -->
      <div style="margin-top:24px">
        <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:4px">
          <div>
            <div style="font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--text-muted);margin-bottom:2px">Drift Details</div>
            <div style="font-size:12px;color:var(--text-muted)">Per-cell balance &amp; drift behaviour &middot; extremes only (SoC &ge;90 % or &le;15 %)</div>
          </div>
          <button class="btn" style="font-size:11px;padding:4px 10px;border:1px solid var(--border);background:var(--bg-card);color:var(--text-secondary);border-radius:6px;cursor:pointer" onclick="driftExpandAll()">Expand all</button>
        </div>
        <div id="drift-section-root">
          <div style="padding:20px;text-align:center;color:var(--text-muted);font-size:13px">Loading drift data…</div>
        </div>
      </div>
    </div>`;
  updateBatteryOverviewCards();
  if (g_drift_timer) { clearInterval(g_drift_timer); g_drift_timer = null; }
  loadDriftDetails();
  g_drift_timer = setInterval(loadDriftDetails, 30000);
}

function setEl(id, text, color) {
  const e = document.getElementById(id);
  if (!e) return;
  e.textContent = text;
  if (color !== undefined) e.style.color = color;
}

function updateBatteryOverviewCards() {
  const grid = document.getElementById('battery-overview-grid');
  if (!grid) return;

  const snap   = (g_live && g_live.snapshot) || {};
  const packs  = snap.packs || [];
  const safety = (g_live && g_live.safety)   || {};
  const sources = (g_live && g_live.sources) || {};

  // ── Status strip ──
  const online = g_live ? (g_live.bms_count_online || 0) : 0;
  const total  = g_live ? (g_live.bms_count_configured || 0) : 0;
  const dot = document.getElementById('batt-strip-dot');
  if (dot) { dot.className = 'pack-status-dot ' + (online > 0 ? 'online' : 'offline'); }
  setEl('batt-strip-text', `${online}/${total} packs online`);
  const pillEl = document.getElementById('batt-strip-pill');
  if (pillEl) pillEl.innerHTML = safety.current_display !== undefined && safety.current_display !== null
    ? chargePill(safety.current_display) : '';
  // ── Aggregate metric boxes ──
  // All three are the Battery Value Sources fused values (see sources.battery_*_src
  // for which source fed each one) — never the raw BMS-only safety.pack_voltage_avg/
  // pack_current_total/soc_avg, which stay reserved for safety/CAN TX.
  const soc  = (safety.soc_display     !== undefined) ? safety.soc_display     : null;
  const volt = (safety.voltage_display !== undefined) ? safety.voltage_display : null;
  const cur  = (safety.current_display !== undefined) ? safety.current_display : null;
  const pow  = (cur !== null && volt !== null) ? cur * volt : null;

  setEl('bagg-soc',  soc  !== null ? fmt(soc,  0)  + ' %' : '—', soc  !== null ? socColor(soc)         : 'var(--text-muted)');
  setEl('bagg-volt', volt !== null ? fmt(volt, 2) + ' V'  : '—', volt !== null ? 'var(--text-primary)'  : 'var(--text-muted)');
  setEl('bagg-curr', cur  !== null ? fmtA(cur)  + ' A'   : '—', cur  !== null ? 'var(--text-primary)'  : 'var(--text-muted)');
  setEl('bagg-pow',  pow  !== null ? fmt(pow,  0)  + ' W' : '—', pow  !== null ? 'var(--text-primary)'  : 'var(--text-muted)');

  // ── Source badges ── each badge MUST come from the same fused result as the
  // number it's labelling (sources.battery_*_src mirrors safety.*_source_shunt),
  // so a badge can never disagree with the value shown next to it. Power has no
  // source of its own (derived V x I) so its badge follows Current's source —
  // same rule already used for the Dashboard's "Battery Power (Total)" card.
  const voltSrcId = sources.battery_voltage_src || 'bms';
  const currSrcId = sources.battery_current_src || 'bms';
  const socSrcId  = sources.battery_soc_src     || 'bms';
  const badgeMap = {
    'bagg-soc-src':  socSrcId,
    'bagg-volt-src': voltSrcId,
    'bagg-curr-src': currSrcId,
    'bagg-pow-src':  currSrcId,
  };
  Object.entries(badgeMap).forEach(([id, srcId]) => {
    const e = document.getElementById(id);
    if (e) { e.textContent = srcId.toUpperCase(); e.className = `card-src card-src-${srcId}`; }
  });

  // ── Freshness indicator ── reflects whichever source is active for Current
  // (the most dynamically live of the three combined metrics, same one the
  // charge/discharge pill above is keyed on) — not hardcoded to the shunt, so
  // it stays meaningful when Battery Value Sources is set to BMS-led.
  {
    const shunt = sources.shunt || {};
    const shuntSeen  = !!shunt.seen;
    const shuntStale = shuntSeen && (shunt.ms_since_last_seen || 0) > 30000;
    const freshDot = document.getElementById('combined-fresh-dot');
    let freshOk, freshText;
    if (currSrcId === 'shunt') {
      freshOk = shuntSeen && !shuntStale;
      freshText = !shuntSeen ? 'SmartShunt: not seen yet'
        : (shuntStale ? `SmartShunt: stale (${Math.round((shunt.ms_since_last_seen || 0) / 1000)}s ago)`
                      : `SmartShunt: receiving data (${Math.round((shunt.ms_since_last_seen || 0) / 1000)}s ago)`);
    } else {
      // BMS freshness: age of the current pack snapshot vs. device uptime
      // (both esp_timer-based clocks, directly comparable).
      const nowDeviceMs = (g_live && g_live.uptime_s !== undefined) ? g_live.uptime_s * 1000 : null;
      const bmsAgeMs = (nowDeviceMs !== null && snap.produced_ms !== undefined)
        ? Math.max(0, nowDeviceMs - snap.produced_ms) : null;
      freshOk = bmsAgeMs !== null && bmsAgeMs < 10000;
      freshText = bmsAgeMs === null ? 'BMS: not yet polled'
        : `BMS: updating (${Math.round(bmsAgeMs / 1000)}s ago)`;
    }
    if (freshDot) freshDot.className = 'pack-status-dot ' + (freshOk ? 'online' : 'offline');
    setEl('combined-fresh-text', freshText);
  }

  // ── Dynamic explanatory sentence ── Auto mode always yields all-shunt or
  // all-bms (uniform policy); Manual mode can mix per metric.
  {
    const allShunt = voltSrcId === 'shunt' && currSrcId === 'shunt' && socSrcId === 'shunt';
    const allBms   = voltSrcId === 'bms'   && currSrcId === 'bms'   && socSrcId === 'bms';
    const sentence = allShunt ? 'Combined values from SmartShunt.'
      : allBms ? 'Combined values from BMS pack average.'
               : 'Combined values sourced per metric — see badges above.';
    setEl('combined-source-sentence', sentence);
  }

  // ── Pack grid ──
  if (packs.length === 0) {
    grid.innerHTML = '<div style="padding:24px;text-align:center;color:var(--text-muted)">Waiting for BMS data…</div>';
    return;
  }

  const existing = grid.querySelectorAll('.battery-overview-card');
  if (existing.length !== packs.length) {
    grid.innerHTML = '';
    packs.forEach((_, i) => {
      const card = el('div', 'card battery-overview-card');
      card.id = `bov-card-${i}`;
      card.style.cursor = 'pointer';
      card.addEventListener('click', () => navigate(`/battery/${i}`));
      grid.appendChild(card);
    });
  }

  packs.forEach((p, i) => {
    const card = document.getElementById(`bov-card-${i}`);
    if (!card) return;
    renderBatteryOverviewCard(card, p);
  });
}

function renderBatteryOverviewCard(card, p) {
  card.className = 'card battery-overview-card' + (p.online ? '' : ' pack-offline');
  renderPackCardInner(card, p, { detailBtn: true });
}

/* ── Battery detail page ────────────────────────────────────────────────────── */
function renderBatteryDetail(packId) {
  stopBatteryDetailPoll();
  const root = document.getElementById('page-root');
  root.innerHTML = `
    <div style="padding:24px" id="bms-detail-root">
      <div style="margin-bottom:16px">
        <a href="/battery" class="back-link" onclick="navigate('/battery');return false;">
          ← Battery overview
        </a>
      </div>
      <div id="bms-detail-content">
        <div style="padding:40px;text-align:center;color:var(--text-muted)">Loading…</div>
      </div>
    </div>`;

  const doFetch = async () => {
    if (!document.getElementById('bms-detail-root')) {
      stopBatteryDetailPoll();
      return;
    }
    try {
      const r = await apiFetch(`/api/bms/${packId}`);
      if (!r) return;
      if (r.status === 404) {
        const c = document.getElementById('bms-detail-content');
        if (c) c.innerHTML = `<p style="color:var(--text-muted)">Pack ${packId + 1} not found or not configured.</p>`;
        return;
      }
      if (!r.ok) return;
      const d = await r.json();
      renderBmsDetailContent(packId, d);
    } catch (_) {}
  };

  doFetch();
  g_battery_detail_timer = setInterval(doFetch, 2000);
}

function renderBmsDetailContent(packId, d) {
  const content = document.getElementById('bms-detail-content');
  if (!content) return;

  const online = d.online;
  const statusCls = online
    ? (d.alarm_bits && d.alarm_bits !== '0x0000000000000000' ? 'alarm' : 'pill-charging')
    : 'pill-discharging';
  const statusLabel = online
    ? (d.alarm_bits && d.alarm_bits !== '0x0000000000000000' ? 'Alarm' : 'Online')
    : 'Offline';

  // Live values table
  const lvRows = [
    ['Pack voltage', fmt(d.pack_v, 2) + ' V'],
    ['Current', fmtA(d.current) + ' A'],
    ['Power', fmt(d.power, 0) + ' W'],
    ['SOC', d.soc !== undefined ? d.soc + '%' : '—'],
    ['SOH', d.soh !== undefined ? d.soh + '%' : '—'],
    ['Cycles', d.cycles !== undefined ? d.cycles : '—'],
    ['Capacity remaining', fmt(d.rem_ah, 1) + ' Ah'],
    ['Capacity full', fmt(d.full_ah, 1) + ' Ah'],
    ['Cell count', d.cell_count !== undefined ? d.cell_count + 'S' : '—'],
    ['Cell min', fmt(d.cell_min_v, 3) + ' V'],
    ['Cell max', fmt(d.cell_max_v, 3) + ' V'],
    ['Cell drift', (d.drift_mv !== undefined ? d.drift_mv + ' mV' : '—')],
  ];

  // Cell voltage bar chart
  const cells = d.cells || [];
  const count = d.cell_count || cells.length;
  const minIdx = d.cell_min_idx;
  const maxIdx = d.cell_max_idx;
  let cellBarsHtml = '';
  if (online && count > 0) {
    cells.slice(0, count).forEach((v, i) => {
      const pct = cellBarPct(v);
      let cls = 'cell-ok';
      if (i === minIdx) cls = 'cell-min';
      else if (i === maxIdx) cls = 'cell-max';
      cellBarsHtml += `
        <div style="display:flex;flex-direction:column;align-items:center;gap:4px;flex:1;min-width:16px">
          <div style="font-size:10px;color:var(--text-muted)">${v.toFixed(3)}</div>
          <div style="flex:1;width:100%;display:flex;flex-direction:column;justify-content:flex-end">
            <div class="cell-bar ${cls}" style="height:${pct.toFixed(0)}%;min-height:4px">
              <div class="cell-tooltip">C${i + 1}: ${v.toFixed(3)}V</div>
            </div>
          </div>
          <div style="font-size:10px;color:var(--text-muted)">${i + 1}</div>
        </div>`;
    });
  }

  // Temps section
  const temps = d.temps || [];
  const tempRows = temps.map(t => {
    const warn = g_config && (t.val > (g_config.charge_temp_max - 5) ||
                               t.val < (g_config.charge_temp_min + 5));
    return `<div style="display:flex;justify-content:space-between;padding:5px 8px;border-bottom:1px solid var(--border-subtle)">
      <span>${escHtml(t.lbl)}</span>
      <span style="font-weight:600;color:${warn ? 'var(--brand-coral)' : 'inherit'}">${fmt(t.val, 1)} °C</span>
    </div>`;
  }).join('');

  // Sysparam section — persists once received; "not yet polled" only before first 0x47.
  const sp = d.sysparam || {};
  let sysparamHtml;
  if (sp.valid) {
    const ageStr = (sp.age_s > 0)
      ? ` <span style="font-size:11px;color:var(--text-muted)">(updated ${sp.age_s}s ago)</span>`
      : '';
    sysparamHtml = `
      <div style="margin-bottom:4px;font-size:12px;color:var(--text-muted)">From pack 0x47${ageStr}</div>
      <div class="diag-kv-grid">
        ${kvRow('Cell OVP', fmt(sp.cell_high_v, 3) + ' V')}
        ${kvRow('Cell UVP', fmt(sp.cell_low_v, 3) + ' V')}
        ${kvRow('Module high', fmt(sp.module_high_v, 2) + ' V')}
        ${kvRow('Module low', fmt(sp.module_low_v, 2) + ' V')}
        ${kvRow('Module under-V', fmt(sp.module_under_v, 2) + ' V')}
        ${kvRow('Charge temp range', fmt(sp.charge_t_min, 1) + ' … ' + fmt(sp.charge_t_max, 1) + ' °C')}
        ${kvRow('Discharge temp range', fmt(sp.discharge_t_min, 1) + ' … ' + fmt(sp.discharge_t_max, 1) + ' °C')}
        ${kvRow('Max charge A', fmt(sp.charge_max_a, 1) + ' A')}
        ${kvRow('Max discharge A', fmt(sp.discharge_max_a, 1) + ' A')}
      </div>`;
  } else {
    sysparamHtml = '<p style="font-size:13px;color:var(--text-muted)">Not yet polled (round-robin, up to 5 min after boot)</p>';
  }

  // RS485 stats section
  const rs = d.rs485 || {};
  const rsHtml = `
    <div class="diag-kv-grid">
      ${kvRow('Polls', rs.polls || 0)}
      ${kvRow('OK', rs.ok || 0)}
      ${kvRow('Timeouts', rs.timeouts || 0)}
      ${kvRow('Errors', rs.errors || 0)}
      ${kvRow('Success', (rs.success_pct || 0) + '%')}
    </div>`;

  content.innerHTML = `
    <div style="display:flex;align-items:center;gap:10px;margin-bottom:20px">
      <h2 style="margin:0;font-size:22px">Pack ${packId + 1}</h2>
      <span class="charging-pill ${statusCls}">${statusLabel}</span>
    </div>

    <div class="settings-section">
      <div class="settings-section-title">Live Values</div>
      <table style="width:100%;border-collapse:collapse;font-size:14px">
        ${lvRows.map(([k, v]) => `
          <tr>
            <td style="padding:5px 8px;color:var(--text-muted);width:55%">${k}</td>
            <td style="padding:5px 8px;font-weight:600">${v}</td>
          </tr>`).join('')}
      </table>
    </div>

    ${count > 0 ? `
    <div class="settings-section">
      <div class="settings-section-title">Cell Voltages</div>
      <div style="display:flex;gap:4px;height:140px;align-items:stretch;margin-top:8px">
        ${cellBarsHtml}
      </div>
    </div>` : ''}

    ${temps.length > 0 ? `
    <div class="settings-section">
      <div class="settings-section-title">Temperatures</div>
      ${tempRows}
    </div>` : ''}

    <div class="settings-section">
      <div class="settings-section-title">Pack Limits (sysparam)</div>
      ${sysparamHtml}
    </div>

    <div class="settings-section">
      <div class="settings-section-title">RS485 Statistics</div>
      ${rsHtml}
    </div>`;
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

/* ── Settings sub-nav definition ────────────────────────────────────────────── */
const SETTINGS_SECTIONS = [
  { id: 'battery',  label: 'Battery',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="7" width="18" height="10" rx="2"/><path d="M20 11h2v2h-2"/></svg>' },
  { id: 'can',      label: 'CAN',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="6" width="18" height="12" rx="2"/><path d="M7 10h.01M7 14h.01M12 10h.01M12 14h.01M17 10h.01M17 14h.01"/></svg>' },
  { id: 'hardware', label: 'Hardware',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><path d="M9 4v16M15 4v16M4 9h16M4 15h16"/></svg>' },
  { id: 'charts',  label: 'Charts',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>' },
  { id: 'time',    label: 'Time',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/></svg>' },
  { id: 'network', label: 'Network',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><line x1="12" y1="20" x2="12" y2="20"/></svg>' },
  { id: 'mqtt',    label: 'MQTT',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="18" cy="5" r="3"/><circle cx="6" cy="12" r="3"/><circle cx="18" cy="19" r="3"/><line x1="8.59" y1="13.51" x2="15.42" y2="17.49"/><line x1="15.41" y1="6.51" x2="8.59" y2="10.49"/></svg>' },
  { id: 'ble',     label: 'BLE',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="6.5 6.5 17.5 17.5 12 23 12 1 17.5 6.5 6.5 17.5"/></svg>' },
  { id: 'notifications', label: 'Notify',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"/><path d="M13.73 21a2 2 0 0 1-3.46 0"/></svg>' },
  { id: 'account', label: 'Account',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"/><circle cx="12" cy="7" r="4"/></svg>' },
  { id: 'system',  label: 'System',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><line x1="8" y1="21" x2="16" y2="21"/><line x1="12" y1="17" x2="12" y2="21"/></svg>' },
  { id: 'reset',   label: 'Reset',
    icon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="1 4 1 10 7 10"/><path d="M3.51 15a9 9 0 1 0 .49-4.95"/></svg>' },
];

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

  const section = (location.hash || '#battery').replace('#', '') || 'battery';
  const activeId = SETTINGS_SECTIONS.some(s => s.id === section) ? section : 'battery';

  root.innerHTML = `
    <div class="settings-layout">
      <nav class="settings-subnav">
        ${SETTINGS_SECTIONS.map(s => `
          <a class="settings-subnav-item${s.id === activeId ? ' active' : ''}"
             href="/settings#${s.id}"
             data-section="${s.id}"
             onclick="showSettingsSection('${s.id}'); return false;">
            ${s.icon}
            <span>${s.label}</span>
          </a>
        `).join('')}
      </nav>
      <div class="settings-content" id="settings-content"></div>
    </div>
  `;
  renderSettingsSection(activeId);
}

function showSettingsSection(id) {
  history.pushState({}, '', '/settings#' + id);
  document.querySelectorAll('.settings-subnav-item').forEach(el => {
    el.classList.toggle('active', el.dataset.section === id);
  });
  renderSettingsSection(id);
}

function renderSettingsSection(id) {
  const content = document.getElementById('settings-content');
  if (!content || !g_config) return;
  // Stop Network status polling left running from a previously-shown Network section.
  if (g_network_timer) { clearInterval(g_network_timer); g_network_timer = null; }
  switch (id) {
    case 'battery':  content.innerHTML = renderSettingsBattery();  break;
    case 'can':      content.innerHTML = renderSettingsCan();      break;
    case 'hardware': content.innerHTML = renderSettingsHardware(); break;
    case 'charts':   content.innerHTML = renderSettingsCharts();   break;
    case 'time':     content.innerHTML = renderSettingsTime();     break;
    case 'network':  content.innerHTML = renderSettingsNetwork();  startNetworkStatusPoll(); break;
    case 'mqtt':          content.innerHTML = renderSettingsMqtt();          break;
    case 'ble':           content.innerHTML = renderSettingsBle();           break;
    case 'notifications': content.innerHTML = renderSettingsNotifications(); loadNotifyData(); break;
    case 'account':       content.innerHTML = renderSettingsAccount();       break;
    case 'system':   content.innerHTML = renderSettingsSystem();   break;
    case 'reset':    content.innerHTML = renderSettingsReset();    break;
    default:         content.innerHTML = renderSettingsBattery();  break;
  }
}

/* ── Settings section renderers ─────────────────────────────────────────────── */

/* ── Hardware / Board section ───────────────────────────────────────────────── */
const WAVESHARE_PINS = { rs485_tx: 17, rs485_rx: 18, rs485_dir: 21, can_tx: 15, can_rx: 16, led: 38 };
// ESP32-S3 reserved GPIO blocklist (conservative — same list as firmware validate()).
// 26-32: SPI0/1 flash; 33-37: Octal PSRAM; 19-20: USB D+/D-; 43-44: UART0; 45-46: strapping.
const S3_RESERVED_GPIOS = new Set([19, 20, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 43, 44, 45, 46]);
const S3_GPIO_MAX = 48;

function renderSettingsHardware() {
  const c = g_config;
  const isManual = (c.board_preset === 1);
  const pins = c.pins || WAVESHARE_PINS;
  const rs485On = (c.rs485_enabled !== false);

  const pinField = (label, id, val, helpText) => `
    <div class="form-group">
      <label>${label}</label>
      <input type="number" id="${id}" value="${val}" min="-1" max="${S3_GPIO_MAX}"
             ${isManual ? '' : 'readonly'}>
      ${helpText ? `<div class="help">${helpText}</div>` : ''}
    </div>`;

  return `
    <div class="settings-page">
      <div class="settings-section" style="border-left:3px solid #f59e0b;background:rgba(245,158,11,.08);padding:12px 16px;border-radius:4px;margin-bottom:16px">
        <strong>ESP32-S3 only.</strong> This firmware requires an ESP32-S3 MCU with
        16 MB flash and 8 MB PSRAM. Classic ESP32 (LX6), ESP32-C3, and boards without
        PSRAM are not supported — they lack the memory capacity required by this firmware.
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Board Type</div>
        <div class="proto-options">
          <label class="proto-option">
            <input type="radio" name="hw_board_radio" value="0"
                   ${!isManual ? 'checked' : ''}
                   onchange="hwBoardTypeChanged(0)">
            <span class="proto-name">Waveshare ESP32-S3-RS485-CAN</span>
            <span class="proto-desc">— Known-good preset (TX=17 RX=18 DIR=21, CAN TX=15 RX=16)</span>
          </label>
          <label class="proto-option">
            <input type="radio" name="hw_board_radio" value="1"
                   ${isManual ? 'checked' : ''}
                   onchange="hwBoardTypeChanged(1)">
            <span class="proto-name">Manual</span>
            <span class="proto-desc">— Set pins yourself (for HATs or boards with external transceiver ICs)</span>
          </label>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">RS485 Interface</div>
        ${isManual ? `
          <div class="form-group" style="display:flex;align-items:center;gap:8px;margin-bottom:12px">
            <input type="checkbox" id="hw-rs485_enabled" ${rs485On ? 'checked' : ''} style="width:auto">
            <label for="hw-rs485_enabled" style="margin:0">RS485 hardware present on this board</label>
          </div>
        ` : ''}
        <div class="form-row">
          ${pinField('TX GPIO', 'hw-rs485_tx', pins.rs485_tx)}
          ${pinField('RX GPIO', 'hw-rs485_rx', pins.rs485_rx)}
          ${pinField('DIR GPIO', 'hw-rs485_dir', pins.rs485_dir, '-1 = hardware auto-direction')}
        </div>
        ${!isManual ? '<div class="help" style="margin-top:4px">Waveshare preset — read-only.</div>' : ''}
      </div>

      <div class="settings-section">
        <div class="settings-section-title">CAN Interface</div>
        <div class="help" style="margin-bottom:8px">
          Enable/disable CAN output via the Inverter Protocol selector in the Battery section.
        </div>
        <div class="form-row">
          ${pinField('TX GPIO', 'hw-can_tx', pins.can_tx)}
          ${pinField('RX GPIO', 'hw-can_rx', pins.can_rx)}
        </div>
        ${!isManual ? '<div class="help" style="margin-top:4px">Waveshare preset — read-only.</div>' : ''}
      </div>

      <div class="settings-section">
        <div class="settings-section-title">LED</div>
        <div class="help" style="margin-bottom:8px">
          NeoPixel GPIO (GPIO 38 on Waveshare). LED driver not yet active in this firmware version
          — pin is reserved and validated to avoid conflicts.
        </div>
        <div class="form-row">
          ${pinField('LED GPIO', 'hw-led', pins.led)}
        </div>
        ${!isManual ? '<div class="help" style="margin-top:4px">Waveshare preset — read-only.</div>' : ''}
      </div>

      <div id="hw-feedback" class="feedback-msg"></div>
      <div class="btn-row">
        <button class="btn btn-primary" onclick="saveHardwareSection()">Save &amp; Reboot</button>
      </div>
      <div class="help" style="margin-top:8px">
        Hardware pin changes require a reboot to take effect. The device will restart
        automatically and reconnect to WiFi.
      </div>
    </div>
  `;
}

function hwBoardTypeChanged(preset) {
  // Re-render so readonly/editable state and checkbox visibility update.
  g_config.board_preset = preset;
  if (preset === 0) {
    // Switching back to Waveshare: restore known-good pins.
    g_config.pins = Object.assign({}, g_config.pins, WAVESHARE_PINS);
    g_config.rs485_enabled = true;
  }
  const content = document.getElementById('settings-content');
  if (content) content.innerHTML = renderSettingsHardware();
}

function validateHwPins(pins, rs485Enabled, fb) {
  const toCheck = [];
  if (rs485Enabled) {
    toCheck.push(['RS485 TX',  pins.rs485_tx]);
    toCheck.push(['RS485 RX',  pins.rs485_rx]);
    if (pins.rs485_dir >= 0) toCheck.push(['RS485 DIR', pins.rs485_dir]);
  }
  toCheck.push(['CAN TX', pins.can_tx]);
  toCheck.push(['CAN RX', pins.can_rx]);
  if (pins.led >= 0) toCheck.push(['LED', pins.led]);

  const errors = [];
  const seen = new Map();
  for (const [name, gpio] of toCheck) {
    if (!Number.isInteger(gpio) || gpio < 0 || gpio > S3_GPIO_MAX) {
      errors.push(`${name}: GPIO ${gpio} out of range (0-${S3_GPIO_MAX})`);
      continue;
    }
    if (S3_RESERVED_GPIOS.has(gpio)) {
      errors.push(`${name}: GPIO ${gpio} is reserved on ESP32-S3 (flash/PSRAM/USB — pick another GPIO)`);
      continue;
    }
    if (seen.has(gpio)) {
      errors.push(`${name} and ${seen.get(gpio)} share the same GPIO ${gpio}`);
    } else {
      seen.set(gpio, name);
    }
  }

  if (errors.length > 0) {
    if (fb) { fb.className = 'feedback-msg err'; fb.textContent = errors.join(' | '); }
    return false;
  }
  return true;
}

async function saveHardwareSection() {
  const fb = document.getElementById('hw-feedback');
  if (fb) { fb.className = 'feedback-msg'; fb.textContent = ''; }

  const presetEl = document.querySelector('input[name="hw_board_radio"]:checked');
  const boardPreset = presetEl ? Number(presetEl.value) : (g_config.board_preset || 0);
  const isManual = (boardPreset === 1);

  const getPin = id => {
    const el = document.getElementById(id);
    return el ? parseInt(el.value, 10) : null;
  };

  const rs485EnabledEl = document.getElementById('hw-rs485_enabled');
  const rs485Enabled = isManual ? (rs485EnabledEl ? rs485EnabledEl.checked : true) : true;

  const pins = {
    rs485_tx:  isManual ? getPin('hw-rs485_tx')  : WAVESHARE_PINS.rs485_tx,
    rs485_rx:  isManual ? getPin('hw-rs485_rx')  : WAVESHARE_PINS.rs485_rx,
    rs485_dir: isManual ? getPin('hw-rs485_dir') : WAVESHARE_PINS.rs485_dir,
    can_tx:    isManual ? getPin('hw-can_tx')    : WAVESHARE_PINS.can_tx,
    can_rx:    isManual ? getPin('hw-can_rx')    : WAVESHARE_PINS.can_rx,
    led:       isManual ? getPin('hw-led')        : WAVESHARE_PINS.led,
  };

  if (isManual && !validateHwPins(pins, rs485Enabled, fb)) return;

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash     = '';
  cfg.mqtt_pass_obf = '';
  cfg.board_preset  = boardPreset;
  cfg.rs485_enabled = rs485Enabled;
  cfg.pins          = pins;

  try {
    const r = await apiFetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg),
    });
    if (!r) return;
    const data = await r.json();
    if (r.ok) {
      g_config = Object.assign(g_config, { board_preset: boardPreset, rs485_enabled: rs485Enabled, pins });
      if (data.restart_required) {
        if (fb) {
          fb.className = 'feedback-msg ok';
          fb.textContent = 'Saved. Device is rebooting — page will reload shortly.';
        }
        setTimeout(() => location.reload(), 6000);
      } else {
        if (fb) { fb.className = 'feedback-msg ok'; fb.textContent = 'Saved.'; }
      }
    } else {
      if (fb) { fb.className = 'feedback-msg err'; fb.textContent = data.error || 'Save failed.'; }
    }
  } catch (e) {
    if (fb) { fb.className = 'feedback-msg err'; fb.textContent = 'Network error: ' + e.message; }
  }
}

/* ── CAN / Inverter (Settings -> CAN) ──────────────────────────────────────────
 * Moved off Settings -> Battery: CAN/inverter output protocol is a distinct
 * concern from battery/BMS/shunt configuration. */
function renderSettingsCan() {
  const c = g_config;
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">CAN / Inverter</div>
        <div class="form-group">
          <label>Inverter Protocol</label>
          <div class="proto-options">
            <label class="proto-option">
              <input type="radio" name="can_protocol_radio" value="-1" ${!c.can_enabled ? 'checked' : ''}>
              <span class="proto-name">Disabled</span>
            </label>
            <label class="proto-option">
              <input type="radio" name="can_protocol_radio" value="0" ${c.can_enabled && c.can_protocol === 0 ? 'checked' : ''}>
              <span class="proto-name">Victron</span><span class="proto-desc">— HIL-verified</span>
            </label>
            <label class="proto-option">
              <input type="radio" name="can_protocol_radio" value="1" ${c.can_enabled && c.can_protocol === 1 ? 'checked' : ''}>
              <span class="proto-name">Pylontech</span><span class="proto-desc">— Spec-derived, community-validated</span>
            </label>
            <label class="proto-option">
              <input type="radio" name="can_protocol_radio" value="2" ${c.can_enabled && c.can_protocol === 2 ? 'checked' : ''}>
              <span class="proto-name">SMA</span><span class="proto-desc">— Spec-derived, community-validated</span>
            </label>
          </div>
          <div class="help" style="margin-top:8px">
            Pylontech and SMA implementations are derived from spec.
            The maintainer cannot HIL-verify these against their respective inverters.
            Reports from users with Pylontech or SMA hardware are welcome.
          </div>
        </div>
      </div>

      <div id="can-feedback" class="feedback-msg"></div>
      <div class="btn-row">
        <button class="btn btn-primary" onclick="saveCanSection()">Save</button>
      </div>
    </div>
  `;
}

function saveCanSection() {
  const protoChecked = document.querySelector('input[name="can_protocol_radio"]:checked');
  const protoVal = protoChecked ? Number(protoChecked.value) : null;

  let overrides = {};
  if (protoVal === -1) {
    overrides = { can_enabled: false };
  } else if (protoVal !== null && protoVal >= 0) {
    overrides = { can_enabled: true, can_protocol: protoVal };
  }

  saveSectionFields([], 'can-feedback', overrides);
}

/* ── Battery Value Sources (Settings -> Battery) ──────────────────────────────
 * Consolidates the former separate "Current Source Mode" / "Bank SOC Source
 * Mode" selectors (previously scattered on the /battery overview page) into
 * one Auto/Manual policy. Display/dashboard/MQTT only — charge-taper and CAN
 * TX are permanently BMS-only regardless of this setting (runSafety.cpp). */
function renderBatteryValueSourcesSection(c) {
  const shuntEnabled = !!c.ble_shunt_enabled;
  const policy = c.battery_source_policy !== undefined ? c.battery_source_policy : 0; // default Auto
  const isManual = policy === 1;
  const metrics = [
    { key: 'voltage', label: 'Voltage' },
    { key: 'current', label: 'Current' },
    { key: 'soc',     label: 'SOC' },
  ];
  return `
    <div class="settings-section">
      <div class="settings-section-title">Battery Value Sources</div>
      <div class="help" style="margin-bottom:10px">
        Controls where the Voltage, Current, and SOC shown on the dashboard and published
        to MQTT come from when the SmartShunt is enabled. Display only &mdash; charge-taper
        safety logic and CAN TX to the inverter always use the BMS value, in Auto or Manual,
        with no exceptions.
      </div>
      <div class="proto-options" id="cfg-bvs-policy-options">
        <label class="proto-option">
          <input type="radio" name="bvs_policy_radio" value="0" ${policy === 0 ? 'checked' : ''}
                 onchange="updateBvsPolicyUI(0)">
          <span class="proto-name">Auto</span><span class="proto-desc">— shunt leads whenever online &amp; fresh, BMS is the fallback, applied the same way to every metric</span>
        </label>
        <label class="proto-option">
          <input type="radio" name="bvs_policy_radio" value="1" ${policy === 1 ? 'checked' : ''}
                 onchange="updateBvsPolicyUI(1)">
          <span class="proto-name">Manual</span><span class="proto-desc">— pick BMS or Shunt independently for each metric below</span>
        </label>
      </div>
      <div id="cfg-bvs-manual-rows" style="margin-top:12px;${isManual ? '' : 'display:none'}">
        ${metrics.map(m => {
          const val = c[m.key + '_source'] !== undefined ? c[m.key + '_source'] : 0;
          return `
          <div class="form-group" style="display:flex;align-items:center;gap:10px;max-width:260px">
            <label style="min-width:70px;margin:0">${m.label}</label>
            <select id="cfg-bvs-${m.key}">
              <option value="0" ${val === 0 ? 'selected' : ''}>BMS</option>
              <option value="1" ${val === 1 ? 'selected' : ''}>Shunt</option>
            </select>
          </div>`;
        }).join('')}
      </div>
      ${!shuntEnabled ? `<div class="help" style="margin-top:8px;color:var(--brand-coral)">SmartShunt is not enabled &mdash; this policy has no effect until enabled in <a href="/settings#ble" onclick="navigate('/settings');showSettingsSection('ble');return false;">Settings &rarr; BLE</a>.</div>` : ''}
    </div>`;
}

function updateBvsPolicyUI(newPolicy) {
  const rows = document.getElementById('cfg-bvs-manual-rows');
  if (rows) rows.style.display = newPolicy === 1 ? '' : 'none';
}

function renderSettingsBattery() {
  const c = g_config;
  const mode = c.battery_config_mode !== undefined ? c.battery_config_mode : 2; // default Manual
  const isAuto = mode === 0 || mode === 1;
  const roAttr = isAuto ? ' readonly style="background:var(--input-bg,#f4f4f4);color:var(--text-muted);cursor:not-allowed"' : '';

  const modeInfo = [
    { label: 'Auto',             desc: 'Follows live pack SYSPARAM (0x47) limits including temperature-dependent DCL. Values are pack-derived and broadcast to the inverter immediately. Hard sanity cap: cell max ≤ 3.65 V.' },
    { label: 'Auto + Margin', desc: 'Same live-follow as Auto with per-quantity safety insets: CCL/DCL 10% inward, CVL 5% inward, cell voltage cap 50 mV inward (effective ceiling 3.60 V/cell), temperature window 3 °C inward on each bound.' },
    { label: 'Manual',           desc: 'You control the values. Pack SYSPARAM is used as a safe-direction cap only. For BMS variants that don’t report all parameters.' },
  ];

  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">Battery Config Mode</div>
        <div class="proto-options" id="cfg-batt-mode-options">
          ${modeInfo.map((m, i) => `
            <label class="proto-option">
              <input type="radio" name="batt_mode_radio" value="${i}" ${mode === i ? 'checked' : ''}
                     onchange="updateBatteryModeUI(${i})">
              <span class="proto-name">${m.label}</span>
            </label>`).join('')}
        </div>
        <div id="cfg-batt-mode-desc" style="margin-top:8px;font-size:13px;color:var(--text-muted)">
          ${modeInfo[mode] ? modeInfo[mode].desc : ''}
        </div>
        ${isAuto ? `<div style="margin-top:8px;font-size:12px;color:var(--brand-coral)">Current limit fields are read-only in Auto modes &mdash; values are derived from pack 0x47 data.</div>` : ''}
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Battery Packs</div>
        <div class="form-row">
          <div class="form-group">
            <label>BMS Pack Count</label>
            <input type="number" id="cfg-bms_count" value="${c.bms_count}" min="1" max="16">
            <div class="help">Number of TopBand packs (1&ndash;16)</div>
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
            <input type="number" id="cfg-charge_amps_per_pack" value="${c.charge_amps_per_pack}" step="0.1"${roAttr}>
            <div class="help">${isAuto ? 'Pack-derived in Auto modes' : 'Max charge current per pack (A)'}</div>
          </div>
          <div class="form-group">
            <label>Discharge Amps per Pack</label>
            <input type="number" id="cfg-discharge_amps_per_pack" value="${c.discharge_amps_per_pack}" step="0.1"${roAttr}>
            <div class="help">${isAuto ? 'Pack-derived in Auto modes' : 'Max discharge current per pack (A)'}</div>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Voltage Limits</div>
        <div class="form-row">
          <div class="form-group">
            <label>CVL (Charge Voltage Limit)</label>
            <input type="number" id="cfg-cvl_voltage" value="${c.cvl_voltage}" step="0.01"${roAttr}>
            <div class="help">${isAuto ? 'Pack-derived in Auto modes' : 'Max pack charge voltage (V)'}</div>
          </div>
          <div class="form-group">
            <label>Safe Pack Voltage</label>
            <input type="number" id="cfg-safe_pack_volt" value="${c.safe_pack_volt}" step="0.01"${roAttr}>
            <div class="help">${isAuto ? 'Pack-derived in Auto modes' : 'Overvolt alarm threshold (V)'}</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Safe Cell Voltage</label>
            <input type="number" id="cfg-safe_cell_volt" value="${c.safe_cell_volt}" step="0.001"${roAttr}>
            <div class="help">${isAuto ? 'Pack-derived; hard cap ≤ 3.65 V always' : 'Cell OVP threshold (V, max 3.65 V)'}</div>
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
            <label>Charge Temp Min (&deg;C)</label>
            <input type="number" id="cfg-charge_temp_min" value="${c.charge_temp_min}" step="0.5"${roAttr}>
          </div>
          <div class="form-group">
            <label>Charge Temp Max (&deg;C)</label>
            <input type="number" id="cfg-charge_temp_max" value="${c.charge_temp_max}" step="0.5"${roAttr}>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Discharge Temp Min (&deg;C)</label>
            <input type="number" id="cfg-discharge_temp_min" value="${c.discharge_temp_min}" step="0.5"${roAttr}>
          </div>
          <div class="form-group">
            <label>Discharge Temp Max (&deg;C)</label>
            <input type="number" id="cfg-discharge_temp_max" value="${c.discharge_temp_max}" step="0.5"${roAttr}>
          </div>
        </div>
        ${isAuto ? `<div style="font-size:12px;color:var(--text-muted);margin-top:4px">In Auto modes temperature limits come from pack 0x47 (most conservative across packs). Auto+Margin adds a 3 °C safety inset on each bound.</div>` : ''}
        <div class="form-row">
          <div class="form-group">
            <label>Soft Zone Width (&deg;C)</label>
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

      ${renderBatteryValueSourcesSection(c)}

      <div id="battery-feedback" class="feedback-msg"></div>
      <div class="btn-row">
        <button class="btn btn-primary" onclick="saveBatterySection()">Save</button>
      </div>
    </div>
  `;
}

function renderSettingsCharts() {
  const cc = getChartConfig();
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">Dashboard Charts</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Choose which data series to display on each of the two dashboard chart cards.
        </p>
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
        <div class="btn-row" style="margin-top:8px">
          <button class="btn btn-primary" onclick="saveChartConfig()">Apply</button>
        </div>
        <div id="chart-cfg-feedback" class="feedback-msg"></div>
      </div>
    </div>
  `;
}

function renderSettingsTime() {
  const c = g_config;
  const nowTs     = (g_live && g_live.now_ts_s) || (g_health && g_health.now_ts_s) || 0;
  const ntpSynced = !!(g_live && g_live.ntp_synced) || !!(g_health && g_health.ntp_synced);
  const deviceTimeStr = nowTs > 1000000 ? new Date(nowTs * 1000).toLocaleString() : 'Unknown';
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">NTP / Timezone</div>
        <div class="form-group">
          <label>NTP Server</label>
          <input type="text" id="cfg-ntp_server" value="${escHtml(c.ntp_server || '')}">
        </div>
        <div class="form-group">
          <label>Timezone Offset (hours)</label>
          <input type="number" id="cfg-timezone_offset_h" value="${c.timezone_offset_h}" min="-12" max="14">
          <div class="help">Offset from UTC, e.g. 1 for CET, 2 for CEST</div>
        </div>
        <div id="time-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="saveTimeSection()">Save</button>
        </div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">Current Time</div>
        <div class="form-row">
          <div class="form-group">
            <label>Device Time</label>
            <div class="settings-info-val">${deviceTimeStr}</div>
            <div class="help">Snapshot from last live data poll</div>
          </div>
          <div class="form-group">
            <label>NTP Status</label>
            <div style="padding:4px 0">
              <span class="charging-pill ${ntpSynced ? 'pill-charging' : 'pill-idle'}">${ntpSynced ? 'Synced' : 'Not synced'}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;
}

function renderSettingsMqtt() {
  const c = g_config;
  const mqttState = g_health && g_health.mqtt ? (g_health.mqtt.state || 'unknown') : 'unknown';
  const mqttStateLabels = { connected: 'Connected', connecting: 'Connecting', disconnected: 'Disconnected', disabled: 'Disabled', failed: 'Failed' };
  const mqttStatePill = mqttState === 'connected'
    ? '<span class="charging-pill pill-charging">Connected</span>'
    : `<span class="charging-pill pill-idle">${mqttStateLabels[mqttState] || mqttState}</span>`;
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">MQTT Broker</div>
        <div class="form-group" style="display:flex;align-items:center;gap:8px">
          <input type="checkbox" id="cfg-mqtt_enabled" ${c.mqtt_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-mqtt_enabled" style="margin:0">MQTT enabled</label>
          <span style="margin-left:8px">${mqttStatePill}</span>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Broker Host / IP</label>
            <input type="text" id="cfg-mqtt_host" value="${escHtml(c.mqtt_host || '')}" placeholder="192.168.1.x">
          </div>
          <div class="form-group">
            <label>Broker Port</label>
            <input type="number" id="cfg-mqtt_port" value="${c.mqtt_port || 1883}" min="1" max="65535">
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Username</label>
            <input type="text" id="cfg-mqtt_user" value="${escHtml(c.mqtt_user || '')}" autocomplete="off">
          </div>
          <div class="form-group">
            <label>Password</label>
            <input type="password" id="cfg-mqtt_pass" value="" autocomplete="new-password" placeholder="Leave blank to keep current">
            <div class="help">Leave blank to keep existing password</div>
          </div>
        </div>
        <div class="form-group">
          <label>Base Topic</label>
          <input type="text" id="cfg-mqtt_base_topic" value="${escHtml(c.mqtt_base_topic || 'topband-bms')}" placeholder="topband-bms">
          <div class="help">Gateway appends -XXXX (last 4 MAC hex) automatically</div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Publish Level</label>
            <select id="cfg-mqtt_level">
              <option value="0" ${c.mqtt_level === 0 ? 'selected' : ''}>Disabled</option>
              <option value="1" ${c.mqtt_level === 1 ? 'selected' : ''}>Status only</option>
              <option value="2" ${c.mqtt_level === 2 ? 'selected' : ''}>Data (system)</option>
              <option value="3" ${c.mqtt_level === 3 ? 'selected' : ''}>Per-pack</option>
              <option value="4" ${c.mqtt_level === 4 ? 'selected' : ''}>Per-cell</option>
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
        <div id="mqtt-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="saveMqttSection()">Save</button>
          <button class="btn btn-secondary" onclick="testMqttConnection()">Test connection</button>
          <button class="btn btn-secondary" onclick="sendHaDiscovery()">Re-send HA Discovery</button>
        </div>
        <div id="mqtt-test-result" style="display:none;margin-top:12px;padding:10px 12px;border-radius:6px;font-size:13px;border:1px solid var(--border)"></div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">Solar Passthrough (OpenDTU)</div>
        <div class="form-group">
          <label>Passthrough MQTT Topic</label>
          <input type="text" id="cfg-mqtt_solar_passthrough_topic"
            value="${escHtml(c.mqtt_solar_passthrough_topic || '')}"
            placeholder="e.g. solar/AC_output_active">
          <div class="help">Topic published by OpenDTU-OnBattery for the Solar Passthrough state.
            Payload: 1/true = active, 0/false = inactive. Leave blank to disable.</div>
        </div>
      </div>
    </div>
  `;
}

/* ── Notifications section ──────────────────────────────────────────────────── */

function renderSettingsNotifications() {
  const c = g_config;
  const pollVal     = (c.notify_poll_interval_s != null) ? c.notify_poll_interval_s : 60;
  const coolVal     = (c.notify_cooldown_s      != null) ? c.notify_cooldown_s      : 120;
  const debounceVal = (c.notify_debounce_s      != null) ? c.notify_debounce_s      : 30;
  return `
    <div class="settings-page">

      <!-- ── Group 1: Connection ──────────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">Connection</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Receive alerts from this gateway directly in a Telegram chat.
          <strong>Setup:</strong>
          (1) Message <a href="https://t.me/BotFather" target="_blank" rel="noopener">@BotFather</a>
          and run <code>/newbot</code> to get a bot token.
          (2) Start a chat with your bot (or add it to a group).
          (3) Find your Chat ID with
          <a href="https://t.me/userinfobot" target="_blank" rel="noopener">@userinfobot</a>
          (personal chats) or
          <a href="https://t.me/getidsbot" target="_blank" rel="noopener">@getidsbot</a>
          (groups — use the negative group ID).
        </p>

        <div class="form-group" style="display:flex;align-items:center;gap:8px;margin-bottom:10px">
          <input type="checkbox" id="cfg-notify_telegram_enabled"
                 ${c.notify_telegram_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-notify_telegram_enabled" style="margin:0">Enable Telegram notifications</label>
        </div>

        <div id="notify-status-line" style="font-size:13px;padding:8px 10px;border-radius:5px;
             margin-bottom:14px;border:1px solid var(--border);background:var(--surface-alt)">
          Loading status…
        </div>

        <div class="form-group">
          <label>Bot Token (API key)</label>
          <input type="password" id="cfg-notify_telegram_token" value=""
                 autocomplete="new-password" placeholder="Leave blank to keep current">
          <div class="help">Secret — never displayed here. Leave blank to keep the saved token.
            Format from BotFather: <code>1234567890:ABCDef...</code></div>
        </div>

        <div class="form-group">
          <label>Chat ID</label>
          <input type="text" id="cfg-notify_telegram_chat_id"
                 value="${escHtml(c.notify_telegram_chat_id || '')}"
                 placeholder="e.g. 123456789 or -1001234567890">
          <div class="help">Your personal numeric ID or a negative group ID.
            Find it with @userinfobot or @getidsbot.</div>
        </div>

        <div class="form-group">
          <label>Sender name</label>
          <input type="text" id="cfg-notify_sender_name"
                 value="${escHtml(c.notify_sender_name || '')}"
                 maxlength="31" placeholder="Leave blank to use device hostname">
          <div class="help">Name shown in outgoing messages so you know which gateway sent it.</div>
        </div>

        <div id="notify-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="saveNotificationsSection()">Save</button>
          <button class="btn btn-secondary" onclick="testTelegramNotification()">Send test</button>
        </div>
        <div id="notify-test-result" style="display:none;margin-top:12px;padding:10px 12px;
             border-radius:6px;font-size:13px;border:1px solid var(--border)"></div>
      </div>

      <!-- ── Group 1b: Network self-test ─────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">Network self-test</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Checks the outbound TLS path stage-by-stage: WiFi association,
          DNS resolution, TCP reachability, TLS handshake, and NTP clock.
          No credentials or bot token required. Use this when "Send test" fails
          to find exactly which stage is broken.
        </p>
        <div class="btn-row">
          <button class="btn btn-secondary" id="net-diag-btn" onclick="runNetDiag()">Run self-test</button>
        </div>
        <div id="net-diag-result" style="display:none;margin-top:14px"></div>
      </div>

      <!-- ── Group 2: Alerts ───────────────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">Alerts</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Choose which conditions trigger a notification. A message is sent when a condition
          begins and again when it clears. Per-alert cooldown and global poll interval
          (Timing section below) prevent flooding.
        </p>
        <div id="notify-alert-types-list" style="font-size:13px;color:var(--text-muted)">
          Loading alert types…
        </div>
      </div>

      <!-- ── Group 3: Timing ───────────────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">Timing</div>
        <p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">
          Poll interval and cooldown floors are enforced at 60 s. Debounce minimum is 0 (disabled).
        </p>

        <div class="form-group">
          <label>Alert debounce (seconds)</label>
          <input type="number" id="cfg-notify_debounce_s"
                 value="${debounceVal}" min="0" max="300" step="1">
          <div class="help">A condition must persist this long before it is logged and notified.
            Filters brief flaps (e.g. a pack that reconnects within seconds).
            0 = disabled (immediate). Does not affect the gateway's safety response,
            which always reacts immediately.</div>
        </div>

        <div class="form-group">
          <label>Poll interval (seconds)</label>
          <input type="number" id="cfg-notify_poll_interval_s"
                 value="${pollVal}" min="60" max="3600" step="1">
          <div class="help">Global minimum time between any two notification sends.
            Floor: 60 s. Example values: 60, 120, 300.</div>
        </div>

        <div class="form-group">
          <label>Per-alert cooldown (seconds)</label>
          <input type="number" id="cfg-notify_cooldown_s"
                 value="${coolVal}" min="60" max="3600" step="1">
          <div class="help">Minimum time before the same alert type can fire again (e.g. repeated
            under-voltage). Floor: 60 s. Does not affect clear/recovery messages.</div>
        </div>
      </div>

    </div>
  `;
}

// Load status and alert-types asynchronously after the notifications page renders.
async function loadNotifyData() {
  loadNotifyStatus();
  loadNotifyAlertTypes();
}

async function loadNotifyStatus() {
  const el = document.getElementById('notify-status-line');
  if (!el) return;
  try {
    const r = await apiFetch('/api/notify/status');
    if (!r) return;
    const s = await r.json();
    let html = '';
    if (!s.token_stored || !s.chat_id_stored) {
      html = '<span style="color:var(--text-muted)">Not configured. Enter a Bot Token and Chat ID above.</span>';
    } else if (!s.verified) {
      html = '<span style="color:var(--color-warning)">API key: stored (&#x2022;&#x2022;&#x2022;&#x2022;) &nbsp;|&nbsp; '
           + 'Chat ID: ' + escHtml(((window.g_config||{}).notify_telegram_chat_id)||'stored') + '</span>'
           + '<br><span style="font-size:12px;color:var(--text-muted)">Not yet verified with Telegram. Use Send test to verify.</span>';
    } else {
      const dt = s.last_ok_ts ? new Date(s.last_ok_ts * 1000).toLocaleString() : '';
      html = '<span style="color:var(--color-success)">API key and Chat ID stored and verified &#x2713;</span>'
           + (dt ? '<br><span style="font-size:12px;color:var(--text-muted)">Last verified: ' + escHtml(dt) + '</span>' : '');
    }
    el.innerHTML = html;
  } catch (e) {
    const el2 = document.getElementById('notify-status-line');
    if (el2) el2.textContent = 'Status unavailable.';
  }
}

async function loadNotifyAlertTypes() {
  const container = document.getElementById('notify-alert-types-list');
  if (!container) return;
  try {
    const r = await apiFetch('/api/notify/alert-types');
    if (!r) return;
    const types = await r.json();
    const flags = (g_config && g_config.notify_alert_flags != null)
                  ? g_config.notify_alert_flags : 0;

    // Group by 'group' field.
    const groups = {};
    for (const t of types) {
      if (!groups[t.group]) groups[t.group] = [];
      groups[t.group].push(t);
    }
    const groupLabels = { voltage: 'Voltage', temperature: 'Temperature', cell: 'Cell', system: 'System' };

    let html = '';
    for (const [gkey, items] of Object.entries(groups)) {
      html += '<div style="margin-bottom:12px">';
      html += '<div style="font-size:12px;font-weight:600;text-transform:uppercase;'
            + 'letter-spacing:.05em;color:var(--text-muted);margin-bottom:6px">'
            + escHtml(groupLabels[gkey] || gkey) + '</div>';
      for (const t of items) {
        const checked = (flags & (1 << t.id)) ? 'checked' : '';
        html += `<div style="display:flex;align-items:center;gap:8px;margin-bottom:6px">
          <input type="checkbox" id="notify-ev-${t.id}" data-ev-id="${t.id}" ${checked} style="width:auto">
          <label for="notify-ev-${t.id}" style="margin:0;font-size:13px">${escHtml(t.name)}</label>
        </div>`;
      }
      html += '</div>';
    }
    container.innerHTML = html || '<span style="color:var(--text-muted)">No alert types available.</span>';
  } catch (e) {
    const c2 = document.getElementById('notify-alert-types-list');
    if (c2) c2.textContent = 'Could not load alert types.';
  }
}

async function saveNotificationsSection() {
  const msg = document.getElementById('notify-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash     = '';
  cfg.mqtt_pass_obf = '';

  const enabledEl = document.getElementById('cfg-notify_telegram_enabled');
  if (enabledEl) cfg.notify_telegram_enabled = enabledEl.checked;

  const chatIdEl = document.getElementById('cfg-notify_telegram_chat_id');
  if (chatIdEl) cfg.notify_telegram_chat_id = chatIdEl.value;

  const senderEl = document.getElementById('cfg-notify_sender_name');
  if (senderEl) cfg.notify_sender_name = senderEl.value;

  // Token: blank means keep existing; non-blank means update.
  const tokenEl = document.getElementById('cfg-notify_telegram_token');
  cfg.notify_telegram_token = (tokenEl && tokenEl.value.length > 0) ? tokenEl.value : '';

  // Timing fields — debounce floor 0, poll/cooldown floor 60.
  const debounceEl = document.getElementById('cfg-notify_debounce_s');
  if (debounceEl) {
    const v = parseInt(debounceEl.value, 10);
    cfg.notify_debounce_s = (isNaN(v) || v < 0) ? 0 : v;
  }
  const pollEl = document.getElementById('cfg-notify_poll_interval_s');
  if (pollEl) {
    const v = parseInt(pollEl.value, 10);
    cfg.notify_poll_interval_s = (isNaN(v) || v < 60) ? 60 : v;
  }
  const coolEl = document.getElementById('cfg-notify_cooldown_s');
  if (coolEl) {
    const v = parseInt(coolEl.value, 10);
    cfg.notify_cooldown_s = (isNaN(v) || v < 60) ? 60 : v;
  }

  // Alert flags: collect checked state from all rendered checkboxes.
  let flags = 0;
  document.querySelectorAll('[data-ev-id]').forEach(cb => {
    const id = parseInt(cb.dataset.evId, 10);
    if (!isNaN(id) && cb.checked) flags |= (1 << id);
  });
  cfg.notify_alert_flags = flags >>> 0;  // ensure uint32

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
      if (tokenEl) tokenEl.value = '';
      if (msg) { msg.className = 'feedback-msg ok'; msg.textContent = 'Saved.'; }
      // Refresh status line — credentials may have changed.
      loadNotifyStatus();
    } else {
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = data.error || 'Save failed.'; }
    }
  } catch (e) {
    if (msg) { msg.className = 'feedback-msg err'; msg.textContent = 'Network error: ' + e.message; }
  }
}

/* ── Telegram notification test ─────────────────────────────────────────────── */

let g_notify_test_poll = null;

async function testTelegramNotification() {
  const resultEl = document.getElementById('notify-test-result');
  if (!resultEl) return;

  if (g_notify_test_poll) { clearInterval(g_notify_test_poll); g_notify_test_poll = null; }

  const enabled = (document.getElementById('cfg-notify_telegram_enabled') || {}).checked || false;
  const tokenEl = document.getElementById('cfg-notify_telegram_token');
  const token   = tokenEl ? tokenEl.value : '';
  const chatId  = (document.getElementById('cfg-notify_telegram_chat_id') || {}).value || '';

  if (!chatId) {
    resultEl.style.display = 'block';
    resultEl.style.borderColor = 'var(--color-alarm)';
    resultEl.style.color = 'var(--color-alarm)';
    resultEl.textContent = 'Enter a Chat ID first.';
    return;
  }

  resultEl.style.display = 'block';
  resultEl.style.borderColor = 'var(--border)';
  resultEl.style.color = 'var(--text-muted)';
  resultEl.innerHTML = '<span class="spinner"></span> Sending test notification…' +
    (token === '' ? ' <em style="font-size:11px">(using saved token)</em>' : '');

  try {
    const r = await apiFetch('/api/notify/telegram/test', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        notify_telegram_enabled: enabled,
        notify_telegram_token:   token,
        notify_telegram_chat_id: chatId,
      }),
    });
    if (!r) return;
    if (!r.ok) {
      const d = await r.json().catch(() => ({}));
      resultEl.style.color = 'var(--color-alarm)';
      resultEl.style.borderColor = 'var(--color-alarm)';
      resultEl.textContent = d.error || 'Failed to start test.';
      return;
    }
  } catch (e) {
    resultEl.style.color = 'var(--color-alarm)';
    resultEl.style.borderColor = 'var(--color-alarm)';
    resultEl.textContent = 'Network error: ' + e.message;
    return;
  }

  // Poll until done (max 60 s at 1 s intervals).
  // TLS handshake under MQTT load: up to ~45 s worst case (9 ops x 5 s timeout).
  // DRAM retry in http_post adds up to 3 s before the handshake.
  let polls = 0;
  g_notify_test_poll = setInterval(async function() {
    polls++;
    if (polls > 60) {
      clearInterval(g_notify_test_poll); g_notify_test_poll = null;
      resultEl.style.color = 'var(--color-alarm)';
      resultEl.style.borderColor = 'var(--color-alarm)';
      resultEl.textContent = 'Test timed out — no result after 60 s.';
      return;
    }
    try {
      const r2 = await apiFetch('/api/notify/telegram/test');
      if (!r2) return;
      const d = await r2.json();
      if (d.status === 'running') return;
      clearInterval(g_notify_test_poll); g_notify_test_poll = null;
      if (d.status === 'ok') {
        resultEl.style.color = 'var(--color-success)';
        resultEl.style.borderColor = 'var(--color-success)';
        resultEl.textContent = d.message || 'Test notification sent.';
        // Refresh the status line to show "verified".
        loadNotifyStatus();
      } else {
        resultEl.style.color = 'var(--color-alarm)';
        resultEl.style.borderColor = 'var(--color-alarm)';
        resultEl.textContent = d.message || 'Test failed.';
      }
    } catch (e) { /* ignore transient poll errors */ }
  }, 1000);
}

/* ── Network self-test ───────────────────────────────────────────────────────── */

let g_net_diag_poll      = null;
let g_net_diag_lastStages = null;  // preserves last known stages so timeout can show them

function renderNetDiagStages(stages) {
  if (!stages || !stages.length) return '<span style="color:var(--text-muted)">No data</span>';
  const colorFor = (s) =>
    !s.run  ? 'var(--text-muted)'    :
    s.pass  ? 'var(--color-success)' : 'var(--color-alarm)';
  const iconFor  = (s) => !s.run ? '–' : s.pass ? '&#x2713;' : '&#x2717;';
  const durationStr = (s) => {
    if (!s.run || !s.duration_ms) return '';
    return s.duration_ms >= 1000
      ? ` <span style="color:var(--text-muted);font-size:11px">${(s.duration_ms / 1000).toFixed(1)} s</span>`
      : ` <span style="color:var(--text-muted);font-size:11px">${s.duration_ms} ms</span>`;
  };
  return stages.map(s => `
    <div style="display:flex;gap:10px;align-items:flex-start;margin-bottom:8px;font-size:13px">
      <span style="min-width:18px;font-weight:bold;color:${colorFor(s)}">${iconFor(s)}</span>
      <div>
        <strong style="color:${colorFor(s)}">${escHtml(s.label)}</strong>${durationStr(s)}<br>
        <span style="color:var(--text-muted)">${escHtml(s.detail || (s.run ? '' : 'Not run'))}</span>
      </div>
    </div>`).join('');
}

function renderNetDiagResult(stages, headerHtml) {
  const passed = stages ? stages.filter(s => s.run && s.pass).length : 0;
  const total  = stages ? stages.filter(s => s.run).length : 0;
  const summary = total > 0 ? `${passed}/${total} passed` : '';
  return `<details open style="border:1px solid var(--border-color);border-radius:6px;padding:10px 14px">
    <summary style="cursor:pointer;font-size:13px;font-weight:600;list-style:none;display:flex;align-items:center;gap:8px">
      <span>Self-test results${summary ? ' — ' + summary : ''}</span>
    </summary>
    <div style="margin-top:10px">${headerHtml}${renderNetDiagStages(stages)}</div>
  </details>`;
}

async function runNetDiag() {
  if (g_net_diag_poll) { clearInterval(g_net_diag_poll); g_net_diag_poll = null; }
  g_net_diag_lastStages = null;
  const resultEl = document.getElementById('net-diag-result');
  const btn      = document.getElementById('net-diag-btn');
  if (!resultEl) return;

  resultEl.style.display = 'block';
  resultEl.innerHTML = '<span class="spinner"></span> Running self-test…';
  if (btn) btn.disabled = true;

  try {
    const r = await apiFetch('/api/net/self-test', { method: 'POST' });
    if (!r) { if (btn) btn.disabled = false; return; }
    if (r.status === 409) {
      resultEl.innerHTML = '<span style="color:var(--text-muted)">Test already running…</span>';
    } else if (!r.ok) {
      const d = await r.json().catch(() => ({}));
      resultEl.innerHTML = `<span style="color:var(--color-alarm)">${escHtml(d.error || 'Failed to start')}</span>`;
      if (btn) btn.disabled = false;
      return;
    }
  } catch (e) {
    resultEl.innerHTML = `<span style="color:var(--color-alarm)">Network error: ${escHtml(e.message)}</span>`;
    if (btn) btn.disabled = false;
    return;
  }

  const STAGE_NAMES = ['WiFi / link', 'DNS resolution', 'TCP connect', 'TLS handshake', 'Time / cert sanity'];

  // Poll until running=false, max 90 s (TLS can take up to ~45 s worst case;
  // extra margin added for slow DNS + TCP on congested networks).
  let polls = 0;
  g_net_diag_poll = setInterval(async function() {
    polls++;
    if (polls > 90) {
      clearInterval(g_net_diag_poll); g_net_diag_poll = null;
      if (btn) btn.disabled = false;
      // Show last known stage data so the hanging stage is visible.
      const stuckStage = g_net_diag_lastStages
        ? g_net_diag_lastStages.find(s => !s.run)
        : null;
      const stuckName = stuckStage ? stuckStage.label : 'unknown stage';
      const timeoutHeader = `<div style="color:var(--color-alarm);font-size:13px;margin-bottom:8px">
        Timed out (90 s) — last stage running: <strong>${escHtml(stuckName)}</strong>. Check serial log.
      </div>`;
      resultEl.innerHTML = renderNetDiagResult(
        g_net_diag_lastStages || [],
        timeoutHeader
      );
      return;
    }
    try {
      const r2 = await apiFetch('/api/net/self-test');
      if (!r2 || !r2.ok) return;
      const d = await r2.json();
      if (d.stages) {
        g_net_diag_lastStages = d.stages;
        if (d.running) {
          const stageName = (d.current_stage >= 0 && d.current_stage < STAGE_NAMES.length)
            ? STAGE_NAMES[d.current_stage] : null;
          const header = stageName
            ? `<div style="margin-bottom:8px;font-size:13px;color:var(--text-muted)"><span class="spinner"></span> Testing: <strong>${escHtml(stageName)}</strong>…</div>`
            : '';
          resultEl.innerHTML = `<div style="margin-bottom:8px">${header}${renderNetDiagStages(d.stages)}</div>`;
        } else {
          resultEl.innerHTML = renderNetDiagResult(d.stages, '');
        }
      }
      if (!d.running) {
        clearInterval(g_net_diag_poll); g_net_diag_poll = null;
        if (btn) btn.disabled = false;
      }
    } catch (e) { /* ignore transient poll errors */ }
  }, 1000);
}

function renderSettingsAccount() {
  const c = g_config;
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">Authentication</div>
        <div style="margin-bottom:20px">
          ${c.auth_enabled
            ? `<div style="display:flex;align-items:center;gap:12px">
                 <span class="charging-pill pill-charging">Enabled</span>
                 <button class="btn btn-secondary"
                         onclick="confirmDisableAuth()">Disable</button>
               </div>`
            : (c.auth_hash
              ? `<div style="display:flex;align-items:center;gap:12px">
                   <span class="charging-pill pill-discharging">Disabled</span>
                   <button class="btn btn-primary"
                           onclick="enableAuth()">Enable</button>
                 </div>`
              : `<div style="display:flex;align-items:center;gap:12px">
                   <span class="charging-pill pill-idle">Disabled</span>
                   <span style="font-size:12px;color:var(--text-muted)">Set a password first to enable authentication</span>
                 </div>`)}
          <div id="auth-toggle-feedback" class="feedback-msg" style="margin-top:6px"></div>
        </div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">Change Password</div>
        ${c.auth_enabled
          ? `<p style="font-size:13px;color:var(--text-muted);margin-bottom:14px">Enter current password to change it.</p>
             ${pwField('pw-current', 'Current password', 'current-password', 'Current password')}`
          : `<p style="font-size:13px;color:var(--color-warning);margin-bottom:14px">
               No password set — set one below to enable authentication.
             </p>`}
        ${pwField('pw-new',     'New password',         'new-password', 'New password')}
        ${pwField('pw-confirm', 'Confirm new password', 'new-password', 'Repeat new password')}
        <div id="pw-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="changePassword()">Save password</button>
        </div>
      </div>
    </div>
  `;
}

function renderSettingsSystem() {
  const h = g_health;
  const psramKb = (h && h.free_psram_b) ? (h.free_psram_b / 1024).toFixed(0) + ' KB' : '—';
  const heapKb  = (h && h.free_heap_b)  ? (h.free_heap_b  / 1024).toFixed(0) + ' KB' : '—';
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">System Info</div>
        <div class="form-row">
          <div class="form-group">
            <label>Firmware</label>
            <div class="settings-info-val">${h ? escHtml(h.version) : '—'}</div>
            <div class="help">${h ? escHtml(h.build) : ''}</div>
          </div>
          <div class="form-group">
            <label>UI Version</label>
            <div class="settings-info-val">${h ? escHtml(h.ui_version || '—') : '—'}</div>
            <div class="help">Served from LittleFS</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Uptime</label>
            <div class="settings-info-val">${h ? formatUptime(h.uptime_s) : '—'}</div>
          </div>
          <div class="form-group">
            <label>Free Heap</label>
            <div class="settings-info-val">${heapKb}</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label>Free PSRAM</label>
            <div class="settings-info-val">${psramKb}</div>
          </div>
        </div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">Firmware Update</div>
        <div class="form-row">
          <div class="form-group" style="flex:1">
            <label>Current firmware</label>
            <div class="settings-info-val">${h ? escHtml(h.version) : '—'}</div>
            <div class="help">${h ? escHtml(h.build || '') : ''}</div>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group" style="flex:1">
            <label>Firmware file (.bin)</label>
            <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
              <input type="file" id="ota-file-input" accept=".bin" style="display:none" onchange="otaFileChanged(event)">
              <button class="btn btn-secondary" onclick="document.getElementById('ota-file-input').click()">Choose file</button>
              <span id="ota-file-info" style="font-size:12px;color:var(--text-muted)">No file selected</span>
            </div>
            <div id="ota-hash" style="font-size:11px;color:var(--text-muted);margin-top:4px;word-break:break-all"></div>
          </div>
        </div>
        <div id="ota-progress" style="display:none;margin:8px 0">
          <div style="background:var(--border);border-radius:4px;height:8px;overflow:hidden">
            <div id="ota-progress-bar" style="background:var(--brand-teal,#76D2D9);height:100%;width:0%;transition:width 0.2s"></div>
          </div>
        </div>
        <div class="btn-row">
          <button id="ota-upload-btn" class="btn btn-primary" disabled onclick="startOtaUpload()">Upload &amp; install</button>
        </div>
        <div id="ota-status" class="feedback-msg" style="margin-top:8px"></div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">Maintenance</div>
        <div class="btn-row">
          <button class="btn btn-secondary" onclick="downloadBackup()">Download Backup</button>
          <button class="btn btn-secondary" onclick="confirmRestart()">Restart Gateway</button>
        </div>
        <div style="margin-top:20px">
          <div class="settings-section-title" style="font-size:13px;margin-bottom:8px">Import Backup</div>
          <p style="font-size:12px;color:var(--text-muted);margin-bottom:10px">
            Restore settings from a previously downloaded backup file.
            Passwords (MQTT, authentication) are NOT included in backups and must be re-entered after import.
          </p>
          <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
            <input type="file" id="restore-file-input" accept=".json" style="display:none" onchange="restoreFileChanged(event)">
            <button class="btn btn-secondary" onclick="document.getElementById('restore-file-input').click()">Choose backup file</button>
            <span id="restore-file-info" style="font-size:12px;color:var(--text-muted)">No file selected</span>
          </div>
          <div id="restore-metadata"></div>
          <div class="btn-row" style="margin-top:10px">
            <button id="restore-import-btn" class="btn btn-secondary" disabled onclick="startRestore()">Import backup</button>
          </div>
          <div id="restore-status" class="feedback-msg" style="margin-top:8px"></div>
        </div>
      </div>
      <div class="settings-section">
        <div class="settings-section-title">About</div>
        <p style="font-size:13px;color:var(--text-secondary);line-height:1.6;margin-bottom:14px">
          Bridges up to 16 TopBand LiFePO4 BMS packs to a solar inverter via CAN bus
          (Victron, Pylontech, or SMA). Aggregates cell voltages, temperatures, and alarms,
          applies safety limits, and controls charge/discharge in real time.
          Publishes live data via MQTT with Home Assistant auto-discovery.
        </p>
        <div class="diag-kv-grid" style="margin-bottom:12px">
          ${kvRow('Version', h ? escHtml(h.version || '—') : '—')}
          ${kvRow('UI Version', h ? escHtml(h.ui_version || '—') : '—')}
          ${kvRow('Build', h ? escHtml(h.build || '—') : '—')}
          ${kvRow('License', 'MIT')}
        </div>
        <p style="font-size:12px;color:var(--text-muted);margin-bottom:4px">With thanks to:</p>
        <ul style="font-size:12px;color:var(--text-muted);line-height:1.8;padding-left:1.2em;margin:0 0 6px">
          <li><a href="https://github.com/linedot/topband-bms" target="_blank" rel="noopener noreferrer">linedot/topband-bms</a> — TopBand RS485 protocol reverse-engineering</li>
          <li><a href="https://github.com/atomi23/Topband-BMS-to-CAN" target="_blank" rel="noopener noreferrer">atomi23/Topband-BMS-to-CAN</a> — original BMS-to-CAN concept</li>
        </ul>
        <div style="font-size:12px;color:var(--text-muted)">
          <a href="https://github.com/swingstate/topband-bms-gateway" target="_blank" rel="noopener noreferrer">
            github.com/swingstate/topband-bms-gateway
          </a>
        </div>
      </div>
    </div>
  `;
}

function renderSettingsReset() {
  return `
    <div class="settings-page">
      <div class="settings-section settings-section-danger">
        <div class="settings-section-title settings-section-title-danger">Factory Reset</div>
        <p style="font-size:13px;font-weight:600;color:var(--text-primary);margin-bottom:4px">Wipe all settings</p>
        <p style="font-size:12px;color:var(--text-muted);margin-bottom:14px">
          Wipes WiFi credentials and admin password. The gateway will reboot into captive portal mode and need to be set up again from scratch.
        </p>
        <div id="reset-feedback" class="feedback-msg"></div>
        <button class="btn btn-danger" onclick="confirmFactoryReset()">Factory Reset</button>
      </div>
    </div>
  `;
}

/* ── Section-specific save functions ────────────────────────────────────────── */

async function saveSectionFields(fields, feedbackId, overrides) {
  const msg = document.getElementById(feedbackId);
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash    = '';  // never re-send
  cfg.mqtt_pass_obf = ''; // preserve unless section explicitly sets it

  fields.forEach(([id, key, type]) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (type === 'num')  cfg[key] = Number(el.value);
    if (type === 'str')  cfg[key] = el.value;
    if (type === 'bool') cfg[key] = el.checked;
  });

  // Apply caller-supplied overrides (e.g. values read from radio groups or
  // other controls that can't be expressed as simple element IDs).
  if (overrides) Object.assign(cfg, overrides);

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
      if (msg) { msg.className = 'feedback-msg ok'; msg.textContent = 'Saved.'; }
    } else {
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = data.error || 'Save failed.'; }
    }
  } catch (e) {
    if (msg) { msg.className = 'feedback-msg err'; msg.textContent = 'Network error: ' + e.message; }
  }
}

// Called when the battery config mode radio changes — updates helper text and
// readonly state of pack-derived fields without a full page reload.
function updateBatteryModeUI(newMode) {
  const modeDescs = [
    'Follows live pack SYSPARAM (0x47) limits including temperature-dependent DCL. Values are pack-derived and broadcast to the inverter immediately. Hard sanity cap: cell max ≤ 3.65 V.',
    'Same live-follow as Auto with per-quantity safety insets: CCL/DCL 10% inward, CVL 5% inward, cell voltage cap 50 mV inward (effective ceiling 3.60 V/cell), temperature window 3 °C inward on each bound.',
    'You control the values. Pack SYSPARAM is used as a safe-direction cap only. For BMS variants that don’t report all parameters.',
  ];
  const descEl = document.getElementById('cfg-batt-mode-desc');
  if (descEl) descEl.textContent = modeDescs[newMode] || '';
  const isAuto = newMode === 0 || newMode === 1;
  const autoFields = ['cfg-charge_amps_per_pack','cfg-discharge_amps_per_pack',
                      'cfg-cvl_voltage','cfg-safe_pack_volt','cfg-safe_cell_volt',
                      'cfg-charge_temp_min','cfg-charge_temp_max',
                      'cfg-discharge_temp_min','cfg-discharge_temp_max'];
  autoFields.forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    if (isAuto) {
      el.setAttribute('readonly', '');
      el.style.background = 'var(--input-bg,#f4f4f4)';
      el.style.color = 'var(--text-muted)';
      el.style.cursor = 'not-allowed';
    } else {
      el.removeAttribute('readonly');
      el.style.background = '';
      el.style.color = '';
      el.style.cursor = '';
    }
  });
}

function saveBatterySection() {
  const modeChecked = document.querySelector('input[name="batt_mode_radio"]:checked');
  const modeVal = modeChecked ? Number(modeChecked.value) : null;

  let overrides = {};
  if (modeVal !== null && modeVal >= 0 && modeVal <= 2) {
    overrides.battery_config_mode = modeVal;
  }

  // Battery Value Sources (Auto/Manual policy + per-metric BMS/Shunt pickers).
  // Display only — never touches charge-taper or CAN TX (see runSafety.cpp).
  const bvsPolicyChecked = document.querySelector('input[name="bvs_policy_radio"]:checked');
  if (bvsPolicyChecked) {
    overrides.battery_source_policy = Number(bvsPolicyChecked.value);
  }
  ['voltage', 'current', 'soc'].forEach(m => {
    const el = document.getElementById(`cfg-bvs-${m}`);
    if (el) overrides[`${m}_source`] = Number(el.value);
  });

  // In Auto/AutoMargin, pack-derived fields are readonly — read the current
  // g_config values so they round-trip unchanged rather than sending whatever
  // the disabled input contains.
  const isAuto = modeVal === 0 || modeVal === 1;
  if (isAuto) {
    overrides.charge_amps_per_pack    = g_config.charge_amps_per_pack;
    overrides.discharge_amps_per_pack = g_config.discharge_amps_per_pack;
    overrides.cvl_voltage             = g_config.cvl_voltage;
    overrides.safe_pack_volt          = g_config.safe_pack_volt;
    overrides.safe_cell_volt          = g_config.safe_cell_volt;
    overrides.charge_temp_min         = g_config.charge_temp_min;
    overrides.charge_temp_max         = g_config.charge_temp_max;
    overrides.discharge_temp_min      = g_config.discharge_temp_min;
    overrides.discharge_temp_max      = g_config.discharge_temp_max;
  }

  saveSectionFields([
    ['cfg-bms_count',              'bms_count',              'num'],
    ['cfg-force_cell_count',       'force_cell_count',       'num'],
    ['cfg-charge_amps_per_pack',   'charge_amps_per_pack',   'num'],
    ['cfg-discharge_amps_per_pack','discharge_amps_per_pack','num'],
    ['cfg-cvl_voltage',            'cvl_voltage',            'num'],
    ['cfg-safe_pack_volt',         'safe_pack_volt',         'num'],
    ['cfg-safe_cell_volt',         'safe_cell_volt',         'num'],
    ['cfg-safe_cell_drift',        'safe_cell_drift',        'num'],
    ['cfg-charge_temp_min',        'charge_temp_min',        'num'],
    ['cfg-charge_temp_max',        'charge_temp_max',        'num'],
    ['cfg-discharge_temp_min',     'discharge_temp_min',     'num'],
    ['cfg-discharge_temp_max',     'discharge_temp_max',     'num'],
    ['cfg-temp_soft_zone',         'temp_soft_zone',         'num'],
    ['cfg-temp_mode',              'temp_mode',              'num'],
    ['cfg-spike_volt_max',         'spike_volt_max',         'num'],
    ['cfg-spike_curr_max',         'spike_curr_max',         'num'],
  ], 'battery-feedback', overrides);
}

function saveTimeSection() {
  saveSectionFields([
    ['cfg-ntp_server',      'ntp_server',      'str'],
    ['cfg-timezone_offset_h','timezone_offset_h','num'],
  ], 'time-feedback');
}

async function saveMqttSection() {
  const msg = document.getElementById('mqtt-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash = '';

  const fields = [
    ['cfg-mqtt_enabled',                    'mqtt_enabled',                    'bool'],
    ['cfg-mqtt_host',                       'mqtt_host',                       'str'],
    ['cfg-mqtt_port',                       'mqtt_port',                       'num'],
    ['cfg-mqtt_user',                       'mqtt_user',                       'str'],
    ['cfg-mqtt_base_topic',                 'mqtt_base_topic',                 'str'],
    ['cfg-mqtt_level',                      'mqtt_level',                      'num'],
    ['cfg-mqtt_diag_enabled',               'mqtt_diag_enabled',               'bool'],
    ['cfg-mqtt_solar_passthrough_topic',    'mqtt_solar_passthrough_topic',    'str'],
    ['cfg-ha_discovery_enabled','ha_discovery_enabled','bool'],
  ];
  fields.forEach(([id, key, type]) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (type === 'num')  cfg[key] = Number(el.value);
    if (type === 'str')  cfg[key] = el.value;
    if (type === 'bool') cfg[key] = el.checked;
  });

  // MQTT password: blank means keep existing; non-blank means update.
  const passEl = document.getElementById('cfg-mqtt_pass');
  cfg.mqtt_pass_obf = (passEl && passEl.value.length > 0) ? passEl.value : '';

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
      if (passEl) passEl.value = '';
      if (msg) { msg.className = 'feedback-msg ok'; msg.textContent = 'Saved.'; }
    } else {
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = data.error || 'Save failed.'; }
    }
  } catch (e) {
    if (msg) { msg.className = 'feedback-msg err'; msg.textContent = 'Network error: ' + e.message; }
  }
}

/* ── BLE Sources section ────────────────────────────────────────────────────── */

function renderSettingsBle() {
  const c = g_config;
  return `
    <div class="settings-page">

      <div class="settings-section" style="border-left:3px solid var(--accent,#3b82f6);background:rgba(59,130,246,.07);padding:12px 16px;border-radius:4px;margin-bottom:16px">
        <strong>Restart required after saving.</strong>
        The NimBLE stack is initialized once at boot; changes only take effect after a reboot.<br>
        Keys come from <strong>VictronConnect</strong> &rarr; device &rarr; share icon &rarr; <em>Encryption key</em>.
      </div>

      <!-- ── SmartShunt ────────────────────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">SmartShunt (battery monitor)</div>
        <div style="padding:12px 16px;background:color-mix(in srgb,var(--neutral-300) 30%,var(--bg-card));border-radius:6px;font-size:12px;color:var(--text-muted);margin-bottom:12px">
          SmartShunt integration adds precise 1&thinsp;Hz current and a shunt-derived bank SOC (see
          <a href="/settings#battery" style="color:var(--accent)" onclick="navigate('/settings');showSettingsSection('battery');return false;">Settings &rarr; Battery &rarr; Battery Value Sources</a>
          once enabled). Disabled by default; the BLE stack is not initialised until a shunt is enabled here.
        </div>
        <div class="form-group" style="display:flex;align-items:center;gap:8px">
          <input type="checkbox" id="cfg-ble_shunt_enabled" ${c.ble_shunt_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-ble_shunt_enabled" style="margin:0">Enable SmartShunt BLE source</label>
        </div>
        <div class="form-group">
          <label>BLE MAC Address</label>
          <input type="text" id="cfg-ble_shunt_mac" value="${escHtml(c.ble_shunt_mac || '')}"
                 placeholder="e3:8d:48:c8:52:b4 or e38d48c852b4" maxlength="17" style="font-family:monospace">
          <div class="help">Colons, hyphens, or no separators — all accepted.</div>
        </div>
        <div class="form-group">
          <label>Encryption Key (32 hex chars)</label>
          <div class="pw-wrap">
            <input type="password" id="cfg-ble_shunt_key" autocomplete="new-password"
                   placeholder="Leave blank to keep current" style="font-family:monospace"
                   oninput="updateKeyHint('cfg-ble_shunt_key','ble-shunt-key-hint')">
            <button type="button" class="pw-toggle" onclick="togglePw('cfg-ble_shunt_key')" title="Show/hide">${EYE_SVG}</button>
          </div>
          <div class="help" id="ble-shunt-key-hint">Write-only — leave blank to keep the saved key. Extra text (label, spaces) is stripped automatically.</div>
        </div>
      </div>

      <!-- ── Smart Solar MPPT ──────────────────────────────────────────────── -->
      <div class="settings-section">
        <div class="settings-section-title">Smart Solar MPPT</div>
        <div class="form-group" style="display:flex;align-items:center;gap:8px">
          <input type="checkbox" id="cfg-ble_mppt_enabled" ${c.ble_mppt_enabled ? 'checked' : ''} style="width:auto">
          <label for="cfg-ble_mppt_enabled" style="margin:0">Enable MPPT BLE source</label>
        </div>
        <div class="form-group">
          <label>BLE MAC Address</label>
          <input type="text" id="cfg-ble_mppt_mac" value="${escHtml(c.ble_mppt_mac || '')}"
                 placeholder="e3:8d:48:c8:52:b4 or e38d48c852b4" maxlength="17" style="font-family:monospace">
          <div class="help">Colons, hyphens, or no separators — all accepted.</div>
        </div>
        <div class="form-group">
          <label>Encryption Key (32 hex chars)</label>
          <div class="pw-wrap">
            <input type="password" id="cfg-ble_mppt_key" autocomplete="new-password"
                   placeholder="Leave blank to keep current" style="font-family:monospace"
                   oninput="updateKeyHint('cfg-ble_mppt_key','ble-mppt-key-hint')">
            <button type="button" class="pw-toggle" onclick="togglePw('cfg-ble_mppt_key')" title="Show/hide">${EYE_SVG}</button>
          </div>
          <div class="help" id="ble-mppt-key-hint">Write-only — leave blank to keep the saved key. Extra text (label, spaces) is stripped automatically.</div>
        </div>
      </div>

      <div id="ble-feedback" class="feedback-msg"></div>
      <div class="btn-row">
        <button class="btn btn-primary" onclick="saveBleSectionSettings()">Save</button>
      </div>
    </div>
  `;
}

// Normalize a raw MAC input to canonical "xx:xx:xx:xx:xx:xx" (lowercase, colons).
// Accepts colons, hyphens, or bare 12 hex chars.  Returns '' for empty/whitespace.
// Returns null for anything that cannot be a valid 6-byte MAC.
function normalizeMacInput(raw) {
  if (!raw || raw.trim() === '') return '';
  const bare = raw.trim().toLowerCase().replace(/[:\-]/g, '');
  if (!/^[0-9a-f]{12}$/.test(bare)) return null;
  return bare.match(/.{2}/g).join(':');
}

// Extract a clean 32-hex-char Victron encryption key from possibly-noisy input.
// Copy-pasting the key from a phone photo (macOS Live Text / OCR) frequently
// captures surrounding text too — a "VE.Direct encryption key" label, a trailing
// period, inserted spaces/newlines — which silently corrupts the stored key so
// the firmware's parse_hex_key() rejects it and decode never runs. Rather than
// trust the raw paste, pull the 32-hex key out of it.
// Returns { key, error, extracted }:
//   key ''      → input left blank (keep the existing stored key)
//   key <32hex> → a clean lowercase key to send (extracted=true if it had to be
//                 recovered from longer pasted text)
//   key null    → non-blank input with no recoverable 32-hex key (error is set)
function extractVictronKey(raw) {
  if (!raw || raw.trim() === '') return { key: '', error: null, extracted: false };
  const stripped = raw.replace(/\s+/g, '');
  if (/^[0-9a-fA-F]{32}$/.test(stripped)) {
    return { key: stripped.toLowerCase(), error: null, extracted: false };
  }
  // A standalone 32-hex run bounded by non-hex (or string edges): handles
  // "label: <key>", "<key>.", "<key>\n…". A 33+ contiguous hex blob is
  // ambiguous and deliberately NOT matched (reject rather than guess wrong).
  const m = stripped.match(/(?:^|[^0-9a-fA-F])([0-9a-fA-F]{32})(?:[^0-9a-fA-F]|$)/);
  if (m) return { key: m[1].toLowerCase(), error: null, extracted: true };
  const hexCount = (stripped.match(/[0-9a-fA-F]/g) || []).length;
  return {
    key: null,
    error: `no 32-character key found (${hexCount} hex character${hexCount === 1 ? '' : 's'} in input). ` +
           `Paste exactly the 32-char key shown in VictronConnect.`,
    extracted: false,
  };
}

// Live feedback under a key input as the user types/pastes. Shows whether a
// clean 32-hex key was found, or was auto-extracted from noisier pasted text.
function updateKeyHint(inputId, hintId) {
  const el = document.getElementById(inputId);
  const hint = document.getElementById(hintId);
  if (!el || !hint) return;
  if (el.value.trim() === '') {
    hint.textContent = 'Write-only — leave blank to keep the saved key. Extra text (label, spaces) is stripped automatically.';
    hint.style.color = '';
    return;
  }
  const res = extractVictronKey(el.value);
  if (res.key === null) {
    hint.textContent = '✗ ' + res.error;
    hint.style.color = 'var(--red)';
  } else if (res.extracted) {
    hint.textContent = '✓ extracted the 32-char key from pasted text (extra characters ignored)';
    hint.style.color = 'var(--color-success)';
  } else {
    hint.textContent = '✓ 32 hex characters';
    hint.style.color = 'var(--color-success)';
  }
}

async function saveBleSectionSettings() {
  const msg = document.getElementById('ble-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash = '';

  const fields = [
    ['cfg-ble_shunt_enabled', 'ble_shunt_enabled', 'bool'],
    ['cfg-ble_mppt_enabled',  'ble_mppt_enabled',  'bool'],
    ['cfg-ble_shunt_mac',     'ble_shunt_mac',     'str'],
    ['cfg-ble_mppt_mac',      'ble_mppt_mac',      'str'],
  ];
  fields.forEach(([id, key, type]) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (type === 'str')  cfg[key] = el.value;
    if (type === 'bool') cfg[key] = el.checked;
  });

  // Normalize and client-side validate MAC fields before the POST.
  // The server also validates, but inline JS feedback is immediate.
  const macFields = [
    ['cfg-ble_shunt_mac', 'ble_shunt_mac', 'SmartShunt MAC'],
    ['cfg-ble_mppt_mac',  'ble_mppt_mac',  'MPPT MAC'],
  ];
  for (const [id, key, label] of macFields) {
    const el = document.getElementById(id);
    if (!el) continue;
    const normalized = normalizeMacInput(el.value);
    if (normalized === null) {
      if (msg) {
        msg.className = 'feedback-msg err';
        msg.textContent = `${label}: must be 6 hex bytes, e.g. e3:8d:48:c8:52:b4 — colons and hyphens optional`;
      }
      return;
    }
    cfg[key] = normalized;   // send canonical form; server also normalizes
  }

  // Keys are write-only: blank = keep existing, non-blank = update.
  // Extract a clean 32-hex key from the (possibly OCR/photo-pasted) input and
  // validate before sending, so surrounding text never gets stored as the key.
  let keyExtracted = false;
  const keyFields = [
    ['cfg-ble_shunt_key', 'ble_shunt_key', 'SmartShunt key'],
    ['cfg-ble_mppt_key',  'ble_mppt_key',  'MPPT key'],
  ];
  for (const [id, key, label] of keyFields) {
    const el = document.getElementById(id);
    const res = extractVictronKey(el ? el.value : '');
    if (res.key === null) {
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = `${label}: ${res.error}`; }
      return;
    }
    cfg[key] = res.key;
    if (res.extracted) keyExtracted = true;
  }

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
      ['cfg-ble_shunt_key', 'cfg-ble_mppt_key'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.value = '';
      });
      if (msg) {
        msg.className = 'feedback-msg ok';
        msg.textContent = 'Saved. Reboot to apply BLE changes.' +
          (keyExtracted ? ' (key auto-extracted from pasted text)' : '');
      }
    } else {
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = data.error || 'Save failed.'; }
    }
  } catch (e) {
    if (msg) { msg.className = 'feedback-msg err'; msg.textContent = 'Network error: ' + e.message; }
  }
}

/* ── Network page ────────────────────────────────────────────────────────────── */

function rssiBarHtml(rssi) {
  // Map RSSI to 0-4 bars: > -55 = 4, > -65 = 3, > -75 = 2, > -85 = 1, else 0.
  const bars = rssi >= -55 ? 4 : rssi >= -65 ? 3 : rssi >= -75 ? 2 : rssi >= -85 ? 1 : 0;
  const label = bars >= 4 ? 'Excellent' : bars === 3 ? 'Good' : bars === 2 ? 'Fair' : bars === 1 ? 'Weak' : 'No signal';
  const heights = [4, 7, 10, 14];
  const barsHtml = heights.map((h, i) =>
    `<div class="rssi-bar${i < bars ? ' lit' : ''}" style="height:${h}px"></div>`
  ).join('');
  return `<span class="rssi-bars">${barsHtml}</span><span>${rssi} dBm</span>
          <span style="margin-left:6px;font-size:11px;color:var(--text-muted)">${label}</span>`;
}

// Three-state BSSID pin status shared between the Network page and the Diag
// WiFi section: "none" (no pin configured — today's behavior, unchanged),
// "pinned" (a target is configured and currently active/connected), or
// "fallback" (a target is configured but not currently active — the pinned
// AP was unreachable and the gateway is running on auto-select so it never
// gets stuck offline).
function bssidPinStatus(pinActive) {
  const target = (g_config && g_config.wifi_bssid) || '';
  if (!target) return { state: 'none', text: 'Not set (auto-select)' };
  if (pinActive) return { state: 'pinned', text: `Pinned to ${target} (connected)` };
  return { state: 'fallback', text: `Pinned to ${target} — unreachable, using auto-select` };
}

function renderNetworkStatus(d) {
  const root = document.getElementById('net-status-panel');
  if (!root) return;
  if (!d || !d.connected) {
    root.innerHTML = '<p style="color:var(--text-muted);font-size:13px">Not connected to any WiFi network.</p>';
    return;
  }
  const pin = bssidPinStatus(d.bssid_pin_active);
  root.innerHTML = `
    <div class="net-kv-grid">
      <div class="net-kv-row"><span>SSID</span><span>${escHtml(d.ssid || '—')}</span></div>
      <div class="net-kv-row"><span>Signal</span><span>${rssiBarHtml(d.rssi || 0)}</span></div>
      <div class="net-kv-row"><span>BSSID</span><span>${escHtml(d.bssid || '—')}</span></div>
      <div class="net-kv-row"><span>BSSID pin</span><span style="${pin.state === 'fallback' ? 'color:var(--color-warning)' : ''}">${escHtml(pin.text)}</span></div>
      <div class="net-kv-row"><span>IP Address</span><span>${escHtml(d.ip || '—')}</span></div>
      <div class="net-kv-row"><span>Gateway</span><span>${escHtml(d.gateway || '—')}</span></div>
      <div class="net-kv-row"><span>Netmask</span><span>${escHtml(d.netmask || '—')}</span></div>
      <div class="net-kv-row"><span>DNS</span><span>${escHtml(d.dns || '—')}</span></div>
      <div class="net-kv-row"><span>mDNS Hostname</span><span>${escHtml(d.mdns_hostname || '—')}</span></div>
      <div class="net-kv-row"><span>Connected for</span><span>${formatUptime(d.connected_for_s || 0)}</span></div>
    </div>
  `;
}

async function fetchNetworkStatus() {
  try {
    const r = await apiFetch('/api/wifi/status');
    if (!r || !r.ok) return;
    const d = await r.json();
    renderNetworkStatus(d);
  } catch (_) {}
}

async function doWifiScan() {
  const btn  = document.getElementById('btn-wifi-scan');
  const list = document.getElementById('scan-results');
  if (!btn || !list) return;
  btn.disabled = true;
  btn.textContent = 'Scanning…';
  list.innerHTML = '<div style="padding:10px;font-size:12px;color:var(--text-muted)">Scanning for networks…</div>';
  try {
    const r = await apiFetch('/api/wifi/scan');
    if (!r || !r.ok) { list.innerHTML = ''; btn.disabled = false; btn.textContent = 'Scan'; return; }
    const networks = await r.json();
    if (!networks.length) {
      list.innerHTML = '<div style="padding:10px;font-size:12px;color:var(--text-muted)">No networks found.</div>';
    } else {
      list.innerHTML = networks.map(n => `
        <div class="scan-row" onclick="prefillSsid(${JSON.stringify(escHtml(n.ssid))})">
          <span class="scan-ssid">${escHtml(n.ssid)}</span>
          ${n.bssid ? `<span class="scan-bssid" title="Pin the gateway to this specific AP"
                 onclick="event.stopPropagation(); pinBssid(${JSON.stringify(escHtml(n.bssid))})">📌 ${escHtml(n.bssid)}</span>` : ''}
          <span class="scan-rssi">${n.rssi} dBm</span>
          <span class="scan-lock">${n.secure
            ? '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" width="14" height="14"><rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 10 0v4"/></svg>'
            : ''
          }</span>
        </div>`).join('');
    }
  } catch (_) {
    list.innerHTML = '<div style="padding:10px;font-size:12px;color:var(--color-alarm)">Scan failed.</div>';
  }
  btn.disabled = false;
  btn.textContent = 'Scan';
}

function prefillSsid(ssid) {
  const inp = document.getElementById('net-ssid');
  if (inp) { inp.value = ssid; inp.focus(); }
  const list = document.getElementById('scan-results');
  if (list) list.innerHTML = '';
}

// Picking a BSSID from scan results only fills the pin field — it does NOT
// save. The owner still has to click "Save" so a stray click never silently
// re-pins the gateway.
function pinBssid(bssid) {
  const inp = document.getElementById('net-bssid-pin');
  if (inp) { inp.value = bssid; inp.focus(); }
  const fb = document.getElementById('bssid-pin-feedback');
  if (fb) { fb.className = 'feedback-msg'; fb.textContent = `Selected ${bssid} — click Save to pin.`; }
}

async function saveBssidPin() {
  const fb = document.getElementById('bssid-pin-feedback');
  if (fb) { fb.className = 'feedback-msg'; fb.textContent = ''; }

  const inp = document.getElementById('net-bssid-pin');
  const raw = inp ? inp.value : '';
  const normalized = normalizeMacInput(raw);
  if (normalized === null) {
    if (fb) {
      fb.className = 'feedback-msg err';
      fb.textContent = 'Pinned BSSID must be 6 hex bytes, e.g. e3:8d:48:c8:52:b4 — colons and hyphens optional. Leave blank for auto-select.';
    }
    return;
  }

  // g_config may not be loaded yet if the Network page was opened directly
  // (fetchConfigOnce() is fire-and-forget at app boot) — never POST a partial
  // config, always work off a freshly-fetched full one.
  if (!g_config) {
    try {
      const r = await apiFetch('/api/config');
      if (r && r.ok) g_config = await r.json();
    } catch (_) {}
  }
  if (!g_config) {
    if (fb) { fb.className = 'feedback-msg err'; fb.textContent = 'Config not loaded yet — try again.'; }
    return;
  }

  const cfg = Object.assign({}, g_config);
  cfg.auth_hash = '';
  cfg.wifi_bssid = normalized;  // server also normalizes; sending canonical form

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
      if (inp) inp.value = data.wifi_bssid || '';
      if (fb) {
        fb.className = 'feedback-msg ok';
        fb.textContent = normalized
          ? 'Saved. Takes effect on the next WiFi reconnect (or reboot).'
          : 'Cleared — auto-select restored.';
      }
      fetchNetworkStatus();
    } else {
      if (fb) { fb.className = 'feedback-msg err'; fb.textContent = data.error || 'Save failed.'; }
    }
  } catch (_) {
    if (fb) { fb.className = 'feedback-msg err'; fb.textContent = 'Save failed — check connection.'; }
  }
}

function confirmWifiConnect() {
  const ssidEl = document.getElementById('net-ssid');
  const ssid = ssidEl ? ssidEl.value.trim() : '';
  if (!ssid) {
    const fb = document.getElementById('net-connect-feedback');
    if (fb) { fb.className = 'feedback-msg err'; fb.textContent = 'SSID is required.'; }
    return;
  }

  const currentSsid = (document.getElementById('net-status-panel')
    ? (document.getElementById('net-status-panel').querySelector('.net-kv-row span:last-child') || {}).textContent
    : '') || '(unknown)';

  const overlay = document.getElementById('modal-overlay');
  document.getElementById('modal-title').textContent = 'Switch network?';
  document.getElementById('modal-body').textContent =
    `Switch to "${ssid}"? The gateway will disconnect from the current network and attempt to join the new one. If unreachable, the captive portal (TopBand-Setup-XXXX) will start automatically.`;
  const confirmBtn = document.getElementById('modal-confirm');
  confirmBtn.textContent = 'Connect';
  confirmBtn.style.display = '';
  overlay.style.display = 'flex';
  confirmBtn.onclick = () => {
    overlay.style.display = 'none';
    doWifiConnect(ssid);
  };
}

async function doWifiConnect(ssid) {
  const passEl = document.getElementById('net-pass');
  const pass = passEl ? passEl.value : '';

  try {
    const r = await apiFetch('/api/wifi/configure', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ssid, password: pass }),
    });
    if (!r) return;
  } catch (_) { /* device is switching — normal to lose connection */ }

  showPageOverlay('Switching network…',
    'The gateway is connecting to the new network. ' +
    'If it cannot reach "' + ssid + '" it will fall back to the setup AP (TopBand-Setup-XXXX). ' +
    'You may need to navigate to the new gateway IP address.');
}

// Network section of Settings (formerly the standalone /network page). g_config
// is guaranteed loaded here because renderSettings() fetches it before rendering
// any section, so the BSSID pin field can prefill safely.
function renderSettingsNetwork() {
  const c = g_config || {};
  return `
    <div class="settings-page">
      <div class="settings-section">
        <div class="settings-section-title">Current Connection</div>
        <div id="net-status-panel">
          <div class="placeholder-page" style="height:80px"><div class="spinner"></div></div>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Switch to a Different Network</div>
        <div class="form-group">
          <label>SSID</label>
          <div style="display:flex;gap:8px">
            <input type="text" id="net-ssid" placeholder="Network name" style="flex:1">
            <button class="btn btn-secondary" id="btn-wifi-scan" onclick="doWifiScan()">Scan</button>
          </div>
          <div id="scan-results" class="scan-list" style="display:block"></div>
        </div>
        <div class="form-group">
          <label>Password</label>
          <div class="pw-wrap">
            <input type="password" id="net-pass" autocomplete="new-password" placeholder="Leave blank for open network">
            <button type="button" class="pw-toggle" onclick="togglePw('net-pass')" title="Show/hide">${EYE_SVG}</button>
          </div>
        </div>
        <div id="net-connect-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="confirmWifiConnect()">Connect to new network</button>
        </div>
      </div>

      <div class="settings-section">
        <div class="settings-section-title">Preferred Access Point (optional)</div>
        <p style="font-size:12px;color:var(--text-muted);margin:0 0 12px 0">
          Pin the gateway to one specific AP's BSSID — useful with multiple APs or a mesh
          sharing one SSID. If the pinned AP becomes unreachable, the gateway automatically
          falls back to the strongest AP of the same SSID, so it never gets stuck offline.
          Use the Scan button above and click a network's 📌 BSSID to fill this in, or type
          one directly.
        </p>
        <div class="form-group">
          <label>Pinned BSSID</label>
          <input type="text" id="net-bssid-pin" placeholder="Auto-select (no pin)"
                 value="${escHtml(c.wifi_bssid || '')}">
        </div>
        <div id="bssid-pin-feedback" class="feedback-msg"></div>
        <div class="btn-row">
          <button class="btn btn-primary" onclick="saveBssidPin()">Save</button>
        </div>
      </div>
    </div>
  `;
}

// Kick off the 5-s WiFi status refresh for the Network settings section. Stopped
// by renderSettingsSection() on section switch and by navigate() on leaving Settings.
function startNetworkStatusPoll() {
  if (g_network_timer) { clearInterval(g_network_timer); g_network_timer = null; }
  fetchNetworkStatus();
  g_network_timer = setInterval(fetchNetworkStatus, 5000);
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
      if (data.restart_required) {
        msg.className = 'feedback-msg ok';
        msg.textContent = 'Settings saved. Device restarting…';
        setTimeout(() => location.reload(), 5000);
      } else {
        msg.className = 'feedback-msg ok';
        msg.textContent = 'Settings saved.';
      }
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

/* ── OTA firmware update ─────────────────────────────────────────────────────── */

let g_ota_file   = null;
let g_ota_sha256 = null;

async function otaFileChanged(event) {
  const file = event.target.files && event.target.files[0];
  if (!file) return;
  g_ota_file   = file;
  g_ota_sha256 = null;

  const infoEl   = document.getElementById('ota-file-info');
  const hashEl   = document.getElementById('ota-hash');
  const uploadBtn = document.getElementById('ota-upload-btn');

  if (infoEl)    infoEl.textContent    = file.name + ' (' + (file.size / 1024).toFixed(0) + ' KB)';
  if (hashEl)    hashEl.textContent    = 'Computing SHA-256…';
  if (uploadBtn) uploadBtn.disabled    = true;

  try {
    const buf = await file.arrayBuffer();
    let hex;
    if (crypto.subtle) {
      // Secure context (HTTPS or localhost): use native SubtleCrypto.
      const hb = await crypto.subtle.digest('SHA-256', buf);
      hex = Array.from(new Uint8Array(hb))
                 .map(b => b.toString(16).padStart(2, '0')).join('');
    } else {
      // Non-secure context (plain HTTP on LAN): crypto.subtle is undefined.
      // Fall back to vendored js-sha256 (web/ui/lib/sha256.min.js).
      hex = sha256(buf);
    }
    g_ota_sha256 = hex;
    if (hashEl)    hashEl.textContent = 'SHA-256: ' + hex.substring(0, 16) + '…';
    if (uploadBtn) uploadBtn.disabled = false;
  } catch (e) {
    if (hashEl) hashEl.textContent = 'Error computing hash: ' + e.message;
  }
}

function startOtaUpload() {
  if (!g_ota_file || !g_ota_sha256) return;
  const overlay = document.getElementById('modal-overlay');
  document.getElementById('modal-title').textContent = 'Install firmware?';
  document.getElementById('modal-body').innerHTML =
    '<p>The gateway will reboot and run a 5-minute self-test. ' +
    'If the new firmware fails, the previous version is restored automatically.</p>' +
    '<p style="font-size:11px;color:var(--text-muted);margin-top:6px;word-break:break-all">' +
    'SHA-256: ' + escHtml(g_ota_sha256) + '</p>';
  document.getElementById('modal-confirm').style.display = '';
  document.getElementById('modal-confirm').textContent   = 'Install';
  overlay.style.display = 'flex';
  document.getElementById('modal-confirm').onclick = () => {
    overlay.style.display = 'none';
    doOtaUpload();
  };
}

function doOtaUpload() {
  const statusEl   = document.getElementById('ota-status');
  const progressEl = document.getElementById('ota-progress');
  const progressBar = document.getElementById('ota-progress-bar');
  const uploadBtn  = document.getElementById('ota-upload-btn');

  if (uploadBtn)  uploadBtn.disabled = true;
  if (statusEl)   { statusEl.className = 'feedback-msg'; statusEl.textContent = 'Uploading…'; }
  if (progressEl) progressEl.style.display = 'block';
  if (progressBar) progressBar.style.width = '0%';

  const prevVersion = g_health ? g_health.version : null;

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/ota/upload');
  xhr.setRequestHeader('Content-Type',       'application/octet-stream');
  xhr.setRequestHeader('X-CSRF-Token',       CSRF_TOKEN);
  xhr.setRequestHeader('X-Firmware-SHA256',  g_ota_sha256);

  xhr.upload.onprogress = function(e) {
    if (e.lengthComputable && progressBar) {
      progressBar.style.width = ((e.loaded / e.total) * 100).toFixed(0) + '%';
    }
  };

  xhr.onload = function() {
    if (progressBar) progressBar.style.width = '100%';
    if (xhr.status === 200) {
      if (statusEl) { statusEl.className = 'feedback-msg ok'; statusEl.textContent = 'Upload complete. Installing…'; }
      setTimeout(function() {
        showPageOverlay('Installing firmware…',
          'The gateway is rebooting. The new firmware will run a 5-minute self-test.');
        setTimeout(function() { pollOtaStatusUntilDone(prevVersion); }, 10000);
      }, 500);
    } else {
      let msg = 'Upload failed';
      try { msg = JSON.parse(xhr.responseText).error || msg; } catch (_) {}
      if (statusEl)   { statusEl.className = 'feedback-msg err'; statusEl.textContent = msg; }
      if (progressEl) progressEl.style.display = 'none';
      if (uploadBtn)  uploadBtn.disabled = false;
    }
  };

  xhr.onerror = function() {
    if (statusEl)   { statusEl.className = 'feedback-msg err'; statusEl.textContent = 'Network error during upload'; }
    if (progressEl) progressEl.style.display = 'none';
    if (uploadBtn)  uploadBtn.disabled = false;
  };

  xhr.send(g_ota_file);
}

function pollOtaStatusUntilDone(prevVersion) {
  const POLL_INTERVAL_MS = 3000;
  const TIMEOUT_MS       = 180000;  // 3 min
  let elapsed = 0;

  function updateOverlay(title, body) {
    const ov = document.getElementById('full-overlay');
    if (ov) ov.innerHTML =
      '<div class="spinner" style="width:36px;height:36px;border-width:3px"></div>' +
      '<h2>' + escHtml(title) + '</h2><p>' + body + '</p>';
  }

  // Finish: hide overlay, SPA-navigate to /settings#system, show toast.
  function finish(newVer, timedOut) {
    const ov = document.getElementById('full-overlay');
    if (ov) ov.style.display = 'none';
    history.pushState({}, '', '/settings#system');
    renderPage('/settings');
    updateSidebarActive('/settings');
    if (timedOut) {
      showToast('Update timeout — check version, gateway may have rolled back', 'warn');
    } else {
      showToast('Firmware updated to ' + escHtml(newVer || '?'));
    }
  }

  function poll() {
    elapsed += POLL_INTERVAL_MS;

    Promise.all([
      fetch('/api/health',     { cache: 'no-store' }).then(function(r) { return r.ok ? r.json() : null; }).catch(function() { return null; }),
      fetch('/api/ota/status', { cache: 'no-store' }).then(function(r) { return r.ok ? r.json() : null; }).catch(function() { return null; }),
    ]).then(function(results) {
      const hd = results[0];
      const sd = results[1];

      // Device not yet reachable — keep waiting.
      if (!hd) {
        if (elapsed >= TIMEOUT_MS) { finish(null, true); return; }
        updateOverlay('Installing firmware…', 'Waiting for gateway to come back online…');
        setTimeout(poll, POLL_INTERVAL_MS);
        return;
      }

      // Success criteria: version changed OR ota status reports "valid".
      const versionChanged = !prevVersion || hd.version !== prevVersion;
      const runningValid   = sd && sd.running_state === 'valid';

      if (versionChanged || runningValid) {
        const st = (sd && sd.self_test) ? sd.self_test : {};
        if (st.in_progress) {
          // Self-test still running — show progress but keep polling.
          const c = st.checks || {};
          const passed = [
            c.wifi_connected     && 'WiFi',
            c.http_server_up     && 'HTTP',
            c.snapshot_published && 'BMS',
            c.controltask_alive  && 'Control',
          ].filter(Boolean).join(', ') || 'none';
          updateOverlay('Self-test in progress…',
            'Checks: ' + passed + ' | ' + (st.elapsed_s || 0) + 's / ' + (st.deadline_s || 300) + 's');
          if (elapsed >= TIMEOUT_MS) { finish(hd.version, true); return; }
          setTimeout(poll, POLL_INTERVAL_MS);
          return;
        }
        // Self-test complete (or not in progress) — done.
        finish(hd.version, false);
        return;
      }

      // Same version back online — rollback occurred.
      updateOverlay('Update failed',
        'The previous firmware has been restored. Check the Alerts page for details.');
      setTimeout(function() {
        const ov = document.getElementById('full-overlay');
        if (ov) ov.style.display = 'none';
        history.pushState({}, '', '/settings#system');
        renderPage('/settings');
        updateSidebarActive('/settings');
        showToast('Update failed — firmware rolled back', 'warn');
      }, 3000);
    });
  }

  setTimeout(poll, POLL_INTERVAL_MS);
}

// Show a brief toast notification. type: 'info' (default) | 'warn'.
function showToast(message, type) {
  const toast = document.createElement('div');
  toast.className = 'toast' + (type === 'warn' ? ' toast-warn' : '');
  toast.textContent = message;
  document.body.appendChild(toast);
  // Trigger fade-in on next paint, then auto-dismiss after 4 s.
  requestAnimationFrame(function() {
    toast.classList.add('toast-visible');
    setTimeout(function() {
      toast.classList.remove('toast-visible');
      setTimeout(function() { toast.remove(); }, 400);
    }, 4000);
  });
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
          // Only redirect to login if auth is actually enabled.
          // With auth disabled the dashboard is served directly; defaulting to
          // login before auth_enabled is known caused the intermittent reboot race.
          window.location.href = data.auth_enabled ? '/login.html' : '/';
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
let g_network_timer  = null;
let g_diag_log_open  = false;  // persists across 5 s polls

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

// Epoch values below 2020-01-01 indicate pre-NTP (clock not yet synced).
const MIN_VALID_EPOCH = 1577836800;

// Format an alert timestamp using the device's configured timezone offset.
// Pre-NTP alerts (ts_epoch < 2020) fall back to a boot-relative indication.
function fmtAlertTs(tsEpoch, uptimeS) {
  if (tsEpoch && tsEpoch >= MIN_VALID_EPOCH) {
    const offsetH = (g_config && g_config.timezone_offset_h != null)
                    ? g_config.timezone_offset_h : 0;
    const d = new Date((tsEpoch + offsetH * 3600) * 1000);
    const yr  = d.getUTCFullYear();
    const mo  = String(d.getUTCMonth() + 1).padStart(2, '0');
    const dy  = String(d.getUTCDate()).padStart(2, '0');
    const hh  = String(d.getUTCHours()).padStart(2, '0');
    const mm  = String(d.getUTCMinutes()).padStart(2, '0');
    const ss  = String(d.getUTCSeconds()).padStart(2, '0');
    return `${yr}-${mo}-${dy} ${hh}:${mm}:${ss}`;
  }
  // Pre-NTP: show uptime-relative so user sees "boot +5m" rather than a 1970 date.
  if (uptimeS !== undefined && uptimeS !== null) {
    return 'boot +' + formatUptime(uptimeS || 0);
  }
  return '—';
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
  const ts     = fmtAlertTs(a.ts_epoch, a.uptime_s);
  return `
    <div class="alert-row" onclick="showAlertDetail(${JSON.stringify(JSON.stringify(a))})" title="${abs}">
      <span class="sev-dot ${sevCls}"></span>
      <div>
        <div style="font-size:13px;color:var(--text-primary)">${escHtml(a.message)}</div>
        <div class="alert-meta">
          <span>${rel}</span>
          <span class="source-pill">${escHtml(a.source)}</span>
          <span style="margin-left:auto;font-size:11px;color:var(--text-muted)">${escHtml(ts)}</span>
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
      <tr><td style="color:var(--text-muted);padding:3px 8px 3px 0">Time</td><td>${fmtAlertTs(a.ts_epoch, a.uptime_s)}</td></tr>
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
        <button class="btn btn-danger" onclick="clearAllAlerts()">Clear all</button>
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

function kvRow(k, v, title) {
  const t = title ? ' title="' + escHtml(String(title)) + '"' : '';
  return `<div class="diag-kv"${t}><span>${escHtml(String(k))}</span><span>${escHtml(String(v))}</span></div>`;
}

// Storage sizes as MB, working memory as KB; raw bytes stay in the tooltip.
function fmtKb(b) { return ((b || 0) / 1024).toFixed(1) + ' KB'; }
function fmtMb(b) { return ((b || 0) / 1048576).toFixed(1) + ' MB'; }

// Age in seconds -> human-readable above 2 minutes.
function fmtAgeS(s) {
  if (s == null || s < 0) return 'never';
  if (s < 120) return s + ' s ago';
  return formatUptime(s) + ' ago';
}

function rssiQuality(rssi) {
  if (rssi == null) return '';
  if (rssi >= -60) return ' (good)';
  if (rssi >= -75) return ' (fair)';
  return ' (weak)';
}

// Decode SafetyState.alarm_flags (the value that FEEDS the CAN encoders) into
// human labels, annotated with the Pylontech 0x359 bit each one drives. Single
// source of the bit meaning — the firmware sends the raw byte, this decodes it.
// Mirrors src/safety/runSafety.cpp (flag assignment) and src/can/pylontech.cpp
// build_0x359() (bit mapping). Keep the two in sync.
function decodeAlarmFlags(af) {
  af = af || 0;
  const out = [];
  if (af & 0x02) out.push('Overvoltage → 0x359 byte0 bit1');
  if (af & 0x10) out.push('Undervoltage → 0x359 byte0 bit2');
  if (af & 0x08) out.push('Temperature stop → 0x359 byte0 bit3/4');
  if (af & 0x40) out.push('BMS critical alarm → 0x359 byte1 bit3');
  if (af & 0x80) out.push('No packs online → 0x359 byte1 bit3');
  if (af & 0x20) out.push('Cell imbalance (UI/MQTT only — no 0x359 bit)');
  return out;
}
function decodeTempAlarm(ta) {
  ta = ta || 0;
  const out = [];
  if (ta & 0x01) out.push('Under-temperature (cold) → 0x359 byte0 bit4');
  if (ta & 0x02) out.push('Over-temperature (hot) → 0x359 byte0 bit3');
  return out;
}

// Per-pack RS485 comms table (id, address, online, last-seen age, poll counts).
function diagRs485Table(packs) {
  if (!packs.length) return '<div style="color:var(--text-muted);font-size:12px">No packs configured.</div>';
  return `<table class="diag-tasks-table">
    <thead><tr><th>Pack</th><th>Addr</th><th>Online</th><th>Last seen</th>
      <th>Polls</th><th>OK</th><th>Timeout</th><th>CRC/err</th><th>Success</th></tr></thead>
    <tbody>${packs.map(p => {
      const bad = p.polls > 0 && p.success_pct < 90;
      return `<tr${bad ? ' class="diag-task-low"' : ''}>
        <td>${p.id}</td>
        <td>${p.bms_id != null ? p.bms_id : p.id}</td>
        <td>${p.online ? 'yes' : 'no'}</td>
        <td>${fmtAgeS(p.last_seen_age_ms != null ? Math.round(p.last_seen_age_ms / 1000) : -1)}</td>
        <td>${(p.polls||0).toLocaleString()}</td>
        <td>${(p.ok||0).toLocaleString()}</td>
        <td>${(p.timeouts||0).toLocaleString()}</td>
        <td>${(p.errors||0).toLocaleString()}</td>
        <td>${p.polls ? (p.success_pct||0) + ' %' : '—'}</td>
      </tr>`;
    }).join('')}</tbody></table>`;
}

// Per-pack Battery/BMS table (cells, drift, SoC/SoH, sysparam limits, alarm bits).
function diagBatteryTable(packs) {
  if (!packs.length) return '<div style="color:var(--text-muted);font-size:12px">No packs configured.</div>';
  return `<table class="diag-tasks-table">
    <thead><tr><th>Pack</th><th>SoC</th><th>SoH</th><th>Cell min</th><th>Cell max</th>
      <th>Drift</th><th>CCL</th><th>DCL</th><th>Cell-OV limit</th><th>Alarm bits</th></tr></thead>
    <tbody>${packs.map(p => {
      const sp = p.sysparam_valid;
      const alarmSet = p.alarm_bits && p.alarm_bits !== '0x0000000000000000';
      return `<tr${alarmSet ? ' class="diag-task-low"' : ''}>
        <td>${p.id}</td>
        <td>${p.online ? (p.soc||0) + ' %' : '—'}</td>
        <td>${p.online ? (p.soh||0) + ' %' : '—'}</td>
        <td>${p.online ? Number(p.cell_min_v||0).toFixed(3) + ' (#' + (p.cell_min_idx+1) + ')' : '—'}</td>
        <td>${p.online ? Number(p.cell_max_v||0).toFixed(3) + ' (#' + (p.cell_max_idx+1) + ')' : '—'}</td>
        <td>${p.online ? (p.drift_mv||0) + ' mV' : '—'}</td>
        <td>${sp ? Number(p.sys_charge_max_a||0).toFixed(1) + ' A' : '—'}</td>
        <td>${sp ? Number(p.sys_discharge_max_a||0).toFixed(1) + ' A' : '—'}</td>
        <td>${sp ? Number(p.sys_cell_high_v||0).toFixed(3) + ' V' : '—'}</td>
        <td title="Raw 0x44 alarm bitmap">${alarmSet ? p.alarm_bits : 'none'}</td>
      </tr>`;
    }).join('')}</tbody></table>`;
}

function renderDiagData(d) {
  const root = document.getElementById('diag-root');
  if (!root) return;

  // Preserve log open/closed state across polls (innerHTML replacement resets it).
  const existingDetails = document.getElementById('diag-log-details');
  if (existingDetails) g_diag_log_open = existingDetails.open;

  const sys = d.system || {};
  const pol = d.poller || {};
  const can = d.can    || {};
  const bat = d.battery || {};
  const packs = d.packs || [];
  const sn  = d.snapshot_bus || {};
  const mq  = d.mqtt   || {};
  const ntp = d.ntp    || {};
  const lfs = d.littlefs || {};
  const ene = d.energy || {};
  const his = d.history || {};
  const ble = d.ble_status || {};

  const tasks = (d.tasks || []).slice().sort((a, b) => (a.stack_hwm||0) - (b.stack_hwm||0));

  // Device time from the diag payload (F4: now_ts_s is current device time).
  let deviceTime = '—';
  if (ntp.now_ts_s) {
    const dt = new Date(ntp.now_ts_s * 1000);
    deviceTime = dt.toLocaleString();
  }

  root.innerHTML = `
    <div class="diag-section">
      <h3>System</h3>
      <div class="diag-kv-grid">
        ${kvRow('Firmware', sys.fw || '—')}
        ${kvRow('Uptime', formatUptime(sys.uptime_s))}
        ${kvRow('Reset reason', sys.reset_reason || '—')}
        ${kvRow('Free heap (all)', fmtKb(sys.free_heap), (sys.free_heap||0) + ' B')}
        ${kvRow('DRAM free', fmtKb(sys.dram_free), (sys.dram_free||0) + ' B')}
        ${kvRow('DRAM min ever', fmtKb(sys.dram_min), (sys.dram_min||0) + ' B')}
        ${kvRow('DRAM largest block', fmtKb(sys.dram_largest_block), (sys.dram_largest_block||0) + ' B')}
        ${kvRow('PSRAM free', fmtMb(sys.psram_free), (sys.psram_free||0) + ' B')}
        ${kvRow('PSRAM largest block', fmtMb(sys.psram_largest_block), (sys.psram_largest_block||0) + ' B')}
        ${kvRow('CPU temp', sys.cpu_temp_c != null ? sys.cpu_temp_c.toFixed(1) + ' °C' : '—')}
        ${kvRow('/api/live latency (last / max)', (ble.handler_last_ms||0) + ' ms / ' + (ble.handler_max_ms||0) + ' ms',
                '> 200 ms while BLE is active indicates CPU starvation of the HTTP task')}
        ${kvRow('Build', sys.build || '—')}
      </div>
    </div>

    <div class="diag-section">
      <h3>Tasks (sorted by stack HWM)</h3>
      <table class="diag-tasks-table">
        <thead><tr><th>Name</th><th>Stack HWM</th><th>Core</th><th>Priority</th></tr></thead>
        <tbody>
          ${tasks.map(t => `<tr${(t.stack_hwm||0) < 512 ? ' class="diag-task-low"' : ''}>
            <td>${escHtml(t.name||'?')}</td>
            <td>${(t.stack_hwm||0).toLocaleString()} B</td>
            <td>${t.core >= 0 ? t.core : 'any'}</td>
            <td>${t.prio}</td>
          </tr>`).join('')}
        </tbody>
      </table>
    </div>

    <div class="diag-section">
      <h3>Poller / cycle</h3>
      <div class="diag-kv-grid">
        ${kvRow('Cycles', pol.cycles_completed||0)}
        ${kvRow('Cycle avg', (pol.cycle_avg_ms||0) + ' ms')}
        ${kvRow('Cycle max', (pol.cycle_max_ms||0) + ' ms')}
        ${kvRow('Snapshot bus (pub / read / retry)',
                (sn.publishes||0) + ' / ' + (sn.reads||0) + ' / ' + (sn.retries||0))}
      </div>
    </div>

    <div class="diag-section">
      <h3>RS485 bus</h3>
      <div class="diag-kv-grid" style="margin-bottom:12px">
        ${kvRow('Analog polls (ok / timeout / parse err)',
                (pol.rs485_ok||0) + ' / ' + (pol.rs485_timeouts||0) + ' / ' + (pol.rs485_parse_err||0),
                'Aggregate 0x42 measurement-frame polls across all packs')}
        ${kvRow('Alarm polls (ok / err)', (pol.alarm_polls_ok||0) + ' / ' + (pol.alarm_polls_err||0),
                '0x44 alarm-frame polls')}
        ${kvRow('Sysparam polls (ok / err)', (pol.sysparam_polls_ok||0) + ' / ' + (pol.sysparam_polls_err||0),
                '0x47 sysparam polls; freshness drives the Auto battery-config modes')}
        ${kvRow('Wrong-address frames', pol.wrong_addr||0,
                'Checksum-valid frames whose responder address ≠ the polled pack; discarded, never mis-attributed')}
      </div>
      ${diagRs485Table(packs)}
    </div>

    <div class="diag-section">
      <h3>Battery / BMS (per pack)</h3>
      ${diagBatteryTable(packs)}
      <div style="color:var(--text-muted);font-size:11px;margin-top:6px">
        CCL/DCL/Cell-OV limit are the pack's own 0x47 sysparam values (Auto modes). A highlighted
        row has a non-zero 0x44 alarm bitmap. Aggregated limits sent to the inverter are in the CAN section.
      </div>
    </div>

    <div class="diag-section">
      <h3>CAN → inverter</h3>
      <div class="diag-kv-grid" style="margin-bottom:12px">
        ${kvRow('Active protocol', (can.protocol||'—').toUpperCase())}
        ${kvRow('Modules on CAN (0x359 byte4)', bat.packs_online != null ? bat.packs_online : '—',
                'Packs reported online to the inverter; should match the RS485 online count above')}
        ${kvRow('Charge voltage limit CVL (0x351)', bat.has_data ? Number(bat.cvl_volts||0).toFixed(2) + ' V' : '—')}
        ${kvRow('Charge current limit CCL (0x351)', bat.has_data ? Number(bat.ccl_amps||0).toFixed(1) + ' A' : '—')}
        ${kvRow('Discharge current limit DCL (0x351)', bat.has_data ? Number(bat.dcl_amps||0).toFixed(1) + ' A' : '—')}
        ${kvRow('Discharge voltage limit DVL (0x351)', bat.has_data ? Number(bat.dvl_volts||0).toFixed(2) + ' V' : '—')}
        ${kvRow('Charge enable (0x35C bit7)', bat.has_data ? ((bat.ccl_amps||0) >= 0.1 ? 'yes' : 'NO') : '—',
                'Derived exactly as the encoder: ccl ≥ 0.1 A')}
        ${kvRow('Discharge enable (0x35C bit6)', bat.has_data ? ((bat.dcl_amps||0) >= 0.1 ? 'yes' : 'NO') : '—',
                'Derived exactly as the encoder: dcl ≥ 0.1 A. \"NO\" here is why an inverter stops discharging.')}
        ${kvRow('Force charge (0x35C bit5)', bat.has_data ? (((bat.alarm_flags||0) & 0x10) ? 'yes' : 'no') : '—',
                'Requested on undervolt only')}
        ${kvRow('Protection lockout', bat.has_data ? (function() {
            const lk = bat.lockout_flags || 0;
            const parts = [];
            if (lk & 0x01) parts.push('charge: ' + lockoutReason(0x01, bat.alarm_flags, bat.temp_alarm));
            if (lk & 0x02) parts.push('discharge: ' + lockoutReason(0x02, bat.alarm_flags, bat.temp_alarm));
            return parts.join(' | ') || 'none';
          })() : '—',
            'Direction-aware: over-voltage blocks charge only, under-voltage blocks discharge only')}
        ${kvRow('Active alarms (0x359)', bat.has_data ? (decodeAlarmFlags(bat.alarm_flags).join('; ') || 'none') : '—')}
        ${(bat.has_data && (bat.temp_alarm||0)) ? kvRow('Temperature direction', decodeTempAlarm(bat.temp_alarm).join('; ')) : ''}
        ${bat.has_data && bat.sys_message ? kvRow('Safety message', bat.sys_message) : ''}
      </div>
      <div class="diag-kv-grid">
        ${kvRow('TX ok', can.tx_ok||0)}
        ${kvRow('TX fail', can.tx_fail||0)}
        ${kvRow('TX fail streak (max)', can.tx_fail_streak_max||0,
                'Longest run of consecutive transmit failures since boot')}
        ${kvRow('Heartbeats', can.heartbeats||0)}
        ${kvRow('Express sends', can.express_sends||0)}
        ${kvRow('Bus-off count', can.bus_off_count||0)}
        ${kvRow('Driver restarts', can.driver_restarts||0)}
      </div>
    </div>

    <div class="diag-section">
      <h3>MQTT</h3>
      <div class="diag-kv-grid">
        ${kvRow('Enabled', mq.enabled ? 'yes' : 'no')}
        ${kvRow('State', mq.state||'—')}
        ${kvRow('Base topic', mq.effective_base || '—')}
        ${kvRow('Publish ok', mq.publish_ok||0)}
        ${kvRow('Publish fail', mq.publish_fail||0)}
        ${kvRow('Drops', mq.publish_drops||0)}
        ${kvRow('Publish max', (mq.publish_max_ms||0) + ' ms')}
      </div>
    </div>

    <div class="diag-section">
      <h3>Time</h3>
      <div class="diag-kv-grid">
        ${kvRow('Device time', deviceTime)}
        ${kvRow('NTP synced', ntp.synced ? 'yes' : 'no')}
        ${kvRow('NTP server', ntp.server||'—')}
      </div>
    </div>

    <div class="diag-section">
      <h3>LittleFS / History / Energy</h3>
      <div class="diag-kv-grid">
        ${kvRow('LFS total', fmtMb(lfs.total_b), (lfs.total_b||0) + ' B')}
        ${kvRow('LFS used', fmtMb(lfs.used_b), (lfs.used_b||0) + ' B')}
        ${kvRow('LFS free', fmtMb(lfs.free_b), (lfs.free_b||0) + ' B')}
        ${kvRow('Fine samples', his.fine_samples||0)}
        ${kvRow('Coarse samples', his.coarse_samples||0)}
        ${kvRow('Today (in / out)', (ene.today_in_kwh||0).toFixed(2) + ' / ' + (ene.today_out_kwh||0).toFixed(2) + ' kWh')}
        ${kvRow('Total (in / out)', (ene.total_in_kwh||0).toFixed(2) + ' / ' + (ene.total_out_kwh||0).toFixed(2) + ' kWh')}
        ${kvRow('Stored alerts', d.alerts_count||0)}
      </div>
    </div>

    ${(function() {
      if (!d.ble_status) return '';
      const mppt  = ble.mppt  || {};
      const shunt = ble.shunt || {};
      const dbg   = ble.ble_debug || {};
      const fmtF  = (v, dec) => (v == null ? '—' : Number(v).toFixed(dec));

      // MPPT filter funnel — the fastest way to see WHY decoding fails
      // (wrong MAC, wrong record type, wrong encryption key).
      const funnelHtml = ble.ble_active ? `
      <div class="diag-kv-grid">
        ${kvRow('Configured MPPT MAC', dbg.configured_mac || '—')}
        ${kvRow('MAC valid', dbg.mppt_mac_valid ? 'yes' : 'no')}
        ${kvRow('Key valid', dbg.mppt_key_valid ? 'yes' : 'NO — re-enter encryption key')}
        ${kvRow('Filter funnel (Victron -> type -> MAC -> decrypt)',
                (dbg.victron_total||0) + ' -> ' + (dbg.mppt_type_match||0) + ' -> ' +
                (dbg.mppt_mac_match||0) + ' -> ' + (dbg.mppt_decrypt_ok||0))}
      </div>` : '';

      return `
    <div class="diag-section">
      <h3>Bluetooth LE</h3>
      <div class="diag-kv-grid">
        ${kvRow('Active', ble.ble_active ? 'yes' : 'no')}
        ${kvRow('Stack', ble.stack || '—')}
        ${kvRow('Advertisements (total / Victron / MPPT)',
                `${(ble.ble_gap_events||0).toLocaleString()} / ${(ble.ble_victron_advs||0).toLocaleString()} / ${(ble.ble_mppt_advs||0).toLocaleString()}`)}
      </div>
      ${funnelHtml}
    </div>

    <div class="diag-section">
      <h3>WiFi</h3>
      <div class="diag-kv-grid">
        ${kvRow('SSID', ble.wifi_ssid || '—')}
        ${kvRow('IP', ble.wifi_ip || '—')}
        ${kvRow('Hostname', ble.wifi_hostname || '—')}
        ${kvRow('Connected for', formatUptime(ble.wifi_connected_for_s))}
        ${kvRow('Disconnects', ble.wifi_disconnects||0)}
        ${kvRow('BSSID', ble.wifi_bssid || '—')}
        ${kvRow('RSSI', ble.wifi_rssi != null ? ble.wifi_rssi + ' dBm' + rssiQuality(ble.wifi_rssi) : '—')}
        ${kvRow('BSSID pin', bssidPinStatus(ble.wifi_bssid_pin_active).text)}
      </div>
    </div>

    <div class="diag-section">
      <h3>MPPT</h3>
      ${mppt.enabled ? `
      <div class="diag-kv-grid">
        ${kvRow('Last seen', fmtAgeS(mppt.last_seen_s))}
        ${kvRow('PV input power', fmtF(mppt.pv_power_w, 1) + ' W')}
        ${kvRow('Charger output voltage', fmtF(mppt.batt_voltage_v, 2) + ' V')}
        ${kvRow('Charger output current', fmtF(mppt.batt_current_a, 2) + ' A')}
        ${kvRow('Charger state', mppt.seen ? csLabel(mppt.charge_state) : '—')}
        ${kvRow('Yield today', mppt.yield_today_wh != null ? (mppt.yield_today_wh / 1000).toFixed(2) + ' kWh' : '—')}
      </div>` : `<div style="color:var(--text-muted);font-size:12px">MPPT disabled</div>`}
    </div>

    <div class="diag-section">
      <h3>Shunt</h3>
      ${shunt.enabled ? `
      <div class="diag-kv-grid">
        ${kvRow('Configured Shunt MAC', dbg.configured_shunt_mac || '—')}
        ${kvRow('MAC valid', dbg.shunt_mac_valid ? 'yes' : 'no')}
        ${kvRow('Key valid', dbg.shunt_key_valid ? 'yes' : 'NO — re-enter encryption key')}
        ${kvRow('Filter funnel (Victron -> type -> MAC -> decrypt)',
                (dbg.victron_total||0) + ' -> ' + (dbg.shunt_type_match||0) + ' -> ' +
                (dbg.shunt_mac_match||0) + ' -> ' + (dbg.shunt_decrypt_ok||0))}
        ${(dbg.shunt_mac_match||0) > 0 && (dbg.shunt_decrypt_ok||0) === 0 ? kvRow(
                'Last matched ad (diagnosing decrypt failure)',
                (dbg.shunt_last_mfg_len||0) + ' bytes, ' + (dbg.shunt_last_new_fmt ? 'new format' : 'old format') +
                ' (needs ≥ ' + (dbg.shunt_last_new_fmt ? '25' : '12') + ' bytes)') : ''}
        ${kvRow('Last seen', fmtAgeS(shunt.last_seen_s))}
        ${kvRow('Current', fmtF(shunt.current_a, 3) + ' A')}
        ${kvRow('BMS total current (cross-check)', fmtF(ble.bms_current_a, 3) + ' A')}
        ${kvRow('Voltage', fmtF(shunt.voltage_v, 2) + ' V')}
        ${kvRow('SOC', shunt.soc_valid === false ? 'Not synced (raw sentinel)' : fmtF(shunt.soc_pct, 1) + ' %')}
        ${kvRow('Consumed Ah (shunt counter, reference only)', shunt.consumed_ah_valid === false ? 'Not available (raw sentinel)' : fmtF(shunt.consumed_ah, 1) + ' Ah')}
        ${kvRow('BMS SOC avg (cross-check)', fmtF(g_live && g_live.safety && g_live.safety.soc_avg, 0) + ' %')}
        ${kvRow('Active source (bank SOC)', ((g_live && g_live.sources && g_live.sources.battery_soc_src) || 'bms').toUpperCase())}
      </div>` : `<div style="color:var(--text-muted);font-size:12px">Shunt disabled — enable in Settings &rarr; Bluetooth LE</div>`}
    </div>`;
    })()}

    ${(function() {
      const cd = d.coredump || {};
      if (!cd.present) return '';
      return `<div class="diag-section">
      <h3>Previous panic (coredump)</h3>
      <div class="diag-kv-grid">
        ${kvRow('Crashing task', cd.crashing_task || '—')}
        ${cd.exc_pc   ? kvRow('Exception PC', cd.exc_pc) : ''}
        ${cd.build_sha256 ? kvRow('Build (SHA256)', cd.build_sha256.slice(0,16) + '…') : ''}
      </div>
      <div style="margin-top:10px">
        <a href="/api/diag/coredump.bin" class="btn btn-secondary" download="coredump.bin">
          Download coredump
        </a>
      </div>
    </div>`;
    })()}

    <details id="diag-log-details" class="diag-section diag-log-details">
      <summary class="diag-log-summary">Log (last ${(d.log_ring||[]).length} lines)</summary>
      <div class="diag-log-box" id="diag-log">
        ${(d.log_ring||[]).map(l => escHtml(l)).join('\n')}
      </div>
    </details>

    `;

  // Restore log open/closed state (innerHTML replacement resets <details> to closed).
  const details = document.getElementById('diag-log-details');
  if (details) {
    if (g_diag_log_open) details.open = true;
    details.addEventListener('toggle', () => { g_diag_log_open = details.open; }, { once: true });
  }

  // Scroll log to bottom when visible.
  const logBox = document.getElementById('diag-log');
  if (logBox && g_diag_log_open) logBox.scrollTop = logBox.scrollHeight;
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
window.addEventListener('popstate', () => {
  renderPage(location.pathname);
  updateSidebarActive(location.pathname);
});

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

/* ── Auth toggle ─────────────────────────────────────────────────────────────── */
function confirmDisableAuth() {
  const overlay = document.getElementById('modal-overlay');
  document.getElementById('modal-title').textContent = 'Disable authentication?';
  document.getElementById('modal-body').textContent =
    'This will turn off login protection. Anyone on your network will be able to view live data, ' +
    'change settings, and trigger a factory reset. This is intended for trusted networks only.';
  const confirmBtn = document.getElementById('modal-confirm');
  confirmBtn.textContent = 'Disable';
  confirmBtn.style.display = '';
  overlay.style.display = 'flex';
  confirmBtn.onclick = async () => {
    overlay.style.display = 'none';
    await setAuthEnabled(false);
  };
}

async function enableAuth() {
  await setAuthEnabled(true);
}

async function setAuthEnabled(enabled) {
  const msg = document.getElementById('auth-toggle-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }
  try {
    const cfg = Object.assign({}, g_config);
    cfg.auth_enabled = enabled;
    cfg.auth_hash = '';        // redacted — server keeps existing hash
    cfg.mqtt_pass_obf = '';    // redacted
    const r = await apiFetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg),
    });
    if (!r) return;
    const data = await r.json();
    if (r.ok) {
      g_config = data;
      // Re-render just the account section (or full settings if layout not present).
      if (document.getElementById('settings-content')) {
        renderSettingsSection('account');
      } else {
        renderSettings();
      }
    } else {
      if (msg) {
        msg.className = 'feedback-msg err';
        msg.textContent = data.error || 'Failed to update authentication state.';
      }
    }
  } catch (e) {
    if (msg) {
      msg.className = 'feedback-msg err';
      msg.textContent = 'Network error: ' + e.message;
    }
  }
}

/* ── Config import (restore) ─────────────────────────────────────────────────── */

let g_restore_backup = null;  // parsed backup JSON object

function restoreFileChanged(event) {
  const file = event.target.files && event.target.files[0];
  const infoEl   = document.getElementById('restore-file-info');
  const importBtn = document.getElementById('restore-import-btn');
  const metaEl   = document.getElementById('restore-metadata');
  const statusEl = document.getElementById('restore-status');
  g_restore_backup = null;
  if (importBtn) importBtn.disabled = true;
  if (metaEl)   metaEl.innerHTML = '';
  if (statusEl) { statusEl.className = 'feedback-msg'; statusEl.textContent = ''; }
  if (!file) { if (infoEl) infoEl.textContent = 'No file selected'; return; }
  if (infoEl) infoEl.textContent = file.name;

  const reader = new FileReader();
  reader.onload = function(e) {
    let parsed;
    try {
      parsed = JSON.parse(e.target.result);
    } catch (_) {
      if (metaEl) metaEl.innerHTML =
        '<p style="color:var(--color-alarm);font-size:13px;margin-top:8px">Invalid JSON — not a valid backup file.</p>';
      return;
    }
    renderRestoreMetadata(parsed);
  };
  reader.readAsText(file);
}

function renderRestoreMetadata(parsed) {
  const metaEl    = document.getElementById('restore-metadata');
  const importBtn = document.getElementById('restore-import-btn');
  if (!metaEl) return;

  const formatOk   = (parsed._format === 'topband-bms-config');
  const metaExported = parsed._exported || null;
  const metaSchema   = (parsed.config && parsed.config.schema_version !== undefined)
                       ? parsed.config.schema_version : null;
  const metaFirmware = parsed._firmware || null;
  const metaDevice   = parsed._device   || null;

  const currentSchema = g_config ? g_config.schema_version : null;

  let schemaOk = true, schemaNote = '';
  if (!formatOk) {
    schemaOk = false; schemaNote = 'N/A';
  } else if (metaSchema === null) {
    schemaOk = false; schemaNote = 'Missing schema_version field';
  } else if (currentSchema !== null && metaSchema > currentSchema) {
    schemaOk = false;
    schemaNote = 'v' + metaSchema + ' is newer than firmware (v' + currentSchema + ') — update firmware first';
  } else if (currentSchema !== null && metaSchema < currentSchema) {
    schemaOk = true;
    schemaNote = 'v' + metaSchema + ' — will migrate to v' + currentSchema;
  } else {
    schemaOk = true;
    schemaNote = 'v' + (metaSchema !== null ? metaSchema : '?') + ' — compatible';
  }

  const allOk = formatOk && schemaOk;
  g_restore_backup = allOk ? parsed : null;
  if (importBtn) importBtn.disabled = !allOk;

  function checkRow(label, ok, note) {
    const c = ok ? 'var(--color-success)' : 'var(--color-alarm)';
    return '<div style="display:flex;gap:8px;padding:3px 0;font-size:12px;border-bottom:1px solid var(--border-subtle)">' +
      '<span style="color:' + c + ';font-weight:700;min-width:14px">' + (ok ? 'OK' : 'FAIL') + '</span>' +
      '<span style="color:var(--text-muted);min-width:130px">' + escHtml(label) + '</span>' +
      '<span style="color:var(--text-primary);flex:1">' + escHtml(note) + '</span>' +
      '</div>';
  }

  metaEl.innerHTML =
    '<div style="border:1px solid var(--border-subtle);border-radius:6px;padding:10px 12px;margin-top:8px">' +
      '<div style="font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.04em;color:var(--text-muted);margin-bottom:6px">Backup Details</div>' +
      '<div class="diag-kv-grid" style="margin-bottom:10px">' +
        kvRow('Exported',      metaExported              || 'unknown') +
        kvRow('Schema',        metaSchema !== null ? 'v' + metaSchema : 'unknown') +
        kvRow('Firmware',      metaFirmware              || 'unknown') +
        kvRow('Source device', metaDevice                || 'unknown') +
      '</div>' +
      '<div style="font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.04em;color:var(--text-muted);margin-bottom:4px">Validation</div>' +
      checkRow('Format',         formatOk, formatOk ? 'topband-bms-config' : 'Not a TopBand BMS backup — wrong _format') +
      checkRow('Schema compat',  schemaOk, schemaNote) +
      checkRow('Value ranges',   allOk,    allOk ? 'Will be checked on import' : 'N/A') +
      '<div style="margin-top:8px;font-size:12px;font-weight:600;color:' +
        (allOk ? 'var(--color-success)' : 'var(--color-alarm)') + '">' +
        (allOk ? 'File looks valid — click "Import backup" to continue.' : 'Import blocked — see failures above.') +
      '</div>' +
    '</div>';
}

function startRestore() {
  if (!g_restore_backup) return;
  const overlay = document.getElementById('modal-overlay');
  const confirmBtn = document.getElementById('modal-confirm');
  const schema   = (g_restore_backup.config && g_restore_backup.config.schema_version !== undefined)
                   ? g_restore_backup.config.schema_version : '?';
  const exported = g_restore_backup._exported || 'unknown date';
  const device   = g_restore_backup._device   || 'unknown';

  document.getElementById('modal-title').textContent = 'Import Backup';
  document.getElementById('modal-body').innerHTML = `
    <p style="font-size:12px;color:var(--text-muted);margin-bottom:10px">
      Exported: <strong>${escHtml(exported)}</strong>
      &nbsp;|&nbsp; Schema: v${escHtml(String(schema))}
      &nbsp;|&nbsp; From device: ${escHtml(device)}
    </p>
    <p style="font-size:13px;font-weight:600;margin-bottom:6px">Import scope:</p>
    <label style="display:flex;gap:8px;align-items:flex-start;margin-bottom:6px;cursor:pointer">
      <input type="radio" name="restore_scope" value="settings" checked style="margin-top:2px;width:auto">
      <span><strong>Settings only</strong> (recommended)<br>
        <span style="font-size:12px;color:var(--text-muted)">Imports all settings except hardware configuration (board preset, pin assignments, RS485). Safe to use when moving settings to a different device.</span>
      </span>
    </label>
    <label style="display:flex;gap:8px;align-items:flex-start;margin-bottom:12px;cursor:pointer">
      <input type="radio" name="restore_scope" value="hardware" style="margin-top:2px;width:auto">
      <span><strong>Settings + Hardware</strong><br>
        <span style="font-size:12px;color:var(--text-muted)">Also imports board preset and pin assignments. Only enable this if the target device uses the same board and wiring as the backup source. On different hardware this can disable RS485/CAN.</span>
      </span>
    </label>
    <div style="background:color-mix(in srgb,var(--color-warning,#E8A44A) 12%,transparent);border:1px solid color-mix(in srgb,var(--color-warning,#E8A44A) 40%,transparent);border-radius:6px;padding:8px 10px;font-size:12px">
      Passwords (MQTT, authentication) are NOT included in backups. You will need to re-enter them after the device reboots.
    </div>
  `;
  confirmBtn.textContent = 'Import & Reboot';
  confirmBtn.style.display = '';
  overlay.style.display = 'flex';

  confirmBtn.onclick = () => {
    overlay.style.display = 'none';
    const scopeEl = document.querySelector('input[name="restore_scope"]:checked');
    const includeHardware = scopeEl && scopeEl.value === 'hardware';
    doRestore(includeHardware);
  };
}

async function doRestore(includeHardware) {
  const statusEl = document.getElementById('restore-status');
  if (statusEl) { statusEl.className = 'feedback-msg'; statusEl.textContent = 'Importing…'; }

  try {
    const r = await apiFetch('/api/restore', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ backup: g_restore_backup, include_hardware: includeHardware }),
    });
    if (!r) return;
    const data = await r.json().catch(() => ({}));
    if (r.ok) {
      showPageOverlay('Import successful — rebooting…',
        'The gateway is applying imported settings. After it restarts, re-enter any passwords (MQTT, authentication) that were not included in the backup.');
      setTimeout(pollUntilOnline, 4000);
    } else {
      if (statusEl) { statusEl.className = 'feedback-msg err'; statusEl.textContent = data.error || 'Import failed.'; }
    }
  } catch (e) {
    if (statusEl) { statusEl.className = 'feedback-msg err'; statusEl.textContent = 'Network error: ' + e.message; }
  }
}

/* ── MQTT connection test ─────────────────────────────────────────────────────── */

let g_mqtt_test_poll = null;

async function testMqttConnection() {
  const resultEl = document.getElementById('mqtt-test-result');
  if (!resultEl) return;

  // Stop any running poll.
  if (g_mqtt_test_poll) { clearInterval(g_mqtt_test_poll); g_mqtt_test_poll = null; }

  // Read current form values (unsaved).
  const host      = (document.getElementById('cfg-mqtt_host')       || {}).value || '';
  const port      = Number((document.getElementById('cfg-mqtt_port') || {}).value || 1883);
  const user      = (document.getElementById('cfg-mqtt_user')        || {}).value || '';
  const passEl    = document.getElementById('cfg-mqtt_pass');
  const pass      = passEl ? passEl.value : '';
  const baseTopic = (document.getElementById('cfg-mqtt_base_topic')  || {}).value || 'topband-bms';

  if (!host) {
    resultEl.style.display = 'block';
    resultEl.style.borderColor = 'var(--color-alarm)';
    resultEl.style.color = 'var(--color-alarm)';
    resultEl.textContent = 'Enter a broker host first.';
    return;
  }

  resultEl.style.display = 'block';
  resultEl.style.borderColor = 'var(--border)';
  resultEl.style.color = 'var(--text-muted)';
  resultEl.innerHTML = '<span class="spinner"></span> Testing connection…' +
    (pass === '' ? ' <em style="font-size:11px">(using saved password)</em>' : '');

  try {
    const r = await apiFetch('/api/mqtt/test', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ host, port, user, pass, base_topic: baseTopic }),
    });
    if (!r) return;
    if (!r.ok) {
      const d = await r.json().catch(() => ({}));
      resultEl.style.color = 'var(--color-alarm)';
      resultEl.style.borderColor = 'var(--color-alarm)';
      resultEl.textContent = d.error || 'Failed to start test.';
      return;
    }
  } catch (e) {
    resultEl.style.color = 'var(--color-alarm)';
    resultEl.style.borderColor = 'var(--color-alarm)';
    resultEl.textContent = 'Network error: ' + e.message;
    return;
  }

  // Poll until done (max ~12 s at 1 s intervals).
  let polls = 0;
  g_mqtt_test_poll = setInterval(async function() {
    polls++;
    if (polls > 12) {
      clearInterval(g_mqtt_test_poll); g_mqtt_test_poll = null;
      resultEl.style.color = 'var(--color-alarm)'; resultEl.style.borderColor = 'var(--color-alarm)';
      resultEl.textContent = 'Test timed out — no result after 12 s.';
      return;
    }
    try {
      const r2 = await apiFetch('/api/mqtt/test');
      if (!r2) return;
      const d = await r2.json();
      if (d.status === 'running') return;  // still in progress
      clearInterval(g_mqtt_test_poll); g_mqtt_test_poll = null;
      if (d.status === 'ok') {
        resultEl.style.color = 'var(--color-success)';
        resultEl.style.borderColor = 'var(--color-success)';
        resultEl.textContent = d.message || 'Connection OK';
      } else {
        resultEl.style.color = 'var(--color-alarm)';
        resultEl.style.borderColor = 'var(--color-alarm)';
        const stageLabel = { tcp: 'TCP connect', auth: 'Authentication', publish: 'Publish' };
        const prefix = stageLabel[d.stage] ? stageLabel[d.stage] + ' failed — ' : '';
        resultEl.textContent = prefix + (d.message || 'Test failed');
      }
    } catch (e) { /* ignore transient poll errors */ }
  }, 1000);
}

/* ── HA Discovery ───────────────────────────────────────────────────────────── */
async function sendHaDiscovery() {
  const msg = document.getElementById('mqtt-feedback');
  if (msg) { msg.className = 'feedback-msg'; msg.textContent = ''; }
  try {
    const r = await apiFetch('/api/svc/ha/discovery/send', { method: 'POST' });
    if (!r) return;
    if (r.ok) {
      showToast('HA discovery sent');
    } else {
      const data = await r.json().catch(() => ({}));
      const errText = data.error || 'Failed to send HA discovery';
      showToast(errText, 'warn');
      if (msg) { msg.className = 'feedback-msg err'; msg.textContent = errText; }
    }
  } catch (e) {
    showToast('Network error: ' + e.message, 'warn');
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

/* ── Auth-disabled banner (reactive on each health/live poll) ─────────────── */
function dismissAuthBanner() {
  sessionStorage.setItem('auth_banner_dismissed', 'true');
  const banner = document.getElementById('auth-banner');
  if (banner) banner.style.display = 'none';
}

function updateAuthBanner() {
  // Use g_health.auth_enabled when available; avoids a separate /api/config call.
  const authEnabled = g_health ? g_health.auth_enabled : undefined;
  if (authEnabled === undefined) return;
  const banner    = document.getElementById('auth-banner');
  const logoutBtn = document.getElementById('logout-btn');
  if (banner) {
    const dismissed = sessionStorage.getItem('auth_banner_dismissed') === 'true';
    banner.style.display = (authEnabled === false && !dismissed) ? 'flex' : 'none';
  }
  if (logoutBtn) logoutBtn.style.display = authEnabled !== false ? 'inline-flex' : 'none';
}

async function checkAuthState() {
  // Called once at boot; subsequent updates come from updateAuthBanner() on each poll.
  updateAuthBanner();
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

  // Render current page and sync sidebar.
  renderPage(location.pathname);
  updateSidebarActive(location.pathname);

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

/* ── Drift Details ──────────────────────────────────────────────────────────── */

// Voltage scale for cell bars: LiFePO4 typical operating range.
const DRIFT_FLOOR_MV = 3200;
const DRIFT_CEIL_MV  = 3680;
const DRIFT_RANGE_MV = DRIFT_CEIL_MV - DRIFT_FLOOR_MV;  // 480 mV

// Guide lines at balance-reference, near-full, and OVP-approach voltages.
const DRIFT_GUIDES = [
  { mv: 3350, label: '3.35' },
  { mv: 3450, label: '3.45' },
  { mv: 3600, label: '3.60' },
];

function driftPct(mv) {
  return Math.max(0, Math.min(100, (mv - DRIFT_FLOOR_MV) / DRIFT_RANGE_MV * 100));
}

function driftBandColor(spreadMv) {
  if (spreadMv >= 110) return 'var(--brand-coral)';
  if (spreadMv >= 40)  return 'var(--color-warning)';
  return 'var(--brand-teal)';
}

function driftStatusLabel(spreadMv) {
  if (spreadMv >= 110) return 'Imbalanced';
  if (spreadMv >= 40)  return 'Monitor drift';
  return 'Balanced';
}

function driftStatusCls(spreadMv) {
  if (spreadMv >= 110) return 'drift-pill-imbalanced';
  if (spreadMv >= 40)  return 'drift-pill-monitor';
  return 'drift-pill-balanced';
}

function driftRateStr(rate) {
  if (rate === null || rate === undefined || isNaN(rate)) return '—';
  const r = Math.round(rate * 10) / 10;
  return (r >= 0 ? '+' : '') + r.toFixed(1);
}

async function loadDriftDetails() {
  if (!document.getElementById('drift-section-root')) return;
  try {
    const r = await apiFetch('/api/drift');
    if (!r || !r.ok) return;
    g_drift_data = await r.json();
    renderDriftSection();
  } catch (_) {}
}

function renderDriftSection() {
  const root = document.getElementById('drift-section-root');
  if (!root || !g_drift_data) return;

  const packs = g_drift_data.packs || [];
  if (packs.length === 0) {
    root.innerHTML = '<p style="padding:16px 4px;color:var(--text-muted);font-size:13px">No online packs — drift data will appear once packs connect.</p>';
    return;
  }

  root.innerHTML = '';
  packs.forEach(pack => root.appendChild(buildDriftCard(pack)));
}

function buildDriftCard(pack) {
  // Pill always reflects the live (now) spread so it matches the battery cards.
  // Historical at-full spread is shown as separate labeled context below.
  const liveSpread = pack.spread_now || 0;
  const isOpen  = g_drift_open.has(pack.id);
  // Rate is meaningful from 2 full-charge days on; show progress before that.
  const nToc = pack.n_toc_days || 0;
  const rateHtml = (nToc >= 2)
    ? '<strong>' + driftRateStr(pack.drift_rate) + '</strong>&thinsp;mV/day'
    : 'trend&thinsp;<strong>' + nToc + '/2</strong>&thinsp;days';

  const wrapper = document.createElement('div');
  wrapper.className = 'drift-pack-card';

  const pillLabel  = driftStatusLabel(liveSpread);
  const atFullStr  = pack.has_toc ? (pack.toc_spread || 0) + '&thinsp;mV' : '—';

  // Toggle header: Live pill + three clearly labeled metrics
  const toggle = document.createElement('div');
  toggle.className = 'drift-toggle';
  toggle.dataset.driftToggle = pack.id;
  toggle.innerHTML =
    '<span class="drift-pack-name">' + escHtml(pack.name) + '</span>' +
    '<div class="drift-summary-items">' +
      '<span class="drift-pill ' + driftStatusCls(liveSpread) + '">' + pillLabel + '</span>' +
      '<span class="drift-meta">Live&thinsp;<strong>' + liveSpread + '</strong>&thinsp;mV</span>' +
      '<span class="drift-summary-sep">&middot;</span>' +
      '<span class="drift-meta">At full&thinsp;<strong>' + atFullStr + '</strong></span>' +
      '<span class="drift-summary-sep">&middot;</span>' +
      '<span class="drift-meta">' + rateHtml + '</span>' +
    '</div>' +
    '<span class="drift-chevron' + (isOpen ? ' open' : '') + '">▼</span>';
  toggle.addEventListener('click', () => toggleDriftPack(pack.id));
  wrapper.appendChild(toggle);

  // Expanded body
  const body = document.createElement('div');
  body.className = 'drift-body';
  body.dataset.driftBody = pack.id;
  body.style.display = isOpen ? '' : 'none';
  body.innerHTML = pack.has_history
    ? buildDriftBodyHtml(pack)
    : buildDriftNoHistoryHtml(pack);
  wrapper.appendChild(body);

  return wrapper;
}

function buildDriftNoHistoryHtml(pack) {
  return '<p class="drift-building">History building — 5-day band appears after the first day accrues. Live cell positions shown below.</p>' +
    buildDriftCellRowsHtml(pack, true);
}

// Repetition rule (review 2.3): a first-full / first-empty cell is shown only
// when the SAME cell won the per-day argmax on >= 2 days AND holds a majority
// of the region days in the window. One-day winners are trivia and stay hidden.
function driftRepeatWinner(modeIdx, daysWon, daysTotal) {
  if (!daysTotal || !daysWon || daysWon < 2) return null;
  if (daysWon * 2 <= daysTotal) return null;
  return { cell: 'C' + (modeIdx + 1), won: daysWon, total: daysTotal };
}

function buildDriftBodyHtml(pack) {
  const cells = pack.cells || [];
  const nc    = Math.min(pack.cell_count || cells.length, 15);

  // Conditional warning strip (review 2.4): cells at/above 3.60 V right now.
  // Zero is the expected state and renders nothing; nonzero is urgent.
  const highCells = cells.slice(0, nc)
    .map((c, i) => ({ i, mv: c.now || 0 }))
    .filter(x => x.mv >= 3600);
  let stripHtml = '';
  if (highCells.length > 0) {
    const crit = highCells.some(x => x.mv >= 3650);
    const list = highCells.map(x => 'C' + (x.i + 1) + ': ' + x.mv + ' mV').join(', ');
    stripHtml = '<div class="drift-warn-strip' + (crit ? ' crit' : '') + '">' +
      highCells.length + (highCells.length === 1 ? ' cell' : ' cells') +
      ' at or above 3.60&thinsp;V (' + escHtml(list) + ')</div>';
  }

  // KPI 1: balance now (anchor tile; matches the header pill).
  const liveSpread = pack.spread_now || 0;
  const kpiNow = driftKpiSub('Balance now', liveSpread + ' mV', driftStatusLabel(liveSpread));

  // KPI 2: at full — worst 5-day spread at top-of-charge, with the worst cell.
  let kpiFull;
  if (pack.has_toc) {
    const worst = (pack.first_full_mv > 0) ? 'worst cell C' + (pack.first_full_idx + 1) : '';
    kpiFull = driftKpiSub('At full', (pack.toc_spread || 0) + ' mV', worst);
  } else {
    kpiFull = driftKpiSub('At full', '—', 'no full charge in 5 days');
  }

  // KPI 3: trend — a rate once 2 full-charge days exist, progress before that
  // (review 2.1: an indefinite "building" reads as a stuck spinner).
  const nToc = pack.n_toc_days || 0;
  let kpiTrend;
  if (nToc >= 2) {
    kpiTrend = driftKpiSub('Trend', driftRateStr(pack.drift_rate) + ' mV/day',
                           'between full-charge days');
  } else {
    kpiTrend = driftKpiSub('Trend', nToc + ' of 2', 'full-charge days collected');
  }

  // Optional KPI: at empty — rendered only when low-SoC data exists
  // (hide-until-data policy, review 2.2). A solar setup may never dip below
  // 15% SoC; a permanent placeholder would be a false promise.
  let kpiEmpty = '';
  if (pack.has_bod) {
    const feWin = driftRepeatWinner(pack.fe_mode_idx, pack.fe_days_won, pack.fe_days_total);
    const sub = feWin
      ? feWin.cell + ' empties first (' + feWin.won + ' of ' + feWin.total + ' days)'
      : (pack.first_empty_mv > 0 ? 'worst cell C' + (pack.first_empty_idx + 1) : '');
    kpiEmpty = driftKpiSub('At empty', (pack.bod_spread || 0) + ' mV', sub);
  }

  // Repetition callout for the fills-first pattern (review 2.3): shown only
  // when the same cell wins repeatedly; a one-day winner is omitted entirely.
  const ffWin = driftRepeatWinner(pack.ff_mode_idx, pack.ff_days_won, pack.ff_days_total);
  let calloutHtml = '';
  if (ffWin) {
    calloutHtml = '<div class="drift-repeat-note">' + ffWin.cell + ' fills first (' +
      ffWin.won + ' of ' + ffWin.total + ' full-charge days)</div>';
  }

  // Capability note when the at-empty region has never been visited.
  const bodNote = pack.has_bod ? '' :
    '<div class="drift-cap-note">No low-SoC data in the 5-day window. ' +
    'At-empty stats appear if SoC dips below 15&thinsp;%.</div>';

  return stripHtml +
    '<div class="drift-kpis">' + kpiNow + kpiFull + kpiTrend + kpiEmpty + '</div>' +
    calloutHtml + bodNote +
    buildDriftCellRowsHtml(pack, false);
}

// KPI with an optional muted sub-label (used for "no data yet" states).
function driftKpiSub(label, value, sub) {
  const subHtml = sub ? '<div class="drift-kpi-sub">' + sub + '</div>' : '';
  return '<div class="drift-kpi"><div class="drift-kpi-label">' + label + '</div>' +
    '<div class="drift-kpi-value">' + value + '</div>' + subHtml + '</div>';
}

function buildDriftCellRowsHtml(pack, noHistory) {
  const cells  = pack.cells || [];
  const nc     = Math.min(pack.cell_count || cells.length, 15);
  const spread = pack.spread_now || 0;
  const color  = driftBandColor(spread);

  const guideHtml = DRIFT_GUIDES.map(g =>
    '<div class="drift-guide" style="left:' + driftPct(g.mv).toFixed(1) + '%"></div>'
  ).join('');

  // Outlier detection (review 2.5): rows are dimmed by default; full opacity
  // goes to the cells that define the pack spread (window worst at full /
  // empty) and to cells whose 5-day band center deviates > 15 mV from the
  // pack median. Keeps the eye on the two or three rows that matter.
  const OUTLIER_MV = 15;
  const centers = [];
  for (let ci = 0; ci < nc; ci++) {
    const c = cells[ci] || {};
    if (!noHistory && c.d5min && c.d5max) centers.push((c.d5min + c.d5max) / 2);
    else if (c.now) centers.push(c.now);
    else centers.push(null);
  }
  const sorted = centers.filter(v => v !== null).sort((a, b) => a - b);
  const median = sorted.length ? sorted[Math.floor(sorted.length / 2)] : null;

  const ffWin = driftRepeatWinner(pack.ff_mode_idx, pack.ff_days_won, pack.ff_days_total);

  const isOutlier = ci => {
    if (nc <= 2) return true;  // tiny packs: nothing to dim
    if (pack.has_toc && ci === pack.first_full_idx) return true;
    if (pack.has_bod && ci === pack.first_empty_idx) return true;
    if (ffWin && ci === pack.ff_mode_idx) return true;
    if (median !== null && centers[ci] !== null &&
        Math.abs(centers[ci] - median) > OUTLIER_MV) return true;
    return false;
  };

  let rows = '';
  for (let ci = 0; ci < nc; ci++) {
    const c = cells[ci] || {};
    const nowMv  = c.now  || 0;
    const nowPct = nowMv  ? driftPct(nowMv).toFixed(1)  : null;

    const atHtml = (!noHistory && c.evMin && c.evMax) ? (() => {
      const l = driftPct(c.evMin).toFixed(1);
      const w = Math.max(0, driftPct(c.evMax) - driftPct(c.evMin)).toFixed(1);
      return '<div class="drift-band-at" style="left:' + l + '%;width:' + w + '%"></div>';
    })() : '';

    const d5Html = (!noHistory && c.d5min && c.d5max) ? (() => {
      const l = driftPct(c.d5min).toFixed(1);
      const w = Math.max(0, driftPct(c.d5max) - driftPct(c.d5min)).toFixed(1);
      return '<div class="drift-band-d5" style="left:' + l + '%;width:' + w + '%;background:' + color + '"></div>';
    })() : '';

    const dotHtml = nowPct !== null
      ? '<div class="drift-dot" data-drift-dot="' + pack.id + '-' + ci +
        '" style="left:' + nowPct + '%;background:' + color + '"></div>'
      : '';

    // Right column: the 5-day band WIDTH is the number this section is about;
    // the endpoints are already encoded by the band position (review 2.5).
    // Raw endpoints stay available as a tooltip.
    const hasBand = !noHistory && c.d5min && c.d5max;
    const numsStr = hasBand
      ? ((c.d5max - c.d5min) + ' mV')
      : (nowMv ? nowMv + ' mV' : '—');
    const numsTitle = hasBand ? (c.d5min + '-' + c.d5max + ' mV over 5 days') : '';

    const tagHtml = (ffWin && ci === pack.ff_mode_idx)
      ? '<div class="drift-cell-tag-line"><span class="drift-cell-tag">fills first</span></div>' : '';

    // The tag is a row-level element (not inside the fixed-width 22px label)
    // so it reflows onto its own line under the bar instead of overflowing the
    // label box and clipping/overlapping the track.
    rows += '<div class="drift-cell-row' + (isOutlier(ci) ? '' : ' dim') + '">' +
      '<div class="drift-cell-lbl">C' + (ci + 1) + '</div>' +
      '<div class="drift-track">' + guideHtml + atHtml + d5Html + dotHtml + '</div>' +
      '<div class="drift-cell-nums"' +
        (numsTitle ? ' title="' + numsTitle + '"' : '') + '>' + numsStr + '</div>' +
      tagHtml +
      '</div>';
  }

  // Scale axis
  const ticksHtml =
    '<div class="drift-scale-tick" style="left:0%">' + (DRIFT_FLOOR_MV / 1000).toFixed(2) + '</div>' +
    DRIFT_GUIDES.map(g =>
      '<div class="drift-scale-tick" style="left:' + driftPct(g.mv).toFixed(1) + '%">' + g.label + '</div>'
    ).join('') +
    '<div class="drift-scale-tick" style="left:100%;transform:translateX(-100%)">' + (DRIFT_CEIL_MV / 1000).toFixed(2) + '</div>';

  // Band legend: the grey band is all-time at ANY SoC while the colored band
  // is 5-day at extremes only — two gates in one graphic need labels.
  const legendHtml = noHistory ? '' :
    '<div class="drift-legend">' +
      '<span><span class="drift-legend-swatch swatch-at"></span>all-time (any SoC)</span>' +
      '<span><span class="drift-legend-swatch" style="background:' + color + '"></span>5-day at extremes</span>' +
      '<span><span class="drift-legend-swatch swatch-dot" style="background:' + color + '"></span>now</span>' +
    '</div>';

  return rows +
    '<div class="drift-scale-row">' +
      '<div class="drift-scale-lbl-spacer"></div>' +
      '<div class="drift-scale-axis">' + ticksHtml + '</div>' +
      '<div class="drift-scale-nums-spacer"></div>' +
    '</div>' +
    legendHtml;
}

function toggleDriftPack(packId) {
  const isOpen = g_drift_open.has(packId);
  if (isOpen) g_drift_open.delete(packId);
  else        g_drift_open.add(packId);

  const body    = document.querySelector('[data-drift-body="' + packId + '"]');
  const chevron = document.querySelector('[data-drift-toggle="' + packId + '"] .drift-chevron');
  if (body)    body.style.display = isOpen ? 'none' : '';
  if (chevron) chevron.className  = 'drift-chevron' + (isOpen ? '' : ' open');
}

function driftExpandAll() {
  if (!g_drift_data) return;
  const packs   = g_drift_data.packs || [];
  const allOpen = packs.every(p => g_drift_open.has(p.id));
  packs.forEach(p => {
    if (allOpen) g_drift_open.delete(p.id);
    else         g_drift_open.add(p.id);
    const body    = document.querySelector('[data-drift-body="' + p.id + '"]');
    const chevron = document.querySelector('[data-drift-toggle="' + p.id + '"] .drift-chevron');
    if (body)    body.style.display = allOpen ? 'none' : '';
    if (chevron) chevron.className  = 'drift-chevron' + (allOpen ? '' : ' open');
  });
}

// Called on every 2-s live poll when on the battery page to update the "now"
// dot positions in-place without re-fetching drift history.
function updateDriftNow() {
  if (!g_live || !g_drift_data) return;
  const livePacks  = (g_live.snapshot && g_live.snapshot.packs) || [];
  const driftPacks = g_drift_data.packs || [];

  driftPacks.forEach(dp => {
    const lp = livePacks.find(p => p.bms_id === dp.id);
    if (!lp || !lp.cells) return;
    const spread = Math.round((lp.cell_drift_v || 0) * 1000);
    const color  = driftBandColor(spread);

    lp.cells.forEach((v, ci) => {
      const mv = Math.round(v * 1000);
      if (mv < 2000 || mv > 5000) return;
      const dotEl = document.querySelector('[data-drift-dot="' + dp.id + '-' + ci + '"]');
      if (!dotEl) return;
      dotEl.style.left       = driftPct(mv).toFixed(1) + '%';
      dotEl.style.background = color;
    });
  });
}
