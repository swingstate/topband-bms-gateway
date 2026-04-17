#!/usr/bin/env python3
"""
Topband BMS Simulator + CAN Monitor mit Web-Dashboard
======================================================
Raspberry Pi + Waveshare RS485/CAN HAT

Alle Szenarien liefern dynamisch schwankende Werte (Sinus-Drift),
damit MQTT-Aenderungen sichtbar werden.

Setup:
  pip3 install pyserial python-can flask

Usage:
  python3 bms_simulator.py                     # Normal mit Drift
  python3 bms_simulator.py -s charge           # Lade-Szenario
  python3 bms_simulator.py -s cold             # Kaelte-Test
  python3 bms_simulator.py --port /dev/ttyAMA0
  python3 bms_simulator.py --can-only          # Nur CAN mitlesen
"""

import serial
import struct
import time
import math
import random
import argparse
import threading
import json
from collections import deque
from datetime import datetime

# --- CONFIG ---
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 9600
CAN_INTERFACE = "can0"
CAN_BITRATE = 500000
BMS_COUNT = 2
CELL_COUNT = 16
WEB_PORT = 5000

# --- SHARED STATE ---
state_lock = threading.RLock()
state = {
    "scenario": "normal",
    "rs485_polls": 0,
    "can_frames": 0,
    "rs485_log": deque(maxlen=50),
    "can_log": deque(maxlen=50),
    "can_last": {},
    "running": True,
    "rs485_ok": False,
    "can_ok": False,
    "started": None,
    "soc_drift": 0.0,
}

# --- SZENARIEN ---
# drift_pct: wie stark Werte prozentual schwanken
# current_wave: [amplitude, offset, period_s] fuer Sinus-Strom
# Max safe: 53.25V pack (3.328V/cell for 16s), 3.55V cell
SCENARIOS = {
    "normal": {
        "desc": "Normalbetrieb mit Drift",
        "cell_v": 3.17, "cell_spread": 0.006,
        "temps": [25.0, 24.5, 25.2, 24.8, 28.0, 22.0, 26.0],
        "current_wave": [3.0, 0.0, 60],
        "soc_pct": 55, "soh": 100, "full_ah": 30.0,
        "drift_pct": 0.02,
    },
    "charge": {
        "desc": "Laden, schwankend 10-20A",
        "cell_v": 3.22, "cell_spread": 0.005,
        "temps": [28.0, 27.5, 28.5, 27.0, 32.0, 22.0, 29.0],
        "current_wave": [5.0, 15.0, 45],
        "soc_pct": 72, "soh": 98, "full_ah": 30.0,
        "drift_pct": 0.015,
    },
    "discharge": {
        "desc": "Entladen, schwankend -15 bis -25A",
        "cell_v": 3.12, "cell_spread": 0.006,
        "temps": [30.0, 29.5, 30.5, 29.0, 35.0, 22.0, 31.0],
        "current_wave": [5.0, -20.0, 40],
        "soc_pct": 45, "soh": 99, "full_ah": 30.0,
        "drift_pct": 0.015,
    },
    "cold": {
        "desc": "Kaelte (<5C) - Laden sollte stoppen",
        "cell_v": 3.15, "cell_spread": 0.003,
        "temps": [3.0, 2.5, 3.5, 2.8, 5.0, 1.0, 4.0],
        "current_wave": [0.5, 0.5, 30],
        "soc_pct": 50, "soh": 100, "full_ah": 30.0,
        "drift_pct": 0.01,
    },
    "hot": {
        "desc": "Hitze (>50C) - Alles sollte stoppen",
        "cell_v": 3.18, "cell_spread": 0.008,
        "temps": [45.0, 44.0, 46.0, 52.0, 55.0, 35.0, 47.0],
        "current_wave": [2.0, -5.0, 50],
        "soc_pct": 60, "soh": 97, "full_ah": 30.0,
        "drift_pct": 0.015,
    },
    "low": {
        "desc": "Niedriger SOC (10%)",
        "cell_v": 2.95, "cell_spread": 0.012,
        "temps": [22.0, 21.5, 22.5, 21.0, 24.0, 18.0, 23.0],
        "current_wave": [1.0, -2.0, 35],
        "soc_pct": 10, "soh": 95, "full_ah": 30.0,
        "drift_pct": 0.02,
    },
    "full": {
        "desc": "Voll geladen (99%)",
        "cell_v": 3.28, "cell_spread": 0.002,
        "temps": [26.0, 25.5, 26.5, 25.0, 28.0, 22.0, 27.0],
        "current_wave": [0.3, 0.2, 90],
        "soc_pct": 99, "soh": 100, "full_ah": 30.0,
        "drift_pct": 0.008,
    },
}


# --- FARBEN ---
class C:
    RESET  = "\033[0m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    CYAN   = "\033[96m"
    BLUE   = "\033[94m"
    BOLD   = "\033[1m"
    DIM    = "\033[2m"


# ===================================================================
#  DYNAMISCHE WERTE
# ===================================================================

def drift_value(base, drift_pct, period_s, phase=0.0):
    """Sinusfoermiger Drift um den Basiswert."""
    t = time.time()
    wave = math.sin(2.0 * math.pi * t / period_s + phase)
    return base * (1.0 + drift_pct * wave)


def get_current(scenario):
    """Zeitabhaengiger Strom aus Sinus-Welle."""
    amp, offset, period = scenario["current_wave"]
    t = time.time()
    wave = math.sin(2.0 * math.pi * t / period)
    return offset + amp * wave


def get_cell_voltage(scenario, cell_idx):
    """Zellspannung mit Drift und Zell-individuellem Phasen-Offset."""
    base = scenario["cell_v"]
    dp = scenario["drift_pct"]
    phase = cell_idx * 0.4
    v = drift_value(base, dp, 47 + cell_idx * 3, phase)
    spread = scenario["cell_spread"]
    v += random.uniform(-spread, spread)
    # Hard limits: ESP32 defaults are 3.55V cell, 53.25V pack (16*3.328V)
    if v < 2.5:
        v = 2.5
    if v > 3.32:
        v = 3.32
    return v


def get_temperature(scenario, temp_idx):
    """Temperatur mit langsamem Drift."""
    base = scenario["temps"][temp_idx]
    dp = scenario["drift_pct"] * 0.5
    phase = temp_idx * 1.2
    t = drift_value(base, dp, 120 + temp_idx * 15, phase)
    t += random.uniform(-0.2, 0.2)
    return t


def get_soc(scenario):
    """SOC mit langsamem Drift basierend auf Stromrichtung."""
    base = scenario["soc_pct"]
    with state_lock:
        d = state["soc_drift"]
    soc = base + d
    if soc > 100:
        soc = 100
    if soc < 0:
        soc = 0
    return int(soc)


def update_soc_drift(current_a, dt_s):
    """SOC-Drift basierend auf Strom aktualisieren."""
    # Grobe Naeherung: 1A fuer 1h = ~3.3% bei 30Ah
    delta = (current_a * dt_s / 3600.0) / 30.0 * 100.0
    with state_lock:
        state["soc_drift"] += delta
        if state["soc_drift"] > 15:
            state["soc_drift"] = 15
        if state["soc_drift"] < -15:
            state["soc_drift"] = -15


# ===================================================================
#  RS485 BMS SIMULATOR
# ===================================================================

def build_bms_response(bms_id, scenario):
    """Baut Topband-BMS-Antwort als Hex-String mit dynamischen Werten."""
    buf = bytearray()
    buf += bytes([0x21, 0x00 + bms_id, 0x46, 0x42])
    buf += bytes([0xD0, 0x7C])
    buf += bytes([0x00, CELL_COUNT])

    for i in range(CELL_COUNT):
        v = get_cell_voltage(scenario, i)
        v_mv = int(v * 1000)
        buf += struct.pack(">H", v_mv)

    temps = scenario["temps"]
    buf += bytes([len(temps)])
    for idx in range(len(temps)):
        t = get_temperature(scenario, idx)
        dk = int(t * 10) + 2731
        if dk < 0:
            dk = 0
        buf += struct.pack(">H", dk)

    current_a = get_current(scenario)
    # Per-BMS leichte Verschiebung
    current_a += (bms_id - BMS_COUNT / 2.0) * 0.3
    current_ca = int(current_a * 100)
    buf += struct.pack(">h", current_ca)

    # Pack-Spannung = Summe Zellspannungen (dynamisch berechnet)
    pack_v = sum(get_cell_voltage(scenario, i) for i in range(CELL_COUNT))
    pack_cv = int(pack_v * 100)
    buf += struct.pack(">H", pack_cv)

    soc = get_soc(scenario)
    full_ah = scenario["full_ah"]
    rem_ah = full_ah * (soc / 100.0)
    buf += struct.pack(">H", int(rem_ah * 100))
    buf += bytes([0x00])
    buf += struct.pack(">H", int(full_ah * 100))

    # Cycle count (2B) + SOC (1B) + SOH (1B) - extended layout
    buf += struct.pack(">H", 42)  # cycle count
    buf += bytes([soc & 0xFF])
    buf += bytes([scenario["soh"] & 0xFF])

    return buf.hex().upper()


def rs485_loop(ser):
    """Liest Poll-Commands vom ESP32 und antwortet mit dynamischen Daten."""
    buffer = ""
    last_current_time = time.time()
    while state["running"]:
        if ser.in_waiting > 0:
            ch = ser.read().decode("ascii", errors="ignore")
            if ch == "\r":
                cmd = buffer.strip()
                buffer = ""
                if cmd.startswith("~"):
                    try:
                        addr = int(cmd[3:5], 16)
                    except (ValueError, IndexError):
                        addr = 0
                    if addr < BMS_COUNT:
                        sc = SCENARIOS[state["scenario"]]
                        response = build_bms_response(addr, sc)
                        ser.write(response.encode("ascii"))
                        ser.write(b"\r")
                        ser.flush()

                        # SOC-Drift aktualisieren
                        now = time.time()
                        dt = now - last_current_time
                        last_current_time = now
                        update_soc_drift(get_current(sc), dt)

                        with state_lock:
                            state["rs485_polls"] += 1
                            ts = datetime.now().strftime("%H:%M:%S")
                            cur = get_current(sc)
                            pv = sc["cell_v"] * CELL_COUNT
                            state["rs485_log"].appendleft(
                                f"[{ts}] BMS #{addr}  {pv:.1f}V  {cur:+.1f}A  SOC:{get_soc(sc)}%")
            elif ch == "~":
                buffer = "~"
            else:
                buffer += ch
        else:
            time.sleep(0.001)


# ===================================================================
#  CAN BUS MONITOR
# ===================================================================

def decode_can(fid, data):
    if len(data) < 2:
        return f"0x{fid:03X}   (empty frame)"
    if fid == 0x351:
        if len(data) < 6:
            return f"LIMITS  (short frame)"
        cvl = struct.unpack_from("<H", data, 0)[0] * 0.1
        ccl = struct.unpack_from("<H", data, 2)[0] * 0.1
        dcl = struct.unpack_from("<H", data, 4)[0] * 0.1
        alert = ""
        if ccl == 0 and dcl == 0:
            alert = " !! ALLES GESPERRT"
        elif ccl == 0:
            alert = " !! LADEN GESPERRT"
        return f"LIMITS  CVL:{cvl:.1f}V CCL:{ccl:.1f}A DCL:{dcl:.1f}A{alert}"
    elif fid == 0x355:
        if len(data) < 6:
            return f"SOC/SOH (short frame)"
        soc = struct.unpack_from("<H", data, 0)[0]
        soh = struct.unpack_from("<H", data, 2)[0]
        cap = struct.unpack_from("<H", data, 4)[0] * 0.1
        return f"SOC/SOH SOC:{soc}% SOH:{soh}% Cap:{cap:.1f}Ah"
    elif fid == 0x356:
        if len(data) < 6:
            return f"MESSUNG (short frame)"
        volt = struct.unpack_from("<H", data, 0)[0] * 0.01
        cur = struct.unpack_from("<h", data, 2)[0] * 0.1
        temp = struct.unpack_from("<h", data, 4)[0] * 0.1
        pwr = volt * cur
        return f"MESSUNG {volt:.2f}V {cur:+.1f}A {temp:.1f}C ({pwr:+.0f}W)"
    elif fid == 0x35A:
        if len(data) < 5:
            return f"ALARM   (short frame)"
        flags = data[4]
        if flags == 0:
            return "ALARM   Keine Alarme"
        parts = []
        if flags & 0x80:
            parts.append("GENERAL")
        if flags & 0x40:
            parts.append("OVERVOLT")
        if flags & 0x20:
            parts.append("TEMP")
        if flags & 0x10:
            parts.append("UNDERVOLT")
        return f"ALARM   Flags:0x{flags:02X} = {','.join(parts)}"
    elif fid == 0x35E:
        name = bytes(data).decode("ascii", errors="replace").rstrip("\x00 ")
        return f"NAME    {name}"
    return f"0x{fid:03X}   {data.hex()}"


def can_loop(interface):
    try:
        import can as canlib
    except ImportError:
        print(f"  {C.RED}python-can nicht installiert: pip3 install python-can{C.RESET}")
        return

    # CAN-Interface automatisch hochfahren falls noetig
    import subprocess
    try:
        result = subprocess.run(["ip", "link", "show", interface],
                                capture_output=True, text=True, timeout=5)
        if "state UP" not in result.stdout:
            print(f"  {C.YELLOW}CAN '{interface}' ist nicht aktiv - starte...{C.RESET}")
            subprocess.run(["sudo", "ip", "link", "set", interface, "up",
                            "type", "can", "bitrate", str(CAN_BITRATE)],
                           capture_output=True, timeout=5)
            time.sleep(0.5)
    except Exception as e:
        print(f"  {C.YELLOW}CAN Auto-Start fehlgeschlagen: {e}{C.RESET}")

    try:
        bus = canlib.interface.Bus(channel=interface, interface="socketcan")
        with state_lock:
            state["can_ok"] = True
        print(f"  {C.GREEN}CAN '{interface}' geoeffnet.{C.RESET}\n")
    except Exception as e:
        print(f"  {C.RED}CAN Fehler: {e}{C.RESET}")
        print(f"  sudo ip link set {interface} up type can bitrate {CAN_BITRATE}")
        return

    while state["running"]:
        try:
            msg = bus.recv(timeout=0.5)
        except Exception:
            with state_lock:
                state["can_ok"] = False
            time.sleep(2)
            continue
        if msg and msg.arbitration_id in (0x351, 0x355, 0x356, 0x35A, 0x35E):
            decoded = decode_can(msg.arbitration_id, bytes(msg.data))
            ts = datetime.now().strftime("%H:%M:%S")
            with state_lock:
                state["can_frames"] += 1
                state["can_last"][msg.arbitration_id] = decoded
                state["can_log"].appendleft(f"[{ts}] {decoded}")


# ===================================================================
#  WEB DASHBOARD (Flask)
# ===================================================================

def web_server():
    try:
        from flask import Flask, jsonify, request
    except ImportError:
        print(f"  {C.YELLOW}Flask nicht installiert - kein Web-Dashboard{C.RESET}")
        print(f"  pip3 install flask")
        return

    app = Flask(__name__)
    import logging
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    DASHBOARD_HTML = """<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMS Simulator</title>
<style>
body{background:#0f172a;color:#e2e8f0;font-family:'Segoe UI',sans-serif;margin:0;padding:16px;}
h1{font-size:1.3rem;margin:0 0 12px;}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;}
.c{background:#1e293b;border-radius:10px;padding:14px;}
.pill{display:inline-block;padding:3px 10px;border-radius:10px;font-size:0.75rem;font-weight:700;cursor:pointer;margin:2px;}
.p-on{background:#166534;color:#4ade80;} .p-off{background:#7f1d1d;color:#fca5a5;}
.p-act{background:#1e40af;color:#93c5fd;border:2px solid #60a5fa;}
.p-inact{background:#334155;color:#94a3b8;border:2px solid transparent;}
.num{font-size:1.6rem;font-weight:700;} .lbl{font-size:0.7rem;color:#64748b;text-transform:uppercase;}
pre{background:#0f172a;padding:8px;border-radius:6px;font-size:0.72rem;max-height:200px;overflow-y:auto;margin:8px 0 0;}
.row{display:flex;gap:20px;flex-wrap:wrap;margin:8px 0;}
.stat{text-align:center;}
</style></head><body>
<h1>Topband BMS Simulator</h1>
<div class="g">
<div class="c">
<div class="lbl">STATUS</div>
<div style="margin:8px 0;">
<span class="pill" id="st_rs485">RS485 --</span>
<span class="pill" id="st_can">CAN --</span>
<span class="pill" id="st_polls">Polls: 0</span>
<span class="pill" id="st_frames">CAN: 0</span>
</div>
<div class="lbl" style="margin-top:12px;">SZENARIO</div>
<div id="sc_btns" style="margin:6px 0;"></div>
</div>
<div class="c">
<div class="lbl">LIVE WERTE</div>
<div class="row" id="live">--</div>
</div>
<div class="c">
<div class="lbl">RS485 LOG</div>
<pre id="rs_log">Warte auf Polls...</pre>
</div>
<div class="c">
<div class="lbl">CAN LOG</div>
<pre id="can_log">Warte auf Frames...</pre>
</div>
</div>
<script>
function upd(){
  fetch('/api').then(function(r){return r.json();}).then(function(d){
    document.getElementById('st_rs485').className='pill '+(d.rs485_ok?'p-on':'p-off');
    document.getElementById('st_rs485').innerText='RS485 '+(d.rs485_ok?'OK':'OFF');
    document.getElementById('st_can').className='pill '+(d.can_ok?'p-on':'p-off');
    document.getElementById('st_can').innerText='CAN '+(d.can_ok?'OK':'OFF');
    document.getElementById('st_polls').className='pill p-on';
    document.getElementById('st_polls').innerText='Polls: '+d.rs485_polls;
    document.getElementById('st_frames').className='pill p-on';
    document.getElementById('st_frames').innerText='CAN: '+d.can_frames;
    var sc=document.getElementById('sc_btns');
    if(sc.children.length===0 && d.scenarios){
      d.scenarios.forEach(function(s){
        var b=document.createElement('span');
        b.className='pill p-inact';
        b.innerText=s;
        b.onclick=function(){fetch('/scenario?s='+s).then(function(){upd();});};
        sc.appendChild(b);
      });
    }
    var btns=sc.children;
    for(var i=0;i<btns.length;i++){
      btns[i].className='pill '+(btns[i].innerText===d.scenario?'p-act':'p-inact');
    }
    var col=d.current>0.5?'#4ade80':d.current<-0.5?'#fbbf24':'#94a3b8';
    document.getElementById('live').innerHTML=
      '<div class="stat"><div class="num">'+d.pack_v.toFixed(1)+'V</div><div class="lbl">Pack</div></div>'
      +'<div class="stat"><div class="num" style="color:'+col+'">'+d.current.toFixed(1)+'A</div><div class="lbl">Strom</div></div>'
      +'<div class="stat"><div class="num">'+d.soc+'%</div><div class="lbl">SOC</div></div>'
      +'<div class="stat"><div class="num">'+d.power.toFixed(0)+'W</div><div class="lbl">Power</div></div>'
      +'<div class="stat"><div class="num">'+d.temp.toFixed(1)+'&deg;</div><div class="lbl">Max Temp</div></div>';
    document.getElementById('rs_log').innerText=(d.rs485_log||[]).join('\\n')||'Warte...';
    document.getElementById('can_log').innerText=(d.can_log||[]).join('\\n')||'Warte...';
  }).catch(function(e){console.log('API error',e);});
}
upd();setInterval(upd,2000);
</script></body></html>"""

    @app.route("/")
    def index():
        return DASHBOARD_HTML

    @app.route("/api")
    def api():
        try:
            sc = SCENARIOS[state["scenario"]]
            cur = get_current(sc)
            pv = sum(get_cell_voltage(sc, i) for i in range(CELL_COUNT))
            max_t = max(get_temperature(sc, i) for i in range(len(sc["temps"])))
            soc = get_soc(sc)
            data = {
                "scenario": state["scenario"],
                "scenarios": list(SCENARIOS.keys()),
                "rs485_ok": state["rs485_ok"],
                "can_ok": state["can_ok"],
                "rs485_polls": state["rs485_polls"],
                "can_frames": state["can_frames"],
                "rs485_log": list(state["rs485_log"]),
                "can_log": list(state["can_log"]),
                "pack_v": round(pv, 2),
                "current": round(cur, 2),
                "soc": soc,
                "power": round(pv * cur, 1),
                "temp": round(max_t, 1),
            }
            return jsonify(data)
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route("/scenario")
    def set_scenario():
        s = request.args.get("s", "normal")
        if s in SCENARIOS:
            state["scenario"] = s
            state["soc_drift"] = 0.0
        return jsonify({"ok": True, "scenario": s})

    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True)


# ===================================================================
#  MAIN
# ===================================================================

def print_status(scenario_name, scenario, mode="Full"):
    pack_v = scenario["cell_v"] * CELL_COUNT
    cur = scenario["current_wave"][1]
    rem_ah = scenario["full_ah"] * (scenario["soc_pct"] / 100.0)
    min_t = min(scenario["temps"])
    max_t = max(scenario["temps"])

    print(f"\n{C.BOLD}{'='*62}")
    print(f"  TOPBAND BMS SIMULATOR (dynamisch)")
    print(f"{'='*62}{C.RESET}")
    print(f"  Modus:      {mode}")
    print(f"  Szenario:   {scenario_name} - {scenario['desc']}")
    print(f"  BMS Count:  {BMS_COUNT}")
    print(f"  Zellen:     {CELL_COUNT} x {scenario['cell_v']:.3f}V = {pack_v:.1f}V")
    print(f"  Drift:      +/- {scenario['drift_pct']*100:.0f}%")
    amp, off, per = scenario["current_wave"]
    print(f"  Strom:      {off:+.1f}A +/- {amp:.1f}A (Periode {per}s)")
    print(f"  SOC/SOH:    {scenario['soc_pct']}% / {scenario['soh']}%")
    print(f"  Temps:      Min {min_t:.1f}C  Max {max_t:.1f}C")

    if max_t > 50:
        print(f"\n  {C.RED}{C.BOLD}>> ERWARTET: Uebertemperatur - alles STOPP{C.RESET}")
    elif min_t < 5:
        print(f"\n  {C.BLUE}{C.BOLD}>> ERWARTET: Kaelte - Laden STOPP{C.RESET}")
    else:
        print(f"\n  {C.GREEN}>> ERWARTET: Normal{C.RESET}")

    print(f"\n  Dashboard:  http://0.0.0.0:{WEB_PORT}")
    print(f"{C.BOLD}{'='*62}{C.RESET}\n")


def main():
    global BMS_COUNT

    parser = argparse.ArgumentParser(
        description="Topband BMS Simulator (dynamische Werte)")
    parser.add_argument("--scenario", "-s", default="normal",
                        choices=list(SCENARIOS.keys()))
    parser.add_argument("--port", "-p", default=SERIAL_PORT)
    parser.add_argument("--can", default=CAN_INTERFACE)
    parser.add_argument("--bms", "-b", type=int, default=BMS_COUNT)
    parser.add_argument("--web-port", "-w", type=int, default=WEB_PORT)
    parser.add_argument("--rs485-only", action="store_true")
    parser.add_argument("--can-only", action="store_true")
    parser.add_argument("--no-web", action="store_true")
    args = parser.parse_args()

    BMS_COUNT = args.bms
    state["scenario"] = args.scenario
    state["started"] = datetime.now().isoformat()

    scenario = SCENARIOS[args.scenario]
    if args.can_only:
        mode = "CAN Monitor"
    elif args.rs485_only:
        mode = "RS485 Simulation"
    else:
        mode = "RS485 + CAN + Web"

    print_status(args.scenario, scenario, mode)
    threads = []

    # --- RS485 ---
    if not args.can_only:
        try:
            ser = serial.Serial(
                port=args.port, baudrate=SERIAL_BAUD,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1)
            with state_lock:
                state["rs485_ok"] = True
            print(f"  {C.GREEN}RS485 '{args.port}' OK{C.RESET}")
            t = threading.Thread(target=rs485_loop, args=(ser,), daemon=True)
            t.start()
            threads.append(t)
        except Exception as e:
            print(f"  {C.RED}RS485 Fehler: {e}{C.RESET}")

    # --- CAN ---
    if not args.rs485_only:
        t = threading.Thread(target=can_loop, args=(args.can,), daemon=True)
        t.start()
        threads.append(t)

    # --- WEB ---
    if not args.no_web:
        t = threading.Thread(target=web_server, daemon=True)
        t.start()
        threads.append(t)

    print(f"\n  Ctrl+C zum Beenden.\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        state["running"] = False
        print(f"\n{C.YELLOW}Simulator beendet.{C.RESET}")


if __name__ == "__main__":
    main()
