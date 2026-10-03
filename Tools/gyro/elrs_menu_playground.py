#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Try the transmitter setup menu in a web browser: SITL runs the gyro
firmware, the real ExpressLRS Lua script runs against it (see
edgetx_sim.py), and a local web page shows the radio screen with the
radio's keys and sliders for the RC channels.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          ELRS_LUA=/path/to/elrsV3.lua python3 Tools/gyro/elrs_menu_playground.py
Then open     http://localhost:8765 (from Windows too, when running in WSL2)

Keys on the page: Up/Down move, Enter selects or edits, Esc or
Backspace goes back. Options: --gps fits a simulated GPS (shows the RTL
entries), --keep keeps the settings from the last run, --port changes
the web port. A ground station can connect to SITL on TCP port 5762
at the same time. Needs pymavlink, lupa and Pillow.
"""
import argparse
import io
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crsf_menu_client as cm  # noqa: E402
from edgetx_sim import EdgeTxSim  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
CRSF_PORT = 5765
BOARD_DEFAULTS = os.path.join(ROOT, "libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/defaults.parm")

CHANNEL_HINTS = ["Aileron", "Elevator", "Throttle", "Rudder", "Arm switch", "Mode switch", "RTL switch",
                 "Gain knob"]

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gyro Menu Playground</title>
<style>
:root { --bg: #f4f4f2; --panel: #ffffff; --text: #1d1d1b; --muted: #6b6b66; --line: #d6d6d0;
        --accent: #2f6fde; --ok: #1f8a4c; --warn: #b54708; }
@media (prefers-color-scheme: dark) {
  :root { --bg: #171716; --panel: #232321; --text: #ecece8; --muted: #9a9a94; --line: #3a3a37;
          --accent: #6d9cff; --ok: #4cc38a; --warn: #f0a35e; }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text);
       font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1060px; margin: 0 auto; padding: 20px 16px; display: grid; gap: 16px;
       grid-template-columns: minmax(0, 1fr) 300px; }
@media (max-width: 860px) { main { grid-template-columns: 1fr; } }
h1 { font-size: 18px; margin: 0 0 4px; }
.sub { color: var(--muted); margin: 0 0 12px; }
.panel { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 14px; }
.screen { width: 100%; aspect-ratio: 480 / 272; display: block; border-radius: 6px;
          border: 1px solid var(--line); background: #fff; }
.keys { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-top: 12px; }
button { font: inherit; padding: 10px 6px; border-radius: 8px; border: 1px solid var(--line);
         background: var(--bg); color: var(--text); cursor: pointer; }
button:hover { border-color: var(--accent); }
button:active { transform: translateY(1px); }
kbd { color: var(--muted); font-size: 12px; display: block; }
.state { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 10px; }
.pill { padding: 3px 10px; border-radius: 999px; border: 1px solid var(--line); font-weight: 600; }
.armed { color: var(--warn); border-color: var(--warn); }
.disarmed { color: var(--ok); border-color: var(--ok); }
h2 { font-size: 14px; margin: 0 0 10px; }
.ch { display: grid; grid-template-columns: 34px 1fr 44px; gap: 8px; align-items: center; margin: 6px 0; }
.ch label { color: var(--muted); font-variant-numeric: tabular-nums; }
.ch input { width: 100%; }
.ch span { text-align: right; font-variant-numeric: tabular-nums; }
.hint { color: var(--muted); font-size: 12px; margin: -4px 0 6px 42px; }
.log { font: 12px/1.5 ui-monospace, Consolas, monospace; color: var(--muted); white-space: pre-wrap;
       max-height: 180px; overflow-y: auto; margin: 0; }
.row { display: flex; gap: 8px; margin-top: 10px; }
.row button { flex: 1; padding: 8px 6px; }
</style>
</head>
<body>
<main>
  <section>
    <h1>Gyro menu playground</h1>
    <p class="sub">The real ExpressLRS script talking to the gyro firmware in SITL.</p>
    <div class="panel">
      <img class="screen" id="screen" alt="Radio screen">
      <div class="keys">
        <button data-key="prev">Up<kbd>&uarr;</kbd></button>
        <button data-key="next">Down<kbd>&darr;</kbd></button>
        <button data-key="enter">Enter<kbd>Enter</kbd></button>
        <button data-key="exit">Back<kbd>Esc</kbd></button>
      </div>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Flight controller messages</h2>
      <pre class="log" id="log"></pre>
    </div>
  </section>
  <aside class="panel">
    <div class="state"><span class="pill" id="armed">&hellip;</span><span class="pill" id="mode">&hellip;</span></div>
    <h2>RC channels</h2>
    <div id="channels"></div>
    <div class="row">
      <button id="center">Center sticks</button>
      <button id="restart">Restart script</button>
    </div>
  </aside>
</main>
<script>
const hints = HINTS;
const screen = document.getElementById("screen");
function post(url) { return fetch(url, {method: "POST"}); }
function refresh() {
  const next = new Image();
  next.onload = () => { screen.src = next.src; setTimeout(refresh, 120); };
  next.onerror = () => setTimeout(refresh, 1000);
  next.src = "/screen.png?t=" + Date.now();
}
refresh();
document.querySelectorAll("[data-key]").forEach(b => b.onclick = () => post("/key?k=" + b.dataset.key));
const keymap = {ArrowUp: "prev", ArrowDown: "next", Enter: "enter", Escape: "exit", Backspace: "exit"};
document.addEventListener("keydown", e => {
  if (e.target.tagName === "INPUT" && e.key.startsWith("Arrow")) return;
  const k = keymap[e.key];
  if (k) { e.preventDefault(); post("/key?k=" + k); }
});
const box = document.getElementById("channels");
const sliders = [];
hints.forEach((hint, i) => {
  const row = document.createElement("div");
  row.className = "ch";
  row.innerHTML = `<label>Ch${i + 1}</label><input type="range" min="988" max="2012" step="1">` +
                  `<span></span>`;
  const h = document.createElement("div");
  h.className = "hint"; h.textContent = hint;
  box.append(row, h);
  const input = row.querySelector("input"), out = row.querySelector("span");
  input.oninput = () => { out.textContent = input.value; post(`/rc?ch=${i + 1}&v=${input.value}`); };
  sliders.push([input, out]);
});
document.getElementById("center").onclick = () => {
  [[0, 1500], [1, 1500], [2, 988], [3, 1500]].forEach(([i, v]) => {
    sliders[i][0].value = v; sliders[i][0].oninput();
  });
};
document.getElementById("restart").onclick = () => post("/restart");
let first = true;
async function poll() {
  try {
    const s = await (await fetch("/state")).json();
    const a = document.getElementById("armed");
    a.textContent = s.armed === null ? "no heartbeat" : (s.armed ? "ARMED" : "DISARMED");
    a.className = "pill " + (s.armed ? "armed" : "disarmed");
    document.getElementById("mode").textContent = s.mode || "\\u2026";
    document.getElementById("log").textContent = s.messages.join("\\n");
    if (first) {
      s.channels.forEach((v, i) => { sliders[i][0].value = v; sliders[i][1].textContent = v; });
      first = false;
    }
  } catch (e) {}
  setTimeout(poll, 500);
}
poll();
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1], formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--lua", default=os.environ.get("ELRS_LUA"), help="ExpressLRS script (default: $ELRS_LUA)")
    ap.add_argument("--gps", action="store_true", help="fit a simulated GPS")
    ap.add_argument("--keep", action="store_true", help="keep the settings from the last run")
    ap.add_argument("--port", type=int, default=8765, help="web page port")
    ap.add_argument("--host", default="127.0.0.1", help="web page address")
    args = ap.parse_args()
    if not args.lua or not os.path.exists(args.lua):
        sys.exit("give the ExpressLRS script (elrsV3.lua) with --lua or ELRS_LUA")
    for port in (5760, CRSF_PORT):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                sys.exit("TCP port %u is already in use; stop the running simulator first" % port)

    work = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_menu_playground")
    os.makedirs(work, exist_ok=True)
    parm = os.path.join(work, "playground.parm")
    with open(parm, "w") as f:
        # the gyro board defaults (modes, switches, failsafe), with the
        # receiver on SITL's CRSF port instead of the board's serial port
        with open(BOARD_DEFAULTS) as board:
            f.writelines(ln for ln in board if not ln.startswith("SERIAL"))
        f.write("SERIAL5_PROTOCOL 23\nSCR_ENABLE 0\n")
        if not args.gps:
            # SITL's disabled GPS still reports "no fix"; removing the
            # driver gives ArduPilot's "no GPS" state
            f.write("SIM_GPS1_ENABLE 0\nGPS1_TYPE 0\n")
    cmd = [BIN, "--model", "plane", "--speedup", "1", "--home", "-35.363261,149.165230,584,353",
           "--serial5=tcp:%u" % CRSF_PORT,
           "--defaults", os.path.join(ROOT, "Tools/autotest/models/plane.parm") + "," + parm]
    if not args.keep:
        cmd.append("-w")
    print("starting SITL in", work, flush=True)
    proc = subprocess.Popen(cmd, cwd=work, stdout=open(os.path.join(work, "sitl_stdout.txt"), "w"),
                            stderr=subprocess.STDOUT)
    try:
        run(args, proc)
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
        proc.wait()


def run(args, proc):
    time.sleep(2)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
    m.wait_heartbeat(timeout=60)

    state = {"armed": None, "mode": None, "messages": []}
    state_lock = threading.Lock()

    # keep reading MAVLink: if nobody reads, SITL's clock nearly stops
    def mav_reader():
        while True:
            try:
                r = m.recv_match(blocking=True, timeout=1)
            except Exception:
                return
            if r is None:
                continue
            with state_lock:
                if r.get_type() == "HEARTBEAT" and r.get_srcComponent() == 1:
                    state["armed"] = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["mode"] = mavutil.mode_string_v10(r)
                elif r.get_type() == "STATUSTEXT":
                    state["messages"] = (state["messages"] + [r.text])[-30:]

    threading.Thread(target=mav_reader, daemon=True).start()

    client = cm.CrsfClient(port=CRSF_PORT)
    print("waiting for the menu", flush=True)
    t0 = time.time()
    while time.time() - t0 < 30:
        with state_lock:
            if any("menu ready" in t for t in state["messages"]):
                break
        time.sleep(0.2)

    sim_lock = threading.Lock()
    sim = [EdgeTxSim(args.lua, client)]
    actions = []

    # the Lua engine is not thread safe: only this thread runs the script
    def script_loop():
        while proc.poll() is None:
            with sim_lock:
                action = actions.pop(0) if actions else None
                if action == "restart":
                    with client.lock:
                        client.frames = []
                    sim[0] = EdgeTxSim(args.lua, client)
                    action = None
                try:
                    exited = sim[0].step(action)
                except Exception as e:
                    with state_lock:
                        state["messages"].append("radio script failed and was restarted: %s" % e)
                    exited = True
                if exited:
                    # EXIT chosen (or a failure): reopen the script, as from the radio's tools list
                    sim[0] = EdgeTxSim(args.lua, client)
            time.sleep(0.02)

    threading.Thread(target=script_loop, daemon=True).start()

    page = PAGE.replace("HINTS", json.dumps(CHANNEL_HINTS)).encode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self.reply(page, "text/html; charset=utf-8")
            elif path == "/screen.png":
                buf = io.BytesIO()
                with sim_lock:
                    sim[0].save_png(buf)
                self.reply(buf.getvalue(), "image/png")
            elif path == "/state":
                with state_lock:
                    body = dict(state, channels=client.channels[:len(CHANNEL_HINTS)])
                self.reply(json.dumps(body).encode(), "application/json")
            else:
                self.reply(b"not found", "text/plain", 404)

        def do_POST(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            if u.path == "/key" and q.get("k") in ("next", "prev", "enter", "exit"):
                with sim_lock:
                    actions.append(q["k"])
            elif u.path == "/rc":
                ch, value = int(q.get("ch", 0)), int(q.get("v", 1500))
                if 1 <= ch <= 16:
                    client.channels[ch - 1] = max(988, min(2012, value))
            elif u.path == "/restart":
                with sim_lock:
                    actions.append("restart")
            else:
                self.reply(b"not found", "text/plain", 404)
                return
            self.reply(b"ok", "text/plain")

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("menu playground on http://localhost:%u  (Ctrl+C to stop)" % args.port, flush=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    while proc.poll() is None:
        time.sleep(0.5)
    print("SITL stopped", flush=True)


if __name__ == "__main__":
    main()
