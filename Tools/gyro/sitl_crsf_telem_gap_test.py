#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL test of the CRSF telemetry gap while the radio reads the gyro menu.

While disarmed, a menu read puts CRSF telemetry in parameter mode, which
stops the GPS, battery, attitude and flight mode frames. A radio that
still sees a good link but silent sensors announces "Sensor lost". This
test measures how long each of those frames stops when the whole menu
loads (device open) and when one folder loads (folder open), and checks
that the gap is about the time of the reads, not several seconds. Armed,
a menu read must not stop telemetry at all.

Build first, with the gyro-like SITL configuration:
    ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef
    ./waf plane
Run:
    python3 Tools/gyro/sitl_crsf_telem_gap_test.py
Needs pymavlink.
"""
import os
import socket
import subprocess
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crsf_menu_client as cm  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_telem_gap")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
CRSF_PORT = 5765

FRAMES = {0x02: "GPS", 0x08: "battery", 0x1E: "attitude", 0x21: "flight mode"}
# time after the last read that telemetry may still be stopped, plus the
# frames' own interval
ALLOWED_AFTER_READS = 1.5
SETTLE = 7.0        # seconds watched after the reads

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


def gaps(arrivals, t0, t1):
    """longest time without each frame type between t0 and t1"""
    out = {}
    for ftype, name in FRAMES.items():
        times = [t for (t, f) in arrivals if f == ftype]
        before = [t for t in times if t < t0]
        inside = [t for t in times if t0 <= t <= t1]
        points = before[-1:] + inside + [t1]
        out[name] = max(b - a for a, b in zip(points, points[1:])) if len(points) > 1 else None
    return out


os.makedirs(HERE, exist_ok=True)
parm = os.path.join(HERE, "test.parm")
with open(parm, "w") as f:
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -81
RC_OPTIONS 96
ARMING_RUDDER 0
SERIAL5_PROTOCOL 23
RC_PROTOCOLS 512
RC5_OPTION 153
FLTMODE_CH 6
SCR_ENABLE 0
""")

for port in (5760, CRSF_PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        if sock.connect_ex(("127.0.0.1", port)) == 0:
            sys.exit("TCP port %u is already in use; stop the running simulator first" % port)

proc = subprocess.Popen(
    [BIN, "--model", "plane", "--speedup", "1", "-w", "--home", "-35.363261,149.165230,584,353",
     "--serial5=tcp:%u" % CRSF_PORT,
     "--defaults", os.path.join(ROOT, "Tools/autotest/models/plane.parm") + "," + parm],
    cwd=HERE, stdout=open(os.path.join(HERE, "sitl_stdout.txt"), "w"), stderr=subprocess.STDOUT)

try:
    time.sleep(2)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
    m.wait_heartbeat(timeout=60)
    m.wait_gps_fix()

    def is_armed():
        while True:
            msg = m.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == "STATUSTEXT" and os.environ.get("GAP_DEBUG"):
                print("   ", msg.text)
        t0 = time.time()
        while time.time() - t0 < 3:
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.get_srcComponent() == 1:
                return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return None

    c = cm.CrsfClient(port=CRSF_PORT)
    c.record = True
    time.sleep(10)          # telemetry running, GPS frames flowing

    t_idle = time.time()
    time.sleep(5)
    idle = gaps(c.arrivals, t_idle, time.time())
    check("all four frames arrive when idle", all(g is not None and g < 2.5 for g in idle.values()),
          {k: round(v, 2) for k, v in idle.items() if v is not None})

    def measure(label, reads):
        t0 = time.time()
        ok = reads()
        t1 = time.time()
        time.sleep(SETTLE)
        g = gaps(c.arrivals, t0, time.time())
        shown = {k: (round(v, 2) if v is not None else None) for k, v in g.items()}
        print("  %s: reads took %.2f s, longest gaps %s" % (label, t1 - t0, shown), flush=True)
        return ok, t1 - t0, g

    # device open: ping, then every entry, as the ExpressLRS script does
    state = {}

    def device_open():
        info = c.ping(timeout=15)
        if info is None:
            return False
        state["entries"], missing = cm.load_device(c, info["field_count"])
        return not missing

    ok, took, g = measure("device open", device_open)
    check("device open loads every entry", ok)
    check("device open: telemetry stops only about as long as the reads",
          all(v is not None and v < took + ALLOWED_AFTER_READS for v in g.values()),
          {k: round(v, 2) for k, v in g.items() if v is not None})

    entries = state["entries"]
    tuning = [pid for pid, e in entries.items() if e["name"] == "6 Tuning"][0]
    children = sorted(pid for pid, e in entries.items() if e["parent"] == tuning)

    def folder_open():
        return all(c.read_entry(pid) is not None for pid in children)

    time.sleep(3)
    ok, took, g = measure("folder open (6 Tuning, %u entries)" % len(children), folder_open)
    check("folder open loads its entries", ok)
    check("folder open: telemetry stops only about as long as the reads",
          all(v is not None and v < took + ALLOWED_AFTER_READS for v in g.values()),
          {k: round(v, 2) for k, v in g.items() if v is not None})

    # armed: menu reads never stop telemetry
    m.set_mode("MANUAL")    # SITL starts in a mode that does not arm
    time.sleep(1)
    c.channels[4] = 2000
    armed = False
    t0 = time.time()
    while not armed and time.time() - t0 < 60:
        # the arm switch arms on its move: move it again until the EKF is ready
        armed = is_armed()
        if not armed:
            c.channels[4] = 988
            time.sleep(1)
            c.channels[4] = 2000
            time.sleep(2)
    check("armed via ch5", armed is True)
    ok, took, g = measure("folder open, armed", folder_open)
    check("armed: telemetry does not stop", all(v is not None and v < 2.5 for v in g.values()),
          {k: round(v, 2) for k, v in g.items() if v is not None})
    c.channels[4] = 988
    time.sleep(2)

    c.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
