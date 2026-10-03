#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL check that autotune tunes both roll and pitch with no GPS and no
airspeed sensor, the way a gyro build usually flies.

The plane is armed with the gyro arm switch (ch5) and takes off in
MANUAL (mode switch low), climbs in FBWA (middle), then the mode switch
high selects AUTOTUNE. Roll and then pitch get alternating full stick
inputs, as in ArduPilot's own AUTOTUNE autotest, until autotune reports
"<axis>: Finished" for each.

The takeoff is in MANUAL because without a GPS the attitude estimate
(DCM) cannot tell the takeoff run's acceleration from a pitch change:
an FBWA takeoff in SITL pushes the nose into the runway.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_autotune_test.py
Needs pymavlink. Scratch files go to $GYRO_TEST_DIR (default /tmp/gyro_sitl_autotune).
"""
import os
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_autotune")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
SPEEDUP = 4
AXIS_TIMEOUT = 300      # simulated seconds allowed per axis

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


os.makedirs(HERE, exist_ok=True)
parm = os.path.join(HERE, "autotune.parm")
with open(parm, "w") as f:
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -1
RC5_OPTION 153
FLTMODE_CH 6
FLTMODE1 0
FLTMODE2 0
FLTMODE3 5
FLTMODE4 5
FLTMODE5 8
FLTMODE6 8
GYRO_GAIN_CH 8
AHRS_EKF_TYPE 0
SIM_GPS1_ENABLE 0
GPS1_TYPE 0
ARSPD_TYPE 0
""")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    if sock.connect_ex(("127.0.0.1", 5760)) == 0:
        sys.exit("TCP port 5760 is already in use; stop the running simulator first")

proc = subprocess.Popen(
    [BIN, "--model", "plane", "--speedup", str(SPEEDUP), "-w", "--home", "-35.363261,149.165230,584,353",
     "--defaults", os.path.join(ROOT, "Tools/autotest/models/plane.parm") + "," + parm],
    cwd=HERE, stdout=open(os.path.join(HERE, "sitl_stdout.txt"), "w"), stderr=subprocess.STDOUT)

try:
    time.sleep(2)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
    m.wait_heartbeat(timeout=60)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    # roll pitch thr yaw | arm mode rtl gain
    rc = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1500]
    state = {"mode": None, "armed": None, "alt": None, "boot_ms": 0, "texts": [], "params": {}}
    lock = threading.Lock()

    def rc_thread():
        while proc.poll() is None:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
            time.sleep(0.05)

    # one reader for everything: SITL slows down when MAVLink is not read
    def reader():
        while proc.poll() is None:
            try:
                r = m.recv_match(blocking=True, timeout=1)
            except Exception:
                return
            if r is None:
                continue
            t = r.get_type()
            with lock:
                if t == "HEARTBEAT" and r.get_srcComponent() == 1:
                    state["mode"] = mavutil.mode_string_v10(r)
                    state["armed"] = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "VFR_HUD":
                    state["alt"] = r.alt
                elif t == "SYSTEM_TIME":
                    state["boot_ms"] = r.time_boot_ms
                elif t == "STATUSTEXT":
                    state["texts"].append(r.text)
                    print("   TEXT:", r.text, flush=True)
                elif t == "PARAM_VALUE":
                    state["params"][r.param_id] = r.param_value

    threading.Thread(target=rc_thread, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()

    def sim_s():
        with lock:
            return state["boot_ms"] / 1000.0

    def wait_sim(seconds):
        t0 = sim_s()
        while sim_s() - t0 < seconds and proc.poll() is None:
            time.sleep(0.02)

    def get(key):
        with lock:
            return state[key]

    def get_param(name):
        with lock:
            state["params"].pop(name, None)
        m.param_fetch_one(name)
        t0 = time.time()
        while time.time() - t0 < 5:
            with lock:
                if name in state["params"]:
                    return state["params"][name]
            time.sleep(0.05)
        return None

    gains = ("RLL_RATE_P", "RLL_RATE_FF", "PTCH_RATE_P", "PTCH_RATE_FF")
    wait_sim(5)
    before = {g: get_param(g) for g in gains}

    def climb_to(height, timeout):
        t0 = sim_s()
        while sim_s() - t0 < timeout and (get("alt") or 0) - ground_alt < height:
            time.sleep(0.05)
        return (get("alt") or 0) - ground_alt

    # takeoff in MANUAL (mode switch low), climb in FBWA (middle)
    ground_alt = get("alt")
    rc[4] = 2000                        # arm switch
    wait_sim(2)
    check("armed with the arm switch", get("armed") is True, get("mode"))
    rc[2] = 2000                        # full throttle, neutral elevator
    climb_to(10, 40)
    rc[5] = 1500                        # FBWA
    climbed = climb_to(100, 60)
    check("took off and climbed without a GPS", climbed >= 100, "%.0f m" % climbed)
    rc[2] = 1700                        # cruise throttle

    # AUTOTUNE on the mode switch, then alternating stick inputs per axis
    rc[5] = 2000
    wait_sim(2)
    check("AUTOTUNE selected", get("mode") == "AUTOTUNE", get("mode"))
    for axis, ch in (("Roll", 0), ("Pitch", 1)):
        value = 1000
        t0 = sim_s()
        finished = False
        while sim_s() - t0 < AXIS_TIMEOUT and proc.poll() is None:
            with lock:
                finished = any(t.startswith("%s: Finished" % axis) for t in state["texts"])
            if finished:
                break
            value = 2000 if value == 1000 else 1000
            rc[ch] = value
            wait_sim(1)
        rc[0], rc[1] = 1500, 1500
        check("%s autotune finished" % axis, finished, "after %.0f s" % (sim_s() - t0))
        wait_sim(10)

    rc[5] = 1500                        # back to FBWA
    wait_sim(2)
    after = {g: get_param(g) for g in gains}
    for g in gains:
        changed = before[g] is not None and after[g] is not None and abs(after[g] - before[g]) > 1e-4
        check("%s changed" % g, changed, "%s -> %s" % (before[g], after[g]))
    m.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
