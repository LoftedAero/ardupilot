#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL check of a held FBWA turn with no GPS and no airspeed sensor.

Without a speed the attitude estimate (DCM) has no centripetal
correction: in a turn it follows the accelerometers towards a false
level, FBWA rolls further to hold the bank it reads, and the plane
spirals down. Stock stall prevention also limits the bank to 25 degrees
when there is no airspeed estimate. The gyro build skips that limit and,
with GYRO_TURN_CORR 1, gives the estimate an assumed airspeed.

The plane takes off in MANUAL, climbs in FBWA, then holds full roll
stick for 20 s. The simulator's true attitude (SIMSTATE) is compared
with the estimate (ATTITUDE).

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_turn_test.py
Needs pymavlink. Scratch files go to $GYRO_TEST_DIR (default /tmp/gyro_sitl_turn).
"""
import math
import os
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_turn")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
SPEEDUP = 4
BANK_LIMIT = 45
HOLD_S = 20

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


os.makedirs(HERE, exist_ok=True)
parm = os.path.join(HERE, "turn.parm")
with open(parm, "w") as f:
    # SCALING_SPEED 19 is near the simulated plane's speed at 45% throttle
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -81
RC_OPTIONS 96
RC5_OPTION 153
FLTMODE_CH 6
FLTMODE1 0
FLTMODE2 0
FLTMODE3 5
FLTMODE4 5
FLTMODE5 5
FLTMODE6 5
GYRO_GAIN_CH 8
GYRO_TURN_CORR 1
SCALING_SPEED 19
ROLL_LIMIT_DEG %u
AHRS_EKF_TYPE 0
SIM_GPS1_ENABLE 0
GPS1_TYPE 0
ARSPD_TYPE 0
""" % BANK_LIMIT)

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
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # roll pitch thr yaw | arm mode rtl gain
    rc = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1500]
    state = {"mode": None, "armed": None, "alt": None, "boot_ms": 0, "roll": 0.0, "pitch": 0.0,
             "true_roll": 0.0, "true_pitch": 0.0, "nav_roll": 0.0}
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
                elif t == "GLOBAL_POSITION_INT":
                    # the height above home from the barometer: with no GPS
                    # position there is no absolute altitude (VFR_HUD reads 0)
                    state["alt"] = r.relative_alt * 0.001
                elif t == "SYSTEM_TIME":
                    state["boot_ms"] = r.time_boot_ms
                elif t == "ATTITUDE":
                    state["roll"] = math.degrees(r.roll)
                    state["pitch"] = math.degrees(r.pitch)
                elif t == "SIMSTATE":
                    state["true_roll"] = math.degrees(r.roll)
                    state["true_pitch"] = math.degrees(r.pitch)
                elif t == "NAV_CONTROLLER_OUTPUT":
                    state["nav_roll"] = r.nav_roll
                elif t == "STATUSTEXT":
                    print("   TEXT:", r.text, flush=True)

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

    def climb_to(height, timeout):
        t0 = sim_s()
        while sim_s() - t0 < timeout and (get("alt") or 0) - ground_alt < height:
            time.sleep(0.05)
        return (get("alt") or 0) - ground_alt

    # takeoff in MANUAL (mode switch low), climb in FBWA (middle)
    wait_sim(5)
    ground_alt = get("alt")
    # the arm switch arms on its move: flip it again while a pre-arm
    # check (SITL's two IMUs must agree for 10 s after boot) refuses
    t_arm = sim_s()
    while not get("armed") and sim_s() - t_arm < 30:
        rc[4] = 2000
        wait_sim(2)
        if not get("armed"):
            rc[4] = 1000
            wait_sim(1)
    check("armed with the arm switch", get("armed") is True, get("mode"))
    rc[2] = 2000                        # full throttle, neutral elevator
    climb_to(10, 40)
    rc[5] = 1500                        # FBWA
    climbed = climb_to(120, 60)
    check("took off and climbed without a GPS", climbed >= 120, "%.0f m" % climbed)
    rc[2] = 1700                        # cruise throttle
    wait_sim(5)

    # hold full roll stick, elevator neutral
    start_height = (get("alt") or 0) - ground_alt
    rc[0] = 2000
    peak_demand = 0.0
    worst_roll = 0.0
    worst_pitch = 0.0
    for second in range(HOLD_S):
        wait_sim(1)
        peak_demand = max(peak_demand, abs(get("nav_roll")))
        if second >= 2:
            # after the roll into the turn
            worst_roll = max(worst_roll, abs(get("true_roll") - get("roll")))
            worst_pitch = max(worst_pitch, abs(get("true_pitch") - get("pitch")))
    lost = start_height - ((get("alt") or 0) - ground_alt)
    check("bank demand reaches ROLL_LIMIT_DEG", peak_demand >= BANK_LIMIT - 1, "%.0f deg" % peak_demand)
    check("bank estimate stays true in the turn", worst_roll < 8, "worst error %.1f deg" % worst_roll)
    check("pitch estimate stays true in the turn", worst_pitch < 8, "worst error %.1f deg" % worst_pitch)
    check("the turn does not become a spiral dive", lost < 30, "%.0f m lost in %u s" % (lost, HOLD_S))

    rc[0] = 1500
    wait_sim(5)
    check("level after the turn", abs(get("true_roll")) < 5, "true bank %.1f deg" % get("true_roll"))
    m.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
