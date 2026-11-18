#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL check of the gyro build with its GPS setting on (menu: 5 Failsafe,
GPS): GPS1_TYPE 1, EKF3 (AHRS_EKF_TYPE 3) and the GPS arming checks, no
compass, no airspeed sensor. With the GPS off the build flies on DCM, as
sitl_turn_test.py and sitl_autotune_test.py check.

The simulator starts with no GPS signal: arming must be refused until
there is a fix and EKF3 has started, so EKF3 never starts in the air
(started in a turn, it took its first attitude from the turn's tilted
accelerations and was 37 degrees wrong for 9 s). With the signal on,
the plane takes off in MANUAL, climbs in FBWA, then:
  1. holds a full-stick turn on EKF3: attitude estimate true, no spiral
  2. flies with the rudder pumped left and right: the airspeed estimate
     stays near the true speed (DCM's GPS wind estimate read such yaw
     swings as wind, the airspeed estimate fell to zero and the gains
     doubled)
  3. loses the GPS: EKF3 goes on without it (dead reckoning; the AHRS
     uses DCM only for a moment) and a held turn stays true
  4. gets the GPS back: EKF3 uses it again with no attitude jump
The simulator's true attitude (SIMSTATE) is compared with the estimate
(ATTITUDE). SITL has no wind, so the true airspeed is the ground speed.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_gps_optional_test.py
Needs pymavlink. Scratch files go to $GYRO_TEST_DIR (default /tmp/gyro_sitl_gps_optional).
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
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_gps_optional")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
SPEEDUP = 4
BANK_LIMIT = 45
HOLD_S = 20

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


os.makedirs(HERE, exist_ok=True)
parm = os.path.join(HERE, "gps_optional.parm")
with open(parm, "w") as f:
    # SCALING_SPEED 19 is near the simulated plane's speed at 45% throttle
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -4185
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
GPS1_TYPE 1
EK3_ENABLE 1
AHRS_EKF_TYPE 3
EK2_ENABLE 0
SIM_GPS1_ENABLE 0
COMPASS_ENABLE 0
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
             "true_roll": 0.0, "true_pitch": 0.0, "nav_roll": 0.0, "airspeed": 0.0, "groundspeed": 0.0,
             "ahrs": None, "fix": 0, "prearm": [], "ahrs_changes": []}
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
                elif t == "VFR_HUD":
                    state["airspeed"] = r.airspeed
                    state["groundspeed"] = r.groundspeed
                elif t == "GPS_RAW_INT":
                    state["fix"] = r.fix_type
                elif t == "STATUSTEXT":
                    print("   TEXT:", r.text, flush=True)
                    if r.text.startswith("PreArm") or r.text.startswith("Arm:"):
                        state["prearm"].append(r.text)
                    if r.text.startswith("AHRS: ") and r.text.endswith(" active"):
                        state["ahrs"] = r.text[6:-7]
                        state["ahrs_changes"].append((state["ahrs"], state["armed"]))

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

    def wait_until(test, timeout):
        t0 = sim_s()
        while sim_s() - t0 < timeout and not test():
            time.sleep(0.05)
        return test(), sim_s() - t0

    def height():
        return (get("alt") or 0) - ground_alt

    def climb_to(target, timeout):
        t0 = sim_s()
        while sim_s() - t0 < timeout and height() < target:
            time.sleep(0.05)
        return height()

    def set_param(name, value):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def held_turn(label):
        start = height()
        rc[0] = 2000
        peak_demand = worst_roll = worst_pitch = 0.0
        for second in range(HOLD_S):
            wait_sim(1)
            peak_demand = max(peak_demand, abs(get("nav_roll")))
            if second >= 2:
                worst_roll = max(worst_roll, abs(get("true_roll") - get("roll")))
                worst_pitch = max(worst_pitch, abs(get("true_pitch") - get("pitch")))
        lost = start - height()
        rc[0] = 1500
        check("%s: bank demand reaches ROLL_LIMIT_DEG" % label, peak_demand >= BANK_LIMIT - 1, "%.0f deg" % peak_demand)
        check("%s: bank estimate stays true" % label, worst_roll < 8, "worst error %.1f deg" % worst_roll)
        check("%s: pitch estimate stays true" % label, worst_pitch < 8, "worst error %.1f deg" % worst_pitch)
        check("%s: no spiral dive" % label, lost < 30, "%.0f m lost in %u s" % (lost, HOLD_S))
        wait_sim(6)

    def try_arm(timeout):
        """flip the arm switch until the plane arms; the switch arms on its move"""
        t0 = sim_s()
        while sim_s() - t0 < timeout:
            rc[4] = 2000
            wait_sim(2)
            if get("armed"):
                return True
            rc[4] = 1000
            wait_sim(1)
        return False

    # no GPS signal: arming is refused with a GPS reason
    wait_sim(20)
    ground_alt = get("alt")
    armed = try_arm(15)
    reasons = sorted(set(get("prearm")))
    check("GPS on, no fix: arming refused", not armed and any("GPS" in t or "AHRS" in t for t in reasons), reasons)
    rc[4] = 1000

    # GPS signal on: arming works once EKF3 has started with the fix
    set_param("SIM_GPS1_ENABLE", 1)
    armed = try_arm(90)
    check("arms once the GPS has a fix", armed and get("fix") >= 3, "fix %s" % get("fix"))
    changes = get("ahrs_changes")
    check("EKF3 had started before arming", ("EKF3", False) in changes, changes)
    ground_alt = get("alt")

    # takeoff in MANUAL (mode switch low), climb in FBWA (middle). Without
    # a compass EKF3 aligns its heading from the GPS track once moving: the
    # AHRS uses DCM until then (stock behavior), then changes to EKF3
    def climb_watching(target, timeout):
        """climb_to, also returning the worst attitude error on the way"""
        worst = 0.0
        t0 = sim_s()
        while sim_s() - t0 < timeout and height() < target:
            worst = max(worst, abs(get("true_roll") - get("roll")), abs(get("true_pitch") - get("pitch")))
            time.sleep(0.05)
        return height(), worst

    rc[2] = 2000
    _, w1 = climb_watching(10, 40)
    rc[5] = 1500
    climbed, w2 = climb_watching(120, 60)
    worst = max(w1, w2)
    check("takeoff and climb: attitude stays true through the change to EKF3", worst < 5,
          "worst error %.1f deg, AHRS changes %s" % (worst, get("ahrs_changes")[len(changes):]))
    check("took off and climbed with a GPS", climbed >= 120, "%.0f m" % climbed)
    rc[2] = 1700
    wait_sim(10)
    check("EKF3 flies with a GPS", get("ahrs") in (None, "EKF3"), get("ahrs"))

    # 1. held turn on EKF3
    held_turn("GPS")

    # 2. rudder pumped left and right in level flight
    worst = 0.0
    for i in range(30):
        rc[3] = 2000 if i % 2 else 1000
        wait_sim(0.35)
        worst = max(worst, abs(get("airspeed") - get("groundspeed")))
    rc[3] = 1500
    for _ in range(10):
        wait_sim(1)
        worst = max(worst, abs(get("airspeed") - get("groundspeed")))
    check("rudder pumping: airspeed estimate stays near the true speed", worst < 4,
          "worst difference %.1f m/s" % worst)
    check("rudder pumping: still on EKF3", get("ahrs") in (None, "EKF3"), get("ahrs"))

    # 3. GPS lost in flight
    set_param("SIM_GPS1_ENABLE", 0)
    ok, took = wait_until(lambda: get("fix") < 3, 30)
    check("GPS lost: the fix goes away", ok, "after %.0f s" % took)
    worst = 0.0
    for _ in range(40):
        wait_sim(0.25)
        worst = max(worst, abs(get("true_roll") - get("roll")), abs(get("true_pitch") - get("pitch")))
    check("GPS lost: attitude stays true", worst < 5, "worst error %.1f deg, AHRS %s" % (worst, get("ahrs")))
    held_turn("no GPS")
    check("no GPS: an attitude estimate is active", get("ahrs") in ("EKF3", "DCM"), get("ahrs"))

    # 4. GPS back
    set_param("SIM_GPS1_ENABLE", 1)
    worst = 0.0
    t0 = sim_s()
    while sim_s() - t0 < 60 and not (get("fix") >= 3 and get("ahrs") == "EKF3"):
        worst = max(worst, abs(get("true_roll") - get("roll")), abs(get("true_pitch") - get("pitch")))
        time.sleep(0.05)
    ok = get("fix") >= 3 and get("ahrs") == "EKF3"
    check("GPS back: fix again, EKF3 active", ok, "after %.0f s" % (sim_s() - t0))
    for _ in range(40):
        wait_sim(0.25)
        worst = max(worst, abs(get("true_roll") - get("roll")), abs(get("true_pitch") - get("pitch")))
    check("GPS back: no attitude jump", worst < 5, "worst error %.1f deg" % worst)
    held_turn("GPS back")
    m.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
