#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL check of the gyro-build behaviours (ArduPlane/gyro.cpp and the RTL guard)
with the arm-switch channel layout used by the Kakute defaults:
  ch5 arm, ch6 mode (MANUAL/ACRO/FBWA), ch7 RTL, ch8 gain knob.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_gyro_test.py
Needs pymavlink. Scratch files and the dataflash log go to $GYRO_TEST_DIR (default /tmp/gyro_sitl_test).

Checks:
  A. RTL refused with no home (disarmed and armed)
  B. trim capture on arming in MANUAL, and again on MANUAL -> ACRO
  F. nothing logged before arming, log starts on arming
  C. long RC failsafe falls back to FBWA when RTL is refused
  D. RTL accepted once GPS gives a home
  E. GYRO.Gain in the dataflash log follows the knob; TrimN counts both captures
  G. a GYRO_CAL calibration request is refused while armed
"""
import glob
import os
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_test")
os.makedirs(HERE, exist_ok=True)
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")

MANUAL, ACRO, FBWA, RTL = 0, 4, 5, 11
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

extra = os.path.join(HERE, "gyro_test.parm")
with open(extra, "w") as f:
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -1
ARMING_RUDDER 0
AHRS_EKF_TYPE 0
RC5_OPTION 153
FLTMODE_CH 6
FLTMODE1 0
FLTMODE2 0
FLTMODE3 4
FLTMODE4 4
FLTMODE5 5
FLTMODE6 5
RC7_OPTION 4
GYRO_TRIM_CAP 1
GYRO_GAIN_CH 8
GYRO_GAIN_MIN 0
GYRO_GAIN_MAX 1.5
FS_SHORT_ACTN 2
FS_SHORT_TIMEOUT 1
FS_LONG_ACTN 1
FS_LONG_TIMEOUT 5
THR_FAILSAFE 1
SIM_GPS1_ENABLE 0
LOG_DISARMED 0
""")

logdir = os.path.join(HERE, "sitl_run")
os.makedirs(logdir, exist_ok=True)
for f in glob.glob(os.path.join(logdir, "logs", "*")):
    os.remove(f)

# a leftover simulator on the same port would silently answer this test
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe_sock:
    if probe_sock.connect_ex(("127.0.0.1", 5760)) == 0:
        sys.exit("TCP port 5760 is already in use; stop the running simulator first")

proc = subprocess.Popen(
    [BIN, "--model", "plane", "--speedup", "5", "-w",
     "--home", "-35.363261,149.165230,584,353",
     "--defaults", os.path.join(ROOT, "Tools/autotest/models/plane.parm") + "," + extra],
    cwd=logdir, stdout=open(os.path.join(logdir, "sitl_stdout.txt"), "w"), stderr=subprocess.STDOUT)

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + detail if detail else ""), flush=True)


def log_bytes():
    return sum(os.path.getsize(f) for f in glob.glob(os.path.join(logdir, "logs", "*.BIN")))


try:
    time.sleep(2)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
    m.wait_heartbeat(timeout=60)
    print("connected, sysid", m.target_system, flush=True)

    # RC override sender thread: roll pitch thr yaw | arm mode rtl gain
    rc = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1500]
    send_rc = [True]

    def rc_thread():
        while proc.poll() is None:
            if send_rc[0]:
                m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
            time.sleep(0.05)
    threading.Thread(target=rc_thread, daemon=True).start()

    texts = []
    state = {"mode": None, "armed": None}

    def pump(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "STATUSTEXT":
                texts.append(msg.text)
                print("   TEXT:", msg.text, flush=True)
            elif t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    print("   HEARTBEAT from component", msg.get_srcComponent(), "mode", msg.custom_mode, flush=True)
                    continue
                if msg.custom_mode != state["mode"]:
                    print("   MODE:", state["mode"], "->", msg.custom_mode, flush=True)
                state["mode"] = msg.custom_mode
                state["armed"] = bool(msg.base_mode & ARMED_FLAG)
        return state["mode"]

    def get_param(name):
        m.param_fetch_one(name)
        for _ in range(10):
            r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
            if r is not None and r.param_id == name:
                return r.param_value
        return None

    def set_param(name, value):
        m.param_set_send(name, float(value))
        return get_param(name)

    def request_mode(mode):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0)

    # let it boot and settle disarmed in MANUAL
    pump(15)
    check("boot in MANUAL, disarmed", state["mode"] == MANUAL and state["armed"] is False,
          f"mode={state['mode']} armed={state['armed']}")
    check("GPS disabled", get_param("SIM_GPS1_ENABLE") == 0)

    # A. RTL refused with no home (disarmed)
    texts.clear()
    request_mode(RTL)
    pump(3)
    check("A: RTL refused without home",
          state["mode"] == MANUAL and any("RTL refused" in t for t in texts),
          f"mode={state['mode']}")

    # F/B. arm with trims offset: log starts, trims captured
    check("F: nothing logged before arming", log_bytes() == 0, f"bytes={log_bytes()}")
    rc[0], rc[1] = 1530, 1480
    pump(2)
    texts.clear()
    rc[4] = 2000  # arm switch
    pump(4)
    check("B: armed via channel 5", state["armed"] is True)
    check("B: trims captured on arming", any("trim complete" in t for t in texts))
    r1, r2 = get_param("RC1_TRIM"), get_param("RC2_TRIM")
    check("B: RC1_TRIM captured", r1 == 1530, f"RC1_TRIM={r1}")
    check("B: RC2_TRIM captured", r2 == 1480, f"RC2_TRIM={r2}")
    check("F: log started on arming", log_bytes() > 0, f"bytes={log_bytes()}")

    # B2. new trim offset in MANUAL, then MANUAL -> ACRO captures again
    rc[0], rc[1] = 1470, 1520
    pump(2)
    texts.clear()
    rc[5] = 1500  # mode switch mid = ACRO
    pump(3)
    check("B2: ACRO entered", state["mode"] == ACRO, f"mode={state['mode']}")
    check("B2: trims captured on MANUAL->ACRO", any("trim complete" in t for t in texts))
    r1 = get_param("RC1_TRIM")
    check("B2: RC1_TRIM updated", r1 == 1470, f"RC1_TRIM={r1}")

    # G. calibration request is refused while armed
    texts.clear()
    set_param("GYRO_CAL", 1)
    pump(2)
    check("G: calibration refused while armed", get_param("GYRO_CAL") == -1
          and any("disarm to calibrate" in t for t in texts))

    # E (part 1): gain knob high, then low, to leave marks in the log
    rc[7] = 2000
    pump(3)
    rc[7] = 1000
    pump(3)
    rc[7] = 1500
    pump(2)

    # C. RC failsafe in ACRO with no home: short -> FBWA, long -> RTL refused -> FBWA
    texts.clear()
    send_rc[0] = False
    set_param("SIM_RC_FAIL", 1)  # no RC pulses from the simulator either
    pump(20)
    check("C: long failsafe ended in FBWA", state["mode"] == FBWA, f"mode={state['mode']}")
    check("C: RTL refused during failsafe", any("RTL refused" in t for t in texts))
    check("C: still armed through failsafe", state["armed"] is True)

    # D. recover RC, enable GPS, wait for home, RTL should now work
    rc[5] = 1000
    set_param("SIM_RC_FAIL", 0)
    send_rc[0] = True
    pump(3)
    set_param("SIM_GPS1_ENABLE", 1)
    got_home = False
    for _ in range(12):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("   TEXT:", msg.text, flush=True)
            if msg.get_type() == "HOME_POSITION" and msg.latitude != 0:
                got_home = True
                break
        if got_home:
            break
    check("D: home set after GPS enable", got_home)
    texts.clear()
    request_mode(RTL)
    pump(3)
    check("D: RTL accepted with home", state["mode"] == RTL, f"mode={state['mode']}")
    request_mode(MANUAL)
    rc[4] = 1000  # disarm
    pump(3)
    check("D: disarmed via channel 5", state["armed"] is False)

finally:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:
        proc.kill()

# E (part 2): inspect the dataflash log
logs = sorted(glob.glob(os.path.join(logdir, "logs", "*.BIN")))
if not logs:
    check("E: log file written", False, "no .BIN in " + logdir)
else:
    from pymavlink import DFReader
    gains = []
    trimn = 0
    d = DFReader.DFReader_binary(logs[-1])
    while True:
        msg = d.recv_msg()
        if msg is None:
            break
        if msg.get_type() == "GYRO":
            gains.append(round(msg.Gain, 2))
            trimn = max(trimn, msg.TrimN)
    check("E: GYRO messages logged", len(gains) > 0, f"n={len(gains)}")
    check("E: gain reached 1.5 at knob high", 1.5 in gains, f"max={max(gains) if gains else None}")
    check("E: gain reached 0.0 at knob low", 0.0 in gains, f"min={min(gains) if gains else None}")
    check("E: gain 1.0 at knob centre", 1.0 in gains)
    check("E: TrimN counts both captures", trimn >= 2, f"TrimN={trimn}")

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
