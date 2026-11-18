#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL test of the menu's 6-position accelerometer calibration
("1 Sensors", "Calibrate accel"), driven over CRSF the way the
ExpressLRS Lua script drives it (see crsf_menu_client.py).

SITL runs its "calibration" model: servo outputs 5-8 turn the model. They
pass through RC channels 9-12 (SERVOn_FUNCTION RCIN9-12), so the test
puts the model in each pose from the CRSF RC frames: channel 9 at 1150
selects attitude control, channels 10-12 give roll, pitch and yaw
(1000-2000 us = -180 to 180 deg).

The simulated accelerometers get offsets and scale errors
(SIM_ACCn_BIAS, SIM_ACCn_SCAL), and the board starts uncalibrated.

Checks: arming is refused until the calibration is done; Exit in the
middle keeps the old offsets; the 6 poses calibrate both accelerometers
(INS_ACCOFFS and INS_ACCSCAL match the simulated errors) and set the level
trim from the first pose; the board restarts and then passes the
accelerometer pre-arm checks; "Calibrate level" afterwards changes only
the trim (AHRS_TRIM) and keeps the 6-position result.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_accel_cal_test.py
Needs pymavlink. Scratch files go to $GYRO_TEST_DIR (default /tmp/gyro_sitl_accel_cal).
"""
import math
import os
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crsf_menu_client as cm  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_accel_cal")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
CRSF_PORT = 5765

CMD_START, CMD_CONFIRM, CMD_CANCEL, CMD_POLL = 1, 4, 5, 6
CMD_READY, CMD_PROGRESS, CMD_CONFIRMATION_NEEDED = 0, 2, 3

# simulated accelerometer errors, as the stock AccelCal autotest
BIAS = [(0.7, -0.3, 1.8), (-2.1, 1.9, 2.3)]
SCALE = [(0.98, 1.12, 1.05), (1.11, 0.98, 0.96)]
# (roll, pitch) of the poses, in AP_AccelCal's order
POSES = [(0, 0), (-90, 0), (90, 0), (0, -89), (0, 89), (179.6, 0)]
LEVEL_PITCH = 3.0

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


def angle_us(deg):
    return int(round(1500 + deg * 1000.0 / 360.0))


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
AHRS_EKF_TYPE 0
GPS1_TYPE 0
SIM_GPS1_ENABLE 0
SERVO5_FUNCTION 59
SERVO6_FUNCTION 60
SERVO7_FUNCTION 61
SERVO8_FUNCTION 62
""")
    for i, (bias, scale) in enumerate(zip(BIAS, SCALE)):
        for axis, b, s in zip("XYZ", bias, scale):
            f.write("SIM_ACC%u_BIAS_%s %g\nSIM_ACC%u_SCAL_%s %g\n" % (i + 1, axis, b, i + 1, axis, s))

for port in (5760, CRSF_PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        if sock.connect_ex(("127.0.0.1", port)) == 0:
            sys.exit("TCP port %u is already in use; stop the running simulator first" % port)

# not -w: SITL restarts with its own arguments, and -w would erase the
# calibration at the restart that follows it
eeprom = os.path.join(HERE, "eeprom.bin")
if os.path.exists(eeprom):
    os.remove(eeprom)
proc = subprocess.Popen(
    [BIN, "--model", "calibration", "--speedup", "1", "--home", "-35.363261,149.165230,584,353",
     "--serial5=tcp:%u" % CRSF_PORT, "--defaults", parm],
    cwd=HERE, stdout=open(os.path.join(HERE, "sitl_stdout.txt"), "w"), stderr=subprocess.STDOUT)


class Link:
    """MAVLink with one reader thread: SITL slows down when nobody reads"""

    def __init__(self):
        self.m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
        self.m.wait_heartbeat(timeout=60)
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.lock = threading.Lock()
        self.params = {}
        self.texts = []
        self.armed = None
        self.att = (0.0, 0.0)
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            try:
                r = self.m.recv_match(blocking=True, timeout=1)
            except Exception:
                return
            if r is None:
                continue
            t = r.get_type()
            with self.lock:
                if t == "PARAM_VALUE":
                    self.params[r.param_id] = r.param_value
                elif t == "STATUSTEXT":
                    self.texts.append(r.text)
                    print("   TEXT:", r.text, flush=True)
                elif t == "HEARTBEAT" and r.get_srcComponent() == 1:
                    self.armed = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "SIMSTATE":
                    self.att = (math.degrees(r.roll), math.degrees(r.pitch))

    def get(self, name):
        with self.lock:
            self.params.pop(name, None)
        self.m.param_fetch_one(name)
        t0 = time.time()
        while time.time() - t0 < 5:
            with self.lock:
                if name in self.params:
                    return self.params[name]
            time.sleep(0.05)
        return None

    def vec(self, prefix):
        return [self.get("%s_%s" % (prefix, a)) for a in "XYZ"]

    def close(self):
        self.running = False
        self.m.close()


def connect():
    t0 = time.time()
    while time.time() - t0 < 60:
        try:
            link = Link()
            client = cm.CrsfClient(port=CRSF_PORT)
            return link, client
        except Exception:
            time.sleep(1)
    raise RuntimeError("SITL did not come back")


try:
    time.sleep(2)
    link, c = connect()
    c.channels[8] = 1150                 # calibration model: attitude control

    def pose(roll, pitch, timeout=20):
        """turn the model to (roll, pitch) and wait until it is there and still"""
        c.channels[9], c.channels[10], c.channels[11] = angle_us(roll), angle_us(pitch), 1500
        t0 = time.time()
        while time.time() - t0 < timeout:
            with link.lock:
                r, p = link.att
            if abs((r - roll + 180) % 360 - 180) < 3 and abs(p - pitch) < 3:
                time.sleep(1.5)          # settle
                return True
            time.sleep(0.1)
        return False

    def load():
        info = c.ping(timeout=15)
        entries, missing = cm.load_device(c, info["field_count"])
        return entries

    entries = load()

    def find(name, parent_name=None):
        for pid, e in sorted(entries.items()):
            if e["name"] == name and (parent_name is None
                                      or entries.get(e["parent"], {}).get("name") == parent_name):
                return pid
        raise KeyError(name)

    cal_folder = find("1 Sensors")
    names = [e["name"] for pid, e in sorted(entries.items()) if e["parent"] == cal_folder]
    check("1 Sensors: orientation, accel, level, GPS",
          names == ["Orientation", "Detect orientation", "Calibrate accel", "Calibrate level", "GPS"], names)

    def command(pid, action):
        r = c.write_entry(pid, [action])
        if r is None:
            return None
        e = cm.parse_entry(pid, r[1][4:])
        if os.environ.get("TRACE"):
            print("   CMD", action, "->", e["status"], e["info"], flush=True)
        return e["status"], e["info"]

    def poll_while_progress(pid, timeout=20):
        state = command(pid, CMD_POLL)
        t0 = time.time()
        while (state is None or state[0] == CMD_PROGRESS) and time.time() - t0 < timeout:
            time.sleep(0.25)
            state = command(pid, CMD_POLL)
        return state

    # 1. an uncalibrated board does not arm
    pose(0, 0)
    time.sleep(5)
    with link.lock:
        link.texts.clear()
    c.channels[4] = 2000
    time.sleep(3)
    c.channels[4] = 988
    with link.lock:
        texts = list(link.texts)
        armed = link.armed
    check("uncalibrated: arming refused with 3D Accel calibration needed",
          not armed and any("3D Accel calibration needed" in t for t in texts), texts)
    offs0 = link.vec("INS_ACCOFFS")

    # 2. Exit after the first pose keeps the old offsets
    acc = find("Calibrate accel", "1 Sensors")
    state = command(acc, CMD_START)
    check("first prompt asks for the level pose", state == (CMD_CONFIRMATION_NEEDED, "Pose 1/6: level, then OK"),
          state)
    command(acc, CMD_CONFIRM)
    state = poll_while_progress(acc)
    check("after pose 1 it asks for pose 2",
          state == (CMD_CONFIRMATION_NEEDED, "Pose 2/6: LEFT wing down, then OK"), state)
    state = command(acc, CMD_CANCEL)
    # late answers to earlier polls: a radio drops them once the popup closes
    time.sleep(1)
    c.purge(acc)
    check("Exit closes the calibration", state is not None and state[0] == CMD_READY, state)
    check("Exit keeps the old offsets", link.vec("INS_ACCOFFS") == offs0, link.vec("INS_ACCOFFS"))

    # 3. the 6 poses
    state = command(acc, CMD_START)
    prompts = []
    for n, (roll, pitch) in enumerate(POSES):
        reached = pose(roll, pitch)
        prompts.append(state[1] if state else None)
        if not reached or state is None or state[0] != CMD_CONFIRMATION_NEEDED:
            break
        command(acc, CMD_CONFIRM)
        state = poll_while_progress(acc)
    expected = all(p and p.startswith("Pose %u/6" % (i + 1)) for i, p in enumerate(prompts))
    check("one prompt for each pose", len(prompts) == 6 and expected, prompts)
    check("result: done, restarting", state == (CMD_CONFIRMATION_NEEDED, "Done, restarting"), state)

    # 4. the restart, then the stored result
    link.close()
    c.close()
    time.sleep(5)
    link, c = connect()
    c.channels[8] = 1150
    for i in range(2):
        prefix = "INS_ACC%sOFFS" % ("" if i == 0 else "2")
        offs = link.vec(prefix)
        scal = link.vec("INS_ACC%sSCAL" % ("" if i == 0 else "2"))
        ok_offs = all(o is not None and abs(o - b) < 0.05 for o, b in zip(offs, BIAS[i]))
        ok_scal = all(s is not None and abs(s - k) < 0.01 for s, k in zip(scal, SCALE[i]))
        check("accel %u offsets match the simulated bias" % (i + 1), ok_offs, (offs, BIAS[i]))
        check("accel %u scale matches the simulated scale" % (i + 1), ok_scal, (scal, SCALE[i]))
    trim = link.vec("AHRS_TRIM")
    check("level trim from pose 1 is near zero",
          all(t is not None and abs(t) < math.radians(1) for t in trim[:2]), trim)

    # 5. calibrated: the accelerometer pre-arm checks pass
    pose(0, 0)
    time.sleep(12)                       # SITL's two IMUs must agree for 10 s
    armed = False
    t0 = time.time()
    while time.time() - t0 < 30 and not armed:
        with link.lock:
            link.texts.clear()
        c.channels[4] = 2000
        time.sleep(2)
        with link.lock:
            armed = bool(link.armed)
            texts = list(link.texts)
        c.channels[4] = 988
        time.sleep(1)
    accel_texts = [t for t in texts if "ccel" in t]
    check("calibrated: arms with the INS checks on", armed, texts)
    check("no accelerometer pre-arm message", not accel_texts, accel_texts)
    c.channels[4] = 988
    time.sleep(2)

    # 6. Calibrate level afterwards: trim only, the 6-position result stays
    offs_before = link.vec("INS_ACCOFFS")
    scal_before = link.vec("INS_ACCSCAL")
    pose(0, LEVEL_PITCH)
    entries = load()
    level = find("Calibrate level", "1 Sensors")
    state = command(level, CMD_START)
    command(level, CMD_CONFIRM)
    state = poll_while_progress(level, timeout=30)
    check("level calibration done", state is not None and state[1] == "Calibration done", state)
    time.sleep(1)
    trim_y = link.get("AHRS_TRIM_Y")
    check("level calibration sets the pitch trim", trim_y is not None
          and abs(abs(math.degrees(trim_y)) - LEVEL_PITCH) < 0.5, None if trim_y is None else math.degrees(trim_y))
    check("level calibration keeps the accel offsets and scale",
          link.vec("INS_ACCOFFS") == offs_before and link.vec("INS_ACCSCAL") == scal_before,
          (link.vec("INS_ACCOFFS"), offs_before))

    link.close()
    c.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
