#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

End-to-end SITL test of the gyro setup menu over CRSF, driven the way the
ExpressLRS Lua script drives it (see crsf_menu_client.py).

Build first, with the gyro-like SITL configuration:
    ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef
    ./waf plane
Run:
    python3 Tools/gyro/sitl_crsf_menu_test.py
Needs pymavlink.

Checks: every entry 1..count loads with a valid parent; selection, number
and mode-position writes reach the parameters; calibration runs as a
confirm -> progress -> result popup; stick learning detects a TAER radio
and a reversed elevator; the arm switch is only assigned while it is off;
the wing template sets output channels 1-4; commands are refused while armed.
"""
import os
import socket
import struct
import subprocess
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crsf_menu_client as cm  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_crsf")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
CRSF_PORT = 5765

CMD_START, CMD_CONFIRM, CMD_CANCEL, CMD_POLL = 1, 4, 5, 6
CMD_READY, CMD_PROGRESS, CMD_CONFIRMATION_NEEDED = 0, 2, 3

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


os.makedirs(HERE, exist_ok=True)
parm = os.path.join(HERE, "test.parm")
with open(parm, "w") as f:
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -1
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

    def drain():
        # ArduPilot broadcasts parameter changes and heartbeats keep queueing:
        # throw away everything already received so checks see fresh data
        while m.recv_match(blocking=False) is not None:
            pass

    def get_param(name):
        drain()
        m.param_fetch_one(name)
        t0 = time.time()
        while time.time() - t0 < 5:
            r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if r is not None and r.param_id == name:
                return r.param_value
        return None

    def set_param(name, value):
        m.param_set_send(name, float(value))
        return get_param(name)

    def is_armed():
        drain()
        t0 = time.time()
        while time.time() - t0 < 3:
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is not None and hb.get_srcComponent() == 1:
                return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return None

    time.sleep(2)
    c = cm.CrsfClient(port=CRSF_PORT)

    # 1. the whole menu loads
    info = c.ping(timeout=15)
    check("device answers ping", info is not None, info)
    with open(os.path.join(ROOT, "Tools/gyro/sitl-gyro.hwdef")) as hw:
        expected = [ln.split('"')[1] for ln in hw if ln.startswith("define AP_CRSF_DEVICE_NAME")][0]
    check("device name", info is not None and info["name"] == expected, info["name"] if info else None)
    entries, missing = cm.load_device(c, info["field_count"])
    check("all entries load", not missing and len(entries) == info["field_count"],
          "%d of %d, missing %s" % (len(entries), info["field_count"], missing))
    folders = {pid for pid, e in entries.items() if e["type"] == "FOLDER"}
    bad_parent = [pid for pid, e in entries.items() if e["parent"] != 0 and e["parent"] not in folders]
    check("every parent is a folder", not bad_parent, bad_parent)
    top = [e["name"] for pid, e in sorted(entries.items()) if e["parent"] == 0]
    check("top level is Status then the folders",
          top == ["Status", "1 Calibration", "2 Radio", "3 Flight modes", "4 Outputs", "5 Failsafe", "6 Tuning",
                  "Tools"], top)

    def find(name, parent_name=None):
        for pid, e in sorted(entries.items()):
            if e["name"] != name:
                continue
            if parent_name is None or entries.get(e["parent"], {}).get("name") == parent_name:
                return pid
        raise KeyError(name)

    def command(pid, action):
        r = c.write_entry(pid, [action])
        if r is None:
            return None
        e = cm.parse_entry(pid, r[1][4:])
        return e["status"], e["info"]

    def run_command(pid, poll_until=CMD_CONFIRMATION_NEEDED, timeout=20):
        """start a command, confirm its first prompt, poll until the given status"""
        first = command(pid, CMD_START)
        if first is None or first[0] != CMD_CONFIRMATION_NEEDED:
            return first, None
        state = command(pid, CMD_CONFIRM)
        t0 = time.time()
        # a calibration blocks the main loop for a while: keep polling
        # through missed replies, as the radio does
        while (state is None or state[0] == CMD_PROGRESS) and time.time() - t0 < timeout:
            time.sleep(0.3)
            state = command(pid, CMD_POLL)
        # polls sent while the main loop was busy are answered late; a radio
        # stops polling once a result is shown, so drop those answers
        time.sleep(1.0)
        c.purge(pid)
        return first, state

    # 2. selection write: output 5 function
    out5_func = find("Function", "Channel 5")
    opts = entries[out5_func]["options"]
    r = c.write_entry(out5_func, [opts.index("Aileron")])
    check("select write: output 5 = Aileron", r is not None and get_param("SERVO5_FUNCTION") == 4,
          get_param("SERVO5_FUNCTION"))
    c.write_entry(out5_func, [opts.index("None")])

    # 3. number write with clamping
    roll_rate = find("ACRO roll rate")
    c.write_entry(roll_rate, struct.pack(">i", 190))
    check("number write: ACRO roll rate 190", get_param("ACRO_ROLL_RATE") == 190, get_param("ACRO_ROLL_RATE"))
    c.write_entry(roll_rate, struct.pack(">i", 5000))
    check("number write clamps to max", get_param("ACRO_ROLL_RATE") == 500, get_param("ACRO_ROLL_RATE"))
    c.write_entry(roll_rate, struct.pack(">i", 180))

    # 4. mode position: Low -> FBWA sets FLTMODE1 and 2
    low = find("Low")
    c.write_entry(low, [entries[low]["options"].index("FBWA")])
    check("mode position Low = FBWA", get_param("FLTMODE1") == 5 and get_param("FLTMODE2") == 5,
          (get_param("FLTMODE1"), get_param("FLTMODE2")))
    c.write_entry(low, [entries[low]["options"].index("MANUAL")])

    # 5. calibration popup flow
    first, final = run_command(find("Calibrate gyros"))
    check("calibration asks to keep still", first is not None and first[0] == CMD_CONFIRMATION_NEEDED, first)
    check("calibration reports done", final is not None and final[1] == "Calibration done", final)
    closed = command(find("Calibrate gyros"), CMD_CONFIRM)
    check("OK closes the result", closed is not None and closed[0] == CMD_READY, closed)

    # 6. wing template
    wing = find("Wing type")
    c.write_entry(wing, [entries[wing]["options"].index("Flying wing")])
    first, final = run_command(find("Apply template"))
    check("template asks to confirm", first is not None and "Flying wing" in first[1], first)
    check("flying wing sets elevons", get_param("SERVO1_FUNCTION") == 77 and get_param("SERVO2_FUNCTION") == 78,
          (get_param("SERVO1_FUNCTION"), get_param("SERVO2_FUNCTION")))
    command(find("Apply template"), CMD_CONFIRM)
    c.write_entry(wing, [entries[wing]["options"].index("Normal")])
    run_command(find("Apply template"))
    command(find("Apply template"), CMD_CONFIRM)
    check("normal template restores outputs", get_param("SERVO1_FUNCTION") == 4 and get_param("SERVO2_FUNCTION") == 19)

    # 7. stick learning on a TAER radio with a reversed elevator
    learn = find("Learn sticks")
    c.channels[:4] = [988, 1500, 1500, 1500]          # throttle on ch1, low
    time.sleep(0.5)
    state = command(learn, CMD_START)
    check("stick learning asks for aileron", state is not None and "AILERON" in state[1], state)
    # aileron ch2 right, elevator ch3 back (reversed), rudder ch4 right, throttle ch1 full
    moves = [(1, 2000), (2, 1000), (3, 2000), (0, 2000)]
    for ch, value in moves:
        c.channels[ch] = value
        time.sleep(0.4)
        state = command(learn, CMD_POLL)
        c.channels[ch] = 988 if ch == 0 else 1500
        time.sleep(0.4)
        state = command(learn, CMD_POLL)
    check("stick learning detects TAER", state is not None and state[1] == "A2 E3 R4 T1 Save?", state)
    command(learn, CMD_CANCEL)
    check("cancel leaves channel map alone", get_param("RCMAP_ROLL") == 1)

    # 8. learning on an AETR radio saves reversals without a restart
    c.channels[:4] = [1500, 1500, 988, 1500]
    time.sleep(0.5)
    command(learn, CMD_START)
    for ch, value in [(0, 2000), (1, 1000), (3, 2000), (2, 2000)]:
        c.channels[ch] = value
        time.sleep(0.4)
        command(learn, CMD_POLL)
        c.channels[ch] = 988 if ch == 2 else 1500
        time.sleep(0.4)
        state = command(learn, CMD_POLL)
    state = command(learn, CMD_CONFIRM)
    check("AETR learning saves", state is not None and state[1] == "Sticks saved", state)
    check("elevator reversal saved", get_param("RC2_REVERSED") == 1, get_param("RC2_REVERSED"))
    c.write_entry(learn, [CMD_CONFIRM])
    set_param("RC2_REVERSED", 0)

    # 9. arm switch learning: only assigned once the switch is off
    learn_arm = find("Learn arm switch")
    state = command(learn_arm, CMD_START)
    check("arm learning prompts", state is not None and "arm switch" in state[1], state)
    c.channels[8] = 2000                                  # flip ch9 high
    time.sleep(0.5)
    state = command(learn_arm, CMD_POLL)
    check("arm learning asks to switch off", state is not None and "switch it off" in state[1], state)
    state = command(learn_arm, CMD_CONFIRM)               # still high: must not assign
    check("arm not assigned while switch on", get_param("RC9_OPTION") == 0 and "off first" in state[1], state)
    check("vehicle did not arm", is_armed() is False)
    c.channels[8] = 988
    time.sleep(0.5)
    state = command(learn_arm, CMD_CONFIRM)
    check("arm assigned once off", get_param("RC9_OPTION") == 153 and get_param("RC5_OPTION") == 0,
          (state, get_param("RC9_OPTION"), get_param("RC5_OPTION")))
    command(learn_arm, CMD_CONFIRM)
    arm_sw = find("Arm switch")
    c.write_entry(arm_sw, [entries[arm_sw]["options"].index("Ch5")])
    check("arm switch back on ch5", get_param("RC5_OPTION") == 153 and get_param("RC9_OPTION") == 0)

    # 10. commands are refused while armed
    c.channels[4] = 2000
    time.sleep(2)
    check("armed via ch5", is_armed() is True)
    state = command(find("Calibrate level"), CMD_START)
    check("calibration refused while armed", state is not None and state[1] == "Disarm first", state)
    command(find("Calibrate level"), CMD_CONFIRM)
    c.channels[4] = 988
    time.sleep(2)
    check("disarmed via ch5", is_armed() is False)

    c.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
