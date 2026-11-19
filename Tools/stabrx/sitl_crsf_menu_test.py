#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

End-to-end SITL test of the gyro setup menu over CRSF, driven the way the
ExpressLRS Lua script drives it (see crsf_menu_client.py).

Build first, with the gyro-like SITL configuration:
    ./waf configure --board sitl --extra-hwdef=Tools/stabrx/sitl-stabrx.hwdef
    ./waf plane
Run:
    python3 Tools/stabrx/sitl_crsf_menu_test.py
Needs pymavlink.

Checks: every entry 1..count loads with a valid parent; selection, number
and mode-position writes reach the parameters; calibration runs as a
confirm -> progress -> result popup; stick learning detects a TAER radio
and a reversed elevator; the arm switch is only assigned while it is off;
the wing type sets output channels 1-4; an open command refuses arming
and ends by itself when the radio stops asking; commands and every write
but the tuning numbers are refused while armed.
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
HERE = os.environ.get("STABRX_TEST_DIR", "/tmp/stabrx_sitl_crsf")
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
    with open(os.path.join(ROOT, "Tools/stabrx/sitl-stabrx.hwdef")) as hw:
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
          top == ["Status", "1 Sensors", "2 Radio", "3 Flight modes", "4 Outputs", "5 Failsafe", "6 Tuning",
                  "Tools"], top)

    def find(name, parent_name=None):
        for pid, e in sorted(entries.items()):
            if e["name"] != name:
                continue
            if parent_name is None or entries.get(e["parent"], {}).get("name") == parent_name:
                return pid
        raise KeyError(name)

    version = entries.get(find("Menu version", "Tools"), {})
    check("the menu version is hidden in Tools", version.get("hidden") and version.get("shown") == "4",
          version)

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

    too_precise = [e["name"] for e in entries.values() if e["type"] == "FLOAT" and e.get("decimals", 0) > 3]
    check("no number has more than 3 decimals (ExpressLRS limit)", not too_precise, too_precise)

    # 3a. 6 Tuning holds the gain and speed folders
    tuning_id = find("6 Tuning")
    tuning_folders = [e["name"] for pid, e in sorted(entries.items())
                      if e["parent"] == tuning_id and e["type"] == "FOLDER"]
    check("6 Tuning holds the gain and speed folders",
          tuning_folders == ["Speeds", "Roll gains", "Pitch gains", "Yaw gains"], tuning_folders)
    for folder, prefix, axis in (("Roll gains", "RLL", "roll"), ("Pitch gains", "PTCH", "pitch")):
        names = [e["name"] for pid, e in sorted(entries.items()) if e["parent"] == find(folder)]
        check("%s holds FF, P, I, D and the ACRO rate" % folder,
              names == ["FF", "P", "I", "D", "ACRO %s rate" % axis], names)
        names = names[:4]
        gains0 = {n: get_param("%s_RATE_%s" % (prefix, n)) for n in names}
        c.write_entry(find("P", folder), struct.pack(">i", 120))
        c.write_entry(find("I", folder), struct.pack(">i", 205))
        c.write_entry(find("D", folder), struct.pack(">i", 4))
        c.write_entry(find("FF", folder), struct.pack(">i", 410))
        got = {n: get_param("%s_RATE_%s" % (prefix, n)) for n in names}
        check("%s write FF 0.41, P 0.12, I 0.205, D 0.004" % folder,
              all(abs(got[n] - v) < 1e-5 for n, v in (("FF", 0.41), ("P", 0.12), ("I", 0.205), ("D", 0.004))), got)
        for n, v in gains0.items():
            set_param("%s_RATE_%s" % (prefix, n), v)

    # 3b. Yaw gains: the yaw damper gain and the rudder mix, with two decimals
    yaw_gain = find("Yaw damper", "Yaw gains")
    c.write_entry(yaw_gain, struct.pack(">i", 35))
    check("yaw damper 0.35 sets YAW2SRV_DAMP", abs(get_param("YAW2SRV_DAMP") - 0.35) < 1e-4, get_param("YAW2SRV_DAMP"))
    c.write_entry(yaw_gain, struct.pack(">i", 0))
    check("yaw damper 0 turns the damper off", get_param("YAW2SRV_DAMP") == 0, get_param("YAW2SRV_DAMP"))
    mix0 = get_param("KFF_RDDRMIX")
    c.write_entry(find("Rudder mix", "Yaw gains"), struct.pack(">i", 30))
    check("rudder mix 0.30 sets KFF_RDDRMIX", abs(get_param("KFF_RDDRMIX") - 0.3) < 1e-4, get_param("KFF_RDDRMIX"))
    set_param("KFF_RDDRMIX", mix0)

    # 3c. speeds: cruise writes two parameters; min < cruise < max is kept
    cruise = find("Cruise speed", "Speeds")
    min_speed = find("Min speed", "Speeds")
    max_speed = find("Max speed", "Speeds")
    speeds0 = [get_param(n) for n in ("SCALING_SPEED", "AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX")]
    c.write_entry(cruise, struct.pack(">i", 17))
    check("cruise speed sets SCALING_SPEED and AIRSPEED_CRUISE",
          get_param("SCALING_SPEED") == 17 and get_param("AIRSPEED_CRUISE") == 17,
          (get_param("SCALING_SPEED"), get_param("AIRSPEED_CRUISE")))
    c.write_entry(min_speed, struct.pack(">i", 11))
    c.write_entry(max_speed, struct.pack(">i", 30))
    check("min and max speed set", get_param("AIRSPEED_MIN") == 11 and get_param("AIRSPEED_MAX") == 30,
          (get_param("AIRSPEED_MIN"), get_param("AIRSPEED_MAX")))
    c.write_entry(min_speed, struct.pack(">i", 25))
    check("min speed stays below cruise", get_param("AIRSPEED_MIN") == 16, get_param("AIRSPEED_MIN"))
    c.write_entry(max_speed, struct.pack(">i", 10))
    check("max speed stays above cruise", get_param("AIRSPEED_MAX") == 18, get_param("AIRSPEED_MAX"))
    c.write_entry(cruise, struct.pack(">i", 40))
    check("cruise speed stays below max", get_param("SCALING_SPEED") == 17 and get_param("AIRSPEED_CRUISE") == 17,
          (get_param("SCALING_SPEED"), get_param("AIRSPEED_CRUISE")))
    for name, value in zip(("SCALING_SPEED", "AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX"), speeds0):
        set_param(name, value)

    # 4. mode position: Low -> FBWA sets FLTMODE1 and 2
    low = find("Low")
    c.write_entry(low, [entries[low]["options"].index("FBWA")])
    check("mode position Low = FBWA", get_param("FLTMODE1") == 5 and get_param("FLTMODE2") == 5,
          (get_param("FLTMODE1"), get_param("FLTMODE2")))
    c.write_entry(low, [entries[low]["options"].index("MANUAL")])

    # 4b. the mode switch is set up in Radio; Flight modes shows it live
    mode_sw = find("Mode switch")
    check("mode switch is in Radio", entries[entries[mode_sw]["parent"]]["name"] == "2 Radio",
          entries[entries[mode_sw]["parent"]]["name"])
    high = find("High")
    c.write_entry(high, [entries[high]["options"].index("FBWA")])
    c.channels[5] = 2000                                  # the mode follows switch moves
    time.sleep(1)
    c.channels[5] = 988
    time.sleep(1)
    check_mode = find("Check mode switch", "3 Flight modes")
    state = command(check_mode, CMD_START)
    check("mode check shows Low", state is not None and state == (CMD_PROGRESS, "Ch6 Low: MANUAL"), state)
    c.channels[5] = 2000
    time.sleep(1)
    state = command(check_mode, CMD_POLL)
    check("mode check follows the switch", state is not None and state == (CMD_PROGRESS, "Ch6 High: FBWA"), state)
    state = command(check_mode, CMD_CANCEL)
    check("Exit closes the mode check", state is not None and state[0] == CMD_READY, state)
    c.channels[5] = 988
    time.sleep(1)

    # 4c. copy the transmitter trims into the servo trims (MANUAL, disarmed)
    copy = find("Copy stick trims", "4 Outputs")
    trims0 = (get_param("SERVO1_TRIM"), get_param("SERVO2_TRIM"))
    c.channels[0] = 1800                                  # a held stick, not a trim
    time.sleep(1)
    state = command(copy, CMD_START)
    check("trim copy asks first", state is not None and state[0] == CMD_CONFIRMATION_NEEDED, state)
    state = command(copy, CMD_CONFIRM)
    check("trim copy refuses a held stick", state is not None and "off centre" in state[1]
          and get_param("SERVO1_TRIM") == trims0[0], state)
    command(copy, CMD_CONFIRM)
    c.channels[0], c.channels[1] = 1530, 1480             # TX trims
    time.sleep(1)
    command(copy, CMD_START)
    state = command(copy, CMD_CONFIRM)
    trims1 = (get_param("SERVO1_TRIM"), get_param("SERVO2_TRIM"))
    check("trim copy asks to center the TX trims", state is not None and "center the TX trims" in state[1], state)
    check("trims copied into SERVO1/2_TRIM", trims1[0] != trims0[0] and trims1[1] != trims0[1]
          and all(abs(a - b) < 100 for a, b in zip(trims0, trims1)), (trims0, trims1))
    state = command(copy, CMD_CONFIRM)                    # TX trims not centred yet
    check("trim copy waits for centred TX trims", state is not None and "not centred" in state[1], state)
    c.channels[0], c.channels[1] = 1500, 1500
    time.sleep(1)
    state = command(copy, CMD_CONFIRM)
    check("trim copy done", state is not None and state[1].startswith("Trims copied"), state)
    command(copy, CMD_CONFIRM)
    c.channels[0] = 1530
    time.sleep(1)
    command(copy, CMD_START)
    command(copy, CMD_CONFIRM)
    moved = get_param("SERVO1_TRIM")
    state = command(copy, CMD_CANCEL)                     # Exit before confirming: undo
    check("Exit undoes an unconfirmed trim copy", moved != trims1[0] and get_param("SERVO1_TRIM") == trims1[0]
          and state is not None and state[0] == CMD_READY, (moved, get_param("SERVO1_TRIM")))
    c.channels[0] = 1500
    c.channels[5] = 2000                                  # FBWA
    time.sleep(1)
    state = command(copy, CMD_START)
    check("trim copy needs MANUAL", state is not None and "MANUAL" in state[1], state)
    command(copy, CMD_CONFIRM)
    c.channels[5] = 988
    set_param("SERVO1_TRIM", trims0[0])
    set_param("SERVO2_TRIM", trims0[1])
    time.sleep(1)

    # 5. calibration popup flow
    check("no Calibrate gyros entry (the gyros calibrate at each start)",
          "Calibrate gyros" not in [e.get("name") for e in entries.values()])
    first, final = run_command(find("Calibrate level"))
    check("calibration asks to hold level", first is not None and first[0] == CMD_CONFIRMATION_NEEDED, first)
    check("calibration reports done", final is not None and final[1] == "Calibration done", final)
    closed = command(find("Calibrate level"), CMD_CONFIRM)
    check("OK closes the result", closed is not None and closed[0] == CMD_READY, closed)

    # 6. wing type: written, it sets channels 1-4 at once
    wing = find("Wing type")
    check("no Apply template entry", "Apply template" not in [e.get("name") for e in entries.values()])
    check("wing type reads Normal from the outputs", entries[wing]["shown"] == "Normal", entries[wing])
    c.write_entry(wing, [entries[wing]["options"].index("Flying wing")])
    time.sleep(0.5)
    check("flying wing sets elevons", get_param("SERVO1_FUNCTION") == 77 and get_param("SERVO2_FUNCTION") == 78,
          (get_param("SERVO1_FUNCTION"), get_param("SERVO2_FUNCTION")))
    data = c.read_entry(wing)
    shown = cm.parse_entry(wing, data) if data else None
    check("wing type reads back Flying wing", shown is not None and shown.get("shown") == "Flying wing", shown)
    c.write_entry(wing, [entries[wing]["options"].index("Normal")])
    time.sleep(0.5)
    check("normal restores outputs", get_param("SERVO1_FUNCTION") == 4 and get_param("SERVO2_FUNCTION") == 19)

    # 7. stick learning on a TAER radio with a reversed elevator
    learn = find("Learn sticks")

    def moves(seq, hold=0.9):
        # each step completes once its stick is held at the end
        for ch, value in seq:
            c.channels[ch] = value
            time.sleep(hold)
        return command(learn, CMD_POLL)

    def learn_run(order, centre, ail=(2012, 988), short=False):
        """order: channel index of (aileron, elevator, rudder, throttle)"""
        a, e, r, t = order
        c.channels[:4] = [0, 0, 0, 0]
        for i, v in zip(order, centre):
            c.channels[i] = v
        time.sleep(0.5)
        command(learn, CMD_START)
        state = command(learn, CMD_CONFIRM)
        if short:
            ail = (1750, 1250)
        state2 = moves([(t, 2012), (r, 2012), (r, 988), (r, 1500), (t, 988),
                        (a, ail[0]), (a, ail[1]), (a, centre[0]),
                        (e, 988), (e, 2012), (e, 1500)])
        return state, state2

    # 7. TAER radio: elevator back reads low (reversed)
    c.channels[:4] = [988, 1500, 1500, 1500]
    time.sleep(0.5)
    state = command(learn, CMD_START)
    check("stick learning asks to center", state is not None and "Center sticks" in state[1], state)
    first, state = learn_run((1, 2, 3, 0), (1500, 1500, 1500, 988))
    check("stick learning starts with the throttle", first is not None and "THROTTLE all the way up" in first[1], first)
    check("review shows aileron and elevator ends", state is not None and state[1] == "Ends A 988-2011 E 988-2011",
          state)
    state = command(learn, CMD_CONFIRM)
    check("review shows rudder and throttle ends", state is not None and state[1] == "Ends R 988-2011 T 988-2011",
          state)
    state = command(learn, CMD_CONFIRM)
    check("stick learning detects TAER", state is not None and state[1] == "A2 E3 R4 T1 Save?", state)
    command(learn, CMD_CANCEL)
    check("cancel leaves channel map alone", get_param("RCMAP_ROLL") == 1)

    # 8. AETR radio: refusals, then a save with endpoints
    _, state = learn_run((0, 1, 3, 2), (1600, 1500, 1500, 988))   # roll stick held off centre
    check("off-centre stick is refused", state is not None and state[1] == "Ch1 not centred at start, retry", state)
    command(learn, CMD_CONFIRM)
    _, state = learn_run((0, 1, 3, 2), (1500, 1500, 1500, 988), short=True)
    check("short travel is refused", state is not None and state[1] == "Ch1 travel too short, retry", state)
    command(learn, CMD_CONFIRM)
    _, state = learn_run((0, 1, 3, 2), (1500, 1500, 1500, 988))
    command(learn, CMD_CONFIRM)
    state = command(learn, CMD_CONFIRM)
    check("AETR summary", state is not None and state[1] == "A1 E2 R4 T3 Save?", state)
    state = command(learn, CMD_CONFIRM)
    check("AETR learning saves", state is not None and state[1] == "Sticks saved", state)
    check("elevator reversal saved", get_param("RC2_REVERSED") == 1, get_param("RC2_REVERSED"))
    ends = [(get_param("RC%d_MIN" % ch), get_param("RC%d_TRIM" % ch), get_param("RC%d_MAX" % ch)) for ch in (1, 3)]
    check("stick endpoints saved", abs(ends[0][0] - 988) <= 4 and abs(ends[0][2] - 2012) <= 4
          and abs(ends[0][1] - 1500) <= 4, ends[0])
    check("throttle trim saved at low", abs(ends[1][1] - ends[1][0]) <= 4 and abs(ends[1][2] - 2012) <= 4, ends[1])
    c.write_entry(learn, [CMD_CONFIRM])
    set_param("RC2_REVERSED", 0)

    # 8b. gain knob learning: its ends set the range and the direction
    learn_knob = find("Learn gain knob")
    state = command(learn_knob, CMD_START)
    check("knob learning prompts", state is not None and "gain knob" in state[1], state)
    c.channels[9] = 2000                                  # turn ch10
    time.sleep(0.5)
    state = command(learn_knob, CMD_POLL)
    check("knob found, asks for the low end", state is not None and "Ch10" in state[1] and "LOW" in state[1], state)
    c.channels[9] = 2012                                  # this knob's low gain end reads high
    time.sleep(0.4)
    state = command(learn_knob, CMD_CONFIRM)
    check("knob asks for the high end", state is not None and "HIGH" in state[1], state)
    c.channels[9] = 1700                                  # not turned far enough
    time.sleep(0.4)
    state = command(learn_knob, CMD_CONFIRM)
    check("short knob travel is refused", state is not None and "further" in state[1], state)
    c.channels[9] = 2012
    time.sleep(0.4)
    command(learn_knob, CMD_CONFIRM)
    c.channels[9] = 988
    time.sleep(0.4)
    state = command(learn_knob, CMD_CONFIRM)
    knob = [get_param(n) for n in ("STABRX_GAIN_CH", "RC10_MIN", "RC10_TRIM", "RC10_MAX", "RC10_REVERSED")]
    check("knob learned reversed", state is not None and state[1] == "Gain knob: Ch10 988-2011, reversed", state)
    # this result makes the entry exactly 54 bytes long: one byte short of a
    # chunk, the length the stock chunk maths sent as an entry with no data
    raw = c.read_entry(learn_knob)
    shown = cm.parse_entry(learn_knob, raw) if raw is not None and len(raw) > 2 else None
    check("a 54 byte entry reads complete", raw is not None and len(raw) == 55 and shown is not None
          and shown["info"] == "Gain knob: Ch10 988-2011, reversed", None if raw is None else len(raw))
    check("knob range and direction saved", knob[0] == 10 and abs(knob[1] - 988) <= 4 and abs(knob[2] - 1500) <= 4
          and abs(knob[3] - 2012) <= 4 and knob[4] == 1, knob)
    command(learn_knob, CMD_CONFIRM)
    c.channels[9] = 1500
    set_param("STABRX_GAIN_CH", 0)
    set_param("RC10_REVERSED", 0)
    time.sleep(1)

    # 9. arm switch learning: the assigned switch stays assigned and does not
    # arm while the command is open; a command the radio stops polling ends
    learn_arm = find("Learn arm switch")
    command(learn_arm, CMD_START)
    c.channels[4] = 2000                                  # the arm switch itself
    time.sleep(2)
    check("no arming while a command is open", is_armed() is False and get_param("RC5_OPTION") == 153,
          get_param("RC5_OPTION"))
    c.channels[4] = 988
    time.sleep(1)
    state = command(learn_arm, CMD_CANCEL)
    check("Exit closes arm learning", state is not None and state[0] == CMD_READY, state)
    command(learn_arm, CMD_START)
    time.sleep(12)                                        # the radio went away: no polls
    state = command(learn_arm, CMD_POLL)
    check("an abandoned command times out", state is not None and state[1] == "Timed out", state)
    command(learn_arm, CMD_CONFIRM)

    # only assigned once the switch is off
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

    # 9b. learning onto a channel in use asks to replace its function; the
    # old function is only cleared at the final save
    set_param("STABRX_GAIN_CH", 10)
    command(learn_arm, CMD_START)
    c.channels[9] = 2000                                  # ch10, the gain knob
    time.sleep(0.5)
    state = command(learn_arm, CMD_POLL)
    check("a channel in use asks to replace", state is not None and state[0] == CMD_CONFIRMATION_NEEDED
          and state[1] == "Ch10 is the Gain knob. Replace?", state)
    state = command(learn_arm, CMD_CANCEL)
    check("cancel keeps the old function", state is not None and state[0] == CMD_READY
          and get_param("STABRX_GAIN_CH") == 10 and get_param("RC9_OPTION") == 153 and get_param("RC10_OPTION") == 0,
          (state, get_param("STABRX_GAIN_CH"), get_param("RC10_OPTION")))
    c.channels[9] = 1500
    time.sleep(0.5)
    # arm switch: after the replace it still asks for the switch off
    command(learn_arm, CMD_START)
    c.channels[9] = 2000
    time.sleep(0.5)
    command(learn_arm, CMD_POLL)
    state = command(learn_arm, CMD_CONFIRM)
    check("replace, then the arm switch asks to switch off", state is not None and "Ch10: switch it off" in state[1],
          state)
    state = command(learn_arm, CMD_CONFIRM)               # still high: must not assign
    check("replace: arm refused while on, old function kept", "off first" in state[1]
          and get_param("RC10_OPTION") == 0 and get_param("STABRX_GAIN_CH") == 10, state)
    c.channels[9] = 988
    time.sleep(0.5)
    state = command(learn_arm, CMD_CONFIRM)
    check("replace: arm saved and the gain knob cleared", state is not None
          and state[1] == "Arm switch: Ch10 (was Gain knob)" and get_param("RC10_OPTION") == 153
          and get_param("STABRX_GAIN_CH") == 0 and get_param("RC9_OPTION") == 0,
          (state, get_param("RC10_OPTION"), get_param("STABRX_GAIN_CH")))
    command(learn_arm, CMD_CONFIRM)
    check("vehicle did not arm while replacing", is_armed() is False)
    # gain knob onto the arm switch's channel: it still learns both ends
    command(learn_knob, CMD_START)
    c.channels[9] = 2000
    time.sleep(0.5)
    state = command(learn_knob, CMD_POLL)
    check("knob onto the arm switch asks to replace", state is not None
          and state[1] == "Ch10 is the Arm switch. Replace?", state)
    state = command(learn_knob, CMD_CONFIRM)
    check("replace, then the knob asks for its low end", state is not None and "LOW" in state[1], state)
    c.channels[9] = 988
    time.sleep(0.4)
    command(learn_knob, CMD_CONFIRM)
    c.channels[9] = 2012
    time.sleep(0.4)
    check("old function kept until the knob is saved", get_param("RC10_OPTION") == 153 and is_armed() is False)
    state = command(learn_knob, CMD_CONFIRM)
    check("replace: knob saved and the arm switch cleared", state is not None
          and state[1] == "Gain knob: Ch10 (was Arm switch)" and get_param("STABRX_GAIN_CH") == 10
          and get_param("RC10_OPTION") == 0, (state, get_param("STABRX_GAIN_CH"), get_param("RC10_OPTION")))
    command(learn_knob, CMD_CONFIRM)
    c.channels[9] = 1500
    time.sleep(0.5)
    # another switch: one OK replaces and saves, with no second question
    learn_at = find("Learn autotune sw")
    command(learn_at, CMD_START)
    c.channels[9] = 2000
    time.sleep(0.5)
    state = command(learn_at, CMD_POLL)
    check("autotune onto the gain knob asks to replace", state is not None
          and state[1] == "Ch10 is the Gain knob. Replace?", state)
    c.channels[9] = 988                                   # switch off before it becomes autotune
    time.sleep(0.4)
    state = command(learn_at, CMD_CONFIRM)
    # "Autotune switch: Ch10 (was Gain knob)" does not fit next to the
    # command name in one reply, so the short result is shown
    check("replace: autotune saved at once", state is not None and state[1] == "Autotune switch: Ch10"
          and get_param("RC10_OPTION") == 107 and get_param("STABRX_GAIN_CH") == 0,
          (state, get_param("RC10_OPTION"), get_param("STABRX_GAIN_CH")))
    command(learn_at, CMD_CONFIRM)
    set_param("RC10_OPTION", 0)
    set_param("RC10_REVERSED", 0)
    c.channels[9] = 1500
    time.sleep(0.5)

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
    state = command(check_mode, CMD_START)
    check("mode check works armed", state is not None and state[0] == CMD_PROGRESS and "Ch6" in state[1], state)
    command(check_mode, CMD_CANCEL)
    c.write_entry(wing, [entries[wing]["options"].index("V-tail")])
    time.sleep(0.5)
    check("wing type refused while armed", get_param("SERVO2_FUNCTION") == 19, get_param("SERVO2_FUNCTION"))
    out1_rev = find("Reversed", "Channel 1")
    rev0 = get_param("SERVO1_REVERSED")
    c.write_entry(out1_rev, [1 - int(rev0)])
    time.sleep(0.5)
    check("output write refused while armed", get_param("SERVO1_REVERSED") == rev0, get_param("SERVO1_REVERSED"))
    out1_trim = find("Trim", "Channel 1")
    trim0 = get_param("SERVO1_TRIM")
    c.write_entry(out1_trim, list(struct.pack(">i", int(trim0) + 20)))
    time.sleep(0.5)
    check("output trim refused while armed", get_param("SERVO1_TRIM") == trim0, get_param("SERVO1_TRIM"))
    acro_roll = find("ACRO roll rate", "Roll gains")
    rate0 = get_param("ACRO_ROLL_RATE")
    c.write_entry(acro_roll, list(struct.pack(">i", int(rate0) + 10)))
    time.sleep(0.5)
    check("tuning number accepted while armed", get_param("ACRO_ROLL_RATE") == rate0 + 10,
          get_param("ACRO_ROLL_RATE"))
    c.write_entry(acro_roll, list(struct.pack(">i", int(rate0))))
    c.write_entry(yaw_gain, list(struct.pack(">i", 20)))
    time.sleep(0.5)
    check("yaw damper accepted while armed", abs(get_param("YAW2SRV_DAMP") - 0.2) < 1e-4, get_param("YAW2SRV_DAMP"))
    c.write_entry(yaw_gain, list(struct.pack(">i", 0)))
    roll_p = find("P", "Roll gains")
    p0 = get_param("RLL_RATE_P")
    c.write_entry(roll_p, list(struct.pack(">i", 100)))
    time.sleep(0.5)
    check("roll P accepted while armed", abs(get_param("RLL_RATE_P") - 0.1) < 1e-5, get_param("RLL_RATE_P"))
    set_param("RLL_RATE_P", p0)
    speed0 = get_param("SCALING_SPEED")
    c.write_entry(cruise, list(struct.pack(">i", int(speed0) + 2)))
    time.sleep(0.5)
    check("speed refused while armed", get_param("SCALING_SPEED") == speed0, get_param("SCALING_SPEED"))
    c.channels[4] = 988
    time.sleep(2)
    check("disarmed via ch5", is_armed() is False)

    # 11. GPS setting (last: switching it restarts the flight controller)
    gps = find("GPS", "1 Sensors")
    e = cm.parse_entry(gps, c.read_entry(gps))
    sensors = [x["name"] for pid, x in sorted(entries.items()) if x["parent"] == entries[gps]["parent"]]
    check("GPS entry is Off/On, last in 1 Sensors", e["options"] == ["Off", "On"]
          and sensors[-1] == "GPS"
          and not any(x["name"] == "GPS" and entries.get(x["parent"], {}).get("name") == "5 Failsafe"
                      for x in entries.values()), sensors)
    set_param("GPS1_TYPE", 0)
    set_param("ARMING_SKIPCHK", -81)
    drain()
    r = c.write_entry(gps, [0])
    time.sleep(2)
    check("GPS Off when already off changes nothing", r is not None and get_param("GPS1_TYPE") == 0
          and get_param("ARMING_SKIPCHK") == -81)
    drain()
    c.write_entry(gps, [1])
    # read before the restart, 1.5 s later; get_param drops older messages
    restarting = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1,
                              condition="'GPS on, restarting' in STATUSTEXT.text")
    on = [get_param(n) for n in ("GPS1_TYPE", "EK3_ENABLE", "AHRS_EKF_TYPE", "ARMING_SKIPCHK")]
    check("GPS On sets the GPS driver, EKF3 and the GPS arming checks", on == [1, 1, 3, -4185], on)
    check("GPS On restarts the flight controller", restarting is not None)

    c.close()
finally:
    proc.kill()
    proc.wait()

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
