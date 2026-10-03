#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Run the real ExpressLRS Lua script (elrsV3.lua) on the PC against SITL and
walk the gyro setup menu the way a pilot would, saving every screen as
text and PNG.

The ExpressLRS script is not part of this repository. Get it from the
radio (SCRIPTS/TOOLS/elrsV3.lua, for example with
"adb pull /storage/emulated/0/AX12LUA/SCRIPTS/TOOLS/elrsV3.lua" on a
RadioMaster AX12) or from the ExpressLRS repository (src/lua/elrsV3.lua),
and point ELRS_LUA at it.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          ELRS_LUA=/path/to/elrsV3.lua python3 Tools/gyro/sitl_elrs_lua_test.py
Screens go to $GYRO_TEST_DIR/screens (default /tmp/gyro_sitl_elrs/screens).
Needs pymavlink, lupa and Pillow.
"""
import os
import socket
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crsf_menu_client as cm  # noqa: E402
from edgetx_sim import EdgeTxSim  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_elrs")
SCREENS = os.path.join(HERE, "screens")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
CRSF_PORT = 5765
SCRIPT = os.environ.get("ELRS_LUA")

if not SCRIPT or not os.path.exists(SCRIPT):
    sys.exit("set ELRS_LUA to the path of the ExpressLRS Lua script (elrsV3.lua)")

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + str(detail) if detail != "" else ""), flush=True)


os.makedirs(SCREENS, exist_ok=True)
for f in os.listdir(SCREENS):
    os.remove(os.path.join(SCREENS, f))
parm = os.path.join(HERE, "test.parm")
with open(parm, "w") as f:
    f.write("""
ARMING_REQUIRE 1
ARMING_SKIPCHK -1
SERIAL5_PROTOCOL 23
RC_PROTOCOLS 512
RC5_OPTION 153
FLTMODE_CH 6
SCR_ENABLE 0
# no GPS fitted: SITL's disabled GPS still reports "no fix", so also
# remove the GPS driver to get ArduPilot's "no GPS" state
SIM_GPS1_ENABLE 0
GPS1_TYPE 0
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

shot_number = [0]

try:
    time.sleep(2)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
    m.wait_heartbeat(timeout=60)

    # keep reading MAVLink in the background: if nobody reads, SITL's
    # output backs up and its clock nearly stops
    mav_lock = threading.Lock()
    param_values = {}
    status_texts = []

    def mav_reader():
        while True:
            try:
                r = m.recv_match(blocking=True, timeout=1)
            except Exception:
                return
            if r is None:
                continue
            with mav_lock:
                if r.get_type() == "PARAM_VALUE":
                    param_values[r.param_id] = r.param_value
                elif r.get_type() == "STATUSTEXT":
                    status_texts.append(r.text)

    threading.Thread(target=mav_reader, daemon=True).start()

    def get_param(name):
        with mav_lock:
            param_values.pop(name, None)
        m.param_fetch_one(name)
        t0 = time.time()
        while time.time() - t0 < 5:
            with mav_lock:
                if name in param_values:
                    return param_values[name]
            time.sleep(0.05)
        return None

    c = cm.CrsfClient(port=CRSF_PORT)
    # the script caches the entry count from its first ping: start it only
    # once the menu exists, as a pilot opening the script after power-up would
    t0 = time.time()
    while time.time() - t0 < 30:
        with mav_lock:
            if any("menu ready" in t for t in status_texts):
                break
        time.sleep(0.2)

    sim = EdgeTxSim(SCRIPT, c)

    def shot(name):
        shot_number[0] += 1
        base = os.path.join(SCREENS, "%02d_%s" % (shot_number[0], name))
        with open(base + ".txt", "w") as f:
            f.write(sim.text() + "\n")
        sim.save_png(base + ".png")

    class EntryMissing(Exception):
        pass

    def open_entry(label):
        # never press Enter on whatever happens to be selected: stop instead
        if not sim.select(label):
            raise EntryMissing("entry '%s' not found on screen:\n%s" % (label, sim.text()))
        sim.press("enter")

    def back():
        # the ExpressLRS script keeps no folder history: Back from any
        # folder, however deep, returns to the device's top level, which
        # is why the menu's folders sit there
        sim.press("exit")
        return sim.contains("1 Calibration") and sim.contains("Tools")

    # 1. root and device list
    sim.idle(3)
    shot("root")
    check("root shows Other Devices", sim.contains("Other Devices"))
    open_entry("Other Devices")
    sim.idle(1)
    shot("other_devices")
    check("device listed as Gyro", sim.contains("[Gyro]"), sim.text())

    # 2. open the device: Status and the folders are its top level
    open_entry("Gyro")
    # entries arrive one by one; the last top-level folder is Tools
    check("all folders load", sim.wait_for("Tools", 60))
    shot("top_level")
    for folder in ("1 Calibration", "2 Radio", "3 Flight modes", "4 Outputs", "5 Failsafe", "6 Tuning",
                   "Tools"):
        check("folder %s shown" % folder, sim.contains(folder))
    check("status line", sim.contains("DISARMED"))

    # 3. edit a number: ACRO roll rate, one step up and back
    open_entry("6 Tuning")
    shot("tuning")
    before = get_param("ACRO_ROLL_RATE")
    sim.select("d/s")               # ACRO roll rate is the first d/s value
    sim.press("enter")              # edit
    sim.press("next")               # one step up
    shot("tuning_editing")
    sim.press("enter")              # save
    sim.idle(1)
    after = get_param("ACRO_ROLL_RATE")
    check("number edit saves +10", after == before + 10, (before, after))
    sim.press("enter")
    sim.press("prev")
    sim.press("enter")
    sim.idle(1)
    check("number edit back", get_param("ACRO_ROLL_RATE") == before)
    check("back returns to the folders", back(), sim.text())

    # 4. edit a selection: Channel 5 function None -> Aileron and back
    open_entry("4 Outputs")
    open_entry("Channel 5")
    shot("output5")
    sim.select("None")
    sim.press("enter")
    sim.press("next")
    sim.press("enter")
    sim.idle(1)
    shot("output5_aileron")
    check("selection edit saves Aileron", get_param("SERVO5_FUNCTION") == 4, get_param("SERVO5_FUNCTION"))
    sim.press("enter")
    sim.press("prev")
    sim.press("enter")
    sim.idle(1)
    check("selection edit back", get_param("SERVO5_FUNCTION") == 0)
    check("back from Channel 5 returns to the folders", back(), sim.text())

    # 5. command popups: calibrate gyros
    open_entry("1 Calibration")
    open_entry("Calibrate gyros")
    check("calibration asks to keep still", sim.wait_for("Keep the model still", 5))
    shot("cal_prompt")
    sim.press("enter")              # OK
    check("calibration result shown", sim.wait_for("Calibration done", 20))
    shot("cal_done")
    sim.press("enter")              # dismiss
    sim.idle(1)
    check("popup closed", sim.popup is None)
    back()

    # 6. stick learning with a TAER radio, cancelled at the summary
    open_entry("2 Radio")
    c.channels[:4] = [988, 1500, 1500, 1500]
    sim.idle(0.5)
    open_entry("Learn sticks")
    check("asks for aileron", sim.wait_for("AILERON", 5))
    shot("learn_aileron")
    for ch, value in [(1, 2000), (2, 1000), (3, 2000), (0, 2000)]:
        c.channels[ch] = value
        sim.idle(1.0)
        c.channels[ch] = 988 if ch == 0 else 1500
        sim.idle(1.0)
    check("summary shows TAER", sim.wait_for("A2 E3 R4 T1", 5), sim.popup)
    shot("learn_summary")
    sim.press("exit")               # cancel
    sim.idle(1)
    check("cancel keeps channel map", get_param("RCMAP_ROLL") == 1)
    back()

    # 7. the mode switch check updates live while the popup is open
    open_entry("3 Flight modes")
    open_entry("Check mode switch")
    check("mode check shows the switch", sim.wait_for("Ch6 Low", 5), sim.popup)
    shot("mode_check_low")
    c.channels[5] = 2000
    check("mode check follows the switch", sim.wait_for("Ch6 High", 5), sim.popup)
    shot("mode_check_high")
    c.channels[5] = 988
    sim.press("exit")
    sim.idle(1)
    check("mode check closes", sim.popup is None, sim.popup)
    back()

    # 8. no GPS: RTL settings hidden, explanation shown
    open_entry("5 Failsafe")
    shot("failsafe_no_gps")
    check("RTL altitude hidden", not sim.contains("RTL altitude"))
    check("needs a GPS shown", sim.contains("needs a GPS"))

    # 9. Back at the device's top level switches to the TX module page;
    # moving from a low line there must not hang the script
    back()
    open_entry("Tools")
    back()
    sim.press("exit")
    sim.press("next")
    check("keys work after leaving the device", sim.contains("Other Devices"), sim.text())

    c.close()
except EntryMissing as e:
    check("navigation", False, e)
finally:
    proc.kill()
    proc.wait()

print()
print("screens saved in", SCREENS)
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
