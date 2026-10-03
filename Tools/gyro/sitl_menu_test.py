#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL checks for the transmitter setup menu (ArduPlane/gyro_menu.cpp), in
two runs:

1. Menu run, with the simulated CRSF receiver on SERIAL5: the native menu
   builds once the receiver is talking.
2. Calibration run, without the CRSF simulator: a Lua probe requests
   calibrations through GYRO_CAL exactly as the menu does, and the gyro
   module runs them without stalling the vehicle.

The runs are separate because, in SITL, any blocking calibration
(including a plain MAVLink PREFLIGHT_CALIBRATION with no Lua involved)
freezes the vehicle when the in-process CRSF simulator is attached. That
is a limitation of the simulator's serial device, not of the firmware.

The radio-side menu interaction itself is not simulated; test that on a
real ELRS transmitter.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef && ./waf plane
Run:          python3 Tools/gyro/sitl_menu_test.py
Needs pymavlink.
"""
import os
import shutil
import socket
import subprocess
import sys
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.environ.get("GYRO_TEST_DIR", "/tmp/gyro_sitl_menu")
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")

PARAMS = """
ARMING_REQUIRE 1
ARMING_SKIPCHK -65
RC_OPTIONS 96
SCR_ENABLE 1
SERIAL5_PROTOCOL 23
RC_PROTOCOLS 512
GYRO_GAIN_CH 8
LOG_DISARMED 1
"""

# Requests the same calibrations the menu requests (GYRO_CAL 1 = gyros,
# 2 = level) and reports what the gyro module wrote back (0 = done,
# -1 = failed).
CAL_PROBE = """
local step = 0
local results = {}
local function probe()
    step = step + 1
    if step == 1 then
        param:set("GYRO_CAL", 1)
    elseif step == 2 then
        results.gyro = param:get("GYRO_CAL")
        param:set("GYRO_CAL", 2)
    else
        results.level = param:get("GYRO_CAL")
        gcs:send_text(6, string.format("PROBE gyro=%d level=%d", results.gyro, results.level))
        return
    end
    return probe, 5000
end
return probe, 15000
"""

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + detail if detail else ""), flush=True)


def run_sitl(name, with_crsf, scripts, done_marker, timeout_s=60):
    """start SITL with the given Lua scripts, collect STATUSTEXTs until
    done_marker has been seen (plus a short drain) and return the texts
    and the longest gap between heartbeats"""
    run = os.path.join(HERE, name)
    shutil.rmtree(run, ignore_errors=True)
    os.makedirs(os.path.join(run, "scripts", "modules"))
    for filename, content in scripts.items():
        with open(os.path.join(run, "scripts", filename), "w") as f:
            f.write(content)
    shutil.copy(os.path.join(ROOT, "libraries/AP_Scripting/modules/crsf_helper.lua"),
                os.path.join(run, "scripts", "modules"))
    parm = os.path.join(run, "test.parm")
    with open(parm, "w") as f:
        f.write(PARAMS)

    # a leftover simulator on the same port would silently answer this test
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        if sock.connect_ex(("127.0.0.1", 5760)) == 0:
            sys.exit("TCP port 5760 is already in use; stop the running simulator first")

    cmd = [BIN, "--model", "plane", "--speedup", "5", "-w",
           "--home", "-35.363261,149.165230,584,353",
           "--defaults", os.path.join(ROOT, "Tools/autotest/models/plane.parm") + "," + parm]
    if with_crsf:
        cmd.append("--serial5=sim:crsf")
    proc = subprocess.Popen(cmd, cwd=run, stdout=open(os.path.join(run, "sitl_stdout.txt"), "w"),
                            stderr=subprocess.STDOUT)
    texts = []
    max_hb_gap = 0.0
    try:
        time.sleep(2)
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, autoreconnect=False)
        m.wait_heartbeat(timeout=60)
        t0 = time.time()
        last_hb = t0
        done_at = None
        while time.time() - t0 < timeout_s:
            msg = m.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=1)
            now = time.time()
            max_hb_gap = max(max_hb_gap, now - last_hb)
            if done_at is not None and now - done_at > 3:
                break
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT":
                last_hb = now
                continue
            texts.append(msg.text)
            print(f"   [{name}] TEXT:", msg.text, flush=True)
            if done_marker in msg.text and done_at is None:
                done_at = now
    finally:
        # kill rather than terminate: a stalled simulator ignores SIGTERM
        proc.kill()
        proc.wait()
    return texts, max_hb_gap


def lua_errors(texts):
    return [t for t in texts
            if ("Lua" in t and "error" in t.lower()) or "CRSF: Failed" in t or "Callback Err" in t]


# 1. menu builds over the simulated CRSF link
texts, _ = run_sitl("menu", True, {}, "setup menu ready", timeout_s=40)
ready = [t for t in texts if "setup menu ready" in t]
check("menu built", bool(ready), ready[0] if ready else "")

# 2. calibration requests serviced by the gyro module
texts, max_hb_gap = run_sitl("calibration", False, {"cal_probe.lua": CAL_PROBE}, "PROBE")
probe = [t for t in texts if t.startswith("PROBE")]
codes = dict(kv.split("=") for kv in probe[0].split()[1:]) if probe else {}
detail = probe[0] if probe else "no PROBE message (vehicle stalled?)"
check("gyro calibration done", codes.get("gyro") == "0", detail)
check("level calibration done", codes.get("level") == "0", detail)
check("calibration texts sent", any("Gyro: gyro calibration done" in t for t in texts)
      and any("Gyro: level calibration done" in t for t in texts))
check("no heartbeat gap over 3 s", max_hb_gap < 3.0, f"longest gap {max_hb_gap:.1f} s")
errors = lua_errors(texts)
check("no Lua errors (calibration run)", not errors, "; ".join(errors))

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
