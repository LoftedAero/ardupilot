#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

SITL check that the transmitter setup menu (ArduPlane/gyro_menu.cpp)
builds once SITL's own simulated CRSF receiver on SERIAL5 is talking.
sitl_crsf_menu_test.py drives the menu itself, Calibrate level included,
and sitl_elrs_lua_test.py runs it in the ExpressLRS Lua script.

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
ARMING_SKIPCHK -81
RC_OPTIONS 96
SERIAL5_PROTOCOL 23
RC_PROTOCOLS 512
GYRO_GAIN_CH 8
LOG_DISARMED 1
"""

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ("  " + detail if detail else ""), flush=True)


def run_sitl(name, with_crsf, done_marker, timeout_s=60):
    """start SITL, collect STATUSTEXTs until done_marker has been seen
    (plus a short drain) and return the texts and the longest gap
    between heartbeats"""
    run = os.path.join(HERE, name)
    shutil.rmtree(run, ignore_errors=True)
    os.makedirs(run)
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


# 1. menu builds over the simulated CRSF link
texts, _ = run_sitl("menu", True, "setup menu ready", timeout_s=40)
ready = [t for t in texts if "setup menu ready" in t]
check("menu built", bool(ready), ready[0] if ready else "")

print()
fails = [r for r in results if not r[1]]
print(f"{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
