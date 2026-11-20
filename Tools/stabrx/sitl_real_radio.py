#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Run the gyro firmware in SITL for a real radio: the radio flies it and
opens the transmitter setup menu over a real ExpressLRS link, through
crsf_serial_bridge.py.

Build first:  ./waf configure --board sitl --extra-hwdef=Tools/stabrx/sitl-stabrx.hwdef && ./waf plane
Run:          python3 Tools/stabrx/sitl_real_radio.py [--model flightaxis:HOST]
Then, on Windows:
              py Tools/stabrx/crsf_serial_bridge.py --port COM53

SITL starts with the LoftedAero-SRX-gyro defaults, the receiver on its
CRSF port (TCP 5765) and RC from that receiver only (RC_PROTOCOLS 512),
so a simulator's own RC input, such as RealFlight's, is ignored. A ground
station can connect on TCP 5762. The settings made in the menu are kept
for the next run unless --wipe is given. The script prints the flight
controller's messages; Ctrl-C stops SITL.

--model flightaxis:HOST uses RealFlight on HOST as the physics model
(RealFlight Link enabled; under WSL2 HOST is the Windows host address,
the default gateway). Only the built-in plane model loads SITL's tune for
that model (Tools/autotest/models/plane.parm); any other model starts
from the ArduPlane defaults, as a new board does. --defaults adds
parameter files, for example the RealFlight model's own.
"""
import argparse
import os
import socket
import subprocess
import sys
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BIN = os.path.join(ROOT, "build/sitl/bin/arduplane")
BOARD_DEFAULTS = os.path.join(ROOT, "libraries/AP_HAL_ChibiOS/hwdef/LoftedAero-SRX-gyro/defaults.parm")
CRSF_PORT = 5765


def windows_host():
    """the Windows host address seen from WSL2, or None"""
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                fields = line.split()
                if fields[1] == "00000000":
                    return socket.inet_ntoa(bytes.fromhex(fields[2])[::-1])
    except OSError:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1], formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--model", default="plane",
                    help="SITL model: plane (default), or flightaxis:HOST for RealFlight "
                         "(flightaxis alone uses the Windows host)")
    ap.add_argument("--defaults", action="append", default=[], help="extra parameter file, may be repeated")
    ap.add_argument("--gps", action="store_true", help="fit a simulated GPS (the built-in plane model only)")
    ap.add_argument("--wipe", action="store_true", help="start from the defaults, forgetting earlier settings")
    ap.add_argument("--dir", default=os.environ.get("STABRX_TEST_DIR", "/tmp/stabrx_sitl_real_radio"),
                    help="working directory (keeps the settings)")
    args = ap.parse_args()

    model = args.model
    if model == "flightaxis":
        host = windows_host()
        if host is None:
            sys.exit("give the RealFlight host: --model flightaxis:HOST")
        model = "flightaxis:" + host
    for port in (5760, 5762, CRSF_PORT):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                sys.exit("TCP port %u is already in use; stop the running simulator first" % port)

    os.makedirs(args.dir, exist_ok=True)
    parm = os.path.join(args.dir, "real_radio.parm")
    with open(parm, "w") as f:
        # the board's defaults, with the receiver on SITL's CRSF port
        # instead of the board's serial port
        with open(BOARD_DEFAULTS) as board:
            f.writelines(ln for ln in board if not ln.startswith("SERIAL"))
        f.write("SERIAL5_PROTOCOL 23\nRC_PROTOCOLS 512\nSCR_ENABLE 0\n")
        # SITL's accelerometers need no calibration, and a simulator model
        # cannot be turned through the 6 poses of "Calibrate accel": small
        # values pass the calibration pre-arm check, as SITL's own plane.parm
        for prefix in ("INS_ACCOFFS", "INS_ACC2OFFS"):
            f.writelines("%s_%s 0.001\n" % (prefix, axis) for axis in "XYZ")
        for prefix in ("INS_ACCSCAL", "INS_ACC2SCAL"):
            f.writelines("%s_%s 1.001\n" % (prefix, axis) for axis in "XYZ")
        if model == "plane" and not args.gps:
            f.write("SIM_GPS1_ENABLE 0\nGPS1_TYPE 0\n")
    defaults = [parm] + [os.path.abspath(d) for d in args.defaults]
    if model == "plane":
        # the built-in model's own tune; another model starts from the
        # ArduPlane defaults, as a new board does
        defaults.insert(0, os.path.join(ROOT, "Tools/autotest/models/plane.parm"))
    cmd = [BIN, "--model", model, "--speedup", "1", "--home", "-35.363261,149.165230,584,353",
           "--serial5=tcp:%u" % CRSF_PORT, "--defaults", ",".join(defaults)]
    eeprom = os.path.join(args.dir, "eeprom.bin")
    if args.wipe and os.path.exists(eeprom):
        # not -w: SITL restarts with its own arguments (the menu's GPS
        # setting restarts it), and -w would erase the settings again
        os.remove(eeprom)
    print("starting SITL (%s) in %s" % (model, args.dir), flush=True)
    print("receiver bridge: crsf_serial_bridge.py --tcp 127.0.0.1:%u; ground station: TCP 5762" % CRSF_PORT,
          flush=True)
    proc = subprocess.Popen(cmd, cwd=args.dir, stdout=open(os.path.join(args.dir, "sitl_stdout.txt"), "w"),
                            stderr=subprocess.STDOUT)
    try:
        time.sleep(2)
        # reconnect after the restart that the menu's GPS setting causes:
        # SITL waits for this connection before it runs again
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=254, autoreconnect=True, retries=60)
        m.wait_heartbeat(timeout=60)
        armed = None
        # keep reading MAVLink: SITL's clock nearly stops when nobody reads
        while proc.poll() is None:
            r = m.recv_match(blocking=True, timeout=1)
            if r is None:
                continue
            if r.get_type() == "STATUSTEXT":
                print(time.strftime("%H:%M:%S"), r.text, flush=True)
            elif r.get_type() == "HEARTBEAT" and r.get_srcComponent() == 1:
                now = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if now != armed:
                    armed = now
                    print(time.strftime("%H:%M:%S"), "ARMED" if armed else "disarmed", m.flightmode, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
        proc.wait()


if __name__ == "__main__":
    main()
