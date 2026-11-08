#!/usr/bin/env python3
"""
Automated SITL tests for F35B.lua (DOORS subsystem).

Runs the F35B_TESTPLAN.md checks in one of two modes:

- default: unattended on the stock quadplane-tilttri SITL model (see
  tilttri.parm for why it stands in for the F-35B), faster than real time.
- --realflight: against RealFlight over FlightAxis, in real time, using the
  F-35B parameter set (F35B_TESTPLAN.param) and its DOOR_ values. Start
  RealFlight first with the F-35B model loaded and RealFlight Link enabled.
  Tests that need SITL-controlled wind or RC loss (4c, 4e, 10f) are skipped.

The runner launches SITL, sets the DOOR_ parameters, drives the mode selector (RC8),
the emergency override (RC5) and the sticks with RC overrides, and checks
flight mode, script messages, door telemetry and the raw servo outputs.

Throughout the run it also checks the core safety rule against the raw
outputs: the doors are never closed while the nacelles are not forward or the
lift motor is still powered, and the VTOL hardware never moves while an
inbound entry is pending. VTOL entries the script cannot withhold (GCS
command, RTL auto-transition) are reported with the measured door lag
instead, as the documented limitation.

Results are printed and written to last_results.md (last_results_realflight.md
with --realflight) next to this file.

usage: run_sitl_tests.py [--realflight [--rf-host IP] [--params FILE]]
                         [--speedup N] [--script PATH] [--results FILE]
                         [--ground-only] [--step] [--keep]

Needs a SITL build of this branch: ./waf configure --board sitl && ./waf plane
"""

import argparse
import datetime
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")    # servo outputs 9-16 are MAVLink2 extension fields
from pymavlink import mavutil              # noqa: E402

HERE = Path(__file__).resolve().parent
APPLETS = HERE.parent
REPO = APPLETS.parents[2]
ARDUPLANE = REPO / "build" / "sitl" / "bin" / "arduplane"
TILTTRI_DEFAULTS = [
    REPO / "Tools" / "autotest" / "default_params" / "quadplane.parm",
    REPO / "Tools" / "autotest" / "default_params" / "quadplane-tilttri.parm",
    HERE / "tilttri.parm",
]
RF_PARAMS = APPLETS / "F35B_TESTPLAN.param"
HOME = "-35.363261,149.165230,584,353"
MAV_URL = "tcp:127.0.0.1:5760"
FLIGHTAXIS_PORT = 18083

# Script parameters for the stand-in model, set over MAVLink once the script has
# created them. The selector mapping matches F35B_TESTPLAN.param: 1000 = QSTABILIZE,
# 1500 = QLOITER, 2000 = FBWA. TILT_FWD and LIFT_OFF are calibrated at run time.
# With --realflight the DOOR_ values come from the parameter file instead.
TILTTRI_DOOR_PARAMS = {
    "DOOR_ENABLE": 1, "DOOR_MODE_CH": 8,
    "DOOR_MODE1": 17, "DOOR_MODE2": 17, "DOOR_MODE3": 19, "DOOR_MODE4": 19,
    "DOOR_MODE5": 5, "DOOR_MODE6": 5,
    "DOOR_FUNC": 94, "DOOR_CLOSED": 1000, "DOOR_PART": 1500, "DOOR_OPEN": 2000,
    "DOOR_ENTRY_MS": 3000, "DOOR_PART_AS": 7, "DOOR_FULL_AS": 15,
    "DOOR_TILT_FN": 41, "DOOR_LIFT_FN": 36, "DOOR_DEBUG": 1, "DOOR_NOZ_MS": 3500,
}

SEL = {"QSTABILIZE": 1000, "QLOITER": 1500, "FBWA": 2000}
OVR_REST, OVR_MANUAL = 2000, 1000
VTOL_MODES = {"QSTABILIZE", "QHOVER", "QLOITER", "QLAND", "QRTL", "QAUTOTUNE", "QACRO", "LOITERALTQLAND"}
MODE_NAMES = dict(mavutil.mode_mapping_apm)                      # number -> name (ArduPlane)
MODE_NUMBERS = {name: num for num, name in MODE_NAMES.items()}
DOOR_ST_NAME = {0: "CLOSED", 1: "PARTIAL", 2: "OPEN", 3: "OPENING"}
PWM_TOL = 30
# A 0.75 s debounce can measure as short as this: DOOR_AS arrives at ~5 Hz, so a crossing is seen
# up to 0.2 s late, and the outputs are sampled at 10 Hz. No debounce at all would read 0-0.3 s.
DEBOUNCE_MIN_S = 0.45
# Flight tests climb to these heights (m above home) in a hover before anything that loses
# height: a transition to fixed-wing, or the MANUAL bailout from a hover (7a).
SAFE_ALT = 100
MANUAL_ALT = 120
MAV_CMD_SCRIPTING = 42701
SCRIPTING_CMD_STOP, SCRIPTING_CMD_STOP_AND_RESTART = 2, 3


class TestFail(Exception):
    pass


class Skip(Exception):
    pass


class Abort(Exception):
    """The flight sequence can't continue (e.g. altitude lost)."""


class Config:
    """What to simulate: the stand-in tilttri model, or RealFlight with the F-35B parameters."""

    def __init__(self, args):
        self.realflight = args.realflight
        if args.realflight:
            self.model = args.model or f"flightaxis:{args.rf_host or windows_host()}"
            self.speedup = 1
            self.defaults = [Path(args.params or RF_PARAMS)]
            self.door_params = read_door_params(self.defaults[0])
            self.door_params["DOOR_ENABLE"] = 1
            self.door_params["DOOR_DEBUG"] = 1
            self.calibrate = False
            self.home = None if self.model.startswith("flightaxis") else HOME
            self.description = (f"RealFlight over FlightAxis with "
                                f"`{os.path.relpath(self.defaults[0], REPO)}`")
            self.results = args.results or str(HERE / "last_results_realflight.md")
        else:
            self.model = args.model or "quadplane-tilttri"
            self.speedup = args.speedup
            self.defaults = TILTTRI_DEFAULTS
            self.door_params = dict(TILTTRI_DOOR_PARAMS)
            self.calibrate = True
            self.home = HOME
            self.description = ("stock `quadplane-tilttri` with `F35B_sitl/tilttri.parm` "
                                "(logic test, not F-35B dynamics)")
            self.results = args.results or str(HERE / "last_results.md")


def windows_host():
    """The Windows host as seen from WSL2 (NAT networking): the default gateway."""
    try:
        out = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True).stdout
        return out.split()[2]
    except Exception:
        return "127.0.0.1"


def read_door_params(path):
    """DOOR_ values from a Mission Planner style parameter file (NAME,VALUE or NAME VALUE)."""
    params = {}
    for line in Path(path).read_text(errors="replace").splitlines():
        parts = re.split(r"[,\s]+", line.strip())
        if len(parts) >= 2 and parts[0].startswith("DOOR_"):
            params[parts[0]] = float(parts[1])
    missing = {"DOOR_MODE_CH", "DOOR_FUNC", "DOOR_TILT_FN", "DOOR_TILT_FWD"} - params.keys()
    if missing:
        sys.exit(f"{path} has no {', '.join(sorted(missing))}")
    return params


def port_open(host, port):
    import socket
    try:
        with socket.create_connection((host, port), timeout=2):
            return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# SITL process
# ---------------------------------------------------------------------------
class Sitl:
    def __init__(self, script, cfg, keep):
        self.cfg = cfg
        self.keep = keep
        self.rundir = Path(tempfile.mkdtemp(prefix="f35b_sitl_"))
        (self.rundir / "scripts").mkdir()
        shutil.copy(script, self.rundir / "scripts" / "F35B.lua")
        self.proc = None
        self.logf = None

    def start(self):
        if not ARDUPLANE.exists():
            sys.exit(f"SITL binary not found: {ARDUPLANE}\n"
                     "Build it first: ./waf configure --board sitl && ./waf plane")
        cfg = self.cfg
        defaults = list(cfg.defaults)
        if cfg.realflight:
            # A parameter dump from hardware enables the watchdog (BRD_OPTIONS 1). In SITL that
            # is a 2 s alarm() that survives the execv of a reboot and kills the restarted process
            # while it waits for a connection. FlightAxis forces it off too; this covers dry runs.
            overrides = self.rundir / "runner_overrides.parm"
            overrides.write_text("BRD_OPTIONS 0\n")
            defaults.append(overrides)
        cmd = [str(ARDUPLANE), "--model", cfg.model, "--speedup", str(cfg.speedup),
               "--defaults", ",".join(str(p) for p in defaults), "-I0"]
        if cfg.home:
            cmd += ["--home", cfg.home]
        self.logf = open(self.rundir / "sitl.log", "w")
        # own session, so a reboot (execv) keeps the same process and Ctrl-C reaches only us
        self.proc = subprocess.Popen(cmd, cwd=self.rundir, stdout=self.logf,
                                     stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.logf:
            self.logf.close()

    def cleanup(self, failed):
        if self.keep or failed:
            print(f"SITL run directory kept: {self.rundir}")
        else:
            shutil.rmtree(self.rundir, ignore_errors=True)


# ---------------------------------------------------------------------------
# MAVLink link, vehicle state and the continuous safety monitor
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, speedup):
        self.speedup = speedup
        self.m = None
        self.rc = {1: 1500, 2: 1500, 3: 1000, 4: 1500, 5: OVR_REST, 6: 1000, 7: 1000, 8: SEL["FBWA"]}
        self.overrides = True
        self.last_rc = 0.0
        # simulated time, kept monotonic across reboots
        self.sim_base = 0
        self.boot_ms = 0
        self.sim_ms = 0
        self.mode = None
        self.armed = False
        self.mode_changes = []     # (sim_ms, mode)
        self.texts = []            # (sim_ms, text)
        self.nvf = {}              # name -> (value, sim_ms)
        self.first_nvf_ms = None   # first named float since the last reboot (the script is running)
        self.servo = {}
        self.airspeed = 0.0
        self.heading = 0
        self.rel_alt = 0.0
        self.params = {}
        # monitor
        self.chan = {}             # "door"/"tilt"/"lift" -> servo channel
        self.monitor_on = False
        self.windows = []          # [start_ms, end_ms or None, label]
        self.violations = []       # (sim_ms, kind, detail, window label or None)
        self.samples = []          # (sim_ms, mode, door, tilt, lift, airspeed)
        self.door_st_last = None
        self.prev_tilt = None
        self.prev_door = None
        self.stowed_since = None   # sim_ms when the tilt and lift outputs last started reading stowed
        self.tilt_span = None      # tilt output's SERVOx_MAX - SERVOx_MIN, for the nozzle estimate
        self.noz = None            # nozzle position estimate (tilt-output PWM), as the script models it
        self.noz_at = None
        self.tilt_fwd_since = None  # sim_ms when the tilt output last started reading forward
        self.lift_off_since = None  # sim_ms when the lift output last started reading off
        self.noz_fwd_since = None   # sim_ms when the nozzle estimate last started reading forward
        self.closes = []            # timeline at each door close: dict of sim_ms values

    # -- connection --------------------------------------------------------
    def connect(self, wall_timeout=90):
        deadline = time.time() + wall_timeout
        last_err = None
        while time.time() < deadline:
            try:
                self.m = mavutil.mavlink_connection(MAV_URL, source_system=255)
                if self.m.wait_heartbeat(timeout=15):
                    self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                    return
            except OSError as e:
                last_err = e
            time.sleep(1)
        raise RuntimeError(f"could not connect to SITL on {MAV_URL}: {last_err}")

    def close(self):
        try:
            self.m.close()
        except Exception:
            pass

    # -- message pump ------------------------------------------------------
    def pump(self, wall_s=0.05):
        end = time.time() + wall_s
        while True:
            now = time.time()
            if self.overrides and now - self.last_rc > 0.1:
                self.send_rc()
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self.handle(msg)
            if time.time() >= end:
                return

    def send_rc(self):
        self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component,
                                             *[self.rc[i] for i in range(1, 9)])
        self.last_rc = time.time()

    def handle(self, msg):
        t = msg.get_type()
        if t == "BAD_DATA":
            return
        tb = getattr(msg, "time_boot_ms", None)
        if tb is not None and msg.get_srcSystem() == self.m.target_system:
            if tb + 5000 < self.boot_ms:            # rebooted
                self.sim_base += self.boot_ms
            self.boot_ms = tb
            self.sim_ms = max(self.sim_ms, self.sim_base + tb)
        if t == "HEARTBEAT":
            if msg.get_srcComponent() != 1 or msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            mode = MODE_NAMES.get(msg.custom_mode, str(msg.custom_mode))
            if mode != self.mode:
                self.mode_changes.append((self.sim_ms, mode))
                if self.mode is not None:
                    print(f"      t={self.sim_ms / 1000:7.1f}s  mode {self.mode} -> {mode}")
                self.mode = mode
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "STATUSTEXT":
            self.texts.append((self.sim_ms, msg.text))
        elif t == "NAMED_VALUE_FLOAT":
            name = msg.name.rstrip("\x00") if isinstance(msg.name, str) else msg.name.decode().rstrip("\x00")
            self.nvf[name] = (msg.value, self.sim_ms)
            if self.first_nvf_ms is None:
                self.first_nvf_ms = self.sim_ms
            if name == "DOOR_ST":
                st = int(round(msg.value))
                if st != self.door_st_last:
                    if self.door_st_last is not None:
                        print(f"      t={self.sim_ms / 1000:7.1f}s  {self.mode or '?':10s} doors "
                              f"{DOOR_ST_NAME.get(self.door_st_last, '?')} -> {DOOR_ST_NAME.get(st, '?')}"
                              f"  airspeed {self.door_airspeed():4.1f}")
                    self.door_st_last = st
        elif t == "SERVO_OUTPUT_RAW":
            if getattr(msg, "port", 0) != 0:
                return
            for i in range(1, 17):
                v = getattr(msg, f"servo{i}_raw", None)
                if v is not None:
                    self.servo[i] = v
            self.check_invariants()
        elif t == "VFR_HUD":
            self.airspeed = msg.airspeed
            self.heading = msg.heading
        elif t == "GLOBAL_POSITION_INT":
            self.rel_alt = msg.relative_alt / 1000.0
        elif t == "PARAM_VALUE":
            self.params[msg.param_id.rstrip("\x00")] = msg.param_value

    def door_airspeed(self):
        """The airspeed the script's thresholds use (DOOR_AS). VFR_HUD reports the airspeed
        sensor whenever one is healthy, even with ARSPD_USE 0, while the script uses the AHRS
        estimate; the two can cross a threshold several tenths of a second apart."""
        v = self.nvf.get("DOOR_AS")
        if v is not None and v[0] >= 0 and self.sim_ms - v[1] < 1000:
            return v[0]
        return self.airspeed

    # -- safety monitor ----------------------------------------------------
    def outputs(self):
        return (self.servo.get(self.chan.get("door")), self.servo.get(self.chan.get("tilt")),
                self.servo.get(self.chan.get("lift")))

    def nacelles_fwd(self, tilt):
        tf = self.params.get("DOOR_TILT_FWD", 1900)
        return tilt >= tf if tf >= 1500 else tilt <= tf

    def lift_off(self, lift):
        if self.params.get("DOOR_LIFT_FN", 0) <= 0:
            return True
        return lift <= self.params.get("DOOR_LIFT_OFF", 1050)

    def door_is(self, door, which):
        return door is not None and abs(door - self.params.get(f"DOOR_{which}", 0)) <= PWM_TOL

    def check_invariants(self):
        door, tilt, lift = self.outputs()
        if None in (door, tilt, lift):
            return
        self.samples.append((self.sim_ms, self.mode, door, tilt, lift, self.door_airspeed()))
        fwd, off = self.nacelles_fwd(tilt), self.lift_off(lift)
        if fwd and off:
            if self.stowed_since is None:
                self.stowed_since = self.sim_ms
        else:
            self.stowed_since = None
        if fwd:
            if self.tilt_fwd_since is None:
                self.tilt_fwd_since = self.sim_ms
        else:
            self.tilt_fwd_since = None
        if off:
            if self.lift_off_since is None:
                self.lift_off_since = self.sim_ms
        else:
            self.lift_off_since = None
        # The script's nozzle model, run independently on the sampled outputs: it follows a deploy
        # (the output moving away from forward) at once, and a stow at most the output's full range
        # per DOOR_NOZ_MS, so it is never closer to forward than the real nozzle.
        noz_ms = self.params.get("DOOR_NOZ_MS", 3500)
        if self.noz is None or self.tilt_span is None or noz_ms <= 0:
            self.noz = tilt
        else:
            step = self.tilt_span * (self.sim_ms - self.noz_at) / noz_ms
            if self.params.get("DOOR_TILT_FWD", 1900) >= 1500:   # forward is the high-PWM end
                self.noz = tilt if tilt <= self.noz else min(tilt, self.noz + step)
            else:                                               # forward is the low-PWM end
                self.noz = tilt if tilt >= self.noz else max(tilt, self.noz - step)
        self.noz_at = self.sim_ms
        noz_fwd = self.nacelles_fwd(self.noz)
        if noz_fwd:
            if self.noz_fwd_since is None:
                self.noz_fwd_since = self.sim_ms
        else:
            self.noz_fwd_since = None
        closing = (self.door_is(door, "CLOSED") and self.prev_door is not None
                   and not self.door_is(self.prev_door, "CLOSED"))
        self.prev_door = door
        if not self.monitor_on:
            return
        window = next((w[2] for w in self.windows if w[0] <= self.sim_ms and (w[1] is None or self.sim_ms <= w[1])),
                      None)
        if self.door_is(door, "CLOSED") and not (fwd and off):
            self.violations.append((self.sim_ms, "closed", f"door {door} closed with tilt {tilt}, lift {lift}",
                                    window))
        # The tilt output is a command and the nozzle lags it, so the doors must not close before
        # the nozzle estimate reads forward (allowing 0.2 s of nozzle travel for the sampling).
        if closing:
            self.closes.append(dict(close=self.sim_ms, tilt=self.tilt_fwd_since, lift=self.lift_off_since,
                                    noz=self.noz_fwd_since))
            tf = self.params.get("DOOR_TILT_FWD", 1900)
            short = (tf - self.noz) if tf >= 1500 else (self.noz - tf)
            tol = (self.tilt_span or 1000) * 200 / max(noz_ms, 1)
            if short > tol:
                self.violations.append((self.sim_ms, "early", f"doors closed with the nozzle estimate at "
                                        f"{self.noz:.0f}, {short:.0f} us short of forward", window))
        # While an entry is pending (still fixed-wing) the VTOL hardware must not deploy: the
        # lift motor stays off and the nacelles don't move away from forward. Nacelles still
        # stowing toward forward (e.g. right after leaving VTOL) are fine.
        prev, self.prev_tilt = self.prev_tilt, tilt
        tf = self.params.get("DOOR_TILT_FWD", 1900)
        deploying = prev is not None and ((tilt < prev - 3) if tf >= 1500 else (tilt > prev + 3))
        # "Pending" comes from the script's own countdown: with time still remaining on a fresh
        # VTOL_DLY report, the script cannot have entered VTOL yet. (The mode from HEARTBEAT is
        # up to 1 s stale, so it can't be used here.)
        st, st_ms = self.nvf.get("DOOR_ST", (None, 0))
        dly, dly_ms = self.nvf.get("VTOL_DLY", (0, 0))
        age = (self.sim_ms - dly_ms) / 1000.0
        pending = st is not None and int(round(st)) == 3 and self.sim_ms - st_ms <= 250 and dly - age >= 0.1
        if pending and (deploying or not off):
            self.violations.append((self.sim_ms, "pending", f"VTOL hardware deployed during pending entry: "
                                    f"tilt {prev}->{tilt}, lift {lift}", window))

    def open_window(self, label):
        self.windows.append([self.sim_ms, None, label])

    def close_window(self, extra_s=0.0):
        self.windows[-1][1] = self.sim_ms + int(extra_s * 1000)

    # -- waiting -----------------------------------------------------------
    def wait(self, pred, timeout_s):
        start = self.sim_ms
        wall_deadline = time.time() + timeout_s / self.speedup * 4 + 20
        while True:
            self.pump()
            if pred():
                return True
            if self.sim_ms - start > timeout_s * 1000 or time.time() > wall_deadline:
                return False

    def sleep(self, s):
        self.wait(lambda: False, s)

    def hold(self, pred, s):
        """True if pred stays true for s seconds of sim time."""
        start = self.sim_ms
        wall_deadline = time.time() + s / self.speedup * 4 + 20
        while self.sim_ms - start < s * 1000 and time.time() < wall_deadline:
            self.pump()
            if not pred():
                return False
        return True

    # -- state accessors ---------------------------------------------------
    def nv(self, name):
        return self.nvf.get(name, (None, 0))[0]

    def door_st(self):
        v = self.nv("DOOR_ST")
        return None if v is None else int(round(v))

    def texts_since(self, mark):
        return [txt for (ms, txt) in self.texts if ms >= mark]

    def text_seen(self, pattern, mark):
        return any(re.search(pattern, txt) for txt in self.texts_since(mark))

    def wait_text(self, pattern, timeout_s, mark):
        return self.wait(lambda: self.text_seen(pattern, mark), timeout_s)

    def modes_since(self, mark):
        return [m for (ms, m) in self.mode_changes if ms >= mark]

    # -- commands ----------------------------------------------------------
    def set_rc(self, **chans):
        for k, v in chans.items():
            self.rc[int(k[2:])] = int(v)
        if self.overrides:
            self.send_rc()

    def select(self, mode):
        self.set_rc(ch8=SEL[mode])

    def set_param(self, name, value):
        for _ in range(5):
            self.params.pop(name, None)
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                      float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            if self.wait(lambda: name in self.params, 3) and abs(self.params[name] - value) < 1e-3:
                return
        raise TestFail(f"could not set {name}={value}")

    def get_param(self, name):
        for _ in range(5):
            self.params.pop(name, None)
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
            if self.wait(lambda: name in self.params, 3):
                return self.params[name]
        return None

    def command(self, cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def gcs_mode(self, name):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_NUMBERS[name])

    def arm(self, timeout_s=150):
        mark = self.sim_ms
        while self.sim_ms - mark < timeout_s * 1000:
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if self.wait(lambda: self.armed, 5):
                return
        prearm = [t for t in self.texts_since(mark) if "PreArm" in t or "Arm" in t]
        raise Abort(f"could not arm: {prearm[-1] if prearm else 'no reason given'}")

    def reboot(self):
        self.command(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
        time.sleep(1.5)
        self.close()
        # HEARTBEAT and STATUSTEXT carry no boot time and take the last one seen; step past it so
        # the new boot's first messages sort after everything from the old one
        self.sim_ms += 1
        self.first_nvf_ms = None
        self.connect()

    def wait_script_restart(self, rebooted, timeout_s=30):
        """After a reboot, wait for the script to be running again. Its one 'script loaded' text
        can go out before the runner has reconnected (1.5 s wall after the reboot, 7.5 s of sim
        time at speedup 5), so its telemetry arriving counts too. Returns the time to measure
        the new boot from (the text if seen, else the reboot), or None."""
        def started():
            return (self.text_seen(r"F35B: script loaded", rebooted)
                    or (self.first_nvf_ms is not None and self.first_nvf_ms >= rebooted))
        if not self.wait(started, timeout_s):
            return None
        loaded = [ms for ms, txt in self.texts if ms >= rebooted and txt.startswith("F35B: script loaded")]
        return max(loaded) if loaded else rebooted


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------
class Runner:
    def __init__(self, link, sitl, cfg, step=False):
        self.L = link
        self.sitl = sitl
        self.cfg = cfg
        self.step = step
        self.saved = {}
        self.results = []         # (id, title, status, detail)
        self.cal = {}
        self.flight_abort = None
        self.alt_sim_ms = None     # for check_alt's stopped-simulation check
        self.alt_wall = 0.0

    # -- bookkeeping -------------------------------------------------------
    def run(self, tid, title, fn, flight=False):
        if flight and self.flight_abort:
            self.record(tid, title, "SKIP", f"flight sequence aborted: {self.flight_abort}")
            return
        print(f"  {tid:4s} {title}")
        if self.step:
            self.wait_for_enter()
        try:
            detail = fn() or ""
            self.record(tid, title, "PASS", detail)
        except TestFail as e:
            self.record(tid, title, "FAIL", str(e))
        except Skip as e:
            self.record(tid, title, "SKIP", str(e))
        except Abort as e:
            self.flight_abort = str(e)
            self.record(tid, title, "FAIL", f"aborted flight sequence: {e}")
            # leave the aircraft hovering rather than in whatever the test was doing (an external
            # VTOL entry, so the doors open reactively: not a finding)
            self.L.open_window("abort recovery")
            self.L.set_rc(ch3=1500)
            self.L.gcs_mode("QLOITER")
        except Exception as e:
            self.record(tid, title, "ERROR", f"{type(e).__name__}: {e}")
            traceback.print_exc()
        if not self.sitl.alive():
            raise RuntimeError("SITL exited unexpectedly; see sitl.log in the run directory")

    def record(self, tid, title, status, detail):
        self.results.append((tid, title, status, detail))
        print(f"  {'':4s} -> {status}  {detail}")

    def check(self, cond, msg):
        if not cond:
            raise TestFail(msg)

    def wait_for_enter(self):
        """Pause until Enter, still pumping so the RC overrides keep holding the controls."""
        import select
        print("       press Enter to run it", end="", flush=True)
        while not select.select([sys.stdin], [], [], 0)[0]:
            self.L.pump(0.1)
        sys.stdin.readline()

    def original(self, name):
        """A parameter's value before any test changed it."""
        if name not in self.saved:
            self.saved[name] = self.L.params.get(name, self.L.get_param(name))
        return self.saved[name]

    def needs_sitl_physics(self, what):
        if self.cfg.realflight:
            raise Skip(f"needs {what}, which RealFlight controls")

    # -- shared steps ------------------------------------------------------
    def ensure_fbwa_closed(self, timeout_s=15):
        L = self.L
        L.set_rc(ch5=OVR_REST)
        L.select("FBWA")
        if not L.wait(lambda: L.mode == "FBWA", 2):
            # selector ignored (e.g. external MANUAL) or no edge: toggle the override switch
            L.set_rc(ch5=OVR_MANUAL)
            L.sleep(1)
            L.set_rc(ch5=OVR_REST)
            if not L.wait(lambda: L.mode == "FBWA", 3):
                L.gcs_mode("FBWA")
                L.wait(lambda: L.mode == "FBWA", 3)
        self.check(L.mode == "FBWA", f"setup: could not get to FBWA (mode {L.mode})")
        self.check(L.wait(lambda: L.door_st() == 0, timeout_s), f"setup: doors did not close (DOOR_ST {L.door_st()})")

    def enter_vtol_via_selector(self, mode="QLOITER", timeout_s=8):
        L = self.L
        L.select(mode)
        self.check(L.wait(lambda: L.mode == mode, timeout_s), f"could not enter {mode} via the selector (mode {L.mode})")

    def gate(self, target, mark):
        """Check an inbound entry: OPENING, held in fixed-wing for DOOR_ENTRY_MS, then target."""
        L = self.L
        entry_s = L.params["DOOR_ENTRY_MS"] / 1000.0
        self.check(L.wait(lambda: L.door_st() == 3, 1.5), f"doors not OPENING (DOOR_ST {L.door_st()})")
        self.check(L.wait_text(r"doors opening", 2, mark), "no 'doors opening' message")
        self.check(L.hold(lambda: L.mode not in VTOL_MODES, entry_s - 0.4 - (L.sim_ms - mark) / 1000.0),
                   f"entered {L.mode} before the door delay ended")
        self.check(L.wait(lambda: L.mode == target, 2.5), f"did not enter {target} after the delay (mode {L.mode})")
        entered = next(ms for ms, m in reversed(L.mode_changes) if m == target)
        # STATUSTEXT can trail the heartbeat when the debug output fills the queue
        self.check(L.wait_text(r"doors open - entering VTOL", 3, mark), "no 'doors open - entering VTOL' message")
        return (entered - mark) / 1000.0

    def no_violations_since(self, mark):
        bad = [v for v in self.L.violations if v[0] >= mark and v[3] is None]
        self.check(not bad, f"{len(bad)} safety violation(s), first at t={bad[0][0] / 1000:.1f}s: {bad[0][2]}"
                   if bad else "")

    def debounce_delay(self, mark, door_is, crossed):
        """Seconds from the airspeed crossing a threshold to the door changing.

        door_is(door) is true once the door has changed; crossed(airspeed) is true on the far
        side of the threshold. The script needs the crossing to persist for 0.75 s."""
        samples = [s for s in self.L.samples if s[0] >= mark]
        change = next((i for i, s in enumerate(samples) if door_is(s[2])), None)
        if change is None:
            return None, None, None
        start = change
        while start > 0 and crossed(samples[start - 1][5]):
            start -= 1
        if start == 0 and crossed(samples[0][5]):
            return None, samples[change][5], None
        return (samples[change][0] - samples[start][0]) / 1000.0, samples[change][5], samples[start][5]

    def climb_hover(self, target_m, timeout_s=150):
        """Climb in the current VTOL mode to target_m before something that loses height. Full
        stick: the climb rate is the VTOL mode's limit (Q_PILOT_SPD_UP)."""
        L = self.L
        if L.mode not in VTOL_MODES:
            raise Abort(f"needs a hover to climb, but the mode is {L.mode}")
        if L.rel_alt >= target_m:
            return
        L.set_rc(ch3=2000)
        ok = L.wait(lambda: L.rel_alt >= target_m, timeout_s)
        L.set_rc(ch3=1500)
        if not ok:
            raise Abort(f"climb stalled at {L.rel_alt:.1f} m (wanted {target_m} m)")
        L.sleep(2)

    def check_alt(self, min_alt=25):
        L = self.L
        if L.rel_alt < min_alt:
            raise Abort(f"altitude {L.rel_alt:.1f} m below {min_alt} m in {L.mode}")
        # RealFlight stops simulation time after a crash, and SITL stops sending with it, so the
        # last altitude never changes and the flight loops (counted in sim time) would never end
        now = time.time()
        if L.sim_ms != self.alt_sim_ms:
            self.alt_sim_ms, self.alt_wall = L.sim_ms, now
        elif now - self.alt_wall > 5:
            raise Abort(f"simulation time stopped for 5 s in {L.mode} (crashed or paused in RealFlight?)")

    # -- setup -------------------------------------------------------------
    def setup(self):
        L = self.L
        print("Setting up: waiting for the script and setting DOOR_ parameters")
        if not L.wait(lambda: L.text_seen(r"F35B: script loaded", 0), 60):
            if L.get_param("DOOR_ENABLE") is None:
                raise RuntimeError("F35B.lua did not load (no 'script loaded' message, no DOOR_ENABLE)")
        for name, value in self.cfg.door_params.items():
            L.set_param(name, value)
        for name, value in (("SR0_EXTRA1", 10), ("SR0_EXTRA2", 10), ("SR0_RC_CHAN", 10),
                            ("SR0_POSITION", 5), ("SR0_EXT_STAT", 2)):
            L.set_param(name, value)
        self.check_selector_layout()
        functions = {}
        for ch in range(1, 17):
            functions[ch] = L.get_param(f"SERVO{ch}_FUNCTION")
        dp = self.cfg.door_params
        for key, fn in (("door", dp["DOOR_FUNC"]), ("tilt", dp["DOOR_TILT_FN"]), ("lift", dp.get("DOOR_LIFT_FN", 0))):
            ch = next((c for c, f in functions.items() if f is not None and int(f) == fn), None)
            if ch is None:
                raise RuntimeError(f"no servo output with function {fn} for the {key}")
            L.chan[key] = ch
        lo, hi = L.get_param(f"SERVO{L.chan['tilt']}_MIN"), L.get_param(f"SERVO{L.chan['tilt']}_MAX")
        L.tilt_span = (hi - lo) if lo is not None and hi is not None and hi > lo else None
        self.ensure_fbwa_closed_initial()
        L.sleep(2)
        door, tilt, lift = L.outputs()
        self.cal.update(tilt_fwd=tilt, lift_stopped=lift)
        if self.cfg.calibrate:
            L.set_param("DOOR_TILT_FWD", tilt - 50 if tilt >= 1500 else tilt + 50)
            L.set_param("DOOR_LIFT_OFF", lift + 30)
        if not L.wait(lambda: L.door_st() == 0, 10):
            raise RuntimeError(f"doors did not close in FBWA on the ground: tilt output {tilt} vs DOOR_TILT_FWD "
                               f"{L.params.get('DOOR_TILT_FWD')}, lift output {lift} vs DOOR_LIFT_OFF "
                               f"{L.params.get('DOOR_LIFT_OFF')}")
        L.monitor_on = True
        print(f"  outputs: door SERVO{L.chan['door']}, tilt SERVO{L.chan['tilt']}, lift SERVO{L.chan['lift']}; "
              f"tilt forward {tilt}, lift stopped {lift}")

    def check_selector_layout(self):
        """The tests drive RC8 at 1000/1500/2000 and RC5 high/low; the parameters must agree."""
        L = self.L
        want = {"DOOR_MODE_CH": 8, "DOOR_MODE1": MODE_NUMBERS["QSTABILIZE"], "DOOR_MODE4": MODE_NUMBERS["QLOITER"],
                "DOOR_MODE6": MODE_NUMBERS["FBWA"], "FLTMODE_CH": 5, "FLTMODE6": MODE_NUMBERS["FBWA"],
                "FLTMODE1": MODE_NUMBERS["MANUAL"]}
        bad = [f"{n} is {L.params.get(n, L.get_param(n))}, expected {v}" for n, v in want.items()
               if L.params.get(n, L.get_param(n)) != v]
        if bad:
            raise RuntimeError("selector/override layout differs from F35B_TESTPLAN.param: " + "; ".join(bad))

    def ensure_fbwa_closed_initial(self):
        L = self.L
        if not L.wait(lambda: L.mode == "FBWA", 10):
            L.set_rc(ch5=OVR_MANUAL)
            L.sleep(1)
            L.set_rc(ch5=OVR_REST)
            L.wait(lambda: L.mode == "FBWA", 5)
        if L.mode != "FBWA":
            raise RuntimeError(f"vehicle not in FBWA after boot (mode {L.mode})")

    # -- ground tests (disarmed) -------------------------------------------
    def t_1a(self):
        L = self.L
        self.check(L.wait(lambda: L.nv("TILT_POS") is not None, 3), "no TILT_POS telemetry")
        tp, tilt = L.nv("TILT_POS"), L.outputs()[1]
        self.check(tp > 0, f"TILT_POS is {tp}: DOOR_TILT_FN is wrong")
        self.check(abs(tp - tilt) <= 5, f"TILT_POS {tp} does not match SERVO{L.chan['tilt']} {tilt}")
        return f"TILT_POS {tp:.0f} = tilt output, nacelles forward"

    def t_1d(self):
        L = self.L
        ok = L.hold(lambda: L.door_st() == 0 and L.door_is(L.outputs()[0], "CLOSED"), 5)
        self.check(ok, f"DOOR_ST/door output changed while idle (DOOR_ST {L.door_st()}, door {L.outputs()[0]})")
        return "closed and stable for 5 s"

    def t_2a(self):
        L = self.L
        self.check(L.get_param("FLTMODE_CH") != L.params["DOOR_MODE_CH"], "FLTMODE_CH equals DOOR_MODE_CH")
        L.set_param("DOOR_ENABLE", 0)
        try:
            ok = True
            for pwm in (1000, 1500, 2000):
                L.set_rc(ch8=pwm)
                ok = ok and L.hold(lambda: L.mode == "FBWA", 2)
            self.check(ok, f"mode changed to {L.mode} with the script disabled: the selector reaches the autopilot")
        finally:
            L.set_rc(ch8=SEL["FBWA"])
            L.set_param("DOOR_ENABLE", 1)
        return f"FLTMODE_CH {L.params['FLTMODE_CH']:.0f} != DOOR_MODE_CH 8; selector sweep alone changed nothing"

    def t_9c(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        L.select("QLOITER")
        delay = self.gate("QLOITER", mark)
        return f"held FBWA, entered QLOITER after {delay:.1f} s"

    def t_9a(self):
        L = self.L
        self.check(L.mode == "QLOITER", f"expected QLOITER from 9c, in {L.mode}")
        # DOOR_ST is reported at 5 Hz, so the last report can still be OPENING
        self.check(L.wait(lambda: L.door_st() in (1, 2), 1), f"doors not open (DOOR_ST {L.door_st()})")
        self.check(L.hold(lambda: L.door_st() in (1, 2), 3), f"doors not open (DOOR_ST {L.door_st()})")
        return f"doors {DOOR_ST_NAME[L.door_st()]} while disarmed in QLOITER"

    def t_9b(self):
        L = self.L
        mark = L.sim_ms
        L.select("FBWA")
        self.check(L.wait(lambda: L.mode == "FBWA", 1), f"FBWA not immediate (mode {L.mode})")
        self.check(L.wait(lambda: L.door_st() == 0, 10), f"doors did not close (DOOR_ST {L.door_st()})")
        self.no_violations_since(mark)
        return f"FBWA immediately, doors closed after {(L.sim_ms - mark) / 1000:.1f} s"

    def t_6b(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        L.select("QLOITER")
        L.sleep(1)
        L.select("FBWA")
        self.check(L.hold(lambda: L.mode == "FBWA", 6), f"entered {L.mode} after the entry was cancelled")
        self.check(not L.text_seen(r"entering VTOL", mark), "VTOL entry happened")
        self.check(L.wait(lambda: L.door_st() == 0, 5), "doors did not close")
        return "pending entry cancelled, stayed FBWA, doors closed"

    def t_6c(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        L.select("QSTABILIZE")
        L.sleep(1.5)
        L.select("QLOITER")
        L.sleep(0.6)
        dly = L.nv("VTOL_DLY")
        self.check(dly is not None and dly >= 2.0, f"timer did not restart (VTOL_DLY {dly})")
        self.check(L.wait(lambda: L.mode == "QLOITER", 4), f"did not end in QLOITER (mode {L.mode})")
        self.check("QSTABILIZE" not in L.modes_since(mark), "passed through QSTABILIZE")
        L.select("FBWA")
        self.check(L.wait(lambda: L.door_st() == 0, 10), "doors did not close afterwards")
        return f"retargeted to QLOITER, timer restarted (VTOL_DLY {dly:.1f} s)"

    def t_6d(self):
        L = self.L
        self.ensure_fbwa_closed()
        mode5 = self.original("DOOR_MODE5")
        L.set_param("DOOR_MODE5", 0)          # selector position 5 -> MANUAL for this test
        try:
            closed = lambda: L.door_st() == 0   # noqa: E731
            L.set_rc(ch8=1700)
            self.check(L.wait(lambda: L.mode == "MANUAL", 1), f"FBWA->MANUAL not immediate (mode {L.mode})")
            self.check(L.hold(closed, 2), "doors moved")
            L.select("FBWA")
            self.check(L.wait(lambda: L.mode == "FBWA", 1), f"MANUAL->FBWA not immediate (mode {L.mode})")
            self.check(L.hold(closed, 2), "doors moved")
        finally:
            L.set_param("DOOR_MODE5", mode5)
        return "FBWA->MANUAL->FBWA immediate, doors stayed closed"

    def t_7f(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        L.select("QLOITER")
        L.sleep(1)
        L.set_rc(ch5=OVR_MANUAL)
        self.check(L.wait_text(r"VTOL entry cancelled", 2, mark), "no 'VTOL entry cancelled' message")
        self.check(L.hold(lambda: L.mode == "MANUAL", 5), f"left MANUAL: now {L.mode}")
        self.check(not any(m in VTOL_MODES for m in L.modes_since(mark)), "a VTOL mode was entered")
        return "entry cancelled, stayed MANUAL"

    def t_7b(self):
        L = self.L
        self.check(L.mode == "MANUAL", f"expected MANUAL from 7f, in {L.mode}")
        mark = L.sim_ms
        L.select("QSTABILIZE")
        self.check(L.wait_text(r"MANUAL override active", 2, mark), "no 'MANUAL override active' message")
        self.check(L.hold(lambda: L.mode == "MANUAL" and L.door_st() == 0, 4),
                   f"mode {L.mode}, DOOR_ST {L.door_st()}")
        return "selector ignored in MANUAL, doors stayed closed"

    def t_7c(self):
        L = self.L
        L.set_rc(ch5=OVR_REST)
        self.check(L.wait(lambda: L.mode == "FBWA", 2), f"override rest did not give FBWA (mode {L.mode})")
        mark = L.sim_ms
        L.select("QLOITER")
        delay = self.gate("QLOITER", mark)
        L.select("FBWA")
        self.check(L.wait(lambda: L.door_st() == 0, 10), "doors did not close afterwards")
        return f"selector works again: QLOITER via the gate after {delay:.1f} s"

    def t_7d(self):
        L = self.L
        self.ensure_fbwa_closed()
        L.set_rc(ch5=OVR_MANUAL)
        self.check(L.wait(lambda: L.mode == "MANUAL", 2), f"no MANUAL (mode {L.mode})")
        self.check(L.hold(lambda: L.door_st() == 0, 3), "doors moved")
        L.set_rc(ch5=OVR_REST)
        L.wait(lambda: L.mode == "FBWA", 2)
        return "MANUAL, doors stayed closed"

    def t_7e(self):
        L = self.L
        self.ensure_fbwa_closed()
        trim = L.get_param(f"SERVO{L.chan['door']}_TRIM")
        L.set_param("DOOR_ENABLE", 0)
        try:
            L.set_rc(ch5=OVR_MANUAL)
            self.check(L.wait(lambda: L.mode == "MANUAL", 2), f"MANUAL did not engage (mode {L.mode})")
            # the script stops refreshing the door command, so the output lapses to its trim
            self.check(L.wait(lambda: L.outputs()[0] == trim, 2),
                       f"door output {L.outputs()[0]} did not return to SERVO{L.chan['door']}_TRIM {trim:.0f}")
            self.check(L.hold(lambda: L.outputs()[0] == trim, 2), f"door output left the trim {trim:.0f}")
        finally:
            L.set_param("DOOR_ENABLE", 1)
            L.set_rc(ch5=OVR_REST)
            L.wait(lambda: L.mode == "FBWA", 2)
        return f"MANUAL engaged with the script disabled; door output returned to its trim ({trim:.0f})"

    def t_10g(self):
        L = self.L
        self.ensure_fbwa_closed()
        trim = L.get_param(f"SERVO{L.chan['door']}_TRIM")
        mark = L.sim_ms
        L.command(MAV_CMD_SCRIPTING, SCRIPTING_CMD_STOP)
        try:
            self.check(L.wait(lambda: L.door_is(L.outputs()[0], "PART"), 3),
                       f"door output {L.outputs()[0]} did not fail to MID with scripting stopped "
                       f"(SERVO{L.chan['door']}_TRIM {trim:.0f}, DOOR_PART {L.params.get('DOOR_PART')})")
            lapse = (L.sim_ms - mark) / 1000
            self.check(L.hold(lambda: L.door_is(L.outputs()[0], "PART"), 3), "door output left MID while stopped")
        finally:
            restart = L.sim_ms
            L.command(MAV_CMD_SCRIPTING, SCRIPTING_CMD_STOP_AND_RESTART)
            restarted = L.wait_text(r"F35B: script loaded", 10, restart)
        self.check(restarted, "script did not restart")
        self.check(L.wait(lambda: L.door_is(L.outputs()[0], "CLOSED"), 10), "doors did not close after the restart")
        return (f"scripting stopped with the doors closed: output went to {trim:.0f} (trim, MID) after "
                f"{lapse:.1f} s and stayed; closed again after the restart")

    def t_8e(self):
        L = self.L
        self.ensure_fbwa_closed()
        rtl_mode = self.original("Q_RTL_MODE")
        L.set_param("Q_RTL_MODE", 0)          # keep RTL from switching to QRTL on the ground
        try:
            mark = L.sim_ms
            L.select("QLOITER")
            L.sleep(1)
            L.gcs_mode("RTL")
            self.check(L.wait_text(r"VTOL entry cancelled", 2, mark), "no 'VTOL entry cancelled' message")
            self.check(L.hold(lambda: L.mode == "RTL", 5), f"left RTL: now {L.mode}")
            self.check(not any(m in VTOL_MODES for m in L.modes_since(mark)), "a VTOL mode was entered")
        finally:
            L.set_param("Q_RTL_MODE", rtl_mode)
            L.gcs_mode("FBWA")
            L.wait(lambda: L.mode == "FBWA", 2)
            L.select("FBWA")
        return "entry cancelled, stayed RTL"

    def t_8a(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        L.open_window("8a GCS QHOVER")
        L.gcs_mode("QHOVER")
        self.check(L.wait(lambda: L.mode == "QHOVER", 2), f"no QHOVER (mode {L.mode})")
        self.check(L.wait_text(r"VTOL entered externally", 2, mark), "no 'VTOL entered externally' warning")
        self.check(L.wait(lambda: L.door_st() in (1, 2), 1.5), f"doors did not open (DOOR_ST {L.door_st()})")
        L.close_window(3)
        L.gcs_mode("FBWA")
        self.check(L.wait(lambda: L.mode == "FBWA" and L.door_st() == 0, 10), "did not return to FBWA, doors closed")
        return "warning sent, doors opened reactively"

    def t_8b(self):
        L = self.L
        self.ensure_fbwa_closed()
        self.enter_vtol_via_selector("QLOITER")
        L.gcs_mode("FBWA")
        self.check(L.wait(lambda: L.mode == "FBWA", 2), f"GCS FBWA not applied (mode {L.mode})")
        self.check(L.hold(lambda: L.mode == "FBWA", 5), f"script dragged the mode back to {L.mode}")
        self.check(L.wait(lambda: L.door_st() == 0, 10), "doors did not close")
        L.select("FBWA")
        return "GCS FBWA respected with the selector still on QLOITER"

    def t_5c(self):
        L = self.L
        self.ensure_fbwa_closed()
        good = L.params["DOOR_TILT_FWD"]
        L.monitor_on = False
        try:
            L.set_param("DOOR_TILT_FWD", 2100 if good >= 1500 else 900)
            self.check(L.wait(lambda: L.door_st() in (1, 2), 2), "doors did not open")
            self.check(L.hold(lambda: L.door_st() in (1, 2), 5), "doors closed although nacelles never read forward")
        finally:
            L.set_param("DOOR_TILT_FWD", good)
            L.wait(lambda: L.door_st() == 0, 5)
            L.monitor_on = True
        return "doors stayed open (fail-safe)"

    def t_10a(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        mode_ch = self.original("DOOR_MODE_CH")
        L.set_param("DOOR_MODE_CH", 0)
        try:
            L.select("QLOITER")
            self.check(L.hold(lambda: L.mode == "FBWA" and L.door_st() == 0, 4), f"mode {L.mode}, DOOR_ST {L.door_st()}")
            self.check(not L.text_seen(r"Internal Error", mark), "script error")
        finally:
            L.select("FBWA")
            L.sleep(0.5)
            L.set_param("DOOR_MODE_CH", mode_ch)
        self.check(L.hold(lambda: L.mode == "FBWA", 2), "mode changed when gating was restored")
        return "no gating, doors tracked phase, no errors"

    def t_10c(self):
        L = self.L
        self.ensure_fbwa_closed()
        mark = L.sim_ms
        for pwm in (1000, 1500, 2000, 1500, 1000, 2000, 1500):
            L.set_rc(ch8=pwm)
            L.sleep(0.3)
        self.check(L.wait(lambda: L.mode == "QLOITER", 5), f"did not settle in QLOITER (mode {L.mode})")
        self.check("QSTABILIZE" not in L.modes_since(mark), "entered QSTABILIZE during the sweep")
        self.check(not L.text_seen(r"Internal Error", mark), "script error")
        L.select("FBWA")
        self.check(L.wait(lambda: L.door_st() == 0, 10), "doors did not close afterwards")
        return "settled in the final selection (QLOITER) only"

    def t_10d(self):
        L = self.L
        self.ensure_fbwa_closed()
        tilt_fn = self.original("DOOR_TILT_FN")
        L.monitor_on = False
        try:
            L.set_param("DOOR_TILT_FN", 0)
            self.check(L.wait(lambda: L.nv("TILT_POS") == 0, 1.5), f"TILT_POS {L.nv('TILT_POS')}, expected 0")
            self.check(L.wait(lambda: L.door_st() in (1, 2), 2), "doors did not open")
            self.check(L.hold(lambda: L.door_st() in (1, 2), 3), "doors closed with no tilt reading")
        finally:
            L.set_param("DOOR_TILT_FN", tilt_fn)
            L.wait(lambda: L.door_st() == 0, 5)
            L.monitor_on = True
        return "TILT_POS 0, doors stayed open (fail-safe)"

    def t_10e(self):
        L = self.L
        mark = L.sim_ms
        self.check(L.wait_text(r"^F35B hb", 4, mark), "no debug heartbeat with DOOR_DEBUG 1")
        L.set_param("DOOR_DEBUG", 0)
        try:
            L.sleep(0.5)
            mark = L.sim_ms
            L.sleep(5)
            dbg = [t for t in L.texts_since(mark) if t.startswith("F35B ")]
            self.check(not dbg, f"debug output with DOOR_DEBUG 0: {dbg[0]}" if dbg else "")
        finally:
            L.set_param("DOOR_DEBUG", 1)
        return "debug output stops with DOOR_DEBUG 0"

    def t_4e(self):
        L = self.L
        self.needs_sitl_physics("SITL-controlled wind")
        self.ensure_fbwa_closed()
        tilt_fn = self.original("DOOR_TILT_FN")
        part_as = L.params["DOOR_PART_AS"]
        L.monitor_on = False
        try:
            # no tilt reading keeps the doors "active" outside VTOL, so the outbound threshold
            # applies; a headwind at that speed holds the airspeed estimate on the threshold
            L.set_param("DOOR_TILT_FN", 0)
            L.set_param("SIM_WIND_T", 1)          # constant wind: the default profile is ~0 at ground level
            L.set_param("SIM_WIND_DIR", L.heading)
            wind = part_as
            for _ in range(3):
                # the airspeed estimate reads below the wind speed; scale the wind until it sits
                # on the threshold
                L.set_param("SIM_WIND_SPD", wind)
                L.sleep(5)
                recent = [s[5] for s in L.samples[-20:]]
                mean = sum(recent) / len(recent)
                if abs(mean - part_as) < 0.15 or mean <= 0.5:
                    break
                wind = round(wind * part_as / mean, 2)
            mark = L.sim_ms
            L.sleep(30)
            window = [s for s in L.samples if s[0] >= mark]
            doors = [s[2] for s in window]
            change_ms = [b[0] for a, b in zip(window, window[1:]) if abs(a[2] - b[2]) > PWM_TOL]
            changes = len(change_ms)
            gaps = [(b - a) / 1000.0 for a, b in zip(change_ms, change_ms[1:])]
            speeds = [s[5] for s in window]
            lo, hi = min(speeds), max(speeds)
            if not (lo <= part_as <= hi):
                raise Skip(f"airspeed {lo:.1f}-{hi:.1f} m/s never crossed {part_as} m/s")
            # the debounce means no two changes closer than 0.75 s (one 0.1 s sample of slack)
            self.check(not gaps or min(gaps) >= 0.65, f"door changes {min(gaps) if gaps else 0:.2f} s apart "
                       f"(debounce 0.75 s); {changes} changes in 30 s")
        finally:
            L.set_param("SIM_WIND_SPD", 0)
            L.set_param("SIM_WIND_T", 0)
            L.set_param("DOOR_TILT_FN", tilt_fn)
            L.wait(lambda: L.door_st() == 0, 5)
            L.monitor_on = True
        spacing = f", at least {min(gaps):.1f} s apart" if gaps else ""
        return (f"{changes} door change(s) in 30 s with airspeed {lo:.1f}-{hi:.1f} m/s around {part_as} m/s"
                f"{spacing}; no change faster than the 0.75 s debounce")

    def t_9d(self):
        L = self.L
        self.ensure_fbwa_closed()
        self.enter_vtol_via_selector("QLOITER")
        L.monitor_on = False
        try:
            L.reboot()
            rebooted = L.sim_ms
            # measure from the restarted script, so nothing from before the reboot counts
            mark = L.wait_script_restart(rebooted)
            self.check(mark is not None, "script did not load after reboot")
        finally:
            L.monitor_on = True      # a failed check here must not leave the run unmonitored
        # Two safe outcomes, depending on which the vehicle reads first after boot: the script
        # sees the selector while the vehicle is still in its initial mode (MANUAL by default),
        # and the MANUAL lockout ignores it; or the override switch has already set FBWA, and
        # the selector goes through the door gate. Either way, no VTOL mode without the gate.
        # SITL's own RC can also show a different selector position until the overrides arrive,
        # so both can happen in turn. Let the boot settle, then require every VTOL entry to
        # have gone through the gate.
        L.sleep(12)
        gated = L.text_seen(r"doors opening", mark)
        locked = L.text_seen(r"MANUAL override active", mark)
        vtol = [m for m in L.modes_since(mark) if m in VTOL_MODES]
        self.check(gated or locked, "selector neither gated nor locked out after reboot")
        self.check(not vtol or (gated and L.text_seen(r"doors open - entering VTOL", mark)),
                   f"entered {vtol[0] if vtol else ''} without the gate")
        L.select("FBWA")
        self.check(L.wait(lambda: L.door_st() == 0, 10), "doors did not close afterwards")
        if vtol:
            return "after reboot on QLOITER: VTOL only through the door gate" + \
                (" (after an initial MANUAL lockout)" if locked else "")
        return ("after reboot on QLOITER: booted in MANUAL (INITIAL_MODE), selector locked out until moved; "
                "no VTOL entry")

    def t_10f(self):
        L = self.L
        self.needs_sitl_physics("simulated RC loss (RealFlight supplies the RC)")
        self.ensure_fbwa_closed()
        # Boot into FBWA: with the default INITIAL_MODE (MANUAL) the MANUAL lockout would hide
        # a script that acts on the selector before RC is valid. Before RC arrives the selector
        # reads 0 us (position 1 = QSTABILIZE).
        initial_mode = L.get_param("INITIAL_MODE")
        L.set_param("INITIAL_MODE", MODE_NUMBERS["FBWA"])
        L.set_param("SIM_RC_FAIL", 1)
        L.overrides = False
        L.monitor_on = False
        try:
            L.reboot()
            rebooted = L.sim_ms
            # measure from the restarted script, so nothing from before the reboot counts
            mark = L.wait_script_restart(rebooted)
            self.check(mark is not None, "script did not load after reboot")
            L.monitor_on = True
            L.sleep(10)
            self.check(L.mode == "FBWA", f"booted into {L.mode}, not FBWA")
            self.check(not L.text_seen(r"doors opening", mark), "script acted on the selector without RC")
            self.check(not any(m in VTOL_MODES for m in L.modes_since(mark)), "a VTOL mode was entered without RC")
        finally:
            L.monitor_on = True      # a failed check above must not leave the run unmonitored
            L.set_param("SIM_RC_FAIL", 0)
            L.set_param("INITIAL_MODE", initial_mode)
            L.overrides = True
            L.set_rc(ch5=OVR_REST, ch8=SEL["FBWA"])
            L.sleep(3)
        self.ensure_fbwa_closed()
        return "booted into FBWA without RC: no selector action for 10 s"

    # -- flight tests ------------------------------------------------------
    def f_takeoff(self):
        L = self.L
        self.ensure_fbwa_closed()
        self.enter_vtol_via_selector("QLOITER")
        L.set_rc(ch3=1000)
        L.arm()
        self.climb_hover(SAFE_ALT)
        L.sleep(1)
        return f"armed in QLOITER, hovering at {L.rel_alt:.0f} m"

    def f_1b(self):
        L = self.L
        tilt = L.outputs()[1]
        self.cal["tilt_hover"] = tilt
        self.check(abs(tilt - self.cal["tilt_fwd"]) > 100, f"tilt output {tilt} barely moved from forward "
                   f"{self.cal['tilt_fwd']}")
        return f"TILT_POS {L.nv('TILT_POS'):.0f} in hover vs {self.cal['tilt_fwd']} forward"

    def f_1c(self):
        L = self.L
        lift = L.outputs()[2]
        self.cal["lift_hover"] = lift
        stopped = self.cal["lift_stopped"]
        self.check(lift > stopped + 50, f"lift output {lift} in hover, {stopped} stopped")
        if not self.cfg.calibrate:
            lift_off, tilt_fwd = L.params["DOOR_LIFT_OFF"], L.params["DOOR_TILT_FWD"]
            self.check(stopped <= lift_off < lift, f"DOOR_LIFT_OFF {lift_off:.0f} is not between the stopped "
                       f"({stopped}) and hover ({lift}) lift outputs")
            self.check(L.nacelles_fwd(self.cal["tilt_fwd"]) and not L.nacelles_fwd(self.cal["tilt_hover"]),
                       f"DOOR_TILT_FWD {tilt_fwd:.0f} doesn't separate forward ({self.cal['tilt_fwd']}) from "
                       f"hover ({self.cal['tilt_hover']})")
            return (f"LIFT_PWM {lift} hover vs {stopped} stopped; DOOR_LIFT_OFF {lift_off:.0f} and "
                    f"DOOR_TILT_FWD {tilt_fwd:.0f} fit the outputs")
        lift_off = round(stopped + 0.3 * (lift - stopped))
        fwd, hover = self.cal["tilt_fwd"], self.cal["tilt_hover"]
        tilt_fwd = round(fwd - max(20, 0.05 * abs(fwd - hover)) * (1 if fwd > hover else -1))
        L.set_param("DOOR_LIFT_OFF", lift_off)
        L.set_param("DOOR_TILT_FWD", tilt_fwd)
        self.cal.update(lift_off=lift_off, tilt_fwd_param=tilt_fwd)
        return f"LIFT_PWM {lift} hover vs {stopped} stopped; set DOOR_LIFT_OFF {lift_off}, DOOR_TILT_FWD {tilt_fwd}"

    def f_3c(self):
        """Outbound; also records the data for 4a, 5a and 5b."""
        L = self.L
        self.climb_hover(SAFE_ALT)
        mark = L.sim_ms
        L.set_rc(ch3=1900)
        L.select("FBWA")
        self.check(L.wait(lambda: L.mode == "FBWA", 1), f"FBWA not immediate (mode {L.mode})")
        start = L.sim_ms
        while L.sim_ms - start < 90000 and L.door_st() != 0:
            L.pump()
            self.check_alt()
        self.check(L.door_st() == 0, f"doors never closed (DOOR_ST {L.door_st()}, airspeed {L.airspeed:.1f})")
        # DOOR_ST can arrive before the servo sample showing the closed door
        self.check(L.wait(lambda: any(s[0] >= mark and L.door_is(s[2], "CLOSED") for s in L.samples[-20:]), 2),
                   "DOOR_ST reads closed but the door output never did")
        # all timings from the same servo samples: first closed sample, and when the nacelles
        # last arrived forward / the lift motor last stopped before it
        samples = [s for s in L.samples if s[0] >= mark]
        close_i = next(i for i, s in enumerate(samples) if L.door_is(s[2], "CLOSED"))

        def arrived(ok):
            i = close_i
            while i > 0 and ok(samples[i - 1]):
                i -= 1
            return samples[i][0] if ok(samples[i]) else None
        self.outbound = dict(mark=mark, close_ms=samples[close_i][0],
                             tilt_fwd_ms=arrived(lambda s: L.nacelles_fwd(s[3])),
                             lift_off_ms=arrived(lambda s: L.lift_off(s[4])))
        close_ms = samples[close_i][0]
        self.no_violations_since(mark)
        return f"FBWA immediately; doors closed {(close_ms - mark) / 1000:.1f} s later at {L.airspeed:.1f} m/s"

    def f_4a(self):
        L = self.L
        o = getattr(self, "outbound", None)
        if not o:
            raise Skip("no outbound data (3c failed)")
        part = L.params["DOOR_PART_AS"]
        delay, at, crossed_at = self.debounce_delay(o["mark"], lambda d: L.door_is(d, "PART"), lambda a: a >= part)
        self.check(at is not None, "doors never went partial on the way out")
        self.check(delay is not None, f"partial at {at:.1f} m/s but airspeed never crossed {part:.0f} m/s first")
        self.check(DEBOUNCE_MIN_S <= delay <= 1.5, f"partial {delay:.2f} s after crossing {part:.0f} m/s (expected ~0.75 s)")
        return (f"partial {delay:.2f} s after airspeed crossed DOOR_PART_AS {part:.0f} m/s "
                f"(debounce 0.75 s; {at:.1f} m/s by then, accelerating)")

    def f_5a(self):
        o = getattr(self, "outbound", None)
        if not o:
            raise Skip("no outbound data (3c failed)")
        self.check(o["tilt_fwd_ms"] is not None, "doors closed with the nacelles not forward")
        self.check(o["close_ms"] >= o["tilt_fwd_ms"], "doors closed before the nacelles were forward")
        return (f"nacelles forward at +{(o['tilt_fwd_ms'] - o['mark']) / 1000:.1f} s, "
                f"doors closed at +{(o['close_ms'] - o['mark']) / 1000:.1f} s")

    def f_5b(self):
        o = getattr(self, "outbound", None)
        if not o:
            raise Skip("no outbound data (3c failed)")
        self.check(o["lift_off_ms"] is not None, "doors closed with the lift motor running")
        self.check(o["close_ms"] >= o["lift_off_ms"], "doors closed while the lift motor was running")
        return (f"lift motor off at +{(o['lift_off_ms'] - o['mark']) / 1000:.1f} s, "
                f"doors closed at +{(o['close_ms'] - o['mark']) / 1000:.1f} s")

    def f_cruise(self):
        L = self.L
        L.set_rc(ch1=1650)                     # gentle turn to stay near home
        ok = True
        start = L.sim_ms
        while L.sim_ms - start < 15000:
            L.pump()
            self.check_alt()
            ok = ok and L.door_st() == 0
        L.set_rc(ch1=1500)
        L.sleep(2)
        self.check(ok, "doors opened during cruise")
        return f"doors closed through 15 s of cruise at {L.airspeed:.1f} m/s, {L.rel_alt:.0f} m"

    def f_3a(self):
        """Fast inbound entry; also records the data for 4b."""
        L = self.L
        fast = L.airspeed
        mark = L.sim_ms
        L.set_rc(ch3=1500)
        L.select("QLOITER")
        self.check(L.wait(lambda: L.door_st() == 3, 1), f"doors not OPENING (DOOR_ST {L.door_st()})")
        partial = L.wait(lambda: L.door_is(L.outputs()[0], "PART"), 1.5)
        partial_ms = L.sim_ms
        delay = self.gate("QLOITER", mark)
        full_as = None
        start = L.sim_ms
        while L.sim_ms - start < 40000:
            L.pump()
            self.check_alt()
            if L.door_is(L.outputs()[0], "OPEN"):
                full_as = L.door_airspeed()
                break
        self.inbound = dict(full_as=full_as, mark=mark, fast=fast)
        self.no_violations_since(mark)
        self.check(partial, f"doors did not go partial during a fast entry ({fast:.1f} m/s)")
        self.check(full_as is not None, f"doors never opened fully (airspeed {L.airspeed:.1f})")
        self.check(L.text_seen(r"doors latched FULL", mark), "no 'doors latched FULL' debug message")
        return (f"from {fast:.1f} m/s: partial after {(partial_ms - mark) / 1000:.1f} s, held FBWA, "
                f"QLOITER after {delay:.1f} s, full at {full_as:.1f} m/s; no VTOL motion before the mode change")

    def f_4b(self):
        L = self.L
        i = getattr(self, "inbound", None)
        if not i or i["full_as"] is None:
            raise Skip("no inbound data (3a failed)")
        full = L.params["DOOR_FULL_AS"]
        if i["fast"] <= full:
            raise Skip(f"entry at {i['fast']:.1f} m/s, not above DOOR_FULL_AS {full:.0f}")
        delay, at, _ = self.debounce_delay(i["mark"], lambda d: L.door_is(d, "OPEN"), lambda a: a < full)
        self.check(delay is not None, f"full at {at} m/s but airspeed never crossed {full:.0f} m/s first")
        self.check(DEBOUNCE_MIN_S <= delay <= 1.5, f"full {delay:.2f} s after crossing {full:.0f} m/s (expected ~0.75 s)")
        return (f"full {delay:.2f} s after airspeed fell through DOOR_FULL_AS {full:.0f} m/s "
                f"(debounce 0.75 s; {at:.1f} m/s by then, decelerating)")

    def f_4c(self):
        L = self.L
        self.needs_sitl_physics("SITL-controlled wind")
        L.wait(lambda: L.airspeed < 5, 30)
        self.check(L.door_st() == 2, f"doors not full in hover (DOOR_ST {L.door_st()})")
        full = L.params["DOOR_FULL_AS"]
        L.set_param("SIM_WIND_T", 1)
        L.set_param("SIM_WIND_DIR", L.heading)
        try:
            wind = None
            for wind in (full + 5, full + 9, full + 13):
                L.set_param("SIM_WIND_SPD", wind)
                if L.wait(lambda: L.door_airspeed() > full + 1, 15):
                    break
                self.check_alt()
            if L.door_airspeed() <= full + 1:
                raise Skip(f"airspeed only reached {L.door_airspeed():.1f} m/s in a {wind:.0f} m/s headwind")
            peak = [L.door_airspeed()]

            def full_and_track():
                peak[0] = max(peak[0], L.door_airspeed())
                return L.door_st() == 2
            self.check(L.hold(full_and_track, 10), f"doors went {DOOR_ST_NAME.get(L.door_st())} at "
                       f"{L.door_airspeed():.1f} m/s after latching full")
        finally:
            L.set_param("SIM_WIND_SPD", 0)
            L.set_param("SIM_WIND_T", 0)
            L.wait(lambda: L.airspeed < 5, 30)
        return f"stayed full for 10 s at up to {peak[0]:.1f} m/s (above DOOR_FULL_AS {full:.0f})"

    def f_6a(self):
        L = self.L
        mark = L.sim_ms
        L.select("QSTABILIZE")
        self.check(L.wait(lambda: L.mode == "QSTABILIZE", 1), f"VTOL->VTOL not immediate (mode {L.mode})")
        self.check(not L.text_seen(r"doors opening", mark), "went through the door delay")
        L.sleep(2)
        self.check_alt()
        L.select("QLOITER")
        self.check(L.wait(lambda: L.mode == "QLOITER", 1), f"back to QLOITER not immediate (mode {L.mode})")
        self.check(L.door_st() in (1, 2), "doors not open")
        return "QLOITER->QSTABILIZE->QLOITER immediate, no delay"

    def f_7a(self):
        L = self.L
        self.climb_hover(MANUAL_ALT)
        mark = L.sim_ms
        L.set_rc(ch5=OVR_MANUAL)
        self.check(L.wait(lambda: L.mode == "MANUAL", 1), f"MANUAL not immediate (mode {L.mode})")
        L.set_rc(ch3=1800)
        # MANUAL from a hover is a bailout: leave it as soon as the tilt output reads forward (about
        # 1 s at 90 deg/s). The nozzle estimate doesn't depend on the mode, so the doors still wait
        # for it in FBWA. Staying in MANUAL until the doors close crashed the RealFlight model.
        start = L.sim_ms
        while L.sim_ms - start < 3000 and not L.nacelles_fwd(L.outputs()[1]):
            L.pump()
            self.check_alt()
        manual_s = (L.sim_ms - mark) / 1000
        L.set_rc(ch5=OVR_REST, ch3=1900)
        self.check(L.wait(lambda: L.mode == "FBWA", 2), f"override rest did not give FBWA (mode {L.mode})")
        start = L.sim_ms
        while L.sim_ms - start < 30000 and L.door_st() != 0:
            L.pump()
            self.check_alt()
        self.check(L.door_st() == 0, f"doors did not close (DOOR_ST {L.door_st()})")
        # DOOR_ST can arrive before the servo sample showing the closed door, and until then
        # L.closes[-1] is an earlier close
        self.check(L.wait(lambda: bool(L.closes) and L.closes[-1]["close"] >= mark, 2),
                   "DOOR_ST reads closed but the door output never did")
        self.no_violations_since(mark)
        # MANUAL tilts at 90 deg/s or more, faster than the nozzle: the doors must wait for the estimate
        c = L.closes[-1]
        self.check(c is not None and c["tilt"] is not None and c["noz"] is not None,
                   "no close with the tilt output and nozzle estimate both forward")
        rel = {k: (v - mark) / 1000.0 for k, v in c.items() if v is not None}
        lag = rel["noz"] - rel["tilt"]
        self.check(lag >= 0.5, f"the nozzle estimate trailed the tilt output by only {lag:.1f} s; the MANUAL "
                   f"swing wasn't faster than the nozzle, so nothing was tested")
        self.check(rel["close"] >= rel["noz"] - 0.2, f"doors closed at +{rel['close']:.1f} s, before the nozzle "
                   f"estimate read forward at +{rel['noz']:.1f} s")
        start = L.sim_ms
        while L.sim_ms - start < 10000:
            L.pump()
            self.check_alt()
        lift = f", lift off +{rel['lift']:.1f} s" if "lift" in rel else ""
        return (f"MANUAL immediately, back to FBWA after {manual_s:.1f} s. After the switch: tilt output forward "
                f"+{rel['tilt']:.1f} s, nozzle estimate forward +{rel['noz']:.1f} s{lift}, doors closed "
                f"+{rel['close']:.1f} s (DOOR_NOZ_MS {L.params.get('DOOR_NOZ_MS', 3500):.0f})")

    def f_4d(self):
        L = self.L
        mark_reset = getattr(self, "inbound", {}).get("mark", 0)
        self.check(L.text_seen(r"full latch reset", mark_reset), "no 'full latch reset' after leaving VTOL")
        L.select("FBWA")                       # FBWA already: makes the next move an edge
        L.sleep(0.5)
        fast = L.airspeed
        mark = L.sim_ms
        L.set_rc(ch3=1500)
        L.select("QLOITER")
        self.check(L.wait(lambda: L.door_st() == 3, 1), "doors not OPENING")
        first_full = L.door_is(L.outputs()[0], "OPEN")
        partial = L.wait(lambda: L.door_is(L.outputs()[0], "PART"), 1.5)
        partial_s = (L.sim_ms - mark) / 1000
        self.gate("QLOITER", mark)
        self.check(L.wait(lambda: L.door_st() == 2, 40), "doors did not latch full again")
        self.no_violations_since(mark)
        self.check(partial, f"doors did not go partial on the fast re-entry ({fast:.1f} m/s)")
        note = " (commanded full for the first 0.75 s: the debounce starts from full)" if first_full else ""
        return f"latch reset seen; re-entry at {fast:.1f} m/s went partial after {partial_s:.1f} s{note}"

    def f_3d(self):
        got = {tid: status for tid, _, status, _ in self.results}
        parts = ["3c", "3a", "7a", "4d"]
        bad = [p for p in parts if got.get(p) != "PASS"]
        self.check(not bad, f"depends on {', '.join(bad)}")
        return "FBWA->QLOITER->hover->FBWA->cruise and back, all legs passed"

    def f_8d(self):
        L = self.L
        self.climb_hover(SAFE_ALT)
        L.set_rc(ch3=1900)
        L.select("FBWA")
        self.check(L.wait(lambda: L.mode == "FBWA", 1), f"no FBWA (mode {L.mode})")
        start = L.sim_ms
        while L.sim_ms - start < 90000 and L.door_st() != 0:
            L.pump()
            self.check_alt()
        self.check(L.door_st() == 0, "doors did not close on the way out")
        start = L.sim_ms
        while L.sim_ms - start < 20000:
            L.pump()
            self.check_alt()
        if self.original("Q_RTL_MODE") != 1:
            L.set_param("Q_RTL_MODE", 1)      # RTL switches to QRTL near home
        mark = L.sim_ms
        L.open_window("8d RTL auto-transition")
        L.gcs_mode("RTL")
        self.check(L.wait(lambda: L.mode == "RTL", 2), f"no RTL (mode {L.mode})")
        self.check(L.wait(lambda: L.mode == "QRTL", 150), f"RTL never switched to QRTL (mode {L.mode})")
        qrtl_ms = L.sim_ms
        self.check(L.wait_text(r"VTOL entered externally", 3, mark), "no 'VTOL entered externally' warning")
        self.check(L.wait(lambda: L.door_st() in (1, 2), 3), "doors did not open in QRTL")
        L.close_window(10)
        # door lag: first sample with VTOL hardware active vs first sample with the doors open
        active = [s for s in L.samples if s[0] >= mark and not (L.nacelles_fwd(s[3]) and L.lift_off(s[4]))]
        opened = [s for s in L.samples if s[0] >= mark and not L.door_is(s[2], "CLOSED")]
        lag = (opened[0][0] - active[0][0]) / 1000 if active and opened else None
        if not L.wait(lambda: not L.armed, 240):
            raise Abort(f"QRTL did not land and disarm (alt {L.rel_alt:.1f} m)")
        L.set_rc(ch3=1000)
        if self.original("Q_RTL_MODE") != 1:
            L.set_param("Q_RTL_MODE", self.original("Q_RTL_MODE"))
        if lag is None:
            lag_txt = "door lag not measured"
        else:
            when = f"{lag:.1f} s after" if lag > 0 else "no later than"
            lag_txt = (f"doors commanded open {when} the VTOL hardware started moving (10 Hz samples), but "
                       f"physical door travel still lags (documented limitation)")
        return f"RTL switched to QRTL after {(qrtl_ms - mark) / 1000:.0f} s, warning sent; {lag_txt}; landed"

    # -- whole-run check ---------------------------------------------------
    def t_monitor(self):
        L = self.L
        bad = [v for v in L.violations if v[3] is None]
        expected = [v for v in L.violations if v[3] is not None]
        self.check(not bad, f"{len(bad)} violation(s); first at t={bad[0][0] / 1000:.1f}s: {bad[0][2]}" if bad else "")
        waits = [(c["noz"] - c["tilt"]) / 1000.0 for c in L.closes if c["noz"] is not None and c["tilt"] is not None]
        hold_txt = (f"{len(L.closes)} close(s), none before the nozzle estimate read forward (it trailed the tilt "
                    f"output by up to {max(waits):.1f} s)" if waits else f"{len(L.closes)} close(s)")
        return (f"{len(L.samples)} output samples; doors never closed on active VTOL hardware and nothing moved "
                f"during a pending entry; {hold_txt}; {len(expected)} sample(s) inside the documented "
                f"external-entry windows")


# ---------------------------------------------------------------------------
def write_results(path, runner, args, cfg, started, wall_s):
    L = runner.L
    counts = {}
    for _, _, status, _ in runner.results:
        counts[status] = counts.get(status, 0) + 1
    try:
        # The last commit that changed the script, not HEAD: a run made before a later docs-only
        # commit (or an amend) would otherwise name a commit that doesn't match the script.
        rev = subprocess.run(["git", "-C", str(REPO), "log", "-1", "--format=%h %s", "--", args.script],
                             capture_output=True, text=True).stdout.strip()
        if subprocess.run(["git", "-C", str(REPO), "diff", "--quiet", "HEAD", "--", args.script]).returncode:
            rev += " + uncommitted changes"
    except Exception:
        rev = "?"
    lines = [
        "# F35B.lua SITL results",
        "",
        f"- Run: {started:%Y-%m-%d %H:%M}, {wall_s / 60:.1f} min wall time, speedup {cfg.speedup}",
        f"- Script: `{os.path.relpath(args.script, REPO)}` at `{rev}`",
        f"- Model: {cfg.description}",
        f"- Totals: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())),
        "",
        "Calibration: " + ", ".join(f"{k} {v}" for k, v in runner.cal.items()),
        "",
        "| ID | Test | Result | Detail |",
        "|----|------|--------|--------|",
    ]
    for tid, title, status, detail in runner.results:
        lines.append(f"| {tid} | {title} | {status} | {detail.replace('|', '/')} |")
    if cfg.realflight:
        lines += ["", "Skipped with RealFlight: 4c and 4e (need SITL-controlled wind) and 10f (needs simulated "
                  "RC loss)."]
    lines += ["", "Not automated: 3b (the model stalls above DOOR_PART_AS, so a slow fixed-wing entry is "
              "impossible), 8c (AUTO mission), 10b (RC failsafe in flight).", ""]
    if L.violations:
        lines += ["## Safety monitor events", ""]
        for ms, kind, detail, window in L.violations[:40]:
            lines.append(f"- t={ms / 1000:.1f}s {kind}: {detail}" + (f" (expected: {window})" if window else ""))
        lines.append("")
    Path(path).write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--realflight", action="store_true", help="fly the F-35B in RealFlight over FlightAxis")
    ap.add_argument("--rf-host", help="RealFlight host IP (default: the Windows host seen from WSL)")
    ap.add_argument("--params", help=f"parameter file for --realflight (default {RF_PARAMS.name})")
    ap.add_argument("--model", help=argparse.SUPPRESS)       # override the SITL model (dry runs)
    ap.add_argument("--speedup", type=int, default=5, help="SITL speedup for the stand-in model (default 5)")
    ap.add_argument("--script", default=str(APPLETS / "F35B.lua"), help="F35B.lua to test")
    ap.add_argument("--results", help="Markdown results file (default last_results[_realflight].md here)")
    ap.add_argument("--ground-only", action="store_true", help="skip the flight tests")
    ap.add_argument("--step", action="store_true", help="pause before each test until Enter is pressed")
    ap.add_argument("--keep", action="store_true", help="keep the SITL run directory (logs, eeprom)")
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)
    args.script = str(Path(args.script).resolve())

    started = datetime.datetime.now()
    t0 = time.time()
    cfg = Config(args)
    if cfg.model.startswith("flightaxis"):
        host = cfg.model.split(":", 1)[1] if ":" in cfg.model else "127.0.0.1"
        if not port_open(host, FLIGHTAXIS_PORT):
            sys.exit(f"RealFlight is not reachable at {host}:{FLIGHTAXIS_PORT}. Start RealFlight, load the F-35B "
                     "model, and enable RealFlight Link (in RealFlight 9: Simulation > Settings > Physics > "
                     "Quality > RealFlight Link Enabled). Use --rf-host if RealFlight runs elsewhere.")
        print(f"RealFlight at {host}: tests run in real time. Watch the door state here; "
              "Ctrl-C stops SITL (RealFlight keeps running).")
    sitl = Sitl(args.script, cfg, args.keep)
    link = Link(cfg.speedup)
    runner = Runner(link, sitl, cfg, step=args.step)
    failed = True
    try:
        sitl.start()
        link.connect()
        runner.setup()

        print("Ground tests (disarmed)")
        for tid, title, fn in [
            ("1a", "Telemetry: TILT_POS matches the tilt output", runner.t_1a),
            ("1d", "DOOR_ST stable while idle", runner.t_1d),
            ("2a", "Selector alone cannot change the mode", runner.t_2a),
            ("9c", "Disarmed FBWA -> QLOITER goes through the door gate", runner.t_9c),
            ("9a", "Disarmed in a VTOL mode: doors open", runner.t_9a),
            ("9b", "Disarmed -> FBWA: doors close once nacelles forward", runner.t_9b),
            ("6b", "Selector moved away during a pending entry cancels it", runner.t_6b),
            ("6c", "New VTOL selection during a pending entry retargets it", runner.t_6c),
            ("6d", "Fixed-wing to fixed-wing via the selector", runner.t_6d),
            ("7f", "MANUAL override during a pending entry cancels it", runner.t_7f),
            ("7b", "Selector ignored in MANUAL", runner.t_7b),
            ("7c", "Override back to rest re-enables the selector", runner.t_7c),
            ("7d", "FBWA -> MANUAL override: doors stay closed", runner.t_7d),
            ("7e", "MANUAL override works with the script disabled", runner.t_7e),
            ("8e", "GCS mode change during a pending entry cancels it", runner.t_8e),
            ("8a", "GCS VTOL mode: warning, doors open reactively", runner.t_8a),
            ("8b", "GCS FBWA respected with the selector on VTOL", runner.t_8b),
            ("5c", "Nacelles never forward: doors never close", runner.t_5c),
            ("10a", "DOOR_MODE_CH 0: no gating, no errors", runner.t_10a),
            ("10c", "Rapid selector sweep", runner.t_10c),
            ("10d", "DOOR_TILT_FN 0: no reading, doors stay open", runner.t_10d),
            ("10e", "DOOR_DEBUG 0 silences debug output", runner.t_10e),
            ("10g", "Scripting stopped: doors fail to MID", runner.t_10g),
            ("4e", "No door chatter at a steady threshold airspeed", runner.t_4e),
            ("9d", "Reboot with the selector on QLOITER: gated entry", runner.t_9d),
            ("10f", "Boot without RC: selector ignored", runner.t_10f),
        ]:
            runner.run(tid, title, fn)

        if not args.ground_only:
            print("Flight tests")
            for tid, title, fn in [
                ("F0", f"Arm in QLOITER and climb to {SAFE_ALT} m", runner.f_takeoff),
                ("1b", "TILT_POS moves with the nacelles in hover", runner.f_1b),
                ("1c", "LIFT_PWM: lift motor running vs stopped; calibrate", runner.f_1c),
                ("3c", "Outbound: QLOITER -> FBWA", runner.f_3c),
                ("4a", "Outbound partial airspeed ~ DOOR_PART_AS", runner.f_4a),
                ("5a", "Close waits for nacelles forward", runner.f_5a),
                ("5b", "Close waits for the lift motor to stop", runner.f_5b),
                ("F1", "Cruise: doors stay closed", runner.f_cruise),
                ("3a", "Fast inbound: FBWA -> QLOITER through the gate", runner.f_3a),
                ("4b", "Inbound full airspeed ~ DOOR_FULL_AS", runner.f_4b),
                ("4c", "Latched full: fast airflow in VTOL keeps doors full", runner.f_4c),
                ("6a", "VTOL -> VTOL switch is immediate", runner.f_6a),
                ("7a", "MANUAL override in hover", runner.f_7a),
                ("4d", "Latch resets per VTOL session", runner.f_4d),
                ("3d", "Full round trip", runner.f_3d),
                ("8d", "RTL auto-transition to QRTL (documented limitation)", runner.f_8d),
            ]:
                runner.run(tid, title, fn, flight=True)

        runner.run("INV", "Safety monitor over the whole run", runner.t_monitor)
        failed = any(r[2] in ("FAIL", "ERROR") for r in runner.results)
    except KeyboardInterrupt:
        print("interrupted")
    except Exception as e:
        print(f"run aborted: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        wall = time.time() - t0
        link.close() if link.m else None
        sitl.stop()
        if runner.results:
            write_results(cfg.results, runner, args, cfg, started, wall)
            print(f"\nResults written to {cfg.results}")
        counts = {}
        for r in runner.results:
            counts[r[2]] = counts.get(r[2], 0) + 1
        print("Summary: " + (", ".join(f"{k} {v}" for k, v in sorted(counts.items())) or "no results"))
        sitl.cleanup(failed)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
