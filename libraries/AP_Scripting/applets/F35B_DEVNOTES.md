# F35B.lua — Development Notes

Working notes for continuing development of `F35B.lua`. `F35B.md` is the user-facing
setup and behavior reference; this file records project status, review findings and open
work, so the context survives between sessions.

## Status (2026-11-02)

- **Preliminary.** Only the DOORS subsystem exists. The automated SITL tests cover most of
  `F35B_TESTPLAN.md`; the manual items have no recorded results.
- **Not flight-tested.** Review findings 1–4 were fixed on 2026-11-01, findings 6–7 on
  2026-11-02, and revised on 2026-11-03 after the failure-mode review. On 2026-11-03 the
  automated SITL suite passed 43/43 on the stand-in tiltrotor model
  (`F35B_sitl/last_results.md`) and 40 with 3 skipped on the F-35B in RealFlight
  (`F35B_sitl/last_results_realflight.md`). Settle finding 5, do the hardware steps under
  Next steps, and bench-test before flying it.
- Developed around July 2026 (files dated 2026-07-05) and committed here on 2026-09-12.
  The original design discussion was not preserved; `F35B.md` and `F35B_TESTPLAN.md` are
  the design record.

## Files

| File | Purpose |
|------|---------|
| `F35B.lua` | The applet (DOORS subsystem) |
| `F35B.md` | Setup, parameters, behavior reference |
| `F35B_TESTPLAN.md` | SITL test plan (current source) |
| `F35B_TESTPLAN.html` | Older rendering of the test plan; out of date (see below) |
| `F35B_TESTPLAN.param` | SITL parameter set the test plan assumes (its "consolidated list") |
| `F35B_offline/` | Offline syntax check and behavioral tests (`run.sh`) |
| `F35B_sitl/` | Automated SITL test runner, its parameters, and `last_results.md` |

### SITL test parameters (`F35B_TESTPLAN.param`)

The test plan's selector and override commands assume this set:

- Selector on RC8 (`DOOR_MODE_CH 8`), used as a 3-position switch:
  1000 → QSTABILIZE (`DOOR_MODE1/2 = 17`), 1500 → QLOITER (`DOOR_MODE3/4 = 19`),
  2000 → FBWA (`DOOR_MODE5/6 = 5`).
- Emergency override on RC5 (`FLTMODE_CH 5`): `FLTMODE1..5 = MANUAL`, `FLTMODE6 = FBWA`,
  so the high position is the "rest" FBWA position and anything lower is MANUAL.
- Door output `SERVO13_FUNCTION 94`; tilt `SERVO8_FUNCTION 41` with forward at the low PWM
  end (`DOOR_TILT_FWD 1010`); lift motor 3 on `SERVO1` (`DOOR_LIFT_FN 35`,
  `DOOR_LIFT_OFF 1010`).
- `DOOR_PART_AS 5`, `DOOR_FULL_AS 15`, `DOOR_ENTRY_MS 3000`, `ARSPD_TYPE 2`.

## Offline tests

```bash
libraries/AP_Scripting/applets/F35B_offline/run.sh
```

`run.sh` builds ArduPilot's bundled Lua (`libraries/AP_Scripting/lua/src`) in a temporary
directory, syntax-checks `F35B.lua` with it, then runs `harness.lua`. The harness stubs the
bindings and drives the script's scheduler through scenarios using the test-plan
parameters. It exits non-zero if any scenario fails.

**Expected: 6/6 pass.** Scenarios 2–6 cover findings 1–4. Each one failed against the
unfixed script (commit `fe69b53045`: 1/6 pass) and passes after the fixes. Add a scenario for
each new fix or behavior. To check a different copy of the script, pass its path:
`run.sh path/to/F35B.lua`.

ArduPilot's Lua strips `print`, `loadfile`, `dofile`, `select` and `os`, so the harness
uses `io.write`, `io.open` + `load`, and `error()` for the exit status.

## SITL tests

```bash
./waf configure --board sitl && ./waf plane      # once per branch switch
libraries/AP_Scripting/applets/F35B_sitl/run_sitl_tests.py
```

`run_sitl_tests.py` runs the test plan unattended and writes `F35B_sitl/last_results.md`.
A full run (ground and flight) takes a few minutes at the default `--speedup 5`.
`--ground-only` skips the flight sequence, and `--script PATH` tests another copy of the
script. It prints each door state change live, and exits non-zero on any failure.

**Model.** There is no SITL physics model for the F-35B frame (`Q_FRAME_CLASS 35`), so the
runner uses the stock `quadplane-tilttri` model with `F35B_sitl/tilttri.parm`. Its two front
motors tilt (function 41) and its rear motor (function 36) is a non-tilting lift motor that
stops in fixed-wing flight. The door script only watches outputs, so the door logic is
exercised as it would be on the airframe, but the flight dynamics are not the F-35B's. The
runner calibrates `DOOR_TILT_FWD` and `DOOR_LIFT_OFF` from the outputs it sees, and uses
`DOOR_PART_AS 7` / `DOOR_FULL_AS 15` to suit the model's 13 m/s stall and 25 m/s cruise.

**What it checks.** Ground tests (disarmed) run first, then one flight: arm and climb in
QLOITER, outbound transition, cruise, fast inbound entry, latched-full in a headwind,
VTOL-to-VTOL switch, MANUAL bailout in hover, a second fast entry, and an RTL that switches
to QRTL and lands. Throughout the run a monitor checks the raw servo outputs: the doors
are never closed while the nacelles are not forward or the lift motor is powered, and the
VTOL hardware never deploys while an entry is pending. Airspeed thresholds (4a, 4b) are
checked as the delay from the threshold crossing to the door change, which should be the
0.75 s debounce.

**Not automated:** 3b (the model stalls above `DOOR_PART_AS`, so a slow fixed-wing entry is
impossible), 8c (AUTO mission) and 10b (RC failsafe in flight).

### With RealFlight

```bash
libraries/AP_Scripting/applets/F35B_sitl/run_sitl_tests.py --realflight [--step]
```

Start RealFlight first with the "Lofted Aero Simple F-35B" model (from the release `.RFX`)
and RealFlight Link enabled (RealFlight 9: Simulation > Settings > Physics > Quality). The
runner connects SITL over FlightAxis to the Windows host as seen from WSL (`--rf-host` to
override) and runs the same tests in real time on the F-35B parameter set,
`F35B_TESTPLAN.param`, with its own `DOOR_` values (no recalibration; 1c instead checks that
`DOOR_TILT_FWD` and `DOOR_LIFT_OFF` fit the outputs). The model has no doors, so the door
state is printed in the terminal as it changes. `--step` pauses before each test until
Enter is pressed, while still holding the controls. Results go to
`F35B_sitl/last_results_realflight.md`.

Skipped with RealFlight: 4c and 4e (need SITL-controlled wind) and 10f (needs simulated RC
loss; RealFlight supplies the RC). 4b is skipped if the inbound entry is not faster than
`DOOR_FULL_AS`.

Two things specific to the F-35B parameter set: it boots in MANUAL (`INITIAL_MODE 0`), so
after a reboot with the selector on a VTOL position the MANUAL lockout ignores the selector
until it is moved (9d accepts that or a gated entry, as both are safe); and it is a hardware
dump with the watchdog on (`BRD_OPTIONS 1`), which in SITL kills the process on reboot, so
the runner sets `BRD_OPTIONS 0` (FlightAxis also forces this).

The first RealFlight run (2026-11-02) exposed two runner problems that the faster tilttri
runs hide, both fixed:

- **Airspeed source.** With an airspeed sensor fitted but unused (`ARSPD_TYPE 2`,
  `ARSPD_USE 0`), VFR_HUD reports the raw sensor, while the script's thresholds use the
  AHRS estimate. Under hard acceleration the two crossed 5 m/s about 0.35 s apart, so 4a
  measured a 0.40 s debounce. The script now publishes the airspeed it uses as `DOOR_AS`,
  and the runner times threshold crossings against that (VFR_HUD only as a fallback).
- **Reboot timestamps.** HEARTBEAT and STATUSTEXT carry no boot time, so they take the
  last time seen. Right after a reboot, the new boot's first messages tied with the old
  boot's last ones, and 9d counted the pre-reboot gated entry as a post-reboot one. The
  runner now steps the clock 1 ms at each reboot.

Flight tests climb in a hover to 100 m before each outbound transition (3c, 8d) and to
120 m before 7a, and abort the flight sequence below 25 m. 7a is a bailout from a hover:
staying in MANUAL until the doors closed (about 3.7 s with the nozzle wait) crashed the
RealFlight model, so the test switches back to FBWA as soon as the tilt output reads
forward and watches the doors close there. After a crash RealFlight stops simulation time
and SITL goes silent, so the runner aborts if sim time stops for 5 s; after any abort it
puts the aircraft in QLOITER. Each new SITL connection resets the RealFlight aircraft
(FlightAxis `ResetAircraft`), so there is nothing to reset by hand between runs; with no
SITL connected, the aircraft flies uncontrolled.

A GCS can watch a run without disturbing it on SITL's spare MAVLink ports, TCP 5762 or
5763. The runner uses 5760. Don't change modes or parameters from it, and keep any GCS
joystick off.

**SITL gotchas** (also in the runner): RC overrides need `source_system=255`; script
parameters can't come from a defaults file, so they're set over MAVLink; SITL's default
wind profile is near zero at ground level, so wind tests set `SIM_WIND_T 1`; and a SITL
build left over from another branch can fail on stale generated MAVLink headers, in which
case delete `build/sitl` and rebuild.

## Review findings (2026-11-01)

### Safety-relevant

Findings 1–4 were **fixed on 2026-11-01**. Each fix is described under its finding.

1. **A pending VTOL entry overrides a later mode change.** Once an inbound entry is
   pending, `F35B.lua` calls `set_mode(target)` after `DOOR_ENTRY_MS` regardless of what
   happened in the meantime. If the pilot flips the emergency override to MANUAL, or a
   failsafe or GCS changes the mode during the delay, the script switches back into VTOL
   anyway. This breaks the documented "MANUAL is a hard lockout" and "external changes
   are respected" guarantees. *Reproduced by harness scenarios 2 and 3.*
   **Fixed:** the mode at the start of the entry is recorded (`pending_from_mode`), and any
   change cancels the entry with "mode changed, VTOL entry cancelled". The pilot re-selects
   to retry. Harness scenarios 2–3; SITL tests 7f, 8e (both fail on the unfixed script).

2. **The selector is acted on before RC is valid.** `rc:get_pwm()` returns the raw
   channel value and doesn't check for a valid signal. Before the receiver connects it
   reads 0 µs, which is position 1. With the test parameters, the script commands
   QSTABILIZE on its own at boot. *Reproduced by harness scenario 4.* The same gap lets a
   receiver's preset failsafe outputs move the selector during an RC failsafe.
   **Fixed:** the selector is ignored while `rc:has_valid_input()` is false. In ArduPlane
   that covers RC failsafe, including throttle-detected failsafe, so preset receiver values
   are ignored too. Harness scenario 4; SITL test 10f. Confirmed in SITL on 2026-11-02:
   booting into FBWA without RC, the unfixed script entered QSTABILIZE on its own. With
   `INITIAL_MODE 0` (MANUAL), as in the F-35B parameter sets, the MANUAL lockout masks the
   boot case, so 10f boots into FBWA to expose it.

3. **`DOOR_TILT_FN = 0` is not "unassigned".** Function 0 is held by every unused servo
   output, so `SRV_Channels:get_output_pwm(0)` returns a spare channel's PWM, not nil.
   With forward at the low PWM end (`DOOR_TILT_FWD < 1500`), a spare channel sitting at
   or below `DOOR_TILT_FWD` reads as "nacelles forward", and the doors could close.
   Found from `SRV_Channels::find_channel` / `update_aux_servo_function`.
   **Fixed:** `DOOR_TILT_FN <= 0` is treated as no reading (doors never close), matching
   `DOOR_LIFT_FN`. Harness scenario 5; SITL test 10d. Not reproducible in SITL, where
   unused outputs read 0; on hardware it depends on what the spare output carries.

4. **No error protection.** A runtime error stops the script, and the doors hold their
   last PWM. If they're closed and VTOL is later entered reactively (Q_ASSIST, failsafe
   QLAND/QRTL), the lift motors spin against closed doors.
   **Fixed:** `update()` now runs inside `protected_wrapper()` (the `pcall` pattern from
   upstream applets such as `RockBlock.lua`). An error is reported as "F35B: Internal
   Error: …" and the loop retries every 1 s. If VTOL is in play (`in_vtol_mode` or
   `in_assisted_flight`), the doors are driven open, so a persistent error can't hold them
   closed. Harness scenario 6.

5. **RTL/AUTO auto-transition opens the doors reactively.** Already documented in
   `F35B.md` as unresolved; still open. On this airframe it includes long RC failsafe:
   `FS_LONG_ACTN 1` is RTL, and `Q_RTL_MODE 1` turns that into QRTL near home.

Findings 6 and 7 came from a review on 2026-11-02 of how the doors could close on a deployed
nozzle. Both were **fixed the same day**. The doors clear the fan blades, so a lift fan coasting
down behind a closed door is not a concern; the nozzle is.

6. **The doors closed on the output commands, with no allowance for the nozzle.** The close
   test reads the tilt and lift *outputs*, and the doors closed on the first cycle all three
   conditions held. The tilt output is always rate-limited (`Tiltrotor::tilt_max_change`):
   `Q_TILT_RATE_DN` (30°/s) normally, but at least 90°/s in MANUAL and in any armed
   fixed-wing mode without assist, which includes the end of every outbound transition.
   ACRO is not special. The 1000–2000 µs output covers 90° of tilt, so 30°/s crosses it in
   about 3 s, and 90°/s outruns a nozzle that moves at about 30°/s.
   **Fixed:** the script estimates the physical nozzle position, following the tilt output
   at most the output's full range per `DOOR_NOZ_MS` (default 3500), and closes only once
   the estimate reads forward. A normal slew adds at most about 0.5 s; a fast swing waits for
   the nozzle. At script start on the ground the estimate starts fully deployed (a full
   stroke's wait after boot); armed (a restart in flight) it starts at the output, so the
   doors don't open at cruise. Published as `NOZ_EST`. Harness scenarios 7–10; SITL 7a
   checks the wait in MANUAL, and the whole-run monitor runs its own copy of the model on
   the sampled outputs and flags any close before it reads forward.
   (First fixed on 2026-11-02 as a fixed `DOOR_CLOSE_MS` hold-off after every close;
   replaced on 2026-11-03 after the failure-mode review below.)
7. **A stopped script froze the doors.** `SRV_Channels:set_output_pwm` sets a raw PWM that
   is never recalculated, so if scripting stopped (out of memory, crash, `SCR_ENABLE 0` at
   runtime) the doors held their last command, closed in fixed-wing, and any later VTOL entry
   met closed doors. Confirmed in SITL: after `MAV_CMD_SCRIPTING` stop the output held 1000.
   **Fixed:** the script drives the door with `set_output_pwm_chan_timeout` (lapses after
   0.5 s) and first calls `set_output_scaled(func, 0)`, without which the lapse holds the raw
   value instead of returning to trim (every channel starts with a raw PWM of 0). With
   `SERVO13_TRIM` at MID (`DOOR_PART`), a stopped, erroring or disabled script leaves the
   lift fan door at its intermediate position and the other doors open. The error path
   holds the doors open in VTOL for longer than its 1 s retry, so they don't drop to MID
   between retries. Confirmed in SITL (10g), harness scenarios 6 and 11. **Not covered:**
   before the script first runs after boot, the door output sends no pulses at all; the
   sequencer holds the nozzle doors open until it sees a valid signal.

### Failure-mode review (2026-11-03)

The owner reviewed 27 failure modes across the script and the door sequencer firmware
(groups A script/FC, B VTOL entry paths, C signal wire, D sequencer, E hardware and
sensors). Decisions:

- **Script stops in flight (A2–A6):** fail to MID, not fully open. Opening the lift fan
  door fully at cruise speed is its own hazard. Done: `SERVO13_TRIM` = `DOOR_PART`.
- **Nozzle lag (B4):** track the nozzle rather than a fixed hold-off. Done: finding 6.
- **Signal wire lost in flight (C1):** the sequencer goes to MID and holds it until the
  signal returns; boot behavior unchanged. Done in the sequencer firmware (separate repo).
- **No airspeed limit** for opening the lift fan door (B1, B2): accepted as is.
- **No door-state feedback** from the sequencer to the flight controller (C1, D2, E1):
  accepted; these stay undetected.
- Accepted or not a concern: A1, A7–A10, B1–B3, B5, B6, C2, C3, D1–D4, E1–E4.

### Minor

- `vehicle:set_mode()` return values are unchecked, so an invalid `DOOR_MODEx` fails
  silently.
- On the first inbound entry after boot, the doors are commanded full for about 0.75 s
  before partial, because the partial/full debounce starts from "full".
- After a non-script mode change within VTOL (e.g. a failsafe QLAND from QLOITER), the
  script still sends "VTOL entered externally - opening doors", although the doors are
  already open.

### Documentation drift

- `F35B_TESTPLAN.md` tables quote old debug strings: 3a "holding fixed-wing", 3c
  "fixed-wing target … immediately", 4d "session ended - full latch reset", 6a "VTOL->VTOL
  … immediately", 10e "F35B[dbg]:". The script prints "inbound M: opening doors X.Xs",
  "FW M now", "full latch reset", "VTOL->VTOL M now", with the prefix "F35B ". Section 0
  of the `.md` already lists the current strings.
- `F35B_TESTPLAN.html` predates the `.md`: its section 0 still has the old strings, and it
  lacks tests 7f, 8e and 10f. Regenerate it from the `.md` or delete it.
- The test plan has no place to record results. Add a results column, or a dated log
  here, when it's run.

## Related

- `f35b-motor-mixer.lua` is a separate Lua replacement for the C++ `AP_MotorsF35B` mixer,
  working in SITL/RealFlight as of 2026-03-17. It isn't committed to this repo.

## Next steps

1. Run `F35B_offline/run.sh` and `F35B_sitl/run_sitl_tests.py` after every script change,
   and commit the updated `last_results.md`. Run `--realflight` too when RealFlight is
   available, and commit `last_results_realflight.md`.
2. Decide the RTL/AUTO auto-transition mitigation (finding 5) before real flight. At
   minimum, choose failsafe actions and mission legs that don't enter VTOL unplanned.
3. On the hardware: set `SERVO13_TRIM` to `DOOR_PART` (MID), measure the nozzle's time
   across the tilt output's full range and set `DOOR_NOZ_MS` from it (check against
   `NOZ_EST`), flash the sequencer firmware with the signal-loss change, then bench-test
   with the fan blades off.
   Run the test plan and the failsafe cases while watching the real door and nozzle,
   especially 7a (MANUAL from hover).
4. Fix the documentation drift and regenerate or delete the HTML test plan.
5. Consider the minor findings, especially checking `set_mode()` return values.
