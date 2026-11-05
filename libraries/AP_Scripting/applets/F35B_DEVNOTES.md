# F35B.lua — Development Notes

Working notes for continuing development of `F35B.lua`. `F35B.md` is the user-facing
setup and behavior reference; this file records project status, review findings and open
work, so the context survives between sessions.

## Status (2026-11-03)

- **Preliminary.** Only the DOORS subsystem exists. The automated SITL tests cover most of
  `F35B_TESTPLAN.md`; the manual items have no recorded results.
- **Not flight-tested.** Review findings 1–4 were fixed on 2026-11-01, findings 6–7 on
  2026-11-02, and revised on 2026-11-03 after the failure-mode review. On 2026-11-03 the
  automated SITL suite passed 43/43 on the stand-in tiltrotor model
  (`F35B_sitl/last_results.md`) and 40 with 3 skipped on the F-35B in RealFlight
  (`F35B_sitl/last_results_realflight.md`). Settle finding 5, do the hardware steps under
  Next steps, and bench-test before flying it.
- **Hardware bring-up in progress** (see "Hardware bring-up" below, and the log under Next
  steps). Stage 1 (sequencer) passed on 2026-11-03, and the sequencer's power-up hold
  (firmware `54b6d26`) on 2026-11-05 with a 30 s hold; stage 2 is next.
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
| `F35B_sitl/` | Automated SITL test runner, its parameters, `last_results.md` and `last_results_realflight.md` |

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
- `DOOR_PART_AS 5`, `DOOR_FULL_AS 15`, `DOOR_ENTRY_MS 3000`, `DOOR_NOZ_MS 3500`,
  `SERVO13_TRIM 1500` (MID), `ARSPD_TYPE 2`.

This file is a dump from the owner's RealFlight SITL setup (it carries `SIM_` parameters).
It is **not** the aircraft's configuration and must not be loaded onto hardware; see
"Hardware bring-up".

## Offline tests

```bash
libraries/AP_Scripting/applets/F35B_offline/run.sh
```

`run.sh` builds ArduPilot's bundled Lua (`libraries/AP_Scripting/lua/src`) in a temporary
directory, syntax-checks `F35B.lua` with it, then runs `harness.lua`. The harness stubs the
bindings and drives the script's scheduler through scenarios using the test-plan
parameters. It exits non-zero if any scenario fails.

**Expected: 11/11 pass.** Scenarios 2–6 cover findings 1–4; each failed against the unfixed
script (commit `fe69b53045`). Scenarios 7–10 cover the nozzle estimate (finding 6) and 11
the fail-to-MID lapse (finding 7). Add a scenario for
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
are never closed while the nacelles are not forward or the lift motor is powered, never
close before its own copy of the nozzle estimate reads forward, and the VTOL hardware never
deploys while an entry is pending. Airspeed thresholds (4a, 4b) are
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

## Hardware bring-up (handoff, 2026-11-03)

Everything so far is simulation. Nothing physical has been tested: door travel times, the
nozzle's real speed, servo directions and endpoints, the sequencer's real timing, or the
tilt and lift outputs on the aircraft's own parameters.

### The system

- **Flight controller:** ArduPlane from this branch (`F35B-4.6`, `Q_FRAME_CLASS 35`, custom
  `AP_MotorsF35B`). Board targets: `CubeOrange-F35B`, `CubeOrangePlus-F35B`,
  `MatekH743-F35B`, `MatekH743-F35B-Mini`, `TBS_H7_WING-F35B-70`, `TBS_H7_WING-F35B-90`.
  `F35B.lua` runs on it and drives one PWM output (`DOOR_FUNC 94`, Scripting1).
- **Door sequencer:** an Arduino Nano (separate repo, `LoftedAero/F35B_Sequencer`;
  firmware version 1 with the power-up hold is commit `54b6d26`, configured with the F-35B
  config app from `LoftedAero/F35B_Sequencer_Config` commit `66670bd` or later). It reads that one PWM signal as three states and
  drives up to nine door servos (lift fan door, lift exhaust, aux inlet, nozzle doors):
  - LOW (`DOOR_CLOSED` 1000): all doors closed.
  - MID (`DOOR_PART` 1500): lift fan door at its intermediate position, all others open.
  - HIGH (`DOOR_OPEN` 2000): all doors open.
  - Matching is ±100 µs around its calibrated `in_low`/`in_mid`/`in_high`; between bands
    it holds its state. It ramps doors at its configured open/close speeds.
  - **Every door holds MID** for `Boot Hold` after power-up (default 20 s, set in the
    config app as "Boot Hold") and whenever the signal isn't trustworthy: before the
    first valid signal, and after losing it (about 0.5 s) until it returns. The nozzle
    homes slowly at power-up and powers up with the sequencer, so the hold is timed from
    the sequencer's own power-up. Measured on the first aircraft: **18 s** from the 105°
    reverse limit, with the Cube booting from the same battery; Boot Hold set to **30 s**. The switch is still read during
    the hold, so the doors go straight to the commanded state when it ends.
  - Configured over USB by the F-35B Door Sequencer config app. In config mode it ignores
    the flight controller (bench only).
- **The script's safety behavior on hardware:** VTOL entries from the selector wait
  `DOOR_ENTRY_MS` with the doors opening; the doors close only when fixed-wing, tilt output
  forward, lift output off, and the nozzle estimate forward; a stopped or erroring script
  lapses the door output to its `SERVOx_TRIM` (set it to MID) within about 0.6 s.
  Entries the script doesn't command (GCS, failsafe, RTL→QRTL, AUTO) open the doors only
  after the fact (finding 5).

### Differences from the SITL setup

- The board `defaults.parm` files don't enable scripting (`SCR_ENABLE`) or configure a door
  output. On the Cube targets `SERVO13` is the rudder (function 26, reversed), so the door
  output is on whichever channel is wired to the sequencer. Set `SERVOx_FUNCTION 94` there,
  `DOOR_FUNC 94`, and that channel's `SERVOx_TRIM` to `DOOR_PART`.
- `Q_TILT_ANGLE_MAX` is 110 in the board defaults (105 in the SITL set and on the first
  aircraft), so the tilt output's forward and hover readings differ between setups.
  Recalibrate `DOOR_TILT_FWD` from `TILT_POS`. The tilt output spans `Q_TILT_ANGLE_MAX`
  degrees, not 90: forward is one end of the range and the nozzle points straight down
  (hover) at (`Q_TILT_ANGLE_MAX` − 90) / `Q_TILT_ANGLE_MAX` of the range from the other end
  (14% at 105, 18% at 110); the rest is reverse tilt.
- Forward may be either end of the tilt output's PWM range: the SITL set reverses it
  (forward 1010), the board defaults don't (forward ≈ 1900).
- The board defaults put VTOL modes on `FLTMODE_CH` (QSTABILIZE on position 1, QLOITER on
  4), which bypasses the door gate, and `RC8_OPTION 153` (arm/disarm), where SITL had the
  selector. Move the selector and make `FLTMODE_CH` a MANUAL/FBWA override first.
- `ARSPD_TYPE 0` on the boards: no airspeed sensor, so `DOOR_AS` (and the partial/full
  thresholds) run on the synthetic estimate.
- `FS_LONG_ACTN 1` and `Q_RTL_MODE 1` on the boards: a long RC failsafe ends in QRTL, a
  reactive VTOL entry (finding 5). Decide before flight.
- `INITIAL_MODE 0` (MANUAL) in the owner's setup: after boot the selector is locked out
  until it is moved.

### Settings that must match (nothing enforces them)

1. `SERVOx_TRIM` of the door output = `DOOR_PART`, and inside the sequencer's MID band.
2. The sequencer's `in_low`/`in_mid`/`in_high` within ±100 µs of `DOOR_CLOSED`/`DOOR_PART`/
   `DOOR_OPEN`. The script's commands are raw PWM, so `SERVOx_MIN`/`MAX`/`REVERSED` of the
   door output don't change them; check what the sequencer actually reads (its `Input:`
   line), which on the first aircraft is 15–35 µs below the commanded value.
3. `DOOR_ENTRY_MS` ≥ the **measured physical** time for every door to get from closed to
   MID (all but the lift fan door fully open), plus margin. The sequencer's speeds are
   commands: a servo or actuator slower than its ramp lags it (the first aircraft's lift
   fan door does), and on a 50 Hz input the loop slips frames.
4. `DOOR_NOZ_MS` ≥ the **measured** time for the physical nozzle to cross the tilt output's
   full range (`SERVOx_MIN` to `SERVOx_MAX` of the `DOOR_TILT_FN` output), plus margin. A
   switch to MANUAL from a hover only sweeps part of the range (86% at `Q_TILT_ANGLE_MAX`
   105), so scale the measured time: full range ÷ the `TILT_POS` span swept.
5. `DOOR_TILT_FWD` from `TILT_POS` with the nozzle fully forward, nudged about 10 µs toward
   mid-travel. `DOOR_LIFT_FN` = the lift fan motor's output function; `DOOR_LIFT_OFF`
   between `LIFT_PWM` stopped and idling.

### Install

- Build and flash this branch for the board, copy `F35B.lua` to `APM/scripts`, set
  `SCR_ENABLE 1`, reboot, check for "F35B: script loaded", then set the `DOOR_` parameters.
- **Reboot after any update that adds a `DOOR_` parameter** (`DOOR_NOZ_MS` was added on
  2026-11-02): the parameter table can't grow on a scripting restart.
- Flash the sequencer firmware (`firmware_1.hex`, bundled with the config app; or
  `arduino-cli compile/upload --fqbn arduino:avr:nano:cpu=atmega328`). It is still version 1
  (never released, so no EEPROM migration). On a board that ran an earlier pre-release
  build, check "Boot Hold" (Doors tab, next to the door speeds) in the config app after flashing: that setting occupies the
  bytes where the old build kept its last switch state. Anything above 60 s reads as the
  20 s default; a smaller stray value would not, so set it explicitly. Opening the config
  app's serial connection resets the board, which restarts the hold. A signal loss does not
  restart the hold: after the signal returns the doors follow the switch at once.

### Suggested order

Lift fan blades off (or the fan otherwise unable to spin) for every bench stage. The doors
and nozzle move at power-up and on mode changes even when disarmed: keep fingers clear of
the linkages. Watch with Mission Planner: `DOOR_ST`, `DOOR_PWM`, `VTOL_DLY`, `TILT_POS`,
`LIFT_PWM`, `NOZ_EST`, `DOOR_AS` on the Quick tab, `DOOR_DEBUG 1` on the bench (0 for flight).

1. **Sequencer alone** (servo tester on its input): every door at MID from power-up for
   the whole `Boot Hold`, then the commanded state; each state's endpoints with no
   binding; directions; the measured closed→MID time. Signal-loss: after a valid signal,
   remove it and check MID within about 0.5 s. Then, with the nozzle connected: power
   the whole system up the way the aircraft does (no USB on the sequencer or the flight
   controller) with the nozzle at its reverse limit, the worst case, and check it is
   fully home well before `Boot Hold` ends. Measure the homing with the doors commanded MID
   first, so they can't close on it. The first aircraft took 18 s, more than the 10–15 s
   first estimated; the 20 s default would have left 2 s.
2. **Flight controller + sequencer, disarmed.** Set the door output and the matching
   settings above. Disarmed, a VTOL mode tilts the nozzle down at `Q_TILT_RATE_DN` and
   MANUAL tilts at 90°/s or more, so the nozzle speed can be measured without arming: from
   QLOITER switch the override to MANUAL and time the physical nozzle (video), then set
   `DOOR_NOZ_MS`. Calibrate `DOOR_TILT_FWD` and `DOOR_LIFT_OFF`.
3. **Ground tests from `F35B_TESTPLAN.md`**, disarmed, watching the real door and nozzle:
   gated entry (doors physically at MID before the mode changes), 9a/9b, 6b/6c, 7b–7e,
   10g (scripting stopped → MID; or `SCR_ENABLE 0` + reboot for the no-script boot),
   signal wire unplugged → MID, and above all MANUAL from a VTOL mode: the doors must stay
   open until the nozzle is physically home.
4. **Powered, restrained,** only after 1–3 are clean.
5. **Flight:** hover first; transitions with height in hand; MANUAL from a hover is a
   bailout only (it crashed the RealFlight model when held). Settle finding 5 first.

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
   sequencer holds every door at MID until it sees a valid signal and its power-up hold
   has run.

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
3. Hardware bring-up: follow "Hardware bring-up" above, and record what is measured and
   set here (log below). Stage 1 passed, and the sequencer's power-up hold firmware was
   checked on 2026-11-05; stage 2 (FC + sequencer, disarmed) is next.
4. Fix the documentation drift and regenerate or delete the HTML test plan.
5. Consider the minor findings, especially checking `set_mode()` return values.

### Bring-up log: first aircraft

**Aircraft (as found, 2026-11-03).** Cube Orange+ (`CubeOrangePlus-F35B`), ArduPlane 4.6.3
from this branch at `03037e3a` (later commits change only other boards' defaults).
`SCR_ENABLE 0`, so no `DOOR_` parameters yet. `Q_TILT_ANGLE_MAX 105` (kept; the board
default is 110), `ARSPD_TYPE 0`, `BRD_SAFETY_DEFLT 0`, `Q_ASSIST_SPEED -1` (assist only in
transitions), no DShot (`SERVO_BLH_MASK 0`).

| | Output | Settings |
|---|---|---|
| Door signal to the sequencer | `SERVO12` (AUX4) | function 94, trim 1500 (= `DOOR_PART`), 1100–1900, not reversed. AUX4 shares TIM1 with motors 1/2 (AUX1/2), so it runs at `Q_RC_SPEED` 490 Hz. |
| Nozzle tilt | `SERVO16`, function 41 | 1100–1900, not reversed: forward is **1900**. The Cube has no pin 15/16; the nozzle controller gets them over SBUS out (`SERIAL6_PROTOCOL 22`, 50 Hz). |
| Lift fan | `SERVO1`, function 35 (motor 3) | 1000–2000. So `DOOR_LIFT_FN 35`. |

Still to set up in stage 2: `FLTMODE_CH 5` carries QSTABILIZE (1) and QLOITER (4), and
`RC8_OPTION 153` is arm/disarm; the selector needs its own channel and `FLTMODE_CH` a
MANUAL/FBWA override.

**Sequencer** (firmware v1 from `ca95edf` for stage 1; from `54b6d26`, with the power-up
hold, since 2026-11-05; EEPROM v1).

| Out | Door | Closed | Open | Drive |
|---|---|---|---|---|
| 1 | Lift fan door | 920 | 2070 | linear actuator |
| 2, 3 | Lift exhaust | 1900, 1115 | 1075, 1950 | servo |
| 4, 5 | Aux inlet | 1115, 1855 | 1965, 1030 | servo |
| 6, 7 | Nozzle doors | 1060, 1985 | 1960, 1085 | servo |

Open/close speed 1500 ms; lift fan door open/close **4500 ms** (was 2000, changed
2026-11-03, see below); Fan Mid 65%; input calibration 1000 / 1500 / 2000 (= `DOOR_CLOSED` /
`DOOR_PART` / `DOOR_OPEN`); **Boot Hold 30000 ms** (set 2026-11-05). `safe_start` is stored
but never read by the firmware.

**Stage 1, sequencer alone (passed 2026-11-03).** Driven from the flight controller rather
than a servo tester: `SERVO12_FUNCTION 135` (Trim) outputs `SERVO12_TRIM`, refreshed at
1 Hz, so the trim value commands the doors without the script (see finding 9). The
sequencer's serial status was logged throughout. A dry run with the servo rail unpowered
came first, ending at HIGH so that powering the rail moved nothing.

- Input: the sequencer reads the 490 Hz signal 15–35 µs low with about 20 µs jitter
  (2000 → 1946–1986, 1500 → 1472–1492, 1000 → 971–999); worst case 46 µs inside a band.
- Commanded ramps run at exactly 20 ms per step on the 490 Hz input (no frame slips).
  Closed→MID 1.46–1.62 s for the general doors.
- **The lift fan door's linear actuator is slower than its ramp:** physically 4.3 s per
  full stroke either way, and **2.8 s closed→MID** (65%), against 1.3 s commanded at the
  old 2000 ms speed. The other doors track their ramps. Its speeds were raised to 4500 ms
  so the ramp governs it (closed→MID now 2.9–3.1 s commanded).
- Endpoints, directions and the output map checked on every door; no binding.
- Between bands (1250, 1750) the state holds.
- Signal loss (lead unplugged): MID 0.5–0.8 s after the last valid pulse (25 frames × the
  25 ms `pulseIn` timeout ≈ 0.63 s, not 0.5 s); `Input: 0` held steady with the servos
  powered (no floating-pin noise). Recovers on reconnect.
- Boot pose (reset with no signal, last state LOW): nozzle doors snap open, the others
  snap to the saved state; on signal the nozzle doors ramp closed. No twitch at attach.
  (Behavior of `ca95edf`; superseded by the power-up hold below.)

**Power-up hold, firmware `54b6d26` (checked 2026-11-05).** Same Trim method for the door
command; logged over USB except where the power-up itself was the test.

- N1, after flashing: every setting kept; Boot Hold read **56320 ms**, a stray value: its
  low byte was the old build's last state (0), its high byte 0xDC was left in EEPROM by
  an earlier build. Under 60000, so used as-is. Set explicitly.
- N2, nozzle homing: whole system powered from the battery (no USB on the sequencer or
  the Cube), nozzle at its 105° reverse limit, doors commanded MID: **18 s** to fully
  home, at a steady speed, including the Cube's boot. Boot Hold set to **30 s** (+67%).
- N3, the same power-up with the doors commanded LOW: every door at MID from power-up,
  nozzle home at 18 s, doors closing just after 30 s: **12 s of margin**. On a power-up
  the hold runs from power-on; after a USB reset the bootloader adds about 1.5 s.
- N4, power-up with no signal: MID 47 s past the reset; on connecting, the doors were
  closing within 170 ms.
- N5, signal loss inside the hold: no motion, and the doors still closed 30.0 s after
  start (the hold is not restarted). After the hold: MID 0.5 s after the loss, closing
  within 160 ms of the return.
- N6, Nano reset with the Cube running: MID for the full hold although LOW arrived 3 s
  in; closed at 30.1 s.
- N7, Boot Hold 0: the doors follow as soon as the signal is valid. Restored to 30000 and
  confirmed after a reset.
- N8, Cube-only reboot (doors commanded MID): the door signal stops about 1.3 s after the
  reboot command and returns about 12 s after it; the doors sit at MID meanwhile. **The
  nozzle does not re-home** when its SBUS feed returns, so the script's ground nozzle wait
  (`DOOR_NOZ_MS`) is enough after a Cube-only reboot.
- Status dump captured for the config app's `test/board-capture.txt`.

**Values so far.**

| Parameter | Value | Source |
|---|---|---|
| Door output | `SERVO12`, trim 1500 | as found; matches rule 1 |
| Sequencer input calibration | 1000 / 1500 / 2000 | matches rule 2 (reads 15–35 µs low) |
| Sequencer Boot Hold | **30000 ms** | nozzle homing 18 s worst case (N2), +67% |
| `DOOR_ENTRY_MS` | **4000** (to set in stage 2) | lift fan door 2.8 s closed→MID unloaded, +40% for air load |
| `DOOR_LIFT_FN` | 35 | `SERVO1` |
| `DOOR_TILT_FN` | 41 | `SERVO16` |
| `DOOR_TILT_FWD` | stage 2 | expect ≈ 1890 (forward is the high end) |
| `DOOR_LIFT_OFF` | stage 2 | |
| `DOOR_NOZ_MS` | stage 2 | scale the MANUAL-from-hover sweep (86% of the range at 105) |

**Findings from bring-up.**

8. **Finding 5 is wide open on this aircraft.** With `FS_LONG_ACTN 1`, `Q_RTL_MODE 1` and
   `RTL_RADIUS 1000`, a long RC failsafe (3 s) from fixed-wing enters RTL, which switches
   to QRTL immediately anywhere within 1 km of home: an ungated VTOL entry at cruise speed
   with the doors closed. The short failsafe (FBWA, or QLAND from a VTOL mode) and the
   other failsafes (battery, fence, GCS: all off) are not a concern. Options: (a)
   `Q_RTL_MODE 0`; (b) `FS_LONG_ACTN 2` (glide); (c) a script change that opens the doors
   to MID while armed with RC invalid, about 1.5–2.5 s before the QRTL switch, which is
   less than the lift fan door's 2.8–2.9 s to MID. **Decide before stage 5.**
9. **Bench method: `SERVOx_FUNCTION 135` latches.** Switching the door output back to 94 at
   runtime leaves the last trim value on the wire (raw PWM), so the doors hold that state
   with no script. It clears at a reboot or when the script starts (`set_output_scaled`
   first). Reboot after using it, and check the output reads 0 before trusting it.
10. **Sequencer reset during a signal loss snaps the doors.** With `ca95edf` the boot pose
    wrote the saved state with no ramp, so a reset (e.g. a brownout) while the doors sat at
    MID after a lost signal moved them at full speed. **Resolved by `54b6d26`:** the boot
    pose is MID. What remains: a reset with the doors closed snaps them to MID at full
    speed and holds MID for the whole Boot Hold (30 s), even in a hover where the FC
    commands fully open. The owner confirmed MID (lift fan door 65%) is enough to hover.
11. **Selector order.** The script acts on any selector position held for one 50 ms cycle.
    If the selector passes through intermediate positions (rotary knob, mixed switches),
    don't put MANUAL or other fixed-wing positions between VTOL positions, or a VTOL→VTOL
    move from a hover sets MANUAL on the way.
12. **Minor sequencer notes:** each serial command stalls the loop for about 1 s
    (`Serial.readString` timeout), so configure on the bench only; `move_toward` truncates
    fractional steps, so opening runs up to 4% slower than the configured speed and
    closing slightly faster. At small steps the effect grows: the lift fan door at 4500 ms
    steps 5.1 µs, which opens at 5 per step (about 4.6 s per stroke) but closes at 6
    (about 3.8 s), so its closing ramp again runs ahead of the 4.3 s actuator. Closing
    speed affects no safety logic.
13. **Power-up hold, the measured worst case.** The nozzle's homing (18 s) was longer than
    the 10–15 s first estimated and close to the firmware's 20 s default. Measure it on
    each airframe, from the reverse limit and with the flight controller booting from the
    same battery, before trusting the default.
14. **Flashing over a used board needs Boot Hold checked.** Our board read 56320 ms after
    flashing `54b6d26` (see N1). A stray value close to zero would mean no hold at all.