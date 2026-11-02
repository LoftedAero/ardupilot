# F35B.lua — Development Notes

Working notes for continuing development of `F35B.lua`. `F35B.md` is the user-facing
setup and behavior reference; this file records project status, review findings and open
work, so the context survives between sessions.

## Status (2026-11-01)

- **Preliminary.** Only the DOORS subsystem exists. It has been exercised in SITL, but no
  test results were recorded, so which `F35B_TESTPLAN.md` items pass is unknown.
- **Not flight-tested.** Fix the safety findings below and work through the test plan
  before flying it.
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

**Expected today: 1/4 pass.** The three failures are findings 1 and 2 below. After those
fixes it should pass 4/4. Add a scenario for each new fix or behavior.

ArduPilot's Lua strips `print`, `loadfile`, `dofile`, `select` and `os`, so the harness
uses `io.write`, `io.open` + `load`, and `error()` for the exit status.

## Review findings (2026-11-01)

### Safety-relevant — fix before flight

1. **A pending VTOL entry overrides a later mode change.** Once an inbound entry is
   pending, `F35B.lua` calls `set_mode(target)` after `DOOR_ENTRY_MS` regardless of what
   happened in the meantime. If the pilot flips the emergency override to MANUAL, or a
   failsafe or GCS changes the mode during the delay, the script switches back into VTOL
   anyway. This breaks the documented "MANUAL is a hard lockout" and "external changes
   are respected" guarantees. *Reproduced by harness scenarios 2 and 3.*
   Fix: remember the mode when the entry starts, and cancel the pending entry if the
   current mode differs (or the mode reason is no longer `SCRIPTING`).

2. **The selector is acted on before RC is valid.** `rc:get_pwm()` returns the raw
   channel value and doesn't check for a valid signal. Before the receiver connects it
   reads 0 µs, which is position 1. With the test parameters, the script commands
   QSTABILIZE on its own at boot. *Reproduced by harness scenario 4.* The same gap lets a
   receiver's preset failsafe outputs move the selector during an RC failsafe.
   Fix: skip selector handling while `rc:has_valid_input()` is false.

3. **`DOOR_TILT_FN = 0` is not "unassigned".** Function 0 is held by every unused servo
   output, so `SRV_Channels:get_output_pwm(0)` returns a spare channel's PWM, not nil.
   With forward at the low PWM end (`DOOR_TILT_FWD < 1500`), a spare channel sitting at
   or below `DOOR_TILT_FWD` reads as "nacelles forward", and the doors could close.
   Found from `SRV_Channels::find_channel` / `update_aux_servo_function`; not covered by the
   harness yet. Fix: treat `DOOR_TILT_FN <= 0` as no reading, as `DOOR_LIFT_FN` already
   is.

4. **No error protection.** A runtime error stops the script, and the doors hold their
   last PWM. If they're closed and VTOL is later entered reactively (Q_ASSIST, failsafe
   QLAND/QRTL), the lift motors spin against closed doors. Fix: wrap `update()` in
   `pcall`, as upstream applets do, so an error is reported and the loop keeps running.

5. **RTL/AUTO auto-transition opens the doors reactively.** Already documented in
   `F35B.md` as unresolved; still open.

### Minor

- `vehicle:set_mode()` return values are unchecked, so an invalid `DOOR_MODEx` fails
  silently.
- On the first inbound entry after boot, the doors are commanded full for about 0.75 s
  before partial, because the partial/full debounce starts from "full".
- After a non-script mode change within VTOL (e.g. a failsafe QLAND from QLOITER), the
  script still sends "VTOL entered externally - opening doors", although the doors are
  already open.
- Stale comment at the `door_tilt_pos` declaration: it says 0..1000, but the value is now
  the tilt PWM.

### Documentation drift

- `F35B_TESTPLAN.md` tables quote old debug strings: 3a "holding fixed-wing", 3c
  "fixed-wing target … immediately", 4d "session ended - full latch reset", 6a "VTOL->VTOL
  … immediately", 10e "F35B[dbg]:". The script prints "inbound M: opening doors X.Xs",
  "FW M now", "full latch reset", "VTOL->VTOL M now", with the prefix "F35B ". Section 0
  of the `.md` already lists the current strings.
- `F35B_TESTPLAN.html` predates the `.md` (its section 0 still has the old strings).
  Regenerate it from the `.md` or delete it.
- Test 10d expects `TILT_POS = 0` with `DOOR_TILT_FN = 0`; see finding 3.
- `F35B.md` behavior row 18 and test 10b say RC loss makes `rc:get_pwm` return nil. It
  doesn't; see finding 2.
- `F35B.md` Notes say the script "assumes higher scaled tilt = more forward". The script
  auto-detects direction from `DOOR_TILT_FWD`.
- The test plan has no place to record results. Add a results column, or a dated log
  here, when it's run.

## Related

- `f35b-motor-mixer.lua` is a separate Lua replacement for the C++ `AP_MotorsF35B` mixer,
  working in SITL/RealFlight as of 2026-03-17. It isn't committed to this repo.

## Next steps

1. Fix findings 1–4; `run.sh` should then pass 4/4. Add a harness scenario for finding 3.
2. Fix the documentation drift and regenerate or delete the HTML test plan.
3. Run the SITL test plan with `F35B_TESTPLAN.param` and record the results.
4. Decide the RTL/AUTO auto-transition mitigation (finding 5) before real flight.
