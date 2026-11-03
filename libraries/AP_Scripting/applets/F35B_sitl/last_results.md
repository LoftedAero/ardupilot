# F35B.lua SITL results

- Run: 2026-11-02 22:44, 2.8 min wall time, speedup 5
- Script: `libraries/AP_Scripting/applets/F35B.lua` at `bbba5ff6b0 F35B: publish DOOR_AS; fix RealFlight test timing + uncommitted changes`
- Model: stock `quadplane-tilttri` with `F35B_sitl/tilttri.parm` (logic test, not F-35B dynamics)
- Totals: PASS 43

Calibration: tilt_fwd 2000, lift_stopped 1000, tilt_hover 1003, lift_hover 1722, lift_off 1217, tilt_fwd_param 1950

| ID | Test | Result | Detail |
|----|------|--------|--------|
| 1a | Telemetry: TILT_POS matches the tilt output | PASS | TILT_POS 2000 = tilt output, nacelles forward |
| 1d | DOOR_ST stable while idle | PASS | closed and stable for 5 s |
| 2a | Selector alone cannot change the mode | PASS | FLTMODE_CH 5 != DOOR_MODE_CH 8; selector sweep alone changed nothing |
| 9c | Disarmed FBWA -> QLOITER goes through the door gate | PASS | held FBWA, entered QLOITER after 3.1 s |
| 9a | Disarmed in a VTOL mode: doors open | PASS | doors OPEN while disarmed in QLOITER |
| 9b | Disarmed -> FBWA: doors close once nacelles forward | PASS | FBWA immediately, doors closed after 3.5 s |
| 6b | Selector moved away during a pending entry cancels it | PASS | pending entry cancelled, stayed FBWA, doors closed |
| 6c | New VTOL selection during a pending entry retargets it | PASS | retargeted to QLOITER, timer restarted (VTOL_DLY 2.7 s) |
| 6d | Fixed-wing to fixed-wing via the selector | PASS | FBWA->MANUAL->FBWA immediate, doors stayed closed |
| 7f | MANUAL override during a pending entry cancels it | PASS | entry cancelled, stayed MANUAL |
| 7b | Selector ignored in MANUAL | PASS | selector ignored in MANUAL, doors stayed closed |
| 7c | Override back to rest re-enables the selector | PASS | selector works again: QLOITER via the gate after 3.0 s |
| 7d | FBWA -> MANUAL override: doors stay closed | PASS | MANUAL, doors stayed closed |
| 7e | MANUAL override works with the script disabled | PASS | MANUAL engaged with the script disabled; door output returned to its trim (2000) |
| 8e | GCS mode change during a pending entry cancels it | PASS | entry cancelled, stayed RTL |
| 8a | GCS VTOL mode: warning, doors open reactively | PASS | warning sent, doors opened reactively |
| 8b | GCS FBWA respected with the selector on VTOL | PASS | GCS FBWA respected with the selector still on QLOITER |
| 5c | Nacelles never forward: doors never close | PASS | doors stayed open (fail-safe) |
| 10a | DOOR_MODE_CH 0: no gating, no errors | PASS | no gating, doors tracked phase, no errors |
| 10c | Rapid selector sweep | PASS | settled in the final selection (QLOITER) only |
| 10d | DOOR_TILT_FN 0: no reading, doors stay open | PASS | TILT_POS 0, doors stayed open (fail-safe) |
| 10e | DOOR_DEBUG 0 silences debug output | PASS | debug output stops with DOOR_DEBUG 0 |
| 10g | Scripting stopped: doors fail open | PASS | scripting stopped with the doors closed: output went to 2000 (trim, open) after 0.8 s and stayed; closed again after the restart |
| 4e | No door chatter at a steady threshold airspeed | PASS | 7 door change(s) in 30 s with airspeed 6.9-7.2 m/s around 7.0 m/s, at least 0.9 s apart; no change faster than the 0.75 s debounce |
| 9d | Reboot with the selector on QLOITER: gated entry | PASS | after reboot on QLOITER: VTOL only through the door gate (after an initial MANUAL lockout) |
| 10f | Boot without RC: selector ignored | PASS | booted into FBWA without RC: no selector action for 10 s |
| F0 | Arm in QLOITER and climb to 50 m | PASS | armed in QLOITER, hovering at 52 m |
| 1b | TILT_POS moves with the nacelles in hover | PASS | TILT_POS 1003 in hover vs 2000 forward |
| 1c | LIFT_PWM: lift motor running vs stopped; calibrate | PASS | LIFT_PWM 1722 hover vs 1000 stopped; set DOOR_LIFT_OFF 1217, DOOR_TILT_FWD 1950 |
| 3c | Outbound: QLOITER -> FBWA | PASS | FBWA immediately; doors closed 6.3 s later at 25.8 m/s |
| 4a | Outbound partial airspeed ~ DOOR_PART_AS | PASS | partial 0.70 s after airspeed crossed DOOR_PART_AS 7 m/s (debounce 0.75 s; 9.4 m/s by then, accelerating) |
| 5a | Close waits for nacelles forward | PASS | nacelles forward at +5.1 s, doors closed at +6.3 s |
| 5b | Close waits for the lift motor to stop | PASS | lift motor off at +5.2 s, doors closed at +6.3 s |
| F1 | Cruise: doors stay closed | PASS | doors closed through 15 s of cruise at 38.0 m/s, 120 m |
| 3a | Fast inbound: FBWA -> QLOITER through the gate | PASS | from 38.0 m/s: partial after 0.5 s, held FBWA, QLOITER after 3.1 s, full at 10.4 m/s; no VTOL motion before the mode change |
| 4b | Inbound full airspeed ~ DOOR_FULL_AS | PASS | full 0.80 s after airspeed fell through DOOR_FULL_AS 15 m/s (debounce 0.75 s; 11.8 m/s by then, decelerating) |
| 4c | Latched full: fast airflow in VTOL keeps doors full | PASS | stayed full for 10 s at up to 17.0 m/s (above DOOR_FULL_AS 15) |
| 6a | VTOL -> VTOL switch is immediate | PASS | QLOITER->QSTABILIZE->QLOITER immediate, no delay |
| 7a | MANUAL override in hover | PASS | MANUAL immediately; doors closed after 2.6 s, 1.0 s after the outputs read stowed (DOOR_CLOSE_MS 1.0 s) |
| 4d | Latch resets per VTOL session | PASS | latch reset seen; re-entry at 37.3 m/s went partial after 1.1 s (commanded full for the first 0.75 s: the debounce starts from full) |
| 3d | Full round trip | PASS | FBWA->QLOITER->hover->FBWA->cruise and back, all legs passed |
| 8d | RTL auto-transition to QRTL (documented limitation) | PASS | RTL switched to QRTL after 32 s, warning sent; doors commanded open no later than the VTOL hardware started moving (10 Hz samples), but physical door travel still lags (documented limitation); landed |
| INV | Safety monitor over the whole run | PASS | 5531 output samples; doors never closed on active VTOL hardware and nothing moved during a pending entry; 19 close(s), each at least 1.0 s after the outputs read stowed; 0 sample(s) inside the documented external-entry windows |

Not automated: 3b (the model stalls above DOOR_PART_AS, so a slow fixed-wing entry is impossible), 8c (AUTO mission), 10b (RC failsafe in flight).
