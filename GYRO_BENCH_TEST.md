# Gyro Build: Bench Test Checklist

Work through this with **propellers removed** before any flight. Each
step says what to do and what you should see. Stop at the first step
that does not match and note what happened; most later steps depend on
the earlier ones.

Keep a ground station (Mission Planner or QGroundControl) connected
over USB throughout and watch its Messages tab. Several checks rely on
the text messages the firmware sends.

How the firmware is meant to behave is described in
[GYRO_README.md](GYRO_README.md).

---

## 0. Preparation

**Board.** Every step applies to both the KakuteF4-Wing and the TBS
Lucid H7 Wing.

**Parameters already on the board.** Flashing keeps saved parameters.
If the board was set up before, either reset parameters to defaults
after flashing (then redo the accelerometer calibration) or check the
values in GYRO_README.md's parameter reference by hand.

**Build and flash.**

```
./waf configure --board TBS_LUCID_H7_WING-gyro    # or KakuteF4-Wing-gyro
./waf plane
```

Flash `build/<board>-gyro/bin/arduplane.apj` with the ground station's
"load custom firmware", or run `./waf plane --upload` with the board on
USB.

**Wiring.**

| | Lucid H7 Wing | KakuteF4-Wing |
|---|---|---|
| ELRS receiver | R6/T6, both wires | UART3 (PC10/PC11), both wires |
| GPS (optional, step 12 only) | R2/T2 | UART1 |
| Aileron, elevator, throttle, rudder | outputs 1, 2, 3, 4 | outputs 1, 2, 3, 4 |

Both receiver wires are needed. The flight controller talks back to
the radio over the receiver's second wire, and the setup menu depends
on that.

**Radio.** On a new model in the transmitter:

- Channels 1 to 4: sticks in AETR order, no mixes, no expo needed.
- Channel 5: a two-position switch, the ELRS AUX1 arm switch.
- Channel 6: a three-position switch.
- Channel 7: a two-position switch.
- Channel 8: a knob or slider.
- ELRS telemetry ratio not set to Off.
- Receiver failsafe left at "No pulses".

---

## 1. Boot

Power the board with the model sitting still. Gyro calibration runs at
power-up and wants no movement for a few seconds.

Expect in the Messages tab:

- [ ] `ArduPilot Ready`
- [ ] `Gyro: setup menu ready (75 entries)` once the receiver is
      linked (may be missed if the ground station connected late; step
      11 checks the menu directly)

## 2. Radio link

Open the ground station's radio page.

- [ ] Channels 1 to 8 move with the matching sticks and switches.
- [ ] Each channel spans roughly 988 to 2012.
- [ ] Channel 5 low reads about 988, high about 2012.

## 3. Disarmed state

Arm switch (channel 5) low.

- [ ] Ground station shows DISARMED.
- [ ] The ESC plays its normal power-up tones (it is receiving minimum
      throttle), but the motor does not turn with the throttle stick.
- [ ] Control surfaces still move with the sticks.

## 4. Surface directions in MANUAL

Mode switch (channel 6) low. The surfaces are driven straight from the
sticks.

- [ ] Right roll stick: right aileron up, left aileron down.
- [ ] Pull back on pitch: elevator trailing edge up.
- [ ] Right rudder: rudder trailing edge moves right.

Wrong direction: fix it in the transmitter menu under 4 Outputs, the
channel's Reversed entry (or `SERVOn_REVERSED` in the ground station),
**not** by reversing in the radio. For elevons or a V-tail, set the
wing type first (4 Outputs: Wing type, then Apply template), then fix
directions.

## 5. Stabilization direction in FBWA

Mode switch high. Hold the sticks centered and move the model by hand.
This is the check that prevents a crash on the first flight.

- [ ] Roll the model right wing down: the ailerons deflect to roll it
      back left (right aileron down, left aileron up).
- [ ] Pitch the nose down: the elevator deflects up.
- [ ] Return the model level: the surfaces return to neutral.

If MANUAL was correct but FBWA corrects the wrong way, the board
orientation is wrong. **Do not fly.** Report which axis and direction.

## 6. ACRO

Mode switch middle, sticks centered.

- [ ] Rotate the model briskly around the roll axis: the ailerons
      oppose the rotation while it is moving.
- [ ] Same for pitch with the elevator.

## 7. Gain knob

In ACRO or FBWA, repeat step 5 or 6 at three knob positions.

- [ ] Knob fully low: little or no correction.
- [ ] Knob centered: the correction seen in steps 5 and 6.
- [ ] Knob fully high: clearly stronger correction.

## 8. Arming

**Propellers off.** Mode switch low (MANUAL).

- [ ] Arm switch high: ground station shows ARMED.
- [ ] Throttle stick now drives the motor.
- [ ] Arm switch low: DISARMED, motor stops.

## 9. Trim capture

Mode switch low, sticks centered, disarmed.

1. Add several clicks of aileron trim on the transmitter so the
   ailerons visibly move.
2. Arm.
   - [ ] Message `trim complete`.
   - [ ] In the ground station, `RC1_TRIM` has moved away from its
         previous value.
3. Hold the model level and switch to FBWA, then ACRO.
   - [ ] The ailerons stay at the trimmed position; they do not jump
         back to the old, untrimmed neutral.
4. Back in MANUAL, add a few more clicks, then switch to ACRO.
   - [ ] Another `trim complete`.

To undo afterwards: zero the transmitter trims, then disarm and arm
again in MANUAL with the sticks centered; the capture moves the
neutrals back.

## 10. Link loss

**Propellers off.** Armed, MANUAL, throttle stick at about a quarter.

1. Switch the transmitter off.
   - [ ] Within about a second: mode FBWA and the motor stops.
   - [ ] After about five seconds: `RTL refused: no home position`
         (no GPS fitted), and the mode stays FBWA.
   - [ ] Still ARMED throughout.
2. Switch the transmitter back on.
   - [ ] Link returns. The mode stays FBWA; it does not jump back on
         its own.
   - [ ] Moving the mode switch changes mode again.

## 11. Transmitter menu

On the radio, open the ExpressLRS Lua script, go to Other Devices and
select Gyro. TBS Agent Lite works too. If you
reflashed while the script was open, exit it and start it again first.

- [ ] Gyro shows Status, then 1 Calibration to 6 Tuning, then Tools, and
      every folder has entries (give it about 20 seconds to load).
- [ ] Back from 4 Outputs > Channel 1 returns to that list.
- [ ] Status reads `DISARMED`, the current mode and the GPS state.
- [ ] 6 Tuning: change ACRO roll rate, then read `ACRO_ROLL_RATE` in
      the ground station: it matches. Set it back.
- [ ] 4 Outputs, Channel 1: change Trim by one step and watch the
      surface move; set it back.
- [ ] 1 Calibration, Calibrate level, disarmed, model at its level flight
      attitude: `Hold model at level attitude`, OK, `Calibrating`, then
      `Calibration done`. The radio may briefly report lost telemetry
      while it runs; that is expected.
- [ ] Same with the arm switch on: `Disarm first`.
- [ ] 2 Radio, Learn sticks: follow the prompts. The summary (for
      example `A1 E2 R4 T3 Save?`) matches your radio's channel order.
      Cancel with Exit unless something needs fixing.
- [ ] 2 Radio, Learn arm switch: flip the arm switch when asked. It
      asks you to switch it off, and only then accepts OK. The plane
      must not arm during this.
- [ ] 1 Calibration, Detect orientation: level then nose up. It reports
      the board orientation you expect for how the board is mounted.

If Gyro is not listed, check that both receiver wires are
connected and that ELRS telemetry is on.

## 12. GPS and RTL (optional)

Only with a GPS connected, outdoors, propellers off.

- [ ] Before a fix, the RTL switch (channel 7) gives
      `RTL refused: no home position`.
- [ ] After a 3D fix, the RTL switch selects RTL.
- [ ] RTL switch back low returns to the mode switch's mode.

## 13. Log

Download the latest log over USB.

- [ ] A log exists covering the armed period.
- [ ] It contains `GYRO` messages whose `Gain` follows the knob.

---

## What to report

For anything that did not match: the step number, what you saw, and
the text of any message in the Messages tab.
