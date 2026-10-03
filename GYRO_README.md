# ArduPilot Gyro Build

A stripped-down ArduPlane that works like a consumer RC gyro or
stabilized receiver (AS3X/SAFE, FrSky S6R/S8R, Aura, HobbyEagle A3),
with ArduPilot's servo mixing, autotune, logging and GPS return-to-launch
underneath. It is set up from the transmitter and needs no ground
station for everyday use.

This document describes how the firmware behaves as flashed with its
default parameters. The design history and roadmap are in
[GYRO_PLAN.md](GYRO_PLAN.md).

Contents

1. [What is in and what is out](#1-what-is-in-and-what-is-out)
2. [Hardware and flashing](#2-hardware-and-flashing)
3. [Transmitter channel layout](#3-transmitter-channel-layout)
4. [Arming](#4-arming)
5. [Flight modes](#5-flight-modes)
6. [Trims](#6-trims)
7. [Gain knob](#7-gain-knob)
8. [Link loss and failsafe](#8-link-loss-and-failsafe)
9. [GPS and return to launch](#9-gps-and-return-to-launch)
10. [Transmitter setup menu](#10-transmitter-setup-menu)
11. [Autotune](#11-autotune)
12. [Logging](#12-logging)
13. [Ground station access](#13-ground-station-access)
14. [Parameter reference](#14-parameter-reference)
15. [Building and testing](#15-building-and-testing)
16. [Known limitations and untested items](#16-known-limitations-and-untested-items)

---

## 1. What is in and what is out

Kept from ArduPlane:

- MANUAL, ACRO (rate stabilized) and FBWA (self-leveling) flight modes
- RTL when a GPS is fitted
- RC input, servo output, trim, reversing and mixing (elevons, V-tail,
  flaperons, differential spoilers)
- Fixed-wing autotune
- Onboard logging (dataflash or SD card, depending on the board)
- MAVLink over USB and a serial port for a ground station

Removed to fit a 1 MB flight controller and to keep the pilot-facing
behavior simple:

- EKF navigation filter (the older DCM attitude estimator is used; it is
  fine for stabilization and GPS return home)
- Terrain, rally points, geofence, ADS-B, soaring
- Quadplane / VTOL
- OSD, video transmitter control, camera and gimbal control
- MSP and DisplayPort
- Lua scripting (the transmitter menu is built into the firmware)

Added on top of ArduPlane, all in one module (`ArduPlane/gyro.cpp`):

- A master gain knob on a transmitter channel
- Return to launch is refused, with a glide fallback, when no home
  position is known
- A throttle-based logging gate for setups that do not arm (off by
  default)
- A transmitter setup menu over CRSF (`ArduPlane/gyro_menu.cpp`)

---

## 2. Hardware and flashing

Two boards have gyro builds. Each uses the stock board ID and
bootloader, so the stock bootloader and the usual flashing tools work.

| Board | Build name | Flash | Logging | Notes |
|---|---|---|---|---|
| Holybro KakuteF4-Wing | `KakuteF4-Wing-gyro` | 1 MB (593 KB used) | onboard dataflash | 7 outputs |
| TBS Lucid H7 Wing | `TBS_LUCID_H7_WING-gyro` | 2 MB (940 KB used) | microSD card | dual IMU, 13 outputs |

Default port assignments on the Kakute:

| Port | Pins | Default use |
|---|---|---|
| USB | | MAVLink for a ground station |
| SERIAL1 (UART1) | TX PB6, RX PB7 | GPS (u-blox) |
| SERIAL2 (UART2) | TX PA2, RX PA3 | MAVLink telemetry radio |
| SERIAL3 (UART3) | TX PC10, RX PC11 | ELRS / CRSF receiver |

Default port assignments on the Lucid H7 Wing (pads are labelled Rn/Tn):

| Port | Pads | Default use |
|---|---|---|
| USB | | MAVLink for a ground station |
| SERIAL2 (UART2) | R2/T2 | GPS (u-blox) |
| SERIAL3 (UART3) | HD VTX connector | unused (DisplayPort OSD is compiled out) |
| SERIAL4 (UART4) | R4/T4 | MAVLink telemetry radio |
| SERIAL6 (UART6) | R6/T6 | ELRS / CRSF receiver, both wires for telemetry |
| SERIAL7 (UART7) | R7/T7 | MAVLink telemetry radio |
| SERIAL8 (UART8) | ESC connector | ESC telemetry |

On the Lucid the VTX power relay defaults on and output 13 drives a
WS2812 LED strip, as in the stock firmware.

Servo outputs M1 to M4 are aileron, elevator, throttle, rudder in the
default "Normal" wing type on both boards. See
[Transmitter setup menu](#10-transmitter-setup-menu) for elevon and
V-tail presets. Battery voltage and current sensing use the board's
built-in pads.

Build and upload from the repository root, using the build name from
the table:

```
./waf configure --board KakuteF4-Wing-gyro
./waf plane
./waf plane --upload        # over USB, through the bootloader
```

Outputs are in `build/<build name>/bin/`:

- `arduplane.apj` for Mission Planner or QGroundControl "load custom firmware"
- `arduplane_with_bl.hex` for DFU flashing when the bootloader is missing

Flashing keeps the parameters already saved on the board when it
already runs ArduPlane. The gyro defaults then only fill in parameters
that were never saved, so a board that was set up before keeps its old
settings for anything that was changed. For a clean start, reset the
parameters to defaults from a ground station after flashing (Mission
Planner: Full Parameter List, Reset to Default), then redo the
accelerometer calibration. Flashing over a different vehicle type or
over Betaflight/INAV starts from the defaults. To go back to stock
ArduPlane, flash the stock firmware for the board the same way.

---

## 3. Transmitter channel layout

The defaults assume an ELRS receiver on the board's RC port (SERIAL3 on
the Kakute, SERIAL6 on the Lucid) with the standard CRSF channel
endpoints (988 to 2012 us) and the usual AETR order.

| Channel | Function | Notes |
|---|---|---|
| 1 | Aileron | |
| 2 | Elevator | |
| 3 | Throttle | |
| 4 | Rudder | |
| 5 | Arm switch | ELRS AUX1, two position, high = armed |
| 6 | Flight mode | three position: MANUAL / ACRO / FBWA |
| 7 | RTL switch | high = return to launch |
| 8 | Gain knob | centre = tuned gains |

Set up the transmitter with plain sticks and switches. Do not add
mixing (elevon, V-tail, flaperon, differential) in the transmitter. The
flight controller does all mixing so that stabilization works in every
mode.

Set the receiver failsafe to "No pulses" (the ELRS default) so the
flight controller sees the link loss.

Channel assignments are parameters and can be changed from a ground
station if a different layout is preferred (see the
[parameter reference](#14-parameter-reference)).

---

## 4. Arming

The arm switch on channel 5 works like a throttle cut with extra
duties.

While disarmed:

- Throttle output is held at minimum PWM, so the ESC stays quiet.
- Control surfaces still move and the stabilization runs, so you can
  check surface directions and gyro response on the bench.
- Calibration from the transmitter menu is allowed.
- Nothing is logged.

Arming (switch high):

- All optional pre-arm checks are skipped, so a board with no GPS will
  arm. The transmitter must be on with a valid link.
- Logging starts.
- If a GPS has a fix, the home position is locked at the arming
  location.

Disarming in flight cuts the motor. Stabilization keeps working and the
plane glides, the same as pulling a throttle cut. Put the arm function
on a switch that is hard to hit by accident.

Rudder-stick arming is disabled.

---

## 5. Flight modes

The three-position switch on channel 6 selects:

| Position | Mode | Behavior |
|---|---|---|
| Low | MANUAL | Direct passthrough. No stabilization. Transmitter trims pass straight to the servos. |
| Middle | ACRO | Rate stabilized. Sticks command roll and pitch rates; centered sticks hold the current attitude against disturbances. Not self-leveling. Yaw is passthrough unless yaw rate control is enabled. |
| High | FBWA | Self-leveling. Sticks command bank and pitch angles up to the configured limits; releasing the sticks returns to level. Throttle is manual. |

Default ACRO rates are 180 deg/s roll and 90 deg/s pitch at full stick.
Default FBWA limits are the ArduPlane defaults (45 degrees of bank,
20 degrees pitch up, 25 degrees pitch down). All of these are adjustable
from the transmitter menu.

The RTL switch on channel 7 overrides the mode switch while high. When
it goes low, the plane returns to whatever the mode switch selects. If
no home position is known, RTL is refused with the message
"RTL refused: no home position" and the mode does not change.

There is no dedicated "panic" button because FBWA on the mode switch
does the same job: flip to high, release the sticks, and the plane
levels itself.

---

## 6. Trims

Trims work the standard ArduPilot way:

- **MANUAL**: transmitter trims pass straight through, as on any
  receiver.
- **ACRO and FBWA**: a transmitter trim click is a small stick input. In
  ACRO it becomes a slow roll or pitch rate, in FBWA a small bank or
  pitch angle.

So set the surface neutrals with the servo trims, not the transmitter
trims: keep the transmitter trims centered and set each surface's
neutral in the transmitter menu (4 Outputs, Channel N, Trim) or from a
ground station (`SERVOn_TRIM`). If the model needs trim in MANUAL,
move that trim into the servo trims and center the transmitter trims
again.

In flight, the I term of the rate controllers holds the model against a
small trim error in ACRO and FBWA. The built-in servo auto-trim
(`SERVO_AUTO_TRIM 1`) moves that correction into the servo trims while
flying level in FBWA, but only when both ground speed and airspeed read
at least 8 m/s, which needs a GPS fix. Without a GPS it does nothing.

Earlier builds captured the transmitter trims automatically on arming
and on MANUAL to ACRO/FBWA (`GYRO_TRIM_CAP`). That was removed: it also
captured any stick held at that moment, up to 20 percent of the throw.
On a board that ran it, check that `RC1_TRIM`, `RC2_TRIM` and
`RC4_TRIM` are at the stick centers (2 Radio, Learn sticks sets them).

---

## 7. Gain knob

Channel 8 scales the roll, pitch and yaw rate controllers, like the gain
setting on a gyro.

| Knob | Scale |
|---|---|
| Fully low | GYRO_GAIN_MIN (default 0.0, feedforward only: no P or D correction, I held) |
| Centre | 1.0, the tuned gains |
| Fully high | GYRO_GAIN_MAX (default 1.5) |

The knob scales only the correction terms: P and D directly, and the
I term by how fast it builds up. What the I term has already built up
is never rescaled, so turning the knob up in flight adds no sudden
correction. The feedforward term that moves the surface with the stick
is not scaled, so stick authority in ACRO stays the same at any knob
position. Knob centre is always exactly 1.0, whatever the min and max
are set to.

At the bottom of the knob the plane flies on feedforward alone, which is
not the same as gyro off: the sticks still command a rate, nothing
corrects gusts, and FBWA still steers toward level without damping.
MANUAL is the true gyro-off mode.

The scale is held at 1.0 during an RC failsafe and while autotune is
running. Its value is recorded in the log as `GYRO.Gain`.

To fly with fixed gains, leave the knob centered or set GYRO_GAIN_CH
to 0.

---

## 8. Link loss and failsafe

When the receiver stops sending frames:

| After | Action |
|---|---|
| 1 second | Short failsafe: FBWA, level, throttle closed. |
| 5 seconds | Long failsafe: RTL if a home position is known, otherwise stay in FBWA and glide level. |

The plane stays armed through a failsafe.

When the link returns during the short failsafe, the previous mode is
restored. When it returns after the long failsafe, the plane stays in
RTL or FBWA until you move the mode switch, so you take control
deliberately.

A ground station link loss does nothing (GCS failsafe is off).

The link-loss action can be changed in the transmitter menu between
"RTL if home" and "Level glide".

---

## 9. GPS and return to launch

A GPS is optional. Everything except RTL works without one, and the
firmware arms without one.

With a u-blox GPS on SERIAL1:

- The home position is set on the first 3D fix, refreshed every few
  seconds while disarmed, and locked at the moment of arming. Arm at
  the place you want the plane to return to.
- No compass is needed. Heading comes from GPS ground track once
  moving.
- RTL climbs or descends to RTL_ALTITUDE (default 100 m above home) and
  circles over home. It does not land. Switch back to FBWA or MANUAL to
  bring it in.
- RTL is refused, with a message, if there is no home position. The
  long failsafe falls back to a level glide in that case.

---

## 10. Transmitter setup menu

The firmware serves a setup menu over CRSF. On an ELRS radio open the
ExpressLRS Lua script, go to Other Devices and select Gyro; on TBS
radios use Agent Lite. No script needs to be installed on the flight
controller.

After flashing new firmware, exit the ExpressLRS script and start it
again. The script remembers how many entries the flight controller had
when it first saw it, and with a stale count part of the menu stays
empty.

The menu follows the Loft GCS setup order. Values are read from and
saved to ArduPilot parameters immediately. Status and the folders sit
at the device's top level: the ExpressLRS script returns there on Back
from any folder, including from a channel inside 4 Outputs.

| Folder | Entries |
|---|---|
| Status | armed state, flight mode, GPS state |
| 1 Calibration | Orientation, Detect orientation, Calibrate level, Calibrate gyros |
| 2 Radio | Learn sticks; Arm switch, Mode switch, RTL switch, Autotune switch, Gain knob, each with a Learn command |
| 3 Flight modes | Check mode switch; Low, Middle and High positions: MANUAL, ACRO, FBWA or AUTOTUNE |
| 4 Outputs | Wing type and Apply template; Channel 1 to 8 (servo outputs), each with Function, Min, Trim, Max, Reversed |
| 5 Failsafe | Link loss (FBWA glide or RTL if GPS), RTL altitude |
| 6 Tuning | Gain knob min and max, ACRO roll and pitch rates, FBWA bank and pitch limits, Autotune level |
| Tools | Firmware version, Reset to defaults |

Commands (the entries that start something) open a popup on the radio.
They all need the arm switch off and answer "Disarm first" otherwise.
Each ends with a result the pilot dismisses with OK; Exit cancels.

- **Apply template** sets the functions of output channels 1 to 4 for the wing
  type chosen above it:

  | Wing type | Channel 1 | Channel 2 | Channel 3 | Channel 4 |
  |---|---|---|---|---|
  | Normal | Aileron | Elevator | Throttle | Rudder |
  | Flying wing | Elevon left | Elevon right | Throttle | Rudder |
  | V-tail | Aileron | V-tail left | Throttle | V-tail right |

  Min, trim, max and reversing are left alone. Check every surface
  direction afterwards and flip the channel's Reversed entry where
  needed; the servo moves as soon as the value changes.
- **Detect orientation**: hold the model level with the nose forward
  and press OK, then raise the nose about 45 degrees and press OK. The
  firmware works out how the board is mounted and sets Orientation.
  Run Calibrate level afterwards.
- **Calibrate level** and **Calibrate gyros** run on the main loop
  like a ground station calibration. The flight controller pauses for
  a second or two, so the radio may briefly report lost telemetry.
- **Learn sticks**: center the sticks with the throttle low, start,
  then move each stick as asked (aileron right, elevator back, rudder
  right, throttle full). The firmware works out the channel order and
  which channels are reversed and shows it, for example
  `A2 E3 R4 T1 Save?`. If the channel order changed, the flight
  controller restarts to apply it.
- **Learn** for a switch: flip the switch or turn the knob when asked.
  A channel already used by another of these functions is refused.
  The arm switch is only assigned once it is in its off position,
  because assigning it while on would arm the plane.
- **Check mode switch** shows the mode switch live while it is open,
  for example `Ch6 High: FBWA`. When the plane is flying another mode
  (the RTL switch or a failsafe), it says so: `Ch6 High: FBWA, now RTL`.
  It changes nothing, so it also works armed. Exit closes it.

Entries that need a GPS (RTL switch, RTL altitude) are hidden when the
menu is opened without one, and a "RTL: needs a GPS" line is shown
instead. The firmware refuses RTL without a home position in any case.

---

## 11. Autotune

ArduPlane's fixed-wing autotune works in this build, with or without a
GPS or airspeed sensor. Put AUTOTUNE on a mode switch position or assign
an autotune switch in the transmitter menu (3 Flight modes, 2 Radio).

Without an airspeed estimate (no airspeed sensor and no GPS fix) stock
ArduPlane locked the roll integrator and skipped roll autotune; pitch
was unaffected. The roll and yaw rate controllers now assume the middle
of AIRSPEED_MIN and AIRSPEED_MAX in that case, as the pitch controller
already did. `Tools/gyro/sitl_autotune_test.py` flies a no-GPS,
no-airspeed autotune in SITL and checks both axes tune.

Procedure:

1. Center the gain knob. The scale is forced to 1.0 during autotune
   anyway, but a centered knob means the tuned gains are what you get
   afterwards.
2. Fly at a safe altitude in FBWA, enable autotune, and make repeated
   sharp full-stick roll and pitch inputs.
3. Disable autotune or land and disarm. The new gains are saved.

The knob then scales the tuned gains. GYRO_GAIN_MAX above 1.5 is rarely
useful after a good autotune.

Speed scaling. ArduPlane also scales the gains with speed, and autotune
tunes against that scaling:

- Without an airspeed estimate it scales by throttle while armed:
  gains are as tuned at 45% throttle, higher at low throttle (up to
  2.8 times P, I and D at idle) and lower at high throttle. Watch for
  oscillation in fast descents at idle.
- With a GPS fix it uses an airspeed estimate from GPS speed and wind:
  gains are as tuned at SCALING_SPEED. Set SCALING_SPEED to the airspeed
  the plane flies at about 45% throttle so both methods agree and gains
  tuned without a GPS still fit after one is added. The wind estimate
  needs a few turns to settle.

---

## 12. Logging

Logs go to the board's dataflash chip (Kakute) or microSD card
(Lucid), one log per arming. Nothing is logged while disarmed. Download logs over USB with Mission Planner or
QGroundControl.

Besides the standard ArduPlane messages, the `GYRO` message records at
10 Hz:

| Field | Meaning |
|---|---|
| Gain | Master gain scale applied to the rate controllers |
| LogOK | Logging gate state (always 1 with the arm switch) |

---

## 13. Ground station access

Mission Planner, QGroundControl or any MAVLink ground station connects
over USB, or over a telemetry radio on SERIAL2. Everything is still a
normal ArduPlane parameter, so any setting not covered by the
transmitter menu can be changed there: channel assignments, servo trims
and endpoints, battery monitor, failsafe timing, and so on.

Calibration from a ground station also requires the arm switch to be
off.

---

## 14. Parameter reference

Parameters added by this build:

| Parameter | Default | Meaning |
|---|---|---|
| GYRO_GAIN_CH | 8 | RC channel for the gain knob, 0 to disable |
| GYRO_GAIN_MIN | 0.0 | Scale at knob minimum |
| GYRO_GAIN_MAX | 1.5 | Scale at knob maximum |
| GYRO_LOG_THR | 0 | Throttle percent that starts logging when ARMING_REQUIRE is 0; unused with the arm switch |
| GYRO_CAL | 0 | Calibration request used by the transmitter menu: 1 = gyros, 2 = level. Reads back 0 when done, -1 when failed or refused. Reset at boot. |

Standard ArduPlane parameters set by the defaults:

| Parameter | Default | Meaning |
|---|---|---|
| ARMING_REQUIRE | 1 | Arm switch required; throttle at minimum PWM when disarmed |
| ARMING_SKIPCHK | -1 | Skip all optional pre-arm checks |
| ARMING_RUDDER | 0 | No rudder-stick arming |
| RC5_OPTION | 153 | Channel 5 arms and disarms |
| FLTMODE_CH | 6 | Mode switch channel |
| FLTMODE1..6 | 0,0,4,4,5,5 | MANUAL, ACRO, FBWA across a three-position switch |
| RC7_OPTION | 4 | Channel 7 is the RTL switch |
| SERIAL3_PROTOCOL / SERIAL6_PROTOCOL | 23 | CRSF receiver on the board's RC port (Kakute / Lucid) |
| RC_PROTOCOLS | 512 | CRSF only |
| AHRS_EKF_TYPE | 0 | DCM attitude estimation |
| COMPASS_ENABLE | 0 | No compass |
| ARSPD_TYPE | 0 | No airspeed sensor |
| SERVO_AUTO_TRIM | 1 | Servo auto-trim in FBWA (needs a GPS fix) |
| STICK_MIXING | 0 | No stick mixing in automatic modes |
| THR_FAILSAFE | 1 | RC failsafe enabled |
| FS_SHORT_ACTN | 2 | Short failsafe: FBWA at zero throttle |
| FS_SHORT_TIMEOUT | 1 | Seconds before the short failsafe |
| FS_LONG_ACTN | 1 | Long failsafe: RTL (falls back to FBWA without home) |
| FS_LONG_TIMEOUT | 5 | Seconds before the long failsafe |
| FS_GCS_ENABL | 0 | No ground station failsafe |
| ACRO_ROLL_RATE | 180 | deg/s at full roll stick |
| ACRO_PITCH_RATE | 90 | deg/s at full pitch stick |
| ACRO_LOCKING | 0 | No attitude locking in ACRO |
| INS_GYR_CAL | 1 | Gyro calibration at boot; keep the model still |
| LOG_DISARMED | 0 | Log only while armed |

---

## 15. Building and testing

Source of the customization:

| Location | What |
|---|---|
| `libraries/AP_HAL_ChibiOS/hwdef/include/gyro.inc` | Feature profile. Include it after any base board hwdef to make a gyro build of that board. |
| `libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/` | Kakute variant: hwdef and defaults |
| `libraries/AP_HAL_ChibiOS/hwdef/TBS_LUCID_H7_WING-gyro/` | Lucid H7 Wing variant, same layout |
| `ArduPlane/gyro.h`, `ArduPlane/gyro.cpp` | The gyro module: gain knob, logging gate, calibration requests, GYRO_ parameters |
| `ArduPlane/gyro_menu.h`, `ArduPlane/gyro_menu.cpp` | The transmitter setup menu |
| `ArduPlane/mode_rtl.cpp`, `ArduPlane/events.cpp` | RTL refused without home; failsafe falls back to FBWA |
| `libraries/APM_Control/AP_FW_Controller.*`, `AP_YawController.*` | Gain scale hook in the rate controllers |
| `libraries/AP_RCTelemetry/AP_CRSF_Telem.*` | CRSF menus without the OSD, numbered the way ExpressLRS expects, with top-level entries |
| `Tools/gyro/` | SITL tests, CRSF radio emulator, SITL build defines |

Porting to another board takes two files: a hwdef that includes the base
board and `gyro.inc`, and a `defaults.parm`.

Simulator tests. Build SITL first with the gyro-like defines (no OSD
parameter screens, CRSF menu engine on):

```
./waf configure --board sitl --extra-hwdef=Tools/gyro/sitl-gyro.hwdef
./waf plane
python3 Tools/gyro/sitl_gyro_test.py        # arming, no trim capture, gain knob, failsafe, RTL guard, logging
python3 Tools/gyro/sitl_menu_test.py        # menu builds; calibration requests complete
python3 Tools/gyro/sitl_crsf_menu_test.py   # the menu over CRSF, driven like the ExpressLRS script
ELRS_LUA=/path/to/elrsV3.lua python3 Tools/gyro/sitl_elrs_lua_test.py   # the real ExpressLRS script
python3 Tools/gyro/sitl_autotune_test.py    # autotune flight with no GPS or airspeed sensor
```

All need pymavlink. `sitl_crsf_menu_test.py` uses
`Tools/gyro/crsf_menu_client.py`, a small CRSF receiver and radio
emulator that loads the menu exactly as the ExpressLRS script does.

`sitl_elrs_lua_test.py` runs the ExpressLRS script itself on the PC
(`Tools/gyro/edgetx_sim.py` provides the parts of the EdgeTX Lua API it
uses) and walks the menu with key presses: opening folders, editing a
number and a selection, the calibration and stick-learning popups, and
the no-GPS Failsafe page. Every screen is saved as text and PNG in
`/tmp/gyro_sitl_elrs/screens`. It also needs lupa and Pillow
(`pip install lupa pillow`). The script is not in this repository:
copy `SCRIPTS/TOOLS/elrsV3.lua` from the radio's SD card, or get it
from the ExpressLRS repository.

To try the menu by hand, run
`ELRS_LUA=/path/to/elrsV3.lua python3 Tools/gyro/elrs_menu_playground.py`
and open http://localhost:8765 (from Windows too when running in
WSL2). The page shows the radio screen with Up, Down, Enter and Back
(arrow keys, Enter and Esc work too) and sliders for channels 1 to 8,
so the arm switch, mode switch and stick learning can be tried. The
simulated plane starts with the KakuteF4-Wing gyro defaults and no GPS;
add `--gps` to fit one.

---

## 16. Known limitations and untested items

- **Not flight tested.** Everything above has been verified in the
  simulator only. Bench test on the Kakute first: surface directions in
  MANUAL, correct stabilization direction in FBWA when tilting the
  model, arm switch, gain knob response, and the menu.
- **Transmitter menu partly verified on hardware.** On a KakuteF4-Wing
  with a RadioMaster AX12 (ELRS Lua r15) the whole menu loads and shows
  live values. Writes and the command popups (calibration, learning,
  templates, the arm-switch safety rule) are verified end to end in
  SITL with the CRSF emulator, not yet on a real radio.
- **The guided "check directions" step from the design is not
  included.** ExpressLRS closes a popup when Exit is pressed, so a
  chain of yes/no questions cannot continue after a "no". Each output
  has a Reversed entry instead.
- **Detect orientation is untested on hardware.** It needs the model
  held still in two attitudes; SITL cannot tilt the simulated plane
  on the bench.
- **No EKF.** DCM is used for attitude. It is adequate for
  stabilization and GPS return home but less precise than the EKF in
  turbulent flight and does not use a compass.
- **Take off in MANUAL or ACRO without a GPS.** Without GPS or airspeed
  the attitude estimate cannot tell the takeoff run's acceleration
  from a pitch change. In SITL an FBWA runway takeoff without GPS
  reported the nose 10-15 degrees low and did not get airborne. Not yet
  checked on a real model or with a hand launch.
- **Missions and AUTO mode are still compiled in** because ArduPlane
  cannot be built without them. They are simply not on any switch.
- **Powered planes only.** Gliders, quadplanes and helicopters are out
  of scope.
