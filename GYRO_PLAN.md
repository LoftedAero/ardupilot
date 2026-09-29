# ArduPilot Gyro Build: Plan

Goal: a stripped ArduPlane firmware that replaces a consumer RC gyro or
stabilized receiver (AS3X/SAFE, FrSky S6R/S8R, Aura, HobbyEagle A3) on
powered fixed-wing models, including flying wings.

Core wants:

1. Easy MANUAL, ACRO (rate) and auto-level (FBWA) modes
2. RTL when a GPS is fitted
3. Keep ArduPilot RC input, servo output/trim/mixing and autotune

Ground rules (decided 2026-09-14):

- This fork will not be merged upstream. Heavy customization is fine, but
  avoid modifying known-working core ArduPilot code where a smaller,
  additive change will do.
- Arm switch on channel 5 (ELRS AUX1), decided 2026-09-14 after first
  trying no arming at all. It lets calibration, logging and home
  position work the stock way and doubles as a throttle cut. Optional
  pre-arm checks are skipped (ARMING_SKIPCHK -1), no rudder arming.
  Boot gyro calibration runs when the model is still.
- Single three-position mode switch on channel 6 (MANUAL / ACRO / FBWA),
  RTL on channel 7, gain knob on channel 8.
- Transmitter trims behave like a gyro (servo offset) in MANUAL and ACRO.
  Trims are captured on arming in MANUAL and on MANUAL -> ACRO/FBWA.
  FBWA uses servo auto-trim and ignores transmitter trims.
- Gains: fixed gains from autotune, plus a transmitter knob master gain.
- Link loss with no GPS home: FBWA, level, zero throttle. With home: RTL.
- Keep dataflash logging. Drop OSD.
- Powered planes only. No gliders, no quadplane.
- Test hardware: Holybro KakuteF4-Wing (F405, 1 MB) and TBS Lucid H7
  Wing (H743, 2 MB, added 2026-09-14). Custom board later.
- Setup UX will eventually live in Loft GCS (separate repo, untouched for
  now) and, if flash allows, a lightweight transmitter Lua menu.

## Comparison with consumer gyros

| Gyro behavior | Stock ArduPlane | Gap |
|---|---|---|
| Rate damping / self-level / off | ACRO / FBWA / MANUAL | none |
| Return home | RTL on DCM + GPS, no compass needed | RTL with no home is unguarded |
| Servo mixing (elevon, V-tail, flaperon) | SERVOn_FUNCTION | none |
| Autotune | AUTOTUNE mode, FW_AUTOTUNE aux | none |
| Power on and fly | Arm switch, like a throttle cut | none |
| TX trims offset servos | Only in MANUAL; stabilized modes treat trim as stick input | needs source |
| Master gain knob, stick priority | TUNE_CHAN tunes one PID set | needs source for a true scalar |
| Panic / SAFE recovery | Aux function 92 forces FBWA | none |
| Configure from transmitter | CRSF scripted menus (needs Lua) | flash cost, script to write |
| No ground station needed | Needs one for accel cal and radio setup | Loft GCS later |

## Phase 1: configuration profile (done, builds clean)

Files:

- `libraries/AP_HAL_ChibiOS/hwdef/include/gyro.inc`: shared feature
  profile. Include it after any base board hwdef. Disables EKF3, terrain,
  rally, fence, ADSB, soaring, landing gear, quadplane, OSD, video TX,
  camera, mount, scripting. Keeps logging, tuning, CRSF telemetry.
- `libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/hwdef.dat`: base
  board plus the profile.
- `libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/defaults.parm`:
  arm switch on channel 5, DCM only, CRSF on SERIAL3, mode switch on
  channel 6, RTL on channel 7, gain knob on channel 8, servo auto-trim,
  RTL-or-glide failsafe, ACRO rates, scripting on.
- `Tools/bootloaders/KakuteF4-Wing-gyro_bl.*`: symlinks to the base
  board bootloader.

Porting to a new board is two files: a hwdef that includes the base
board and `gyro.inc`, and a defaults.parm. `TBS_LUCID_H7_WING-gyro` is
the second such port (RC on SERIAL6, GPS on SERIAL2, SD-card logging).

## Phase 2: small source changes (done, builds clean)

All new behavior lives in `ArduPlane/gyro.h` and `ArduPlane/gyro.cpp`
(class `PlaneGyro`, parameter group `GYRO_`, registered as ParametersG2
subgroup 42, scheduler task at 10 Hz). Every hook into existing code is
a few lines, guarded by `AP_PLANE_GYRO_ENABLED`, and a no-op until the
matching parameter is set, so stock behavior is unchanged by default.

1. **Failsafe: RTL only when home is set.** `ModeRTL::_enter()` refuses
   with "RTL refused: no home position" when AHRS has no home. Long
   failsafe in `ArduPlane/events.cpp` falls back to FBWA (glide) when
   RTL entry fails. The RTL aux switch is refused the same way. Home is
   set on the first 3D fix regardless of arming, so RTL works with
   ARMING_REQUIRE 0 once a GPS has locked. Not parameter gated.
2. **Master gain scalar.** `GYRO_GAIN_CH`, `GYRO_GAIN_MIN`,
   `GYRO_GAIN_MAX`. The knob is read each cycle; knob centre is always
   1.0, low end maps to MIN, high end to MAX. The scale is pushed into
   `AP_FW_Controller` (roll, pitch) and `AP_YawController` via a new
   `set_gain_scale()`, applied to the P, I and D outputs only. FF is
   untouched so ACRO stick authority is unchanged. Held at 1.0 during
   RC failsafe and while autotune is running. Logged as `GYRO.Gain`.
3. **Gyro-style trims.** `GYRO_TRIM_CAP 1` calls the existing
   `trim_radio()` at the moment of arming in MANUAL (like a gyro learning
   stick centre at power-up) and on MANUAL -> ACRO/FBWA, when roll and
   pitch sticks are within 20% of centre, rotation is under 30 deg/s and
   RC is valid. Servo trims take the current surface position and RC
   trims take the current stick centre, so the stabilised mode starts
   with zero demand. Skipped silently otherwise. Trim clicks made in
   ACRO or FBWA are captured on the next pass through MANUAL. In FBWA,
   `SERVO_AUTO_TRIM` continues to refine the servo trims.
4. **Logging without arming.** `GYRO_LOG_THR` (percent), only for
   ARMING_REQUIRE 0 setups. Off by default now that the arm switch
   drives logging the stock way. One-line change in
   `AP_Arming_Plane::update_soft_armed()`.
5. **Boot calibration.** Unchanged. Gyro calibration retries for up to
   30 s waiting for the model to be still, then uses the best average
   and flags the calibration as failed. With no arming the aircraft
   still flies, just with less accurate offsets.

Kakute defaults now set `GYRO_GAIN_CH 8`, `GYRO_GAIN_MIN 0`,
`GYRO_GAIN_MAX 1.5`, `GYRO_TRIM_CAP 1` and `FS_LONG_ACTN 1` (RTL, glide
fallback).

Verified in SITL with `Tools/gyro/sitl_gyro_test.py` (24 checks): RTL
refused without home, arming on channel 5 with trim capture, a second
capture on MANUAL -> ACRO, no log bytes before arming, long RC failsafe
ending in FBWA with RTL refused while staying armed, RTL accepted once
GPS sets home, disarm on channel 5, and GYRO.Gain following the knob
(0.0, 1.0, 1.5). Not yet flight tested.

## Phase 3: transmitter menu (done, builds clean, load-tested in SITL)

- `gyro.inc` now keeps Lua scripting on and forces
  `HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED 1` so the CRSF parameter-menu
  code is compiled without the OSD.
- `AP_CRSF_Telem` assumed the OSD parameter screens exist whenever the
  menu code is on. Guarded the OSD-only parts with `OSD_PARAM_ENABLED`
  so scripted menus work on their own; the root folder then lists only
  scripted menus. No behaviour change when the OSD is present.
- `libraries/AP_Scripting/applets/gyro_menu.lua` builds a "Gyro Setup"
  menu on CRSF/ELRS radios (EdgeTX ELRS Lua or TBS Agent Lite):
  Level & cal (calibrate level, calibrate gyros, capture trims),
  Servos (wing type Normal/Elevons/V-tail, output 1-4 direction),
  Rates & limits (acro rates, bank and pitch limits), Gyro (gain knob
  range, trim capture, yaw rate mode), RTL & link loss (RTL altitude,
  link-loss action, RC failsafe on/off). Values are read from and
  saved to parameters directly.
- Calibration commands are refused while armed; the applet answers
  "Disarm first" in that case.
- The applet and `crsf_helper.lua` are embedded in ROMFS by the Kakute
  hwdef; `SCR_ENABLE 1` is in the defaults.
- `Tools/gyro/sitl_menu_test.py` starts SITL with a simulated CRSF
  receiver and confirms the menu builds without Lua errors. Radio-side
  interaction is not simulated; test on a real ELRS transmitter.
- Fixed 2026-09-28 before hardware testing, three problems in the
  menu's calibration commands, found while writing the bench checklist:
  1. `gcs:run_command_int` returns a MAV_RESULT number (0 = success)
     and any number is true in Lua, so the menu said "Done" even on
     failure.
  2. The binding only reads `p1`-`p4`, `x`, `y`, `z`; param5 must be
     passed as `x`. "Calibrate level" passed `p5 = 4`, which was
     silently dropped, so no level calibration ever ran.
  3. Running a calibration from Lua at all froze the SITL vehicle: the
     binding holds the scheduler lock while the calibration waits for
     IMU samples that only the blocked main loop produces. Hardware may
     behave differently (IMU reads run on their own thread there), but
     it is not a path to rely on.
  The upstream `crsf-calibrate.lua` applet has all three. The fix: the
  menu now sets the new `GYRO_CAL` parameter and `PlaneGyro` runs the
  calibration on the main thread, like a ground station calibration,
  then writes back 0 (done) or -1 (failed or armed).
  `sitl_menu_test.py` verifies this path; `sitl_gyro_test.py` checks it
  is refused while armed.
- SITL limitation found on the way: with the in-process CRSF simulator
  attached (`--serial5=sim:crsf`), any blocking calibration, including
  a plain MAVLink PREFLIGHT_CALIBRATION, freezes the simulated vehicle
  afterwards (main loop stuck on the SERIAL5 UART lock). Stock
  behaviour, not ours; the menu test therefore runs its calibration
  checks without the CRSF simulator.
- Open risk: the F405 defaults to a 43 KB Lua heap. If the menu fails
  to load on hardware with an out-of-memory message, raise
  SCR_HEAP_SIZE (RAM permitting) or trim the menu.

## Considered and rejected

- **Assumed airspeed for DCM centripetal correction without GPS**
  (2026-09-14). DCM has no centripetal correction when there is no GPS
  and no airspeed sensor, so the horizon drifts a few degrees in long
  turns. Substituting SCALING_SPEED was analysed and would be safe if
  gated on armed and only used before any GPS estimate exists, but
  stock ArduPlane flies FBWA without GPS routinely, so the code stays
  untouched. Fit a GPS if the drift matters.

## Phase 4: Loft GCS guided setup

Separate repo. Not started until asked. Scope sketch: board flash,
accel level, radio check, servo direction and mixing wizard, gain knob
range, autotune runner, log pull.

## Phase 5: custom hardware

Decided 2026-09-28: the custom board uses a supported STM32 flight
controller MCU (F405 or H743 class) with an ESP-based ELRS receiver on
the same PCB, the way commercial AIO boards do it. New hwdef once the
MCU, IMU, ELRS chip and servo count are chosen; `gyro.inc` applies
unchanged.

Considered and not pursued:

- RP2350: would need a new ArduPilot HAL (ESP32-style, several
  months minimum), PIO-based UARTs and DShot, and a storage backend on
  external flash. No benefit that outweighs that.
- STM32F722 (512 KB flash): only fits after replacing the Lua menu
  with a native C++ one, cutting compass, airspeed, BLHeli, extra RC
  protocols and much of MAVLink, and probably dropping logging and
  compiling out missions. Revisit only if the hardware forces it.

## Flash budget (KakuteF4-Wing, 976 KB usable)

| Build | Text |
|---|---|
| Previous gyro build (no logging, OSD on) | 731 KB |
| Phase 1 (logging on, OSD off, quadplane off, scripting off) | 580 KB |
| Phase 2 (gyro module added) | 581 KB |
| Phase 3 (scripting + CRSF menu + embedded Lua) | 725 KB |
