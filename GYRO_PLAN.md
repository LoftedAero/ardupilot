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
  pre-arm checks are skipped except RC and INS (ARMING_SKIPCHK -81 since
  2026-11-18, was -65; RC_OPTIONS 96 so only the throttle-low RC check
  runs; INS catches a failed boot gyro calibration and an uncalibrated
  accelerometer: a new board needs the menu's 6-position Calibrate accel
  once, added 2026-11-18), no rudder arming. The menu's GPS setting adds the GPS
  checks while it is On.
  RC_OPTIONS bit 12 is also on (4192, added 2026-11-08) so radios on
  plain CRSF telemetry can show armed or disarmed (the flight mode
  carries a star while disarmed); Lofty PFD reads it.
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
  Wing (H743, 2 MB, added 2026-09-14). The custom board is the
  LoftedAero-SRX (F405, 1 MB, ELRS receiver on the board, added
  2026-11-13).
- Setup UX will eventually live in Loft GCS (separate repo, untouched for
  now) and, if flash allows, a lightweight transmitter Lua menu.

## Comparison with consumer gyros

| Gyro behavior | Stock ArduPlane | Gap |
|---|---|---|
| Rate damping / self-level / off | ACRO / FBWA / MANUAL | none |
| Return home | RTL with the GPS setting On (EKF3), no compass needed | RTL with no home is unguarded |
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
  profile. Include it after any base board hwdef. Disables terrain,
  rally, fence, ADSB, soaring, landing gear, quadplane, OSD, video TX,
  camera, mount, scripting. Keeps logging, tuning, CRSF telemetry.
- `libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/hwdef.dat`: base
  board plus the profile.
- `libraries/AP_HAL_ChibiOS/hwdef/KakuteF4-Wing-gyro/defaults.parm`:
  arm switch on channel 5, GPS off (DCM; the menu GPS setting selects EKF3), CRSF on SERIAL3, mode switch on
  channel 6, RTL on channel 7, gain knob on channel 8, servo auto-trim,
  RTL-or-glide failsafe, ACRO rates, scripting on.
- `Tools/bootloaders/KakuteF4-Wing-gyro_bl.*`: symlinks to the base
  board bootloader.

Porting to a new board is two files: a hwdef that includes the base
board and `gyro.inc`, and a defaults.parm. `TBS_LUCID_H7_WING-gyro` is
the second such port (RC on SERIAL6, GPS on SERIAL2, SD-card logging).

### Base moved to master of 2026-11-06, LoftedAero-SRX added (2026-11-13)

The branch was rebased from master `104af030` (2026-01-31) onto
`a485091a` (2026-11-06) plus the two commits that add the
`LoftedAero-SRX` base board. The SRX IMU (LSM6DSV16X) has no driver on
the old base. The tag `gyro-pre-rebase-2026-11-13` keeps the old branch.

What the new base changed for the gyro code:

- `AP_FW_Controller` already assumes the mid airspeed for the roll
  axis when there is no estimate, so the gyro's own change of the roll
  controller is gone. The yaw rate controller still needs it.
- `RC_Channels::get_radio_in(chan)` and the `AP_GPS::` fix names are
  gone; `gyro_menu.cpp` has a local `radio_in()` and uses
  `AP_GPS_FixType`.
- DCM reports no location until it has had a GPS position, so MAVLink
  `VFR_HUD.alt` reads 0 with no GPS. The height above home
  (`GLOBAL_POSITION_INT.relative_alt`, CRSF altitude) still comes from
  the barometer. The flight tests read that.
- `FS_SHORT_TIMEOUT` in the gyro defaults was never a parameter and was
  ignored. The defaults now name `RC_FS_TIMEOUT`, with the same value
  as its stock default.

`LoftedAero-SRX-gyro` is the third port: the base board's hwdef and
bootloader, `gyro.inc`, and the Kakute defaults with the receiver on
SERIAL2 (set by the hwdef), `RC_OPTIONS 12640` (adds CRSF passthrough
telemetry and the 420 kbaud ELRS link) and `RSSI_TYPE 3`. It keeps board ID 5999, the placeholder that
the bootloader on board 1 accepts. No relay is assigned: GPIO 70 and 71
are the ELRS receiver's EN and BOOT pins. Built, 609 KB of 983 KB; not
yet run on a board.

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
   `set_gain_scale()`, passed to AC_PID's `update_all()` as `pd_scale`
   and `i_scale`: P and D scale directly, I scales its build-up rate so
   a knob change never steps the I output (scaling the I output instead
   wound up the integrator at low gain and kicked when the knob was
   raised). The yaw damper scales D only. FF is untouched so ACRO stick
   authority is unchanged. Held at 1.0 during
   RC failsafe and while autotune is running. Logged as `GYRO.Gain`.
3. **Gyro-style trims (removed 2026-10-03: it also captured any stick
   held at that moment; trims now work the standard ArduPilot way).**
   `GYRO_TRIM_CAP 1` called the existing
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
- Measured on a KakuteF4-Wing on 2026-10-02 (ELRS receiver on SERIAL2):
  1. The 15 KB `crsf_helper` module could not even be opened: compressed
     ROMFS files are decompressed into a heap buffer, and only ~14 KB of
     RAM is free once the 43 KB Lua heap is allocated. Fixed by
     `env ROMFS_UNCOMPRESSED True` in `gyro.inc` (files read straight
     from flash, +24 KB flash).
  2. The Lua heap is then too small: `not enough mem, increase
     SCR_HEAP_SIZE`. With SCR_HEAP_SIZE 52 KB plus Lua's automatic heap
     expansion it still fails, with 3.5 KB of RAM left. **The Lua menu
     cannot run on an F405.** It is fine on H7 boards (200 KB heap).
  3. Freeing RAM works: with LOG_FILE_BUFSIZE 4 (dataflash buffer
     16 KB -> 4 KB), SCR_HEAP_SIZE 61440 and Lua heap expansion off
     (SCR_DEBUG_OPTS 64), the menu loads and ~7 KB of RAM stays free
     (bench experiment, 2026-10-02). Not yet checked: whether logging
     keeps up with a 4 KB buffer.
  Where the RAM goes (board @SYS/threads.txt, linker map): static
  ~61 KB (16 KB parameter storage mirror, 15 KB Plane object), the
  scripting thread stack 17.8 KB, the Lua heap, the dataflash buffer
  16 KB; the menu itself needs ~50 KB of Lua heap on 64-bit SITL on
  top of a ~16 KB Lua baseline.
  4. With the menu loaded, the ELRS Lua script still showed an empty
     device: ArduPilot spaced scripted ids in blocks and advertised a
     count of 0 without OSD, while ELRS Lua loads ids 1..count. Fixed
     in AP_CRSF_Telem (contiguous ids, real count). Verified on a
     RadioMaster AX12 (Android, ELRS Lua r15): menu, values and a
     parameter write all work.
  Options: ship the reduced log buffer as the F405 default (cheapest);
  add margin with a slimmer Lua menu without the generic crsf_helper,
  or by dropping MAVLink FTP and spare serial ports; or a native C++
  menu (no Lua: frees ~43 KB heap + 17.8 KB stack, and ~120 KB flash
  with scripting off). Decision pending.

## Phase 3b: native C++ transmitter menu (built 2026-10-02)

Status: built in `ArduPlane/gyro_menu.cpp` and served by the gyro
module at 50 Hz. Gyro builds now have Lua scripting off. Verified: the
whole menu loads with live values on a RadioMaster AX12 (ELRS Lua r15)
against a KakuteF4-Wing; writes, command popups, stick and switch
learning, templates and the arm-switch safety rule pass 27 end-to-end
checks in SITL (`Tools/gyro/sitl_crsf_menu_test.py`, using the CRSF
radio emulator `Tools/gyro/crsf_menu_client.py`).

Differences from the design below:

- No guided "Check directions" step: ExpressLRS closes a popup on Exit,
  so a chain of yes/no questions cannot continue after a "no". Each
  output's Reversed entry applies immediately instead.
- No trim capture: removed 2026-10-03. Servo trims are set per channel
  (4 Outputs, Channel N, Trim).
- Command results are shown as a confirmation the pilot dismisses,
  because ExpressLRS closes a popup as soon as a command reports ready.
- The arm function is only assigned (by Learn or by the channel
  selection) while that switch is off: ArduPilot only tracks a
  switch's position while it has a function, so assigning it in the
  on position would arm the plane.
- SITL must be built with `--extra-hwdef=Tools/gyro/sitl-gyro.hwdef`
  (no OSD parameter screens). With them, reading an OSD parameter
  entry over CRSF crashes SITL in AP_OSD_ParamSetting::update (stock
  issue, not in gyro builds).
- After reflashing, the ExpressLRS script must be restarted: it caches
  the entry count from its first device ping.
- The ExpressLRS script (r15) keeps no folder history: Back from any
  folder, however deep, returns to the device's top level. Found with
  `Tools/gyro/sitl_elrs_lua_test.py`, which runs the real script on the
  PC against SITL (`Tools/gyro/edgetx_sim.py` stands in for EdgeTX).
  So there is no "Gyro Setup" folder: Status and the numbered folders
  are the device's top level (AP_CRSF_Telem now serves entries added
  to its root menu).

Design:

Decided: the transmitter menu moves from Lua to C++ in the gyro module,
using the same CRSF scripted-menu structures. Reasons: the Lua menu only
fits on an F405 with bench-only memory settings, and the planned menu is
roughly twice its size. With the menu native, F405 gyro builds can turn
scripting off (frees ~60 KB of RAM and ~120 KB of flash).

Design rules (user feedback 2026-10-02):

- Follow the Loft GCS setup order: Configuration (frame), Sensors,
  Radio, Flight Modes, Outputs, Failsafe, Tuning. Ports, Power, OSD and
  Filters stay ground-station only. Steps are numbered, per the Loft GCS
  rule that an ordered process numbers its steps.
- ArduPilot naming throughout: MANUAL, ACRO, FBWA, AUTOTUNE, RTL.
- Outputs use the ArduPilot model: each servo output has a function,
  min, trim, max and reversed. Wing type sets the output functions when
  it is changed (a separate Apply step read as if the choice alone took
  effect), not a separate mixing layer.
- Borrowed from AS3X: learn the transmitter by moving sticks and
  switches; detect board orientation by holding the model level and
  then raising the nose; one gain plus a knob.
- Avoided: per-mode gain tables, firmware parameter names on the radio.
  Multi-position calibration was avoided until 2026-11-18; it is now the
  stock 6-position accelerometer calibration (Calibrate accel), which
  the INS arming checks require.

Menu (ELRS Lua shows folders, numbers, selections, info lines and
commands with OK/Cancel popups; values refresh when a menu is reopened):

```
Gyro (device top level)
├─ Status                   DISARMED · FBWA · GPS: none      (info)
├─ 1 Calibration ▸
│   ├─ Board orientation    current value          [Detect]: level, then nose up
│   ├─ Accel calibration    [Run]  6 poses, then restarts (stock AP_AccelCal)
│   ├─ Level calibration    [Run]  level trim only (GYRO_CAL 2)
│   └─ Gyro calibration     [Run]                  (GYRO_CAL 1)
├─ 2 Radio ▸
│   ├─ Learn sticks         [Start]  center, then 8 held moves (Loft GCS order):
│   │                                RCMAP_*, RCn_REVERSED/MIN/MAX/TRIM
│   ├─ Arm switch           Off / Ch5..Ch16        [Learn]
│   ├─ Mode switch          Ch5..Ch16              [Learn]
│   ├─ RTL switch           Off / Ch5..Ch16        [Learn]   (only with a GPS)
│   ├─ Autotune switch      Off / Ch5..Ch16        [Learn]   (works in FBWA)
│   └─ Gain knob            Off / Ch5..Ch16        [Learn]
├─ 3 Flight modes ▸
│   ├─ Check mode switch    [Start]  live: "Ch6 High: FBWA" (", now RTL" if different)
│   └─ Low / Mid / High     MANUAL / ACRO / FBWA / AUTOTUNE
├─ 4 Outputs ▸
│   ├─ Wing type            Normal / Flying wing / V-tail: sets channels 1-4
│   ├─ Copy stick trims     [Run]  MANUAL: outputs' positions -> SERVOn_TRIM,
│   │                              then center TX trims; Exit undoes
│   ├─ Check directions     [Start]  yes/no per surface, flips Reversed
│   └─ Channel 1..N ▸       Function, Min, Trim, Max, Reversed
├─ 5 Failsafe ▸
│   ├─ GPS                  Off / On: GPS driver, EKF3, GPS arming checks; restarts
│   ├─ Link loss            FBWA glide / RTL       (RTL only with a GPS)
│   └─ RTL altitude         m                      (only with a GPS)
├─ 6 Tuning ▸
│   ├─ Gain knob min / max
│   ├─ FBWA bank / pitch limits
│   ├─ Autotune level
│   ├─ Speeds ▸             (locked while armed)
│   │   ├─ Cruise speed     m/s  SCALING_SPEED + AIRSPEED_CRUISE
│   │   ├─ Min speed        m/s  AIRSPEED_MIN, kept below cruise
│   │   └─ Max speed        m/s  AIRSPEED_MAX, kept above cruise
│   ├─ Roll gains ▸         FF, P, I, D (RLL_RATE_*), ACRO roll rate
│   ├─ Pitch gains ▸        FF, P, I, D (PTCH_RATE_*), ACRO pitch rate
│   └─ Yaw gains ▸          Yaw damper (YAW2SRV_DAMP, 0 = off; knob scales it),
│                           Rudder mix (KFF_RDDRMIX)
└─ Tools ▸
    ├─ Firmware             version, board          (info)
    └─ Reset to defaults    [Run]  confirm
```

Notes:

- Mode positions write FLTMODE pairs (1-2 low, 3-4 mid, 5-6 high).
- The autotune switch (RCn_OPTION 107) only tunes in FBWA among the
  allowed modes; AUTOTUNE as a mode position is FBWA with tuning on.
  Both are offered.
- GPS presence: AP_GPS reports NO_GPS when the GPS is disabled or
  nothing has been heard for 4 s, NO_FIX when it is talking without a
  lock. When the menu is opened without a GPS, the RTL items are
  replaced by an info line "RTL: no GPS". The firmware already refuses
  RTL without a home and glides instead, so this is presentation only.
- Output functions offered: None, Aileron, Elevator, Rudder, Throttle,
  Elevon L/R, V-tail L/R, Flaperon L/R, Flap. Outputs listed are the
  board's PWM outputs.
- The menu needs new firmware support for the learn steps (live RC
  channel detection) and orientation detection (accelerometer), in the
  gyro module, triggered from the menu like GYRO_CAL.

## Autotune without an airspeed estimate (2026-10-03)

Stock AP_RollController and the yaw rate controller assume 0 airspeed
when there is no estimate (no airspeed sensor, no GPS fix). That counts
as underspeed: the integrator is locked and autotune skips the axis.
The pitch controller assumes the middle of AIRSPEED_MIN/MAX instead.
Roll and yaw rate now do the same. Verified with
`Tools/gyro/sitl_autotune_test.py` (MANUAL takeoff, FBWA climb,
AUTOTUNE on the mode switch, ARSPD_TYPE 0, no GPS): before the change
roll never finished in 300 s and RLL_RATE_P stayed put while pitch
tuned; after it both axes finish in under a minute.

## Turns without a GPS (2026-11-13)

Reverses the rejection of 2026-09-14, which rested on "the horizon
drifts a few degrees in long turns". Measured in SITL with no GPS and
no airspeed sensor, full roll stick held in FBWA:

| Case | Estimate | True attitude | Result |
|---|---|---|---|
| Stock, 25 deg bank (stall prevention limit) | 24 roll, 0 pitch | 46 roll, -20 pitch | ground from 148 m in 24 s |
| Stock, 45 deg bank | 43 roll, +2 pitch | 66 roll, -29 pitch | ground in 16 s |
| AHRS_RP_P 0.05, 25 deg | 25 roll, 0 pitch | 33 roll, -10 pitch | 40 m lost in 25 s |
| With GPS, 45 deg | 45 roll | 44.6 roll | held |
| GYRO_TURN_CORR 1, SCALING_SPEED 15 (20% low) | 45 roll, +2 pitch | 48 roll, -5 pitch | 30 m lost in 25 s |
| GYRO_TURN_CORR 1, SCALING_SPEED 19 (matched) | 45 roll, +2 pitch | 46 roll, 0 pitch | held |
| GYRO_TURN_CORR 1, SCALING_SPEED 30 (57% high) | 45 roll, +3 pitch | 42 roll, +10 pitch | climbs |

Design:

- `AP_AHRS::set_assumed_airspeed_EAS()`. DCM uses it in its no-GPS
  branch only when there is no airspeed sensor and no earlier GPS
  estimate. `airspeed_EAS()` is unchanged, so the controllers and the
  throttle based gain scaling behave as before.
- A change of the assumed speed rescales the previous velocity, so only
  the change of direction is corrected (the centripetal part). A
  throttle change is not read as an acceleration.
- `PlaneGyro::update_assumed_airspeed()` supplies
  `SCALING_SPEED / speed_scaler` (the speed the throttle scaling stands
  for) while the model counts as airborne: from 1 s of armed throttle
  above 30% until it is disarmed and still for 3 s. A disarm in flight
  is a throttle cut, so it does not end the correction.
- `GYRO_TURN_CORR` (0 in the source, 1 in the board defaults).
- Stall prevention's 25 degree limit is skipped when there is no
  airspeed estimate (`Plane::apply_load_factor_roll_limits`).

Not flight tested. Open: whether `is_still()` can read true in a very
smooth disarmed glide, which would end the correction early.

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
| Phase 3b (native C++ menu, scripting off) | 593 KB |
| On master of 2026-11-06 | 601 KB |
