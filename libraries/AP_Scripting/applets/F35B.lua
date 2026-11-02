--[[
   F35B.lua - custom flight logic for an F-35B style tiltrotor VTOL.

   This is a single, modular applet. Each subsystem is gated by its own
   *_ENABLE parameter so it can be turned on/off independently. The file is
   intentionally the *sole* owner of flight-mode changes so that subsystems
   which react to (or command) the flight mode never race each other.

   Subsystems:
     DOORS - drives a PWM output for the motor-cover doors, and gates entry
             into VTOL (Q) modes until the doors have physically opened.

   See F35B.md for setup and parameter documentation.
--]]

-- luacheck: only 0

local RUN_INTERVAL_MS = 50            -- main loop period (20 Hz)
local ERROR_RETRY_MS = 1000           -- loop period after a runtime error (limits error spam)
local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_WARNING = 4
local MODE_REASON_SCRIPTING = 32      -- ModeReason::SCRIPTING (libraries/AP_Vehicle/ModeReason.h)

assert(quadplane, "F35B: requires a quadplane build")

-- ArduPlane flight mode numbers
local MODE = {
   MANUAL        = 0,
   CIRCLE        = 1,
   STABILIZE     = 2,
   TRAINING      = 3,
   ACRO          = 4,
   FLY_BY_WIRE_A = 5,
   FLY_BY_WIRE_B = 6,
   CRUISE        = 7,
   AUTOTUNE      = 8,
   AUTO          = 10,
   RTL           = 11,
   LOITER        = 12,
   TAKEOFF       = 13,
   AVOID_ADSB    = 14,
   GUIDED        = 15,
   INITIALISING  = 16,
   QSTABILIZE    = 17,
   QHOVER        = 18,
   QLOITER       = 19,
   QLAND         = 20,
   QRTL          = 21,
   QAUTOTUNE     = 22,
   QACRO         = 23,
   THERMAL       = 24,
   LOITER_ALT_QLAND = 25,
}

-- set of modes that spin the lift motors / move the tilts (blocked by closed doors)
local is_vtol_mode = {
   [MODE.QSTABILIZE] = true,
   [MODE.QHOVER]     = true,
   [MODE.QLOITER]    = true,
   [MODE.QLAND]      = true,
   [MODE.QRTL]       = true,
   [MODE.QAUTOTUNE]  = true,
   [MODE.QACRO]      = true,
   [MODE.LOITER_ALT_QLAND] = true,
}

-- ArduPilot standard 6-position switch PWM bands -> position 1..6
local function pwm_to_pos(pwm)
   if     pwm <= 1230 then return 1
   elseif pwm <= 1360 then return 2
   elseif pwm <= 1490 then return 3
   elseif pwm <= 1620 then return 4
   elseif pwm <= 1749 then return 5
   else                    return 6 end
end

-------------------------------------------------------------------------------
-- === DOORS ===
-------------------------------------------------------------------------------
local doors = {}

do
   local PARAM_TABLE_KEY = 100
   local PARAM_TABLE_PREFIX = "DOOR_"
   assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), "F35B: could not add DOOR_ param table")

   local function add(idx, name, default)
      assert(param:add_param(PARAM_TABLE_KEY, idx, name, default),
             string.format("F35B: could not add param %s%s", PARAM_TABLE_PREFIX, name))
      return Parameter(PARAM_TABLE_PREFIX .. name)
   end

   --[[
      DOOR_ENABLE   : 0 disabled, 1 enabled
      DOOR_MODE_CH  : RC input channel carrying the rerouted 6-position mode selector.
                      MUST NOT be the same channel as FLTMODE_CH.
      DOOR_MODE1..6 : target flight mode number for each selector position.
      DOOR_FUNC     : SERVOx_FUNCTION assigned to the door output (94..109 = scripting1..16).
      DOOR_CLOSED   : PWM commanded for closed (fixed wing).
      DOOR_PART     : PWM commanded for partially open (fast/transition, reduced drag).
      DOOR_OPEN     : PWM commanded for fully open (low speed / hover).
      DOOR_ENTRY_MS : time from commanding the doors open until it is safe to enter VTOL,
                      i.e. how long the doors take to reach the partial (operating clearance)
                      position. Inbound VTOL entry is withheld this long.
      DOOR_PART_AS  : OUTBOUND threshold (m/s). While transitioning VTOL->FW (a fixed-wing
                      mode with assisted flight) the doors neck to partial at/above this
                      airspeed, full below. Roughly the speed during the forward transition
                      where reduced drag is wanted (e.g. about half the transition-complete
                      airspeed).
      DOOR_FULL_AS  : INBOUND / VTOL threshold (m/s). While in a VTOL mode the doors open
                      fully below this airspeed, partial above. Raise it to open fully sooner
                      (at higher speed) as the aircraft decelerates back into VTOL. The two
                      thresholds are chosen by direction and are independent - there is no
                      ordering requirement between them (FULL_AS is typically > PART_AS).
                      Uses sensed or synthetic airspeed; with no estimate the doors stay
                      fully open whenever the VTOL system is in play.
      DOOR_TILT_FN  : SERVOx_FUNCTION of the nacelle-tilt output to watch (41 = motor tilt,
                      75/76 = vectored left/right). The script reads this output's PWM
                      (1000..2000) to know when the nacelles are stowed forward and it is safe
                      to close the doors. Watch the TILT_POS telemetry (now the tilt PWM) to
                      confirm the right function and the fully-forward value. 0 = no tilt
                      reading, so the doors never close (function 0 is held by every unused
                      output, so it can't be read as "unassigned").
      DOOR_TILT_FWD : tilt-output PWM that marks nacelles fully forward (doors may close).
                      Direction is auto-detected: if this is >= 1500 the forward end is high
                      PWM (nacelles fwd when TILT_POS >= this); if < 1500 the forward end is
                      low PWM (nacelles fwd when TILT_POS <= this). Set it from the TILT_POS
                      value seen with the nacelles fully forward, nudged toward mid-stick.
      DOOR_LIFT_FN  : SERVOx_FUNCTION of a dedicated (non-tilting) lift motor to watch, so the
                      doors don't close while it is still powered/spooling down. Motor N
                      function = 32+N for motors 1..8, 73+N for motors 9..12 (e.g. motor 1 = 33).
                      0 disables this gate (pure tiltrotor - nacelle position is sufficient).
      DOOR_LIFT_OFF : output PWM at/below which the watched lift motor counts as off. Set from
                      the LIFT_PWM telemetry: between the value seen with the motor stopped and
                      the value when it is idling/running.
      DOOR_DEBUG    : 0 normal, 1 verbose. When 1 the script sends a GCS message on every door
                      state change (with the deciding inputs) and on every mode/latch action,
                      for SITL testing. Leave 0 for flight.
   --]]
   doors.ENABLE   = add(1,  "ENABLE",   0)
   doors.MODE_CH  = add(2,  "MODE_CH",  0)
   doors.MODE1    = add(3,  "MODE1",    MODE.MANUAL)
   doors.MODE2    = add(4,  "MODE2",    MODE.FLY_BY_WIRE_A)
   doors.MODE3    = add(5,  "MODE3",    MODE.CRUISE)
   doors.MODE4    = add(6,  "MODE4",    MODE.QLOITER)
   doors.MODE5    = add(7,  "MODE5",    MODE.QHOVER)
   doors.MODE6    = add(8,  "MODE6",    MODE.QRTL)
   doors.FUNC     = add(9,  "FUNC",     94)
   doors.CLOSED   = add(10, "CLOSED",   1000)
   doors.PART     = add(11, "PART",     1500)
   doors.OPEN     = add(12, "OPEN",     2000)
   doors.ENTRY_MS = add(13, "ENTRY_MS", 3000)
   doors.PART_AS  = add(14, "PART_AS",  6)
   doors.FULL_AS  = add(15, "FULL_AS",  10)
   doors.TILT_FN  = add(16, "TILT_FN",  41)
   doors.TILT_FWD = add(17, "TILT_FWD", 1900)
   doors.LIFT_FN  = add(18, "LIFT_FN",  0)
   doors.LIFT_OFF = add(19, "LIFT_OFF", 1050)
   doors.DEBUG    = add(20, "DEBUG",    0)

   doors.mode_params = { doors.MODE1, doors.MODE2, doors.MODE3, doors.MODE4, doors.MODE5, doors.MODE6 }
end

-- door state codes (also published as DOOR_ST telemetry)
local DOOR_ST = { CLOSED = 0, PARTIAL = 1, OPEN = 2, OPENING = 3 }
local DOOR_ST_NAME = { [0] = "CLOSED", [1] = "PARTIAL", [2] = "OPEN", [3] = "OPENING" }

local LATCH_PERSIST_MS = 750         -- a partial<->full change must hold this long (rejects airspeed noise)

-- runtime state
local door_last_pos = nil            -- last acted-on selector position (edge trigger)
local pending_vtol_target = nil      -- VTOL mode awaiting the door-open delay (nil = none)
local pending_from_mode = nil        -- flight mode when the pending entry began (a change cancels it)
local door_entry_start = nil         -- millis() when the doors were commanded open for a pending entry
local vtol_door_partial = false      -- directional-band latch: true when the reduced (partial) opening is selected
local latch_want_since = nil         -- millis() when the opposite band state first appeared (persistence filter)
local vtol_reached_full = false      -- true once the doors have opened fully in the current VTOL session
local door_last_state = nil          -- for edge-triggered gcs:send_text
local door_warned_nonscript = false
local door_tilt_pos = 0              -- last read nacelle-tilt output PWM (0 = no reading), for telemetry
local door_lift_pwm = 0              -- last read dedicated lift-motor output PWM, for telemetry
local door_aspd = nil                -- last airspeed estimate (m/s) this cycle
local dbg_mode = 0                   -- snapshot of current mode for debug output
local dbg_in_vtol = false            -- snapshot of in_vtol for debug output

-- verbose debug message, only when DOOR_DEBUG > 0 (kept short: MAVLink STATUSTEXT is 50 chars)
local function dbg(fmt, ...)
   if doors.DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY_INFO, "F35B " .. string.format(fmt, ...))
   end
end

local function door_selected_mode()
   local ch = math.floor(doors.MODE_CH:get() or 0)
   if ch <= 0 then
      return nil, nil
   end
   -- get_pwm returns the raw channel value without checking the link: 0 before the receiver
   -- connects (= position 1) and receiver-preset values during an RC failsafe. Ignore the
   -- selector until the input is valid.
   if not rc:has_valid_input() then
      return nil, nil
   end
   local pwm = rc:get_pwm(ch)
   if not pwm then
      return nil, nil
   end
   local pos = pwm_to_pos(pwm)
   local target = math.floor(doors.mode_params[pos]:get() or -1)
   return pos, target
end

-- Announce discrete door events once. The routine partial<->full airspeed-driven position
-- changes are NOT sent to the message log (they would spam it) - watch DOOR_ST telemetry
-- for the live position instead.
local function door_announce(state, extra)
   if state == door_last_state then
      return
   end
   door_last_state = state
   if state == DOOR_ST.OPENING then
      gcs:send_text(MAV_SEVERITY_INFO, string.format("F35B: doors opening, VTOL in %.1fs", extra or 0))
   elseif state == DOOR_ST.CLOSED then
      gcs:send_text(MAV_SEVERITY_INFO, "F35B: doors closed")
   end
   dbg("%s m=%d v=%s t=%d l=%d as=%.1f", DOOR_ST_NAME[state] or "?",
       dbg_mode, dbg_in_vtol and "Y" or "N", door_tilt_pos, door_lift_pwm, door_aspd or -1)
end

local function door_output(pwm)
   SRV_Channels:set_output_pwm(math.floor(doors.FUNC:get()), math.floor(pwm))
end

-- Door position to use while the VTOL system is in play. The partial<->full airspeed
-- threshold is chosen by transition direction, which lets the two directions use different
-- speeds without the instability an overlapping hysteresis band would cause:
--   * in a VTOL mode (inbound / hover)     -> full below DOOR_FULL_AS, partial above
--   * outbound transition (FW + assisted)  -> full below DOOR_PART_AS, partial above
-- Once the doors have opened fully in a VTOL session (vtol_reached_full, cleared on leaving
-- VTOL) they stay full for the rest of that session, so partial only appears during the
-- initial fast inbound entry and never necks the doors during deliberate fast VTOL flight.
-- A change must persist for LATCH_PERSIST_MS to take effect, which rejects airspeed noise.
-- Falls back to full open when no airspeed estimate is available. Returns pwm, is_partial.
local function vtol_active_door_pwm(in_vtol)
   local now = millis()
   local aspd = door_aspd
   local want
   if aspd == nil then
      want = false
   elseif in_vtol and vtol_reached_full then
      want = false
   else
      local thresh = in_vtol and doors.FULL_AS:get() or doors.PART_AS:get()
      want = aspd >= thresh
   end
   if want == vtol_door_partial then
      latch_want_since = nil
   else
      -- only flip once the new state has persisted long enough (debounce)
      if latch_want_since == nil then
         latch_want_since = now
      end
      if (now - latch_want_since):tofloat() >= LATCH_PERSIST_MS then
         vtol_door_partial = want
         latch_want_since = nil
      end
   end
   -- remember once we are fully open in a VTOL mode (latched full for the session)
   if in_vtol and not vtol_door_partial then
      if not vtol_reached_full then
         dbg("doors latched FULL")
      end
      vtol_reached_full = true
   end
   if vtol_door_partial then
      return doors.PART:get(), true
   end
   return doors.OPEN:get(), false
end

-- returns door_state, door_pwm, vtol_delay_remaining_s
local function doors_update()
   local now = millis()
   local in_vtol = quadplane:in_vtol_mode()
   local assisted = quadplane:in_assisted_flight()
   local armed = arming:is_armed()
   local current = vehicle:get_mode()
   local reason = vehicle:get_control_mode_reason()
   door_aspd = ahrs:airspeed_estimate()
   dbg_mode = current
   dbg_in_vtol = in_vtol

   -- end the "full latched" VTOL session whenever we are not in a VTOL mode
   if not in_vtol then
      if vtol_reached_full then
         dbg("full latch reset")
      end
      vtol_reached_full = false
   end

   -- Watch the actual nacelle tilt output PWM (1000..2000). This is the true "is the VTOL
   -- geometry deployed" signal - it sits at the intermediate angle while waiting for airspeed,
   -- and is independent of Q_ASSIST. "Forward" (doors may close) is the end matching your
   -- firmware: direction is inferred from where DOOR_TILT_FWD sits relative to mid-stick (1500),
   -- so it works whether forward is the high-PWM or the low-PWM end.
   -- DOOR_TILT_FN 0 must not be read: function 0 is held by every unused output, so it would
   -- return a spare channel's PWM, which could look like "forward".
   local tilt_fn = math.floor(doors.TILT_FN:get())
   door_tilt_pos = 0
   if tilt_fn > 0 then
      door_tilt_pos = SRV_Channels:get_output_pwm(tilt_fn) or 0
   end
   local tfwd = doors.TILT_FWD:get()
   local nacelles_fwd
   if door_tilt_pos <= 0 then
      nacelles_fwd = false                       -- no reading (0 or unassigned): fail-safe open
   elseif tfwd >= 1500 then
      nacelles_fwd = door_tilt_pos >= tfwd       -- forward is the high-PWM end
   else
      nacelles_fwd = door_tilt_pos <= tfwd       -- forward is the low-PWM end
   end

   -- Watch a dedicated (non-tilting) lift motor's output so the doors don't close while it is
   -- still powered / spooling down. The per-motor PWM rides the spool-down ramp to minimum;
   -- the group spool state can't isolate one motor on a hybrid. 0 = gate disabled.
   local lift_off = true
   local lift_fn = math.floor(doors.LIFT_FN:get())
   if lift_fn > 0 then
      local lp = SRV_Channels:get_output_pwm(lift_fn)
      door_lift_pwm = lp or 0
      -- if the function isn't assigned (nil) treat the motor as powered (fail-safe: stay open)
      lift_off = (lp ~= nil) and (lp <= doors.LIFT_OFF:get())
   else
      door_lift_pwm = 0
   end

   -- Detect a VTOL entry the script did not command (GCS / failsafe / AUTO leg).
   -- We cannot withhold these, so just open the doors immediately and warn once.
   if in_vtol and reason ~= MODE_REASON_SCRIPTING then
      if not door_warned_nonscript then
         gcs:send_text(MAV_SEVERITY_WARNING, "F35B: VTOL entered externally - opening doors")
         door_warned_nonscript = true
      end
   else
      door_warned_nonscript = false
   end

   local pos, target = door_selected_mode()

   -- Edge-triggered: act ONLY when the selector position changes. Between changes the
   -- script never re-commands the mode, so an override / failsafe / GCS mode change is
   -- respected until the pilot physically moves the selector.
   if pos ~= nil and pos ~= door_last_pos then
      door_last_pos = pos
      pending_vtol_target = nil            -- any selector move cancels a prior pending entry
      dbg("sel pos %d -> mode %d (cur %d)", pos, target or -1, current)
      if current == MODE.MANUAL and reason ~= MODE_REASON_SCRIPTING then
         -- emergency MANUAL override engaged (set outside the script): hand control to the
         -- pilot and ignore the selector until they leave MANUAL (e.g. override back to rest)
         gcs:send_text(MAV_SEVERITY_WARNING, "F35B: MANUAL override active - selector ignored")
      elseif target >= 0 and target ~= current then
         if is_vtol_mode[target] then
            if in_vtol then
               -- already in a VTOL mode: doors are open, switch immediately (no delay)
               vehicle:set_mode(target)
               dbg("VTOL->VTOL %d now", target)
            else
               -- coming from fixed wing: open doors to operating clearance, then enter VTOL
               pending_vtol_target = target
               pending_from_mode = current
               door_entry_start = now
               dbg("inbound %d: opening doors %.1fs", target, doors.ENTRY_MS:get()/1000.0)
            end
         else
            -- fixed-wing target: always safe to switch immediately
            vehicle:set_mode(target)
            dbg("FW %d now", target)
         end
      end
   end

   -- Withheld VTOL entry (inbound FW->VTOL). Driven only by the edge-triggered state above,
   -- so it never fights external changes. The doors open to their operating position (partial
   -- when fast); once they have had DOOR_ENTRY_MS to reach clearance, VTOL is entered - it is
   -- the VTOL mode that then decelerates the aircraft, and the doors open fully later (below)
   -- once airspeed has dropped through DOOR_FULL_AS.
   -- Any mode change while waiting (MANUAL override, failsafe, GCS) cancels the entry, so the
   -- script never switches the aircraft out of a mode it didn't set. The pilot re-selects to
   -- retry.
   if pending_vtol_target ~= nil and current ~= pending_from_mode then
      pending_vtol_target = nil
      gcs:send_text(MAV_SEVERITY_WARNING, "F35B: mode changed, VTOL entry cancelled")
   end
   if pending_vtol_target ~= nil then
      local pwm = vtol_active_door_pwm(in_vtol)
      door_output(pwm)
      local elapsed = (now - door_entry_start):tofloat()
      local entry_ms = doors.ENTRY_MS:get()
      local remaining = math.max(0.0, (entry_ms - elapsed) / 1000.0)
      if elapsed >= entry_ms then
         dbg("entering VTOL %d", pending_vtol_target)
         vehicle:set_mode(pending_vtol_target)
         pending_vtol_target = nil
         gcs:send_text(MAV_SEVERITY_INFO, "F35B: doors open - entering VTOL")
         return DOOR_ST.OPENING, pwm, 0.0
      end
      door_announce(DOOR_ST.OPENING, remaining)
      return DOOR_ST.OPENING, pwm, remaining
   end

   -- VTOL geometry / propulsion active: in a VTOL mode, the nacelles are not yet fully forward
   -- (still slewing / waiting for airspeed on an outbound transition), a dedicated lift motor
   -- is still powered, or an armed assist is active. In that case the doors are open, and
   -- airspeed sets partial vs full. The doors close only once we are in fixed wing AND the
   -- nacelles are forward AND the lift motor is off - all watched directly, so it is correct
   -- regardless of Q_ASSIST and while disarmed.
   if in_vtol or (not nacelles_fwd) or (not lift_off) or (assisted and armed) then
      local pwm, is_partial = vtol_active_door_pwm(in_vtol)
      door_output(pwm)
      if is_partial then
         door_announce(DOOR_ST.PARTIAL)
         return DOOR_ST.PARTIAL, pwm, 0.0
      end
      door_announce(DOOR_ST.OPEN)
      return DOOR_ST.OPEN, pwm, 0.0
   end

   -- Pure fixed wing: transition complete and lift motors stowed - close immediately.
   door_output(doors.CLOSED:get())
   door_announce(DOOR_ST.CLOSED)
   return DOOR_ST.CLOSED, doors.CLOSED:get(), 0.0
end

-------------------------------------------------------------------------------
-- main loop
-------------------------------------------------------------------------------
local TELEM_DECIMATE = 4             -- publish named floats every 4th cycle (~5 Hz)
local telem_count = 0
local DBG_HEARTBEAT_MS = 2000        -- when DOOR_DEBUG > 0, print a status line this often
local dbg_hb_ms = uint32_t(0)

local function update()
   if doors.ENABLE:get() > 0 then
      local state, pwm, delay = doors_update()
      telem_count = telem_count + 1
      if telem_count >= TELEM_DECIMATE then
         telem_count = 0
         gcs:send_named_float('DOOR_ST',  state)
         gcs:send_named_float('DOOR_PWM', pwm)
         gcs:send_named_float('VTOL_DLY', delay)
         gcs:send_named_float('TILT_POS', door_tilt_pos)
         gcs:send_named_float('LIFT_PWM', door_lift_pwm)
      end
      -- text heartbeat so the signals are visible in the GCS Messages window (not just as
      -- NAMED_VALUE_FLOAT telemetry, which some GCSes don't surface). Debug only.
      if doors.DEBUG:get() > 0 and (millis() - dbg_hb_ms):tofloat() >= DBG_HEARTBEAT_MS then
         dbg_hb_ms = millis()
         gcs:send_text(MAV_SEVERITY_INFO, string.format(
            "F35B hb st=%d t=%d l=%d as=%.1f", state, door_tilt_pos, door_lift_pwm, door_aspd or -1))
      end
   end
end

-- After a runtime error, open the doors if VTOL hardware may be in use, so a lift motor
-- never spins against doors frozen closed. Protected itself: the error may come from a
-- binding this calls.
local function error_fallback()
   if doors.ENABLE:get() > 0 and (quadplane:in_vtol_mode() or quadplane:in_assisted_flight()) then
      door_output(doors.OPEN:get())
   end
end

-- A runtime error would otherwise stop the script and freeze the door output. Report it,
-- apply the fallback, and keep retrying at a slower rate.
local function protected_wrapper()
   local ok, err = pcall(update)
   if not ok then
      gcs:send_text(MAV_SEVERITY_ERROR, "F35B: Internal Error: " .. tostring(err))
      pcall(error_fallback)
      return protected_wrapper, ERROR_RETRY_MS
   end
   return protected_wrapper, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "F35B: script loaded")
return protected_wrapper()
