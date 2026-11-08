-- luacheck: ignore
--[[
   Offline behavioral tests for F35B.lua (DOORS subsystem).

   Stubs the ArduPilot bindings F35B.lua uses, then drives the script's
   (update, delay) scheduler through scenarios and checks the outcome against
   the intended behavior. Run via run.sh, which builds ArduPilot's own Lua.

   usage: aprun harness.lua <path/to/F35B.lua>
   exit status is non-zero if any scenario fails.

   ArduPilot's Lua strips print/loadfile/os, so this uses io.write and load().
--]]

local SCRIPT = arg[1]
local function out(...) io.write(string.format(...), "\n") end

local fh = assert(io.open(SCRIPT, "r"))
local SRC = fh:read("a")
fh:close()

local MODE_NAME = { [0]="MANUAL", [5]="FBWA", [11]="RTL", [17]="QSTABILIZE", [18]="QHOVER", [19]="QLOITER", [20]="QLAND", [21]="QRTL" }
local function mode_name(m) return MODE_NAME[m] or tostring(m) end
local VTOL = { [17]=true, [18]=true, [19]=true, [20]=true, [21]=true, [22]=true, [23]=true, [25]=true }

-- ModeReason values (libraries/AP_Vehicle/ModeReason.h)
local REASON_RC_COMMAND = 1
local REASON_RADIO_FAILSAFE = 5
local REASON_SCRIPTING = 32

local S       -- simulated vehicle state for the current scenario
local update  -- the script's scheduled function
local delay   -- ms until update is due, as returned by the script

-- Fresh vehicle + bindings, then load the applet. Defaults are the
-- F35B_TESTPLAN.param SITL values: 3-position selector on RC8
-- (1000 = QSTABILIZE, 1500 = QLOITER, 2000 = FBWA), tilt forward at the low
-- PWM end, lift motor = motor 3.
local function new_world(overrides)
   S = {
      now = 0, mode = 5, reason = REASON_RC_COMMAND, in_vtol = false, assisted = false,
      armed = true, aspd = 20.0, rc = { [8] = 2000 }, rc_valid = true, aspd_error = false,
      srv = { [41] = 1000, [35] = 1000 },
      set_mode_calls = 0, door_pwm = nil, door_lapse = 0, door_trim = 1500,
      texts = {}, reject = {}, logs = {},
      servo_func = { [8] = 41, [13] = 94 },
      servo_range = { [8] = { 1000, 2000 }, [13] = { 1000, 2000 } },
      P = {
         DOOR_ENABLE=1, DOOR_MODE_CH=8, DOOR_MODE1=17, DOOR_MODE2=17, DOOR_MODE3=19,
         DOOR_MODE4=19, DOOR_MODE5=5, DOOR_MODE6=5, DOOR_FUNC=94, DOOR_CLOSED=1000,
         DOOR_PART=1500, DOOR_OPEN=2000, DOOR_ENTRY_MS=3000, DOOR_PART_AS=5,
         DOOR_FULL_AS=15, DOOR_TILT_FN=41, DOOR_TILT_FWD=1010, DOOR_LIFT_FN=35,
         DOOR_LIFT_OFF=1010, DOOR_DEBUG=0, DOOR_NOZ_MS=3500,
      },
   }
   for k, v in pairs(overrides or {}) do S[k] = v end

   local U = {}
   U.__index = U
   U.__sub = function(a, b) return setmetatable({ v = a.v - b.v }, U) end
   function U:tofloat() return self.v end
   millis = function() return setmetatable({ v = S.now }, U) end
   uint32_t = function(n) return setmetatable({ v = n }, U) end

   param = {
      add_table = function() return true end,
      add_param = function(_, _, _, name, default)
         if S.P["DOOR_" .. name] == nil then S.P["DOOR_" .. name] = default end
         return true
      end,
      get = function(_, name)
         local n, field = name:match("^SERVO(%d+)_(%u+)$")
         if n then
            n = tonumber(n)
            if n > 16 then return nil end
            if field == "FUNCTION" then return S.servo_func[n] or 0 end
            local r = S.servo_range[n] or { 1000, 2000 }
            if field == "MIN" then return r[1] end
            if field == "MAX" then return r[2] end
            return nil
         end
         return S.P[name]
      end,
   }
   Parameter = function(name) return { get = function() return S.P[name] end } end
   quadplane = {
      in_vtol_mode = function() return S.in_vtol end,
      in_assisted_flight = function() return S.assisted end,
   }
   arming = { is_armed = function() return S.armed end }
   ahrs = {
      airspeed_estimate = function()
         if S.aspd_error then error("injected airspeed fault") end
         return S.aspd
      end,
   }
   rc = {
      get_pwm = function(_, ch) return S.rc[ch] end,
      has_valid_input = function() return S.rc_valid end,
   }
   SRV_Channels = {
      get_output_pwm = function(_, fn) return S.srv[fn] end,
      set_output_scaled = function() end,
      -- chan is 0-based; the command lapses to the output's trim after timeout_ms
      set_output_pwm_chan_timeout = function(_, chan, pwm, timeout_ms)
         if S.servo_func[chan + 1] == S.P.DOOR_FUNC then
            S.door_pwm, S.door_lapse = pwm, S.now + timeout_ms
         end
      end,
   }
   gcs = {
      send_text = function(_, _, txt)
         out("    [t=%5.2fs] GCS: %s", S.now / 1000, txt)
         S.texts[#S.texts + 1] = txt
      end,
      send_named_float = function() end,
   }
   -- the same checks as ArduPilot's logger:write binding (lua_bindings.cpp): format, units and
   -- multipliers the same length, one label and one value per field, integer fields given
   -- integral values in range
   local INT_RANGE = { B = { 0, 255 }, M = { 0, 255 }, H = { 0, 65535 }, h = { -32768, 32767 } }
   logger = {
      write = function(_, name, labels, fmt, units, mults, ...)
         local vals = table.pack(...)
         assert(#name <= 4, "log name longer than 4")
         assert(#units == #fmt and #mults == #fmt, "units/multipliers not the length of the format")
         local _, commas = labels:gsub(",", "")
         local nlabels = commas + 1
         assert(nlabels == #fmt, "label count differs from the format")
         assert(vals.n == #fmt, string.format("%d values for a %d-field format", vals.n, #fmt))
         local row = {}
         for i = 1, #fmt do
            local c, v = fmt:sub(i, i), vals[i]
            assert(type(v) == "number", "field " .. i .. " is not a number")
            local range = INT_RANGE[c]
            if range then
               assert(math.tointeger(v) ~= nil, string.format("field %d (%s) not integral: %s", i, c, v))
               assert(v >= range[1] and v <= range[2], string.format("field %d (%s) out of range: %s", i, c, v))
            else
               assert(c == "f", "unhandled format " .. c)
            end
            row[i] = v
         end
         row.t = S.now
         S.logs[#S.logs + 1] = row
      end,
   }
   vehicle = {
      get_mode = function() return S.mode end,
      get_control_mode_reason = function() return S.reason end,
      set_mode = function(_, m)
         S.set_mode_calls = S.set_mode_calls + 1
         if S.reject[m] then
            out("    [t=%5.2fs] script set_mode(%s) refused", S.now / 1000, mode_name(m))
            return false
         end
         out("    [t=%5.2fs] script set_mode(%s)", S.now / 1000, mode_name(m))
         S.mode, S.reason, S.in_vtol = m, REASON_SCRIPTING, VTOL[m] or false
         return true
      end,
   }

   -- the applet's trailing `return protected_wrapper()` runs its first cycle
   update, delay = assert(load(SRC, "@F35B.lua"))()
end

local function run_until(t_ms)
   while S.now + delay <= t_ms do
      S.now = S.now + delay
      update, delay = update()
   end
   S.now = t_ms
end

-- the door output as the servo sees it: the last command until it lapses, then the trim
local function door_out()
   if S.door_pwm ~= nil and S.now < S.door_lapse then return S.door_pwm end
   return S.door_trim
end

-- true if any GCS text so far matches the Lua pattern
local function said(pat)
   for _, t in ipairs(S.texts) do
      if t:find(pat) then return true end
   end
   return false
end

local function external_mode(m, reason, label)
   S.mode, S.reason, S.in_vtol = m, reason, VTOL[m] or false
   out("    [t=%5.2fs] %s -> %s", S.now / 1000, label, mode_name(m))
end

-- A deploy cut short: the tilt output jumps from forward to `deployed`, stays 0.7 s and jumps
-- back. A real nozzle may deploy faster than DOOR_NOZ_MS, so the doors must wait the stow from
-- where the output went (500 us at 1000 us per 3.5 s, minus the 10 us DOOR_TILT_FWD margin:
-- 1.715 s), not just the 0.7 s the output spent away from forward.
local function short_deploy(label, tilt_fwd, fwd, deployed)
   return {
      name = "deploy cut short (" .. label .. "): doors wait the stow from the output",
      run = function()
         new_world({ srv = { [41] = fwd, [35] = 1000 } })
         S.P.DOOR_TILT_FWD = tilt_fwd
         run_until(500)
         local closed_at_start = (door_out() == S.P.DOOR_CLOSED)
         S.srv[41] = deployed
         run_until(1200)
         S.srv[41] = fwd
         run_until(1200 + 1600)
         local still_open = (door_out() ~= S.P.DOOR_CLOSED)
         run_until(1200 + 1800)
         return closed_at_start and still_open and door_out() == S.P.DOOR_CLOSED,
                "closed, open 1.6 s after the output returns, closed by 1.8 s (estimate 1.715 s)"
      end,
   }
end

-- DOOR log rows since t_ms, as tables keyed by label
local function door_logs(t_ms)
   local rows = {}
   for _, r in ipairs(S.logs) do
      if r.t >= (t_ms or 0) then
         rows[#rows + 1] = { t = r.t, St = r[1], Pwm = r[2], Dly = r[3], Tilt = r[4], Noz = r[5],
                             Lift = r[6], AS = r[7], Md = r[8], Fl = r[9] }
      end
   end
   return rows
end

local scenarios = {
   {
      name = "inbound entry waits DOOR_ENTRY_MS, then enters VTOL",
      run = function()
         new_world()
         run_until(1000)
         S.rc[8] = 1500
         run_until(3000)
         local held = (S.mode == 5)
         run_until(6000)
         return held and S.mode == 19, "held FBWA during delay, then QLOITER"
      end,
   },
   {
      name = "MANUAL override during a pending entry is respected",
      run = function()
         new_world()
         run_until(1000)
         S.rc[8] = 1500
         run_until(2000)
         external_mode(0, REASON_RC_COMMAND, "pilot override")
         run_until(6000)
         return S.mode == 0, "final mode MANUAL"
      end,
   },
   {
      name = "failsafe during a pending entry is respected",
      run = function()
         new_world()
         run_until(1000)
         S.rc[8] = 1500
         run_until(2000)
         external_mode(11, REASON_RADIO_FAILSAFE, "RC failsafe")
         run_until(6000)
         return S.mode == 11, "final mode RTL"
      end,
   },
   {
      name = "boot before the receiver connects commands no mode",
      run = function()
         -- before RC arrives the selector channel reads 0us (= position 1)
         new_world({ armed = false, aspd = 0, rc = { [8] = 0 }, rc_valid = false })
         run_until(5000)
         return S.set_mode_calls == 0, "no set_mode before valid RC"
      end,
   },
   {
      name = "DOOR_TILT_FN 0 never reads as nacelles forward",
      run = function()
         -- every unused output holds function 0; this spare one sits at the forward end
         new_world()
         S.P.DOOR_TILT_FN = 0
         S.srv[0] = 1000
         run_until(3000)
         return door_out() ~= S.P.DOOR_CLOSED, "doors not closed"
      end,
   },
   {
      name = "runtime error with VTOL in play opens doors and keeps running",
      run = function()
         new_world()
         run_until(2000)
         local was_closed = (door_out() == S.P.DOOR_CLOSED)
         external_mode(18, REASON_RC_COMMAND, "GCS")
         S.aspd_error = true
         out("    [t=%5.2fs] airspeed binding starts faulting", S.now / 1000)
         run_until(4000)
         local opened = (door_out() == S.P.DOOR_OPEN)
         S.aspd_error = false
         S.aspd = 3.0
         run_until(6000)
         return was_closed and opened and door_out() == S.P.DOOR_OPEN,
                "doors closed -> open on fault, still open after recovery"
      end,
   },
   {
      name = "switch to MANUAL from hover: doors wait for the nozzle estimate",
      run = function()
         new_world({ mode = 18, in_vtol = true, aspd = 0, srv = { [41] = 1800, [35] = 1500 } })
         run_until(2000)
         local open_in_hover = (door_out() == S.P.DOOR_OPEN)
         -- MANUAL: take the tilt output straight to forward (faster than the nozzle can follow)
         -- and stop the lift motor. The nozzle needs 790/1000 of DOOR_NOZ_MS to get home.
         external_mode(0, REASON_RC_COMMAND, "pilot override")
         S.srv[41], S.srv[35] = 1000, 1000
         run_until(2000 + 2600)
         local still_open = (door_out() ~= S.P.DOOR_CLOSED)
         run_until(2000 + 2900)
         return open_in_hover and still_open and door_out() == S.P.DOOR_CLOSED,
                "open in hover, still open 2.6 s after the jump, closed by 2.9 s (estimate 2.77 s)"
      end,
   },
   {
      name = "slew no faster than the nozzle: doors close as soon as the output is forward",
      run = function()
         new_world({ mode = 5, aspd = 20, srv = { [41] = 1400, [35] = 1000 } })
         run_until(500)
         local open_at_start = (door_out() ~= S.P.DOOR_CLOSED)
         -- 200 us/s, slower than the nozzle's 1000 us per 3.5 s
         local t, closed_at, fwd_at = 500, nil, nil
         while t < 4000 do
            t = t + 50
            S.srv[41] = math.max(1000, S.srv[41] - 10)
            run_until(t)
            if not fwd_at and S.srv[41] <= S.P.DOOR_TILT_FWD then fwd_at = t end
            if not closed_at and door_out() == S.P.DOOR_CLOSED then closed_at = t end
         end
         return open_at_start and fwd_at and closed_at and closed_at - fwd_at <= 100,
                "closed within 0.1 s of the output reaching forward"
      end,
   },
   short_deploy("forward at the low PWM end", 1010, 1000, 1500),
   short_deploy("forward at the high PWM end", 1990, 2000, 1500),
   {
      name = "boot on the ground: doors wait a full nozzle stroke",
      run = function()
         -- outputs already stowed, but the nozzle could be anywhere after power-up
         new_world({ mode = 0, armed = false, aspd = 0 })
         run_until(3300)
         local still_open = (door_out() ~= S.P.DOOR_CLOSED)
         run_until(3600)
         return still_open and door_out() == S.P.DOOR_CLOSED, "open at 3.3 s, closed by 3.6 s (estimate 3.47 s)"
      end,
   },
   {
      name = "restart in flight: doors stay closed",
      run = function()
         -- armed: the nozzle is taken to be where the output says, so no opening at cruise
         new_world()
         run_until(100)
         return door_out() == S.P.DOOR_CLOSED, "closed from the first cycle"
      end,
   },
   {
      name = "DOOR_ENABLE 0 then 1: the estimate kept up while disabled",
      run = function()
         -- disabled in a hover; much later a MANUAL bailout, then re-enabled at once
         new_world({ mode = 18, in_vtol = true, aspd = 0, srv = { [41] = 1800, [35] = 1500 } })
         run_until(2000)
         S.P.DOOR_ENABLE = 0
         run_until(6000)
         external_mode(0, REASON_RC_COMMAND, "pilot override")
         S.srv[41], S.srv[35] = 1000, 1000
         run_until(6300)
         S.P.DOOR_ENABLE = 1
         out("    [t=%5.2fs] DOOR_ENABLE 1", S.now / 1000)
         run_until(6000 + 2600)
         local still_open = (door_out() ~= S.P.DOOR_CLOSED)
         run_until(6000 + 2900)
         return still_open and door_out() == S.P.DOOR_CLOSED,
                "open 2.6 s after the stow (re-enabled at 0.3 s), closed by 2.9 s (estimate 2.77 s)"
      end,
   },
   {
      name = "DOOR_ENABLE 0 then 1: a selector move made while disabled is not acted on",
      run = function()
         new_world()
         run_until(1000)
         S.P.DOOR_ENABLE = 0
         run_until(1500)
         S.rc[8] = 1500
         out("    [t=%5.2fs] selector -> QLOITER while disabled", S.now / 1000)
         run_until(3000)
         S.P.DOOR_ENABLE = 1
         out("    [t=%5.2fs] DOOR_ENABLE 1", S.now / 1000)
         run_until(9000)
         return S.mode == 5 and S.set_mode_calls == 0 and door_out() == S.P.DOOR_CLOSED,
                "stayed FBWA, no set_mode, doors closed"
      end,
   },
   {
      name = "a rejected VTOL mode is reported and the doors close",
      run = function()
         new_world()
         S.reject[19] = true
         run_until(1000)
         S.rc[8] = 1500
         run_until(6000)
         return S.mode == 5 and said("mode 19 rejected") and not said("entering VTOL")
                and door_out() == S.P.DOOR_CLOSED,
                "FBWA, 'mode 19 rejected', no 'entering VTOL', doors closed"
      end,
   },
   {
      name = "external VTOL entry warns from fixed wing only",
      run = function()
         new_world({ mode = 19, reason = REASON_SCRIPTING, in_vtol = true, aspd = 0,
                     rc = { [8] = 1500 }, srv = { [41] = 1800, [35] = 1500 } })
         run_until(1000)
         external_mode(20, REASON_RADIO_FAILSAFE, "failsafe")
         run_until(2000)
         local quiet = not said("entered externally")
         new_world()
         run_until(1000)
         external_mode(21, REASON_RADIO_FAILSAFE, "failsafe")
         run_until(2000)
         return quiet and said("entered externally"),
                "no warning QLOITER -> QLAND; warning FBWA -> QRTL"
      end,
   },
   {
      name = "doors fail to MID if the script stops",
      run = function()
         new_world()
         run_until(2000)
         local was_closed = (door_out() == S.P.DOOR_CLOSED)
         S.now = S.now + 1000                   -- no more script cycles
         return was_closed and door_out() == S.door_trim and S.door_trim == S.P.DOOR_PART,
                "closed, then the output lapses to the trim (MID)"
      end,
   },
   {
      name = "DOOR log: 10 Hz, and its fields follow a gated entry",
      run = function()
         new_world()
         run_until(1000)
         S.rc[8] = 1500
         run_until(2000)
         local mid = door_logs(1900)          -- 1 s into the 3 s gate
         run_until(5000)
         local n = #door_logs(4000)           -- QLOITER, doors open
         local last = door_logs(4900)
         local m, l = mid[#mid], last[#last]
         -- gate: OPENING (partial at 20 m/s), FBWA, entry pending, about 2 s to go; then still
         -- partial in QLOITER (20 m/s is above DOOR_FULL_AS)
         return n >= 9 and n <= 11
            and m.St == 3 and m.Md == 5 and m.Fl & 64 ~= 0 and m.Dly > 1.8 and m.Dly < 2.2
            and m.Pwm == S.P.DOOR_PART and m.Tilt == 1000 and m.AS == 20.0
            and l.St == 1 and l.Md == 19 and l.Fl & 1 ~= 0 and l.Fl & 64 == 0,
            string.format("about 10 rows per second (got %d); OPENING/FBWA/pending mid-gate, then QLOITER", n)
      end,
   },
   {
      name = "observer only: no selector, no door output, decisions still logged",
      run = function()
         -- DOOR_MODE_CH 0 and nothing assigned to DOOR_FUNC, on an aircraft flown with its own
         -- mode switch: the script must never change the mode or drive an output
         new_world({ servo_func = { [8] = 41 } })
         S.P.DOOR_MODE_CH = 0
         run_until(1000)
         local closed = door_logs(500)
         external_mode(19, REASON_RC_COMMAND, "pilot")
         S.srv[41], S.srv[35] = 1800, 1700
         run_until(3000)
         local hover = door_logs(2500)
         external_mode(5, REASON_RC_COMMAND, "pilot")
         S.srv[41], S.srv[35] = 1000, 1000     -- MANUAL-fast stow: the estimate trails 2.8 s
         run_until(4000)
         local waiting = door_logs(3900)
         run_until(7000)
         local after = door_logs(6900)
         local c, h, w, a = closed[#closed], hover[#hover], waiting[#waiting], after[#after]
         return S.set_mode_calls == 0 and S.door_pwm == nil
            and c.St == 0 and h.St == 2 and h.Lift == 1700 and h.Noz == 1800
            and w.St ~= 0 and w.Fl & 8 ~= 0 and w.Fl & 32 == 0 and a.St == 0 and a.Fl & 32 ~= 0,
            "no set_mode, no door output; closed, open in the hover, held for the nozzle, closed"
      end,
   },
}

local failed = 0
for i, sc in ipairs(scenarios) do
   out("[%d] %s", i, sc.name)
   local ran, ok, expect = pcall(sc.run)
   if not ran then
      out("    script crashed: %s", tostring(ok))
      ok, expect = false, "no crash"
   end
   out("    %s (expected: %s; got %s)", ok and "PASS" or "FAIL", expect, mode_name(S.mode))
   if not ok then failed = failed + 1 end
end
out("%d/%d scenarios passed", #scenarios - failed, #scenarios)
-- ArduPilot's Lua has no `os`; raising an error makes the runner exit non-zero
if failed > 0 then error(string.format("%d scenario(s) failed", failed), 0) end
