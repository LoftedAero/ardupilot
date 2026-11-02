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

local MODE_NAME = { [0]="MANUAL", [5]="FBWA", [11]="RTL", [17]="QSTABILIZE", [18]="QHOVER", [19]="QLOITER" }
local function mode_name(m) return MODE_NAME[m] or tostring(m) end
local VTOL = { [17]=true, [18]=true, [19]=true, [20]=true, [21]=true, [22]=true, [23]=true, [25]=true }

-- ModeReason values (libraries/AP_Vehicle/ModeReason.h)
local REASON_RC_COMMAND = 1
local REASON_RADIO_FAILSAFE = 5
local REASON_SCRIPTING = 32

local S       -- simulated vehicle state for the current scenario
local update  -- the script's scheduled function

-- Fresh vehicle + bindings, then load the applet. Defaults are the
-- F35B_TESTPLAN.param SITL values: 3-position selector on RC8
-- (1000 = QSTABILIZE, 1500 = QLOITER, 2000 = FBWA), tilt forward at the low
-- PWM end, lift motor = motor 3.
local function new_world(overrides)
   S = {
      now = 0, mode = 5, reason = REASON_RC_COMMAND, in_vtol = false, assisted = false,
      armed = true, aspd = 20.0, rc = { [8] = 2000 },
      srv = { [41] = 1000, [35] = 1000 },
      set_mode_calls = 0, door_pwm = nil,
      P = {
         DOOR_ENABLE=1, DOOR_MODE_CH=8, DOOR_MODE1=17, DOOR_MODE2=17, DOOR_MODE3=19,
         DOOR_MODE4=19, DOOR_MODE5=5, DOOR_MODE6=5, DOOR_FUNC=94, DOOR_CLOSED=1000,
         DOOR_PART=1500, DOOR_OPEN=2000, DOOR_ENTRY_MS=3000, DOOR_PART_AS=5,
         DOOR_FULL_AS=15, DOOR_TILT_FN=41, DOOR_TILT_FWD=1010, DOOR_LIFT_FN=35,
         DOOR_LIFT_OFF=1010, DOOR_DEBUG=0,
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
   }
   Parameter = function(name) return { get = function() return S.P[name] end } end
   quadplane = {
      in_vtol_mode = function() return S.in_vtol end,
      in_assisted_flight = function() return S.assisted end,
   }
   arming = { is_armed = function() return S.armed end }
   ahrs = { airspeed_estimate = function() return S.aspd end }
   rc = { get_pwm = function(_, ch) return S.rc[ch] end }
   SRV_Channels = {
      get_output_pwm = function(_, fn) return S.srv[fn] end,
      set_output_pwm = function(_, fn, pwm) if fn == S.P.DOOR_FUNC then S.door_pwm = pwm end end,
   }
   gcs = {
      send_text = function(_, _, txt) out("    [t=%5.2fs] GCS: %s", S.now / 1000, txt) end,
      send_named_float = function() end,
   }
   vehicle = {
      get_mode = function() return S.mode end,
      get_control_mode_reason = function() return S.reason end,
      set_mode = function(_, m)
         S.set_mode_calls = S.set_mode_calls + 1
         out("    [t=%5.2fs] script set_mode(%s)", S.now / 1000, mode_name(m))
         S.mode, S.reason, S.in_vtol = m, REASON_SCRIPTING, VTOL[m] or false
         return true
      end,
   }

   -- the applet's trailing `return update()` runs its first cycle
   update = assert(load(SRC, "@F35B.lua"))()
end

local function run_until(t_ms)
   while S.now < t_ms do
      S.now = S.now + 50
      update = update()
   end
end

local function external_mode(m, reason, label)
   S.mode, S.reason, S.in_vtol = m, reason, VTOL[m] or false
   out("    [t=%5.2fs] %s -> %s", S.now / 1000, label, mode_name(m))
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
         new_world({ armed = false, aspd = 0, rc = { [8] = 0 } })
         run_until(5000)
         return S.set_mode_calls == 0, "no set_mode before valid RC"
      end,
   },
}

local failed = 0
for i, sc in ipairs(scenarios) do
   out("[%d] %s", i, sc.name)
   local ok, expect = sc.run()
   out("    %s (expected: %s; got %s)", ok and "PASS" or "FAIL", expect, mode_name(S.mode))
   if not ok then failed = failed + 1 end
end
out("%d/%d scenarios passed", #scenarios - failed, #scenarios)
-- ArduPilot's Lua has no `os`; raising an error makes the runner exit non-zero
if failed > 0 then error(string.format("%d scenario(s) failed", failed), 0) end
