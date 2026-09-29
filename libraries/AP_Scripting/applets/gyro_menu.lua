--[[
  Gyro Setup: a transmitter setup menu for the "gyro" build of ArduPlane.

  Shows up on CRSF/ELRS radios (EdgeTX "ELRS" Lua or TBS Agent Lite) as
  "Gyro Setup" and covers the things a pilot would otherwise need a
  ground station for: level calibration, servo direction and wing type,
  acro rates and bank limits, gain-knob range, trim capture and the
  return-to-launch / link-loss behaviour.

  Requires the crsf_helper module. Both live in ROMFS on gyro boards.

  Calibration commands need the vehicle disarmed (arm switch off) and
  are run by the firmware through the GYRO_CAL parameter.
--]]

local h = require('crsf_helper')
local SEV = h.MAV_SEVERITY
local CMD = h.CRSF_COMMAND_STATUS
local AUX_TRIM_TO_CURRENT_SERVO_RC = 155

local function pget(name)
    return param:get(name) or 0
end

local function pset(name, v)
    if not param:set_and_save(name, v) then
        gcs:send_text(SEV.WARNING, "Gyro: set " .. name .. " failed")
    end
end

-- NUMBER item bound to a parameter
local function num(label, pname, min, max, step, dp, unit)
    return {
        type = 'NUMBER', name = label, min = min, max = max, step = step,
        dpoint = dp or 0, unit = unit, default = pget(pname),
        callback = function(v) pset(pname, v) end,
    }
end

-- SELECTION item bound to a parameter; options[i] writes values[i]
local function sel(label, pname, options, values)
    local cur, idx = pget(pname), 1
    for i, v in ipairs(values) do
        if v == cur then idx = i end
    end
    return {
        type = 'SELECTION', name = label, options = options, default = idx,
        callback = function(opt)
            for i, o in ipairs(options) do
                if o == opt then pset(pname, values[i]) end
            end
        end,
    }
end

-- ---------------------------------------------------------------
-- calibration commands (vehicle must be disarmed)
--
-- The script does not run the calibration itself: calling it from Lua
-- blocks the main loop while it waits for IMU samples. Instead it sets
-- GYRO_CAL and the firmware's gyro module runs the calibration on the
-- main thread, then writes back 0 (done) or -1 (failed).
-- ---------------------------------------------------------------
local CAL_GYROS = 1
local CAL_LEVEL = 2

local function cal_command(request)
    return function(action)
        if action == CMD.START then
            if arming:is_armed() then
                return CMD.READY, "Disarm first"
            end
            param:set("GYRO_CAL", request)
            return CMD.PROGRESS, "Hold still"
        elseif action == CMD.POLL then
            local state = param:get("GYRO_CAL")
            if state == 0 then
                return CMD.READY, "Done"
            elseif state == -1 then
                return CMD.READY, "Failed"
            end
            return CMD.PROGRESS, "Hold still"
        end
        return CMD.READY, "Execute"
    end
end

local function trim_command(action)
    if action == CMD.START then
        if vehicle:get_mode() ~= 0 then
            return CMD.READY, "MANUAL only"
        end
        rc:run_aux_function(AUX_TRIM_TO_CURRENT_SERVO_RC, 2)
        return CMD.READY, "Done"
    end
    return CMD.READY, "Execute"
end

-- ---------------------------------------------------------------
-- wing type: sets the functions of outputs 1-4
-- ---------------------------------------------------------------
local WING_NAMES  = { "Normal", "Elevons", "V-tail", "Custom" }
local WING_FUNCS  = {
    { 4, 19, 70, 21 },   -- aileron, elevator, throttle, rudder
    { 77, 78, 70, 21 },  -- elevon L, elevon R, throttle, rudder
    { 4, 79, 70, 80 },   -- aileron, vtail L, throttle, vtail R
}

local function wing_current()
    for w, funcs in ipairs(WING_FUNCS) do
        local match = true
        for i = 1, 4 do
            if pget(string.format("SERVO%d_FUNCTION", i)) ~= funcs[i] then match = false end
        end
        if match then return w end
    end
    return 4
end

local function wing_item()
    return {
        type = 'SELECTION', name = 'Wing type', options = WING_NAMES, default = wing_current(),
        callback = function(opt)
            for w, name in ipairs(WING_NAMES) do
                if name == opt and WING_FUNCS[w] then
                    for i = 1, 4 do
                        pset(string.format("SERVO%d_FUNCTION", i), WING_FUNCS[w][i])
                    end
                    gcs:send_text(SEV.INFO, "Gyro: wing type " .. name)
                end
            end
        end,
    }
end

local DIR = { "Normal", "Reversed" }
local DIRV = { 0, 1 }

-- ---------------------------------------------------------------
-- menu
-- ---------------------------------------------------------------
local menu = {
    name = 'Gyro Setup',
    items = {
        { type = 'MENU', name = 'Level & cal', items = {
            { type = 'INFO', name = 'Place level, then', info = 'run' },
            { type = 'COMMAND', name = 'Calibrate level', callback = cal_command(CAL_LEVEL) },
            { type = 'COMMAND', name = 'Calibrate gyros', callback = cal_command(CAL_GYROS) },
            { type = 'COMMAND', name = 'Capture trims', callback = trim_command },
        } },
        { type = 'MENU', name = 'Servos', items = {
            wing_item(),
            sel('Output 1', 'SERVO1_REVERSED', DIR, DIRV),
            sel('Output 2', 'SERVO2_REVERSED', DIR, DIRV),
            sel('Output 3', 'SERVO3_REVERSED', DIR, DIRV),
            sel('Output 4', 'SERVO4_REVERSED', DIR, DIRV),
        } },
        { type = 'MENU', name = 'Rates & limits', items = {
            num('Acro roll rate', 'ACRO_ROLL_RATE', 30, 500, 10, 0, 'd/s'),
            num('Acro pitch rate', 'ACRO_PITCH_RATE', 30, 500, 10, 0, 'd/s'),
            num('Bank limit', 'ROLL_LIMIT_DEG', 10, 85, 5, 0, 'deg'),
            num('Pitch up limit', 'PTCH_LIM_MAX_DEG', 5, 60, 5, 0, 'deg'),
            num('Pitch down limit', 'PTCH_LIM_MIN_DEG', -60, -5, 5, 0, 'deg'),
        } },
        { type = 'MENU', name = 'Gyro', items = {
            num('Gain knob min', 'GYRO_GAIN_MIN', 0, 1, 0.05, 2),
            num('Gain knob max', 'GYRO_GAIN_MAX', 1, 3, 0.05, 2),
            sel('Trim capture', 'GYRO_TRIM_CAP', { "Off", "On" }, { 0, 1 }),
            sel('Yaw rate mode', 'YAW_RATE_ENABLE', { "Off", "On" }, { 0, 1 }),
        } },
        { type = 'MENU', name = 'RTL & link loss', items = {
            num('RTL altitude', 'RTL_ALTITUDE', 20, 400, 10, 0, 'm'),
            sel('Link loss', 'FS_LONG_ACTN', { "Level glide", "RTL if home" }, { 2, 1 }),
            sel('RC failsafe', 'THR_FAILSAFE', { "Off", "On" }, { 0, 1 }),
        } },
    },
}

return h.register_menu(menu)
