-- Hydrone water mode for ArduPlane (QuadPlane): drives the four water thrusters from the VTOL controller.
--
-- In the air the QuadPlane flies on its four air rotors (SERVO1..4) as usual. Under water those rotors give no thrust;
-- this script then turns the VTOL attitude and position controllers' demands (AP_Motors: throttle, pitch, yaw, and the
-- forward throttle of Q_FWD_THR_USE) into the water thrusters on SERVO5..8:
--   SERVO5 front vertical (+: pushes down), SERVO6 right forward, SERVO7 rear vertical (+: down), SERVO8 left forward.
-- Each axis is scaled so that a demand gives about the acceleration it gives in the air (HYD_K* parameters), so the
-- air-tuned loops keep their gain under water. Roll has no water actuator: it is left to the vehicle.
-- Under water the script also flags the motor limits (roll always, the other axes when the water thrusters
-- saturate) so the controllers' integrators do not wind up on demands nothing carries out: wound up, they threw the
-- vehicle (40 deg of roll, ~9 g of climb on the air rotors) as it broke the surface.
--
-- The medium comes from the altitude against the water surface (HYD_SURF, m AMSL: ArduPilot's home altitude is the
-- simulator's water level), with HYD_HYST of hysteresis. Entering the water it turns on Q_FWD_THR_USE (forward thrust
-- instead of tilt: the horizontal thrusters move the vehicle) and keeps the hover throttle as the zero of the heave.

local PARAM_TABLE_KEY = 72
local PARAM_PREFIX = "HYD_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 8), "could not add the HYD_ parameters")

local function add(index, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, index, name, default), "could not add " .. PARAM_PREFIX .. name)
    return Parameter(PARAM_PREFIX .. name)
end

local SURF  = add(1, "SURF", 0.0)    -- water surface altitude, m AMSL
local HYST  = add(2, "HYST", 0.15)   -- m: under water below SURF - HYST, in the air above SURF + HYST
local KH    = add(3, "KH", 0.45)     -- heave per (throttle - hover) / hover: m g / (two vertical thrusters' thrust)
local KP    = add(4, "KP", 2.1)      -- pitch: air pitch torque per demand / water pitch torque per demand
local KY    = add(5, "KY", 0.53)     -- yaw: air reaction torque per demand / water differential torque per demand
local KF    = add(6, "KF", 1.0)      -- forward thrust per forward throttle
local DEBUG = add(7, "DEBUG", 0)

local CHANNELS = {4, 5, 6, 7}        -- SERVO5..8, 0-based
local NEUTRAL, SPAN = 1500, 400
local K_THROTTLE = 70                -- SRV_Channel::k_throttle (the QuadPlane's forward throttle, -100..100)

local fwd_thr_use = Parameter("Q_FWD_THR_USE")
local hover = Parameter("Q_M_THST_HOVER")
local hover_learn = Parameter("Q_M_HOVER_LEARN")

local in_water = nil
local throttle_zero = 0.05
local air_hover_learn = hover_learn:get()
local last_debug = 0

local function constrain(v, lo, hi) return math.max(lo, math.min(hi, v)) end

local function output(values)
    for i, v in ipairs(values) do
        SRV_Channels:set_output_pwm_chan_timeout(CHANNELS[i], math.floor(NEUTRAL + SPAN * constrain(v, -1, 1) + 0.5), 100)
    end
end

local function set_medium(water)
    if water == in_water then return end
    in_water = water
    if water then
        throttle_zero = math.max(hover:get(), 0.01)
        air_hover_learn = hover_learn:get()
        hover_learn:set(0)           -- the throttle that holds depth is not the air hover throttle
        fwd_thr_use:set(2)
        gcs:send_text(6, "Hydrone: water mode")
    else
        hover_learn:set(air_hover_learn)
        fwd_thr_use:set(0)
        gcs:send_text(6, "Hydrone: air mode")
    end
end

local function update()
    local location = ahrs:get_location()
    if location then
        local depth = SURF:get() - location:alt() * 0.01
        if in_water == nil then
            set_medium(depth > 0)
        elseif depth > HYST:get() then
            set_medium(true)
        elseif depth < -HYST:get() then
            set_medium(false)
        end
    end

    if not in_water or not arming:is_armed() then
        motors:set_external_limits(false, false, false, false, false)
        output({0, 0, 0, 0})
        return update, 1
    end

    local heave = KH:get() * (motors:get_throttle() - throttle_zero) / throttle_zero
    local pitch = KP:get() * motors:get_pitch()
    local yaw = KY:get() * motors:get_yaw()
    local forward = KF:get() * math.max(0, SRV_Channels:get_output_scaled(K_THROTTLE) * 0.01)

    -- front vertical: down force noses down; rear vertical: down force noses up; right forward yaws left
    local thrusters = {-(heave + pitch), forward - yaw, pitch - heave, forward + yaw}
    output(thrusters)
    local vertical_saturated = math.abs(thrusters[1]) >= 1 or math.abs(thrusters[3]) >= 1
    local horizontal_saturated = math.abs(thrusters[2]) >= 1 or math.abs(thrusters[4]) >= 1
    motors:set_external_limits(true, vertical_saturated, horizontal_saturated,
                               vertical_saturated and heave < 0, vertical_saturated and heave > 0)

    if DEBUG:get() > 0 and millis():tofloat() - last_debug > 1000 then
        last_debug = millis():tofloat()
        gcs:send_text(7, string.format("HYD h %.2f p %.2f y %.2f f %.2f", heave, pitch, yaw, forward))
    end
    return update, 1
end

gcs:send_text(6, "Hydrone medium script running")
return update, 1000
