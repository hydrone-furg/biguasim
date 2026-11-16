#include "AP_MotorsHydrone.h"

#include <SRV_Channel/SRV_Channel.h>

// water thrusters: motor index (0-based) of MOT5..8
static constexpr uint8_t WATER_FRONT = 4;   // vertical, front: + pushes down
static constexpr uint8_t WATER_RIGHT = 5;   // forward, right: + pushes forward
static constexpr uint8_t WATER_REAR = 6;    // vertical, rear: + pushes down
static constexpr uint8_t WATER_LEFT = 7;    // forward, left: + pushes forward

void AP_MotorsHydrone::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // the air rotors: the configured Copter matrix on MOT1..4
    AP_MotorsMatrix::init(frame_class, frame_type);

    // the water thrusters: outputs only, outside the air matrix (motor_enabled stays false, so Copter's mixer,
    // spool-up checks and motor test leave them alone)
    for (uint8_t i = WATER_FRONT; i <= WATER_LEFT; i++) {
        add_motor_num(i);
    }
    for (uint8_t i = 0; i < 4; i++) {
        _water_out[i] = 0.0f;
    }
    _override_heave = _override_pitch = _override_yaw = 0.0f;
}

void AP_MotorsHydrone::set_water_override(float heave, float pitch, float yaw)
{
    _override_heave = constrain_float(heave, -1.0f, 1.0f);
    _override_pitch = constrain_float(pitch, -1.0f, 1.0f);
    _override_yaw = constrain_float(yaw, -1.0f, 1.0f);
}

float AP_MotorsHydrone::get_throttle_hover() const
{
    if (_medium == Medium::WATER) {
        return 0.5f;
    }
    return AP_MotorsMatrix::get_throttle_hover();
}

uint32_t AP_MotorsHydrone::get_motor_mask()
{
    uint32_t mask = 0;
    for (uint8_t i = WATER_FRONT; i <= WATER_LEFT; i++) {
        mask |= 1U << i;
    }
    return AP_MotorsMatrix::get_motor_mask() | motor_mask_to_srv_channel_mask(mask);
}

// heave: up positive; pitch: nose up positive; yaw: nose right positive; forward: forward positive (all -1..1).
// Pitch has priority over heave on the vertical pair, yaw over forward on the forward pair.
void AP_MotorsHydrone::output_water_mix(float heave, float pitch, float yaw, float forward, bool set_limits)
{
    pitch = constrain_float(pitch, -1.0f, 1.0f);
    yaw = constrain_float(yaw, -1.0f, 1.0f);
    heave = constrain_float(heave, -1.0f, 1.0f);
    forward = constrain_float(forward, -1.0f, 1.0f);

    // vertical pair: nose up = front pushes up (negative), rear pushes down (positive); heave up = both negative
    const float heave_room = 1.0f - fabsf(pitch);
    const float heave_out = constrain_float(heave, -heave_room, heave_room);
    _water_out[WATER_FRONT - 4] = -heave_out - pitch;
    _water_out[WATER_REAR - 4] = -heave_out + pitch;

    // forward pair: nose right = left pushes forward more than right
    const float forward_room = 1.0f - fabsf(yaw);
    const float forward_out = constrain_float(forward, -forward_room, forward_room);
    _water_out[WATER_RIGHT - 4] = forward_out - yaw;
    _water_out[WATER_LEFT - 4] = forward_out + yaw;

    if (set_limits) {
        limit.roll = true;      // roll is not actuated in the water
        limit.pitch = fabsf(pitch) >= 1.0f;
        limit.yaw = fabsf(yaw) >= 1.0f;
        limit.throttle_lower = heave_out <= -heave_room && heave < 0.0f;
        limit.throttle_upper = heave_out >= heave_room && heave > 0.0f;
    }
}

void AP_MotorsHydrone::output_armed_stabilizing()
{
    switch (_medium) {
    case Medium::AIR:
        AP_MotorsMatrix::output_armed_stabilizing();
        output_water_mix(0.0f, 0.0f, 0.0f, 0.0f, false);
        break;

    case Medium::WATER: {
        // ArduSub's inputs: throttle 0.5 is neutral heave
        output_water_mix(get_throttle_bidirectional(), _pitch_in + _pitch_in_ff, _yaw_in + _yaw_in_ff, _forward_in,
                         true);
        // air rotors idle (they push nothing in the water)
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = 0.0f;
            }
        }
        break;
    }

    case Medium::BREACH:
        AP_MotorsMatrix::output_armed_stabilizing();
        output_water_mix(_override_heave, _override_pitch, _override_yaw, 0.0f, false);
        break;
    }
}

void AP_MotorsHydrone::output_to_motors()
{
    // air rotors: Copter's output (spool states, thrust linearisation, slew)
    AP_MotorsMatrix::output_to_motors();

    // water thrusters: neutral unless the motors are running
    const bool running = _spool_state == SpoolState::SPOOLING_UP || _spool_state == SpoolState::THROTTLE_UNLIMITED ||
                         _spool_state == SpoolState::SPOOLING_DOWN;
    const float neutral = 0.5f * (get_pwm_output_min() + get_pwm_output_max());
    for (uint8_t i = WATER_FRONT; i <= WATER_LEFT; i++) {
        const float out = running ? _water_out[i - 4] : 0.0f;
        rc_write(i, (uint16_t)(neutral + WATER_PWM_SPAN * out));
    }
}
