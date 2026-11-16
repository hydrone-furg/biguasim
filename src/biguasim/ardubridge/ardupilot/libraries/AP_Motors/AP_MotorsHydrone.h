/// @file	AP_MotorsHydrone.h
/// @brief	Motors for the Hydrone, a hybrid aerial-underwater quad: four air rotors and four reversible water thrusters.
///
/// MOT1..4: the air rotors, a Copter quad matrix (FRAME_CLASS/FRAME_TYPE), push only out of the water.
/// MOT5..8: the water thrusters, push only in it, bidirectional (neutral in the middle of the PWM range):
///   MOT5 front vertical, MOT6 right forward, MOT7 rear vertical, MOT8 left forward. A positive output pushes the
///   vertical ones down and the forward ones forward. In the water they carry heave, pitch, yaw and surge; roll and
///   sway are not actuated.
///
/// The vehicle picks which set the roll/pitch/yaw/throttle/forward inputs drive (set_medium):
///   AIR:    the air matrix only (Copter's mixer), water thrusters at neutral.
///   WATER:  the water mix only, ArduSub style (throttle 0.5 is neutral heave), air rotors at idle.
///   BREACH: the air matrix from the inputs, plus the water thrusters from set_water_override (heave, pitch, yaw),
///           while the hull crosses the surface.
#pragma once

#include "AP_MotorsMatrix.h"

class AP_MotorsHydrone : public AP_MotorsMatrix {
public:
    using AP_MotorsMatrix::AP_MotorsMatrix;

    enum class Medium : uint8_t {
        AIR = 0,
        WATER = 1,
        BREACH = 2,
    };

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void set_medium(Medium medium) { _medium = medium; }
    Medium get_medium() const { return _medium; }

    // water thruster demands (-1..1) used in BREACH: heave up positive, pitch nose up positive, yaw nose right positive
    void set_water_override(float heave, float pitch, float yaw);

    // ArduSub's convention in the water: throttle 0.5 is zero heave
    float get_throttle_hover() const override;

    // the water thrusters' last outputs (-1..1), for logging
    float get_water_output(uint8_t i) const { return i < 4 ? _water_out[i] : 0.0f; }

    uint32_t get_motor_mask() override;

    // PWM half-range of the water thrusters around the neutral (MOT_PWM_MIN + MOT_PWM_MAX) / 2
    static constexpr float WATER_PWM_SPAN = 400.0f;

protected:
    void output_armed_stabilizing() override;
    void output_to_motors() override;

private:
    void output_water_mix(float heave, float pitch, float yaw, float forward, bool set_limits);

    Medium _medium = Medium::AIR;
    float _water_out[4];             // MOT5..8 thrust demands, -1..1
    float _override_heave, _override_pitch, _override_yaw;
};
