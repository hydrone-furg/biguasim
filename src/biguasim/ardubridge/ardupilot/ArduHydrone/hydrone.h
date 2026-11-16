#pragma once

/*
  ArduHydrone: the water side and the medium manager (ArduCopter + ArduSub fusion).

  The Copter host flies the air side unchanged. Here:
  - the medium manager picks AIR / WATER / BREACH from the height above the water surface (HYD_SURF_AMSL) and the
    guided target, and tells the motors (AP_MotorsHydrone) which set to drive;
  - in WATER, ArduSub's attitude controller (ATW_) and a second position controller (PSW) hold attitude and depth,
    and a nonholonomic guidance (turn, then surge forward or backward: the Hydrone has no sway thruster and no roll
    actuator under water) drives toward the target;
  - BREACH: rising through the surface level (rolled more than HYD_BRCH_ANG, it first floats there, where the
    hull rights the roll picked up under water, then dives HYD_BRCH_RUN to cross it rising again); then, the water thrusters push up at full heave while the air rotors run at a
    fixed throttle under Copter's attitude control; once clear of the water (HYD_AIR_H), Copter takes over;
  - a dive from the air: under HYD_DIVE_H with a target under water, the air rotors stop and the water stack holds.
*/

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>

class Hydrone {
public:
    enum class Medium : uint8_t {
        AIR = 0,
        WATER = 1,
        BREACH = 2,
    };

    // in WATER with a target in the air
    enum class BreachPrep : uint8_t {
        RISE,       // climbing; at BRCH_H: BREACH if level (BRCH_ANG), else LEVEL
        LEVEL,      // floating free at the surface while the hull rights the roll (BRCH_WAIT at most)
        RUNUP,      // down to BRCH_RUN, then RISE again
    };

    Hydrone();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 enable;          // HYD_ENABLE
    AP_Float surf_amsl;      // HYD_SURF_AMSL: water surface altitude, m AMSL
    AP_Float breach_h;       // HYD_BRCH_H: height (m, center of mass over the surface) where a breach starts
    AP_Float air_h;          // HYD_AIR_H: height where a breach hands over to the air side
    AP_Float dive_h;         // HYD_DIVE_H: height under which a dive (target under water) stops the air rotors
    AP_Float sub_h;          // HYD_SUB_H: height under which the vehicle counts as under water whatever the target
    AP_Float breach_thr;     // HYD_BRCH_THR: air throttle during the breach
    AP_Float breach_heave;   // HYD_BRCH_HV: water heave during the breach (0..1, up)
    AP_Float breach_tout;    // HYD_BRCH_TOUT: breach timeout, s (back to WATER)
    AP_Float w_min_depth;    // HYD_W_MIN_D: shallowest depth the water stack holds, m
    AP_Float w_spd;          // HYD_W_SPD: surge speed limit, m/s
    AP_Float w_acc;          // HYD_W_ACC: surge acceleration limit, m/s/s
    AP_Float w_pos_p;        // HYD_W_POS_P: along-track position gain, 1/s
    AP_Float w_hdg_err;      // HYD_W_HDG_ERR: heading error (deg) beyond which there is no surge
    AP_Float w_radius;       // HYD_W_RAD: approach radius, m: inside it the vehicle may back up toward the target
    AP_Float w_spd_z;        // HYD_W_SPD_Z: climb and descent speed, m/s
    AP_Float w_acc_z;        // HYD_W_ACC_Z: vertical acceleration, m/s/s
    AP_Float w_vel_ff;       // HYD_W_VEL_FF: surge feed forward, output per (m/s)^2 (water drag)
    AC_PID w_vel_pid;        // HYD_WV_: surge speed PID, output per m/s
    AP_Float breach_angle;   // HYD_BRCH_ANG: roll and pitch (deg) to wait for at the surface before a breach
    AP_Float breach_wait;    // HYD_BRCH_WAIT: longest wait at the surface for that, s
    AP_Float breach_runup_depth; // HYD_BRCH_RUN: run-up depth after levelling, m

    // state
    Medium medium = Medium::AIR;
    bool medium_valid;               // medium set from a valid height once
    float height_m;                  // center of mass over the water surface, m
    uint32_t breach_start_ms;
    BreachPrep breach_prep = BreachPrep::RISE;
    uint32_t surface_since_ms;       // LEVEL since
    uint32_t log_counter;

    // the last position target (from GUIDED), NED from the EKF origin
    bool has_target;
    Vector3p target_ned_m;
    Location target_loc;
    bool target_is_loc;

    // water guidance
    Vector3p hold_ned_m;             // position held when there is no target
    float yaw_target_rad;
    float speed_target_ms;           // shaped surge speed target
    bool holding;                    // within HYDRONE_HOLD_RADIUS_M of the target: heading frozen
};
