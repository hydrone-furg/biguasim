#include "Copter.h"

/*
  ArduHydrone: medium manager and water side (see hydrone.h).
*/

// under water, within this distance of the target the heading is held and only surge acts (until beyond W_RAD)
static constexpr float HYDRONE_HOLD_RADIUS_M = 0.3f;
// rising to a breach, the depth target is above the surface, so that the vehicle crosses it at climb speed
static constexpr float HYDRONE_BREACH_RISE_HEIGHT_M = 0.3f;

const AP_Param::GroupInfo Hydrone::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Hydrone water side enable
    // @Description: 1: the medium manager and the water stack run; 0: air only (plain ArduCopter)
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, Hydrone, enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: SURF_AMSL
    // @DisplayName: Water surface altitude
    // @Units: m
    AP_GROUPINFO("SURF_AMSL", 2, Hydrone, surf_amsl, 0.0f),

    // @Param: BRCH_H
    // @DisplayName: Breach start height
    // @Description: Height of the center of mass over the water surface where a breach (water to air) starts
    // @Units: m
    AP_GROUPINFO("BRCH_H", 3, Hydrone, breach_h, -0.3f),

    // @Param: AIR_H
    // @DisplayName: Air handover height
    // @Description: Height over the water surface where a breach hands over to the air side
    // @Units: m
    AP_GROUPINFO("AIR_H", 4, Hydrone, air_h, 0.4f),

    // @Param: DIVE_H
    // @DisplayName: Dive height
    // @Description: Height over the water surface under which, with a target under water, the air rotors stop and the water side takes over
    // @Units: m
    AP_GROUPINFO("DIVE_H", 5, Hydrone, dive_h, 0.3f),

    // @Param: SUB_H
    // @DisplayName: Submerged height
    // @Description: Height over the water surface under which the vehicle is under water whatever its target
    // @Units: m
    AP_GROUPINFO("SUB_H", 6, Hydrone, sub_h, -0.15f),

    // @Param: BRCH_THR
    // @DisplayName: Breach air throttle
    // @Range: 0 1
    AP_GROUPINFO("BRCH_THR", 7, Hydrone, breach_thr, 0.1f),

    // @Param: BRCH_HV
    // @DisplayName: Breach water heave
    // @Range: 0 1
    AP_GROUPINFO("BRCH_HV", 8, Hydrone, breach_heave, 1.0f),

    // @Param: BRCH_TOUT
    // @DisplayName: Breach timeout
    // @Units: s
    AP_GROUPINFO("BRCH_TOUT", 9, Hydrone, breach_tout, 5.0f),

    // @Param: W_MIN_D
    // @DisplayName: Minimum water depth
    // @Description: Shallowest depth of the center of mass the water side holds
    // @Units: m
    AP_GROUPINFO("W_MIN_D", 10, Hydrone, w_min_depth, 0.3f),

    // @Param: W_SPD
    // @DisplayName: Water surge speed
    // @Units: m/s
    AP_GROUPINFO("W_SPD", 11, Hydrone, w_spd, 1.0f),

    // @Param: W_ACC
    // @DisplayName: Water surge acceleration
    // @Units: m/s/s
    AP_GROUPINFO("W_ACC", 12, Hydrone, w_acc, 0.5f),

    // @Param: W_POS_P
    // @DisplayName: Water along-track position gain
    AP_GROUPINFO("W_POS_P", 13, Hydrone, w_pos_p, 0.5f),

    // @Param: W_HDG_ERR
    // @DisplayName: Water heading error for surge
    // @Description: Beyond this heading error to the target the vehicle turns in place without surge
    // @Units: deg
    AP_GROUPINFO("W_HDG_ERR", 14, Hydrone, w_hdg_err, 60.0f),

    // @Param: W_RAD
    // @DisplayName: Water approach radius
    // @Description: Beyond this distance the vehicle faces the target; inside it, it faces the target or backs up toward it, whichever turn is smaller
    // @Units: m
    AP_GROUPINFO("W_RAD", 15, Hydrone, w_radius, 0.6f),

    // @Param: W_SPD_Z
    // @DisplayName: Water climb and descent speed
    // @Units: m/s
    AP_GROUPINFO("W_SPD_Z", 16, Hydrone, w_spd_z, 0.5f),

    // @Param: W_ACC_Z
    // @DisplayName: Water vertical acceleration
    // @Units: m/s/s
    AP_GROUPINFO("W_ACC_Z", 17, Hydrone, w_acc_z, 1.0f),

    // @Param: W_VEL_FF
    // @DisplayName: Water surge drag feed forward
    // @Description: Forward output per (m/s)^2 of surge speed target
    AP_GROUPINFO("W_VEL_FF", 18, Hydrone, w_vel_ff, 0.3f),

    // @Group: WV_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(w_vel_pid, "WV_", 19, Hydrone, AC_PID),

    // @Param: BRCH_ANG
    // @DisplayName: Breach level angle
    // @Description: A breach starts only with roll and pitch within this angle. Rolled more, the vehicle first floats at the surface, where the hull rights it (fully under water nothing corrects roll), then dives BRCH_RUN for a run-up and rises through the surface again
    // @Units: deg
    AP_GROUPINFO("BRCH_ANG", 20, Hydrone, breach_angle, 15.0f),

    // @Param: BRCH_WAIT
    // @DisplayName: Breach levelling wait
    // @Description: Longest wait floating at the surface for BRCH_ANG before the run-up
    // @Units: s
    AP_GROUPINFO("BRCH_WAIT", 21, Hydrone, breach_wait, 10.0f),

    // @Param: BRCH_RUN
    // @DisplayName: Breach run-up depth
    // @Description: Depth the vehicle dives to after levelling at the surface, to cross it again rising: a breach from rest leaves the air rotors in the splash zone, wet unevenly, and can flip the vehicle
    // @Units: m
    AP_GROUPINFO("BRCH_RUN", 22, Hydrone, breach_runup_depth, 0.8f),

    AP_GROUPEND
};

Hydrone::Hydrone() :
    w_vel_pid(0.3f, 0.1f, 0.0f, 0.0f, 0.5f, 0.0f, 2.0f, 0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

static const char *medium_name(Hydrone::Medium medium)
{
    switch (medium) {
    case Hydrone::Medium::AIR:
        return "air";
    case Hydrone::Medium::WATER:
        return "water";
    case Hydrone::Medium::BREACH:
        return "breach";
    }
    return "?";
}

// the water-side controllers; allocated before the air ones, so that the air ones are the attitude and position
// control singletons
void Copter::hydrone_allocate_water_stack()
{
    attitude_control_water = NEW_NOTHROW AC_AttitudeControl_Sub(*ahrs_view, *motors);
    if (attitude_control_water == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControlWater");
    }
    AP_Param::load_object_from_eeprom(attitude_control_water, AC_AttitudeControl_Sub::var_info);

    pos_control_water = NEW_NOTHROW AC_PosControl(*ahrs_view, *motors, *attitude_control_water);
    if (pos_control_water == nullptr) {
        AP_BoardConfig::allocation_error("PosControlWater");
    }
    AP_Param::load_object_from_eeprom(pos_control_water, pos_control_water->var_info);
}

bool Copter::hydrone_water_stack_active() const
{
    return hydrone.enable && motors->armed() && hydrone.medium != Hydrone::Medium::AIR;
}

// center of mass over the water surface, m
bool Copter::hydrone_height(float &height_m) const
{
    Location loc;
    float alt_amsl_m;
    if (!ahrs.get_location(loc) || !loc.get_alt_m(Location::AltFrame::ABSOLUTE, alt_amsl_m)) {
        return false;
    }
    height_m = alt_amsl_m - hydrone.surf_amsl;
    return true;
}

// called by GUIDED with every position target
void Copter::hydrone_set_target(const Vector3p &target_ned_m, const Location *loc)
{
    // a new target (not the same one sent again) ends the hold on the previous one
    if (!hydrone.has_target || (target_ned_m - hydrone.target_ned_m).length() > 0.1) {
        hydrone.holding = false;
    }
    hydrone.has_target = true;
    hydrone.target_ned_m = target_ned_m;
    hydrone.target_is_loc = loc != nullptr;
    if (loc != nullptr) {
        hydrone.target_loc = *loc;
    }
}

// height of the guided target over the water surface; false without one
bool Copter::hydrone_target_height(float &target_h_m) const
{
    if (flightmode != &mode_guided || !hydrone.has_target) {
        return false;
    }
    Vector3p pos_ned_m;
    if (!ahrs.get_relative_position_NED_origin(pos_ned_m)) {
        return false;
    }
    target_h_m = hydrone.height_m + float(pos_ned_m.z - hydrone.target_ned_m.z);
    return true;
}

void Copter::hydrone_set_medium(Hydrone::Medium medium)
{
    if (medium == hydrone.medium) {
        return;
    }
    const Hydrone::Medium previous = hydrone.medium;
    hydrone.medium = medium;
    motors_hydrone->set_medium((AP_MotorsHydrone::Medium)medium);
    motors->set_forward(0.0f);
    motors->set_lateral(0.0f);

    switch (medium) {
    case Hydrone::Medium::WATER: {
        attitude_control_water->reset_rate_controller_I_terms();
        attitude_control_water->relax_attitude_controllers();
        pos_control_water->D_init_controller();
        hydrone.yaw_target_rad = ahrs.get_yaw_rad();
        Vector3p pos_ned_m;
        if (ahrs.get_relative_position_NED_origin(pos_ned_m)) {
            hydrone.hold_ned_m = pos_ned_m;
        }
        hydrone.speed_target_ms = 0.0f;
        hydrone.w_vel_pid.reset_I();
        hydrone.holding = false;
        break;
    }

    case Hydrone::Medium::BREACH:
        hydrone.breach_start_ms = millis();
        hydrone.breach_prep = Hydrone::BreachPrep::RISE;
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->relax_attitude_controllers();
        break;

    case Hydrone::Medium::AIR:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->relax_attitude_controllers();
        if (motors->armed()) {
            // hand over to the flight mode at hover throttle, flying
            attitude_control->set_throttle_out(motors->get_throttle_hover(), true, 0.0f);
            set_land_complete(false);
            set_land_complete_maybe(false);
            set_auto_armed(true);
            flightmode->init(true);
            if (flightmode == &mode_guided && hydrone.has_target) {
                if (hydrone.target_is_loc) {
                    mode_guided.set_destination(hydrone.target_loc);
                } else {
                    mode_guided.set_pos_NED_m(hydrone.target_ned_m);
                }
            }
        }
        break;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hydrone: %s -> %s (h %.2f m)", medium_name(previous), medium_name(medium),
                    hydrone.height_m);
}

// fast loop, before the flight mode
void Copter::hydrone_update_medium()
{
    if (!hydrone.enable) {
        hydrone_set_medium(Hydrone::Medium::AIR);
        return;
    }
    float h;
    if (!hydrone_height(h)) {
        return;
    }
    hydrone.height_m = h;

    if (!hydrone.medium_valid) {
        hydrone.medium_valid = true;
        hydrone_set_medium(h < 0.0f ? Hydrone::Medium::WATER : Hydrone::Medium::AIR);
        return;
    }

    const bool armed = motors->armed();
    float target_h;
    const bool has_target = hydrone_target_height(target_h);

    switch (hydrone.medium) {
    case Hydrone::Medium::AIR:
        if (h < hydrone.sub_h || (armed && has_target && target_h < 0.0f && h < hydrone.dive_h)) {
            hydrone_set_medium(Hydrone::Medium::WATER);
        }
        break;

    case Hydrone::Medium::WATER:
        if (armed && has_target && target_h > 0.0f) {
            // a target in the air: rise through the surface level, or level first (see BreachPrep)
            const float max_angle = radians(hydrone.breach_angle);
            const bool level = fabsf(ahrs.get_roll_rad()) < max_angle && fabsf(ahrs.get_pitch_rad()) < max_angle;
            switch (hydrone.breach_prep) {
            case Hydrone::BreachPrep::RISE:
                if (h > hydrone.breach_h) {
                    if (level) {
                        hydrone_set_medium(Hydrone::Medium::BREACH);
                    } else {
                        hydrone.breach_prep = Hydrone::BreachPrep::LEVEL;
                        hydrone.surface_since_ms = millis();
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hydrone: levelling at the surface (roll %.0f)",
                                      degrees(ahrs.get_roll_rad()));
                    }
                }
                break;
            case Hydrone::BreachPrep::LEVEL:
                if (level || millis() - hydrone.surface_since_ms > hydrone.breach_wait * 1000.0f) {
                    hydrone.breach_prep = Hydrone::BreachPrep::RUNUP;
                }
                break;
            case Hydrone::BreachPrep::RUNUP:
                if (h < -0.9f * hydrone.breach_runup_depth) {
                    hydrone.breach_prep = Hydrone::BreachPrep::RISE;
                }
                break;
            }
        } else if (!armed && h > hydrone.air_h) {
            hydrone_set_medium(Hydrone::Medium::AIR);
        }
        break;

    case Hydrone::Medium::BREACH:
        if (h > hydrone.air_h) {
            hydrone_set_medium(Hydrone::Medium::AIR);
        } else if (!armed || millis() - hydrone.breach_start_ms > hydrone.breach_tout * 1000.0f) {
            hydrone_set_medium(Hydrone::Medium::WATER);
        }
        break;
    }

    hydrone_log();
}

// rate controllers when the water side is active: ArduSub's in the water; in a breach both, the water thrusters
// taking the water controller's pitch and yaw, the air rotors Copter's
void Copter::hydrone_run_rate_controllers(float dt_s)
{
    attitude_control_water->set_dt_s(dt_s);
    pos_control_water->set_dt_s(dt_s);
    attitude_control_water->rate_controller_run();
    if (hydrone.medium == Hydrone::Medium::BREACH) {
        motors_hydrone->set_water_override(hydrone.breach_heave, motors->get_pitch(), motors->get_yaw());
        attitude_control->rate_controller_run();
    }
    attitude_control_water->rate_controller_target_reset();
}

// replaces the flight mode while the water side is active (armed, WATER or BREACH)
void Copter::hydrone_water_run()
{
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (hydrone.medium == Hydrone::Medium::BREACH) {
        // level, heading held; water heave (set_water_override) and a fixed air throttle lift it through the surface
        attitude_control_water->input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hydrone.yaw_target_rad, true);
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hydrone.yaw_target_rad, true);
        attitude_control->set_throttle_out(hydrone.breach_thr, false, g.throttle_filt);
        motors->set_forward(0.0f);
        return;
    }

    Vector3p pos_ned_m;
    Vector3f vel_ned_ms;
    if (!ahrs.get_relative_position_NED_origin(pos_ned_m) || !ahrs.get_velocity_NED(vel_ned_ms)) {
        attitude_control_water->input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hydrone.yaw_target_rad, true);
        attitude_control_water->set_throttle_out(0.5f, false, g.throttle_filt);
        motors->set_forward(0.0f);
        return;
    }
    const float dt_s = scheduler.get_last_loop_time_s();
    const float yaw_rad = ahrs.get_yaw_rad();

    float target_h;
    const bool guided = hydrone_target_height(target_h);
    const bool breach_pending = guided && target_h > 0.0f;
    const Vector3p target_ned_m = (guided && !breach_pending) ? hydrone.target_ned_m : hydrone.hold_ned_m;

    // depth: to the target, never shallower than W_MIN_D; before a breach (see BreachPrep) up through the surface,
    // or down for the run-up
    float hold_h = guided ? target_h : hydrone.height_m + float(pos_ned_m.z - target_ned_m.z);
    hold_h = MIN(hold_h, -hydrone.w_min_depth.get());
    if (!breach_pending) {
        hydrone.breach_prep = Hydrone::BreachPrep::RISE;
    } else if (hydrone.breach_prep == Hydrone::BreachPrep::RUNUP) {
        hold_h = -hydrone.breach_runup_depth;
    } else {
        hold_h = HYDRONE_BREACH_RISE_HEIGHT_M;
    }
    if (breach_pending && hydrone.breach_prep == Hydrone::BreachPrep::LEVEL) {
        // floating at the surface, free (no heave: pushed up the hull is less wet, pushed down it is held at a
        // depth, either way the righting is weaker); pitch and yaw stay controlled
        pos_control_water->D_relax_controller(0.5f);
        attitude_control_water->set_throttle_out(0.5f, false, g.throttle_filt);
    } else {
        const float target_d_m = float(pos_ned_m.z) + (hydrone.height_m - hold_h);
        pos_control_water->D_set_max_speed_accel_m(hydrone.w_spd_z, hydrone.w_spd_z, hydrone.w_acc_z);
        pos_control_water->D_set_alt_target_with_slew_m(-target_d_m);
        pos_control_water->D_update_controller();
    }

    // horizontal: turn toward the target, then surge (forward or backward) along the heading
    const Vector2f to_target{float(target_ned_m.x - pos_ned_m.x), float(target_ned_m.y - pos_ned_m.y)};
    const float dist_m = to_target.length();
    // Far (beyond W_RAD): face the target. Near: face it or back up toward it, whichever turn is smaller (at most
    // 90 deg), which also takes out the sideways drift (no sway thruster) without spinning in place: every turn adds
    // roll, which nothing under water corrects. Within the hold radius the heading is frozen.
    // holding: entered close to the target, left only beyond W_RAD, so drifting around a nearby target (no sway
    // thruster) does not make the bearing to it, and the heading after it, spin
    if (dist_m < HYDRONE_HOLD_RADIUS_M) {
        hydrone.holding = true;
    } else if (dist_m > hydrone.w_radius) {
        hydrone.holding = false;
    }
    if (!hydrone.holding && !breach_pending) {
        const float bearing_rad = atan2f(to_target.y, to_target.x);
        if (dist_m > hydrone.w_radius || fabsf(wrap_PI(bearing_rad - yaw_rad)) <= M_PI_2) {
            hydrone.yaw_target_rad = bearing_rad;
        } else {
            hydrone.yaw_target_rad = wrap_PI(bearing_rad + M_PI);
        }
    }
    const Vector2f heading{cosf(yaw_rad), sinf(yaw_rad)};
    const float along_m = to_target * heading;
    float speed_ms = constrain_float(sqrt_controller(along_m, hydrone.w_pos_p, hydrone.w_acc, dt_s),
                                     -hydrone.w_spd, hydrone.w_spd);
    if (!hydrone.holding) {
        // no surge while turning hard; less as the heading error grows
        const float heading_error = fabsf(wrap_PI(hydrone.yaw_target_rad - yaw_rad));
        speed_ms = heading_error > radians(hydrone.w_hdg_err) ? 0.0f : speed_ms * cosf(heading_error);
    }
    const float dv = hydrone.w_acc * dt_s;
    hydrone.speed_target_ms = constrain_float(speed_ms, hydrone.speed_target_ms - dv, hydrone.speed_target_ms + dv);
    const float surge_ms = Vector2f{vel_ned_ms.x, vel_ned_ms.y} * heading;
    const float forward = hydrone.w_vel_pid.update_all(hydrone.speed_target_ms, surge_ms, dt_s,
                                                       fabsf(motors->get_forward()) >= 1.0f)
                          + hydrone.w_vel_ff * hydrone.speed_target_ms * fabsf(hydrone.speed_target_ms);
    motors->set_forward(constrain_float(forward, -1.0f, 1.0f));

    // attitude: level (roll is not actuated, the hull keeps it), heading from the guidance
    attitude_control_water->input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hydrone.yaw_target_rad, true);
}

void Copter::hydrone_log()
{
#if HAL_LOGGING_ENABLED
    // 10 Hz at a 400 Hz loop
    if (++hydrone.log_counter % 40 != 0) {
        return;
    }
    // @LoggerMessage: HYDM
    // @Description: Hydrone medium manager and water side
    // @Field: TimeUS: Time since system startup
    // @Field: Med: medium (0 air, 1 water, 2 breach)
    // @Field: H: center of mass over the water surface
    // @Field: YT: water heading target
    // @Field: ST: water surge speed target
    // @Field: Fwd: forward output
    // @Field: W5: water thruster outputs (MOT5..8)
    AP::logger().WriteStreaming("HYDM", "TimeUS,Med,H,YT,ST,Fwd,W5,W6,W7,W8", "s-mdn-----", "F-00000000",
                                "QBffffffff",
                                AP_HAL::micros64(), (uint8_t)hydrone.medium, hydrone.height_m,
                                degrees(hydrone.yaw_target_rad), hydrone.speed_target_ms, motors->get_forward(),
                                motors_hydrone->get_water_output(0), motors_hydrone->get_water_output(1),
                                motors_hydrone->get_water_output(2), motors_hydrone->get_water_output(3));
#endif
}
