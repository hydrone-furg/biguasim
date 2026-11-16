# Hydrone ArduPilot firmware — plan (ArduCopter + ArduSub fusion)

Status (2026-11-09), in a local `hydrone` branch of the ardupilot submodule (not committed or pushed):

- **M0 done**: `ArduHydrone/` (a copy of ArduCopter, program `arduhydrone`, `./waf hydrone`), build type
  `APM_BUILD_ArduHydrone`, hover-thrust floor 0.02 (`AP_MotorsMulticopter.cpp`).
- **M1-M3 and the breach of M4/M5, for GUIDED**: the 8-waypoint mission (4 at 2 m depth, breach, 4 at 4 m altitude)
  completes offline and live in Pier-Harbor, every waypoint within 0.5 m horizontally and 0.06 m vertically.
  - `libraries/AP_Motors/AP_MotorsHydrone`: Copter matrix on MOT1..4, water mix (heave, pitch, yaw, forward; reversible)
    on MOT5..8, medium AIR / WATER / BREACH.
  - `ArduHydrone/hydrone.{h,cpp}`: medium manager (`HYD_`), ArduSub's `AC_AttitudeControl_Sub` as `ATW_`, a second
    `AC_PosControl` as `PSW` for depth, water guidance (beyond `HYD_W_RAD` face the target and surge; inside it face
    it or back up toward it, whichever turn is smaller, so sideways drift is taken out without spinning; hold from
    0.3 m until beyond `HYD_W_RAD`), breach (rising through the surface level with roll and pitch under `HYD_BRCH_ANG`;
    rolled more, it floats at the surface until the hull has levelled it, dives `HYD_BRCH_RUN` and rises again; then
    water heave + fixed air throttle, 0.2-0.4 s through the surface), dive (air rotors stop under `HYD_DIVE_H`), `HYDM` log.
  - Hooks in Copter: `system.cpp` (allocation), `Attitude.cpp` (rate controller, hover learning), `mode.cpp` (flight
    mode), `land_detector.cpp`, `inertia.cpp`, `mode_guided.cpp` (targets), `Copter.cpp` (fast task).
- **Tuning**: `PSC_D_ACC_P/I` scaled to the hover throttle (0.0076 / 0.015) removed the ~6 Hz collective limit cycle
  and the 0.3-0.7 m altitude offset; `PSC_NE_*` 1 / 2 / 1 / 0.25 (6 m step: 0.09 m overshoot, settled in 4 s,
  against 0.51 m and 14 s with the QuadPlane values).
- Open: water modes other than GUIDED (STABILIZE/ALT_HOLD/AUTO under water), dive from the air not yet flown in a
  test, failsafes per medium (M6). Under water the model has no roll righting moment (see §8): roll creeps ~3-4° per
  90° turn.

Goal: one firmware that flies the Hydrone as **ArduCopter in the air** and drives it as **ArduSub in the water**, and
switches between the two across the surface on its own.

## 1. Where it stands

The Hydrone flies today on **stock ArduPlane as a QuadPlane** (VTOL only, quad plus) with:

- `patches/arduplane-vfwd-alt-off.patch` — a negative `Q_VFWD_ALT` disables the forward-thrust ground cutoff;
- `scripts/hydrone_medium.lua` — picks the medium from the altitude against `HYD_SURF`, and under water re-mixes the
  VTOL demands onto the four water thrusters;
- `params/hydrone.parm` — VTOL-only setup, `Q_FWD_THR_USE`, rate gains at 1/5 of `quadplane.parm`.

ArduPilot itself knows nothing about water, which breaks autonomous navigation under water. In the 8-waypoint goto
mission (4 under water, 4 in the air), offline:

| Symptom | Cause in the firmware |
|---|---|
| A waypoint off the nose is never reached | QuadPlane's VTOL approach (`QPOS_POSITION1`) moves sideways by **rolling** and turns only by a roll-coordinated rate; nothing carries roll under water |
| The vehicle pitches 30° nose up arriving at a waypoint | Braking is a pitch-up: the forward throttle is 0..100 %, so it cannot push backward though the water thrusters are bidirectional |
| Capsizes (roll -180°) when turning after a waypoint | Yaw torque while pitched 30°, with roll unactuated |
| Guided overshoot logic yaws to the "apparent wind" | Airspeed / wind estimates are meaningless under water |

## 2. What the vehicle needs

From `dynamics/hauv.py` (`OctaCopterFourAirFourWaterCross`) and the bridge profile (`vehicle.py`):

| Medium | Actuators | Controllable DOF | Not actuated |
|---|---|---|---|
| Air | MOT1..4: quad plus rotors, 0..592 rad/s, push only out of the water | roll, pitch, yaw, thrust (horizontal motion by tilt) | — |
| Water | MOT5..8: 2 vertical (front, rear), 2 forward (left, right), **bidirectional** -279..279 rad/s, push only in it | heave, pitch, yaw, **surge forward and backward** | roll, sway |

- **Air**: a standard quad, which is ArduCopter's job.
- **Water**: an underactuated ROV. It is close to ArduSub's `SimpleROV-4` layout (2 forward + 2 vertical thrusters),
  but with the vertical pair front/rear (pitch) instead of left/right (roll). ArduSub's depth hold, reversible
  thrusters and surface detection fit it.
- **No sway actuator**: ArduSub's position control assumes lateral thrust, so water guidance needs a nonholonomic
  layer: **turn, then surge**. This is the one water piece neither upstream vehicle has. The BiguaSim model's own
  controller (`hauv.py`, the `dive()` branches: level, heading to the target, surge along the heading) is the
  reference.
- **Crossing the surface**: a distinct phase. The air rotors lose thrust as they submerge, and the water thrusters as
  they leave the water (`_compute_body_wrench`).

## 3. How to fuse them

ArduCopter and ArduSub are separate vehicles (`ArduCopter/`, `ArduSub/`). Each has its own main object
(`Copter copter`, `Sub sub`), scheduler and parameter tables, so they cannot both be linked into one binary as they
are. Their building blocks are compatible, though:

| Piece | ArduCopter | ArduSub | Shared base |
|---|---|---|---|
| Motors | `AP_MotorsMatrix` | `AP_Motors6DOF` (forward/lateral axes, reversible motors) | `AP_MotorsMatrix` → `AP_MotorsMulticopter` |
| Attitude | `AC_AttitudeControl_Multi` | `AC_AttitudeControl_Sub` | `AC_AttitudeControl` |
| Position | `AC_PosControl` | `AC_PosControl` | same class |
| Modes | 25+ (`mode_*.cpp`) | 10 (`mode_*.cpp`: manual, stabilize, althold, poshold, guided, auto, circle, surface, surftrak, motordetect) | `Mode` pattern, different base classes |

Options:

| Option | What it is | Verdict |
|---|---|---|
| A. **New vehicle `ArduHydrone/`, Copter host + Sub water stack (recommended)** | Starts from ArduCopter's vehicle code (main loop, arming, failsafes, takeoff/land, air modes). Ports ArduSub's water modes, depth control and surface/bottom detector into it. Adds a medium manager and a combined motors class | The air side is the bigger and more safety-critical one, and stays upstream Copter code; Sub's ten modes are the smaller part to bring over |
| B. New vehicle, Sub host + Copter air stack | Same, the other way round | Would port Copter's takeoff/land, land detector, hover-throttle learning and air failsafes: more code |
| C. Two binaries handing over at the surface | Copter process in the air, Sub process in the water | One flight controller runs one firmware; state (EKF, mission) would not carry over. Rejected |

## 4. Architecture (option A)

1. **Medium manager** (`ArduHydrone/medium.cpp`): AIR / TRANSITION / WATER from the depth (barometer in SITL;
   water-pressure sensor on real hardware, as ArduSub uses), with hysteresis and the vehicle's extent (rotors submerge
   before the body). Logs `HYDM`, sends a STATUSTEXT on each change.
2. **Motors** (`AP_MotorsHydrone : AP_Motors6DOF`): 8 motors with two factor sets.
   - Air rows: Copter quad plus on MOT1..4 (roll, pitch, yaw, throttle; not reversible).
   - Water rows: a Sub-style 6-DOF row set on MOT5..8 (heave, pitch, yaw, forward; reversible). Roll and lateral
     are reported as limited in water so integrators don't wind up.
   - TRANSITION: both sets active, weighted by the immersion of each rotor.
   - One `AP::motors()` singleton, so arming, the safety switch, ESC calibration and motor test work as usual.
3. **Two controller stacks, one per medium**, which gives per-medium gains for free:
   - Air: `AC_AttitudeControl_Multi` (`ATC_*`) + `AC_PosControl` (`PSC_*`): Copter's.
   - Water: `AC_AttitudeControl_Sub` (`ATCW_*`) + a second `AC_PosControl` (`PSCW_*`): Sub's, with Sub's defaults.
   - On a medium change the new stack is reset to the current attitude, rates and target, and its integrators
     cleared.
4. **Modes**: the user sees one set, and each mode has an air and a water implementation.

   | Mode | Air (from Copter) | Water (from Sub, adapted) |
   |---|---|---|
   | STABILIZE / MANUAL | Copter stabilize | Sub stabilize / manual |
   | ALT_HOLD | Copter alt hold | Sub depth hold |
   | LOITER / POSHOLD | Copter loiter | Sub poshold + heading-first guidance |
   | GUIDED | Copter guided | Sub guided + **nonholonomic guidance** (§4.5) |
   | AUTO | Copter auto | Sub auto + nonholonomic guidance; mission items with negative altitude are under water |
   | RTL | Copter RTL | Sub SURFACE, then air RTL |
   | LAND | Copter land on the ground; on the water: land = settle floating | — |

   A target across the surface makes GUIDED/AUTO run a transition (§4.6) and then continue in the other medium.
5. **Water guidance** (the part neither upstream has):
   - Target bearing → heading target (Sub's yaw control); surge scaled by the cosine of the heading error, zero
     beyond ±60°.
   - Braking and position hold with reverse surge. No pitch-to-move, no roll-to-move.
   - Cross-track correction on legs; acceptance radius and pass-abeam for AUTO waypoints.
   - Envelope protection: yaw rate limited while pitched and pitch limited while turning (the capsize seen offline).
6. **Transitions** (state machines inside the medium manager):
   - **Dive**: descend on Copter control to the surface, hand over as the rotors submerge (air thrust ramps down,
     water heave ramps up), then Sub depth hold to the target depth.
   - **Breach**: rise on Sub control at a set climb rate, spool the air rotors to hover throttle before they clear
     the water, then Copter climb to the target altitude.
   - Timeouts, abort back to the previous medium, and log entries for each step.
7. **Safety**:
   - Land detector and crash check only in AIR.
   - Sub's leak and depth failsafes, and the GCS failsafe, in WATER.
   - EKF/GPS failsafe relaxed under water on real hardware (no GPS there).
   - Arming checks for both stacks.
8. **Sensors and EKF**:
   - In SITL the barometer altitude from the bridge's JSON position already goes negative under water.
   - Real hardware: a water-pressure sensor as the primary depth source in water, and an EKF source set switched by
     medium (`EK3_SRC*` options).
   - SITL option to drop GPS below the surface, for realism.
9. **BiguaSim side**:
   - `firmware/build.sh` builds a `hydrone` target.
   - The Hydrone profile (`vehicle.py`) switches from `arduplane` + Lua to it, with a new `hydrone.parm`.
   - The ArduPlane patch and the Lua script are retired.
   - Update `test_ardubridge.py`, `test_ardupilot_sitl.py`, the ardubridge docs and `ai/domains/ardubridge.md`.

## 5. Milestones (each with its acceptance test)

Offline tests use `OfflineWorld` + SITL (`tests/unit/test_ardupilot_sitl.py`, `-m sitl`). Live tests use the engine
(`tests/ardupilot_goto_fly.py`, `tests/live_ardupilot.py`).

| # | Deliverable | Accepted when |
|---|---|---|
| M0 | `ArduHydrone/` vehicle as a copy of ArduCopter in the fork; waf target; profile switch | SITL boots and arms as a plain quad (air table only); the existing air hover test passes |
| M1 | `AP_MotorsHydrone` (both tables), medium manager (detection + logging only) | Air takeoff, hover and 4-waypoint air square at 4 m within 0.5 m; medium reported correctly when pushed under |
| M2 | Sub water stack (`ATCW_`/`PSCW_`), Sub STABILIZE/MANUAL/ALT_HOLD as water modes | From the surface: hold 2 m depth ±0.1 m, roll/pitch < 5° for 60 s |
| M3 | Water guidance in GUIDED/POSHOLD | Underwater 4-waypoint square (8 m at 2 m depth) within 1 m, no capsize, pitch < 15° |
| M4 | Dive and breach | 10 dive/breach cycles offline without a failure; breach to 4 m within 10 s |
| M5 | Medium-aware GUIDED/AUTO missions | The 8-waypoint mission (4 under, 4 in the air) completes offline and live in Pier-Harbor |
| M6 | Failsafes per medium, RTL via SURFACE, docs | Failsafe tests per medium; docs and AI notes updated |
| M7 | (Optional) real hardware: pressure sensor, EKF source switching | Hardware-in-the-loop checklist |

The video's Hydrone clip gets re-rendered after M5.

## 6. Repository strategy

The fusion is a new vehicle directory plus new libraries, thousands of lines. Patch files (today's approach, 2 small
patches) won't hold up. Recommended:

- a fork of `ArduPilot/ardupilot` (e.g. under `hydrone-furg`) with a `hydrone` branch based on a tagged ArduCopter
  release;
- `ArduHydrone/` added beside `ArduCopter/` and `ArduSub/`, which stay untouched so upstream rebases stay simple;
- the `ardubridge/ardupilot` submodule pointing to the fork;
- `build.sh` building `copter sub rover hydrone`.

## 7. Decisions needed

1. **Host**: Copter host + Sub water stack (recommended), or Sub host + Copter air stack.
2. **Scope**: simulation only, or also the real Hydrone (flight controller, ESCs, bidirectional water thrusters,
   pressure sensor). This changes M7 and the sensor assumptions.
3. **Repository**: fork branch + submodule (recommended), or patch files.
4. **Gains**: air gains from today's `hydrone.parm`. Water gains start from ArduSub's defaults scaled by the Lua
   script's factors (`HYD_K*`). Further tuning only with your approval.
5. **Realism in SITL**: keep GPS under water (simpler), or drop it below the surface.

## 8. Risks

- **Roll stability under water**: roll is unactuated, and the offline capsizes show little margin when pitched and
  turning. Measured (2026-11-09): fully submerged, the model's hull gives **no** righting moment (a 20° roll stays
  20° with the motors off), so roll changes only through the pitch-rate x yaw-rate coupling, and yaw is the
  intermediate inertia axis (Ixx 0.0145 < Izz 0.0154 < Iyy 0.0288): every water turn grows the roll (−11° after the
  offline square; floating at the surface rights it, 30° → 5° in 2 s). The firmware protects the envelope (§4.5). If the hull's righting moment in the model is too weak, that is
  a dynamics question to raise separately, not something to fix with dynamics parameter changes.
- **Two controller stacks**: the handover between `AC_AttitudeControl_Multi` and `_Sub` in mid-transition is the
  riskiest code path. It needs its own SITL tests (M4) and logs.
- **Upstream drift**: copying Copter and porting Sub modes means periodic merges from both vehicles' upstream
  changes. Keeping `ArduHydrone/` thin (shared libraries, few copied files) limits it.
- **EKF across the surface** (real hardware): barometer and GPS steps at entry and exit.
