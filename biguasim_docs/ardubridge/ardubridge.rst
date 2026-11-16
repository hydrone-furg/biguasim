.. _ardupilot:

=====================
ArduPilot (SITL)
=====================

Every BiguaSim vehicle can be flown by its ArduPilot firmware, running as Software In The Loop (SITL). The firmware
sees the vehicle through ArduPilot's JSON physics backend: it sends its servo outputs, BiguaSim steps the vehicle's
dynamics model with them, and returns the vehicle's state. Ground stations (QGroundControl, MAVProxy, Mission
Planner) and companion software connect to the firmware as to a real vehicle.

.. list-table::
   :widths: 16 16 34 34
   :header-rows: 1

   * - Vehicle
     - Firmware
     - Frame
     - BiguaSim adaptation
   * - ``DjiMatrice``
     - ArduCopter
     - Quad X (``FRAME_CLASS 1``, ``FRAME_TYPE 1``)
     - Parameters: thrust curve, hover throttle, rate gains
   * - ``BlueBoat``
     - Rover
     - Boat (``FRAME_CLASS 2``), skid steering (``SERVO1/3_FUNCTION 73/74``)
     - Parameters: cruise speed and throttle
   * - ``BlueROV2``
     - ArduSub
     - Vectored (``FRAME_CONFIG 1``)
     - Parameters: guided speed
   * - ``BlueROV2Heavy``
     - ArduSub
     - Vectored 6DOF (``FRAME_CONFIG 2``)
     - Parameters: guided speed
   * - ``TorpedoAUV``
     - ArduSub
     - Torpedo (``FRAME_CONFIG 8``): propeller, two rudders, two stern planes
     - Firmware patch (the frame), parameters: depth through pitch
   * - ``Hydrone``
     - ArduPlane (QuadPlane)
     - Quad plus (``Q_FRAME_TYPE 0``), VTOL only
     - Lua script (water mode), firmware patch, parameters

The bridge lives in ``biguasim.ardubridge``; the firmware side (parameter files, scripts, patches, build script) in
``src/biguasim/ardubridge/firmware``.

Quick start
===========

1. Build the SITL firmwares once (the ``ardupilot`` submodule of the bridge, with BiguaSim's patches applied). The
   ArduPilot code generators need ``empy`` 3 (``empy`` 4 breaks them), so use an environment of their own:

   .. code-block:: bash

      conda create -n ardupilot python=3.11
      conda run -n ardupilot pip install "empy==3.3.4" pexpect future dronecan pymavlink
      PYTHON=$(conda run -n ardupilot which python) src/biguasim/ardubridge/firmware/build.sh

2. Install ``pymavlink`` in the BiguaSim environment (``pip install pymavlink``) for the ground station helper.

3. Fly:

   .. code-block:: bash

      python -m biguasim.ardubridge BlueROV2 --package SkyDive --world Pier-Harbor --location 38 -301 0
      python -m biguasim.ardubridge --list

   This starts the world, the vehicle's firmware with its parameters and scripts, and the bridge. Connect a ground
   station to UDP 14550 (QGroundControl and MAVProxy listen there by default) or TCP 5760.

The same from Python:

.. code-block:: python

   import biguasim
   from biguasim.ardubridge import ArduBiguaSimRunner, SITL, VEHICLE_REGISTRY

   profile = VEHICLE_REGISTRY["BlueROV2"]
   scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Pier-Harbor",
                                                location=[38, -301, 0])
   env = biguasim.make(scenario_cfg=scenario)
   with SITL(profile) as sitl:
       ArduBiguaSimRunner(profile, env, port=sitl.json_port).run(sitl=sitl)

``build_scenario`` gives the vehicle the only sensor the bridge needs, a DynamicsSensor (``UseCOM``, quaternion,
exported geometry), and sets ``ticks_per_sec`` to the profile's physics rate. Add the sensors your own software needs
to the agent; they do not affect the bridge.

Scripting a mission
-------------------

:class:`~biguasim.ardubridge.gcs.GroundStation` is a small pymavlink ground station: it sends heartbeats (ArduSub's
GCS failsafe disarms without them) and keeps RC overrides alive (ArduPilot drops an override after
``RC_OVERRIDE_TIME``). Run the bridge in a thread and drive the vehicle from the main one:

.. code-block:: python

   import threading
   from biguasim.ardubridge.gcs import GroundStation

   sitl = SITL(profile).start()
   runner = ArduBiguaSimRunner(profile, env, port=sitl.json_port)
   threading.Thread(target=runner.run, kwargs={"sitl": sitl}, daemon=True).start()

   gcs = GroundStation(sitl.mavlink_url)
   gcs.wait_ready()                     # pre-arm checks pass and the EKF uses the GPS
   gcs.set_mode("ALT_HOLD"); gcs.arm()
   gcs.set_mode("GUIDED")
   gcs.goto_global(lat, lon, -3.0)      # 3 m below the surface

How it works
============

ArduPilot runs in lockstep with the physics: it sends one servo packet (16 or 32 channels) per physics step and
waits for the state. :class:`~biguasim.ardubridge.runner.ArduBiguaSimRunner` answers each packet with exactly one
``env.step``; a packet ArduPilot resends after a lost reply gets the same state again, without stepping. Simulated
time is the environment's, so the firmware runs as fast as the simulation, faster or slower than real time.

**Actuators.** Each :class:`~biguasim.ardubridge.vehicle.VehicleProfile` maps the vehicle's command (its
``cmd_motor_speeds``, or the TorpedoAUV's ``cmd_rudders_sterns_motor_speed``) to ArduPilot servo channels, with a
sign per actuator. The mappings follow ArduPilot's motor tables (``AP_MotorsMatrix``, ``AP_Motors6DOF``), and a test
checks each one against the dynamics models: a positive output on every channel must roll, pitch, yaw, lift or push
the vehicle the way ArduPilot means it to. PWM converts to the actuator's range from the model's own limits:

- multirotor motors (``1000..2000`` µs): rotor speed, ``0..rotor_speed_max``; the parameters set ArduPilot's
  thrust curve (``MOT_THST_EXPO 1``) for thrust growing with the square of the speed;
- thrusters, boat throttles and fins (``1100..1900`` µs, 1500 neutral): thrusters are *thrust-linear*, the pulse
  sets the thrust (as a T200 does, and as ArduSub and Rover expect), fins take the deflection linearly.

**State.** The bridge sends ArduPilot the position (NED metres from home), velocity, attitude quaternion and an IMU
built from the DynamicsSensor: the accelerometer reads the specific force (acceleration minus gravity, so
``[0, 0, -g]`` at rest) and the gyro the body rates.

**Physics rate.** ArduPilot gets one IMU sample per physics step and refuses to arm unless they come at 1.8 times its
loop rate or more. The physics runs at the profile's ``physics_hz`` (200 Hz; 400 Hz for the Hydrone) and the
parameter files set ``SCHED_LOOP_RATE`` to match (100 Hz; 200 Hz for the Hydrone; Rover keeps its 50 Hz).

.. note::
    **Rendering sets the pace.** In lockstep every tick waits for the engine's frame. Against the editor build,
    Pier-Harbor rendered off-screen at 1280x720 ticked at ~4 Hz (0.02x real time at 200 Hz: ArduSub took 30 min to
    boot); at 160x120 with the scalability settings at their lowest, ~18 Hz (0.09x); without a renderer
    (``-nullrhi``) ~200 Hz (the BlueROV2 flew at ~0.6x real time). Without a renderer the engine throws the
    BlueBoat at spawn (5 rad/s on the first tick; it capsizes and sinks), so fly the boat with rendering.

Coordinates, home and depth
---------------------------

ArduPilot's home (the SITL ``--home``, altitude 0) stands for a world point, the runner's ``origin``. By default it
is the world's ``x = y = 0`` at the height of the water surface under the vehicle when the run starts, so:

- north/east are the world's ``+x`` / ``-y`` (BiguaSim is north-west-up, ArduPilot north-east-down);
- an altitude above mean sea level (AMSL) is a height above the water surface, and for ArduSub
  ``alt = -3`` is 3 m deep.

.. note::
    **Boot ArduSub vehicles at the surface.** ArduSub takes the pressure at boot as zero depth, and moves that zero
    up whenever it reads a positive altitude. A vehicle spawned under water reports depths from where it booted.

.. note::
    GUIDED local targets (``SET_POSITION_TARGET_LOCAL_NED``, :meth:`~biguasim.ardubridge.gcs.GroundStation.goto_local`)
    count from the EKF origin, where the vehicle was when the EKF first used the GPS (about 20-40 s into the run).
    For absolute positions and depths use global targets
    (:meth:`~biguasim.ardubridge.gcs.GroundStation.goto_global`).

Vehicles
========

DjiMatrice — ArduCopter
-----------------------

Quad X: ``MOT1..4`` drive the front-right, back-left, front-left and back-right rotors. The rate gains are about a
third of ArduCopter's defaults: the Matrice has ~5x the torque per inertia of the ~1.5 kg quad those suit, and with
them the rotors swung between minimum and 300 rad/s every few ticks. Offline it takes off, hovers steady within
0.25°, and settles a 15 m step in 4.4 s with 0.13 m of overshoot.

Spawn it on the ground: ArduCopter boots and arms on it. Pier-Harbor has open water where the other vehicles fly;
a box prop as a pad (``env.spawn_prop("box", ...)`` after ``reset``, then ``run(reset=False)``) does not work yet:
the Matrice tips 25-55° standing on it, and ArduCopter will not arm leaning.

BlueROV2 and BlueROV2Heavy — ArduSub
------------------------------------

The vectored and vectored-6DOF frames, with ArduSub's default gains. The BlueROV2 has no pitch actuator: horizontal
thrust tilts it until its righting moment holds it. BiguaSim's own controller limits it to 0.25 m/s for that tilt
to stay under 30°, and the parameters set ArduSub's guided speed to the same (``WP_SPD 0.25``; at 0.5 m/s it tipped
to 72°). The Heavy controls its pitch; its speed is limited by thrust saturation, about the same.

BlueBoat — Rover
----------------

Rover as a boat with skid steering on SERVO1 (port) and SERVO3 (starboard), as on the real BlueBoat. Cruise
1.5 m/s at 25 % throttle (drag grows with the square of the speed; full thrust gives 3 m/s).

The hull has no keel and does not run straight by itself: with even thrust it drifts into a 15-20 °/s turn within
8 s. With Rover's default turn-rate gains the heading swung ±50° on a straight leg, so the parameters raise them
(``ATC_STR_RAT_P 2``, ``ATC_STR_RAT_I 1``); the boat then holds its path heading within a few degrees.

TorpedoAUV — ArduSub, torpedo frame
-----------------------------------

ArduSub has no frame with fins, and frames are only added in its C++ (``AP_Motors6DOF``), so BiguaSim adds one
(``firmware/patches/ardusub-torpedo-frame.patch``, ``FRAME_CONFIG 8``):

.. list-table::
   :widths: 15 25 60
   :header-rows: 1

   * - Motor
     - Actuator
     - Factors
   * - MOT1
     - propeller
     - forward
   * - MOT2, MOT3
     - top, bottom rudder
     - yaw
   * - MOT4, MOT5
     - left, right stern plane
     - pitch, and climb

The fins only act while it moves: drive it with the forward input (RC 5 or the joystick's forward axis), in
ALT_HOLD or MANUAL. Depth is held by pitching: the climb demand raises the planes like a pitch demand. The depth and
pitch gains are lower than ArduSub's (which bang the planes between ±20° and swing the depth by 3 m); offline it
holds depth within 0.1 m at 2 m/s and keeps a new heading within 2° after a turn. It has no roll actuator (its roll
is passively stable) and cannot hover.

Hydrone — ArduPlane QuadPlane, water mode
-----------------------------------------

The Hydrone flies as a VTOL-only QuadPlane: its four air rotors are the quad (plus frame, ``SERVO1..4``), the
QuadPlane never transitions to fixed-wing flight (``Q_ASSIST_SPEED -1``) and GUIDED targets are flown in VTOL
(``Q_GUIDED_MODE 1``). Use the Q modes: QSTABILIZE, QHOVER, QLOITER, GUIDED, AUTO with VTOL items.

Under water the air rotors give no thrust. The ``hydrone_medium.lua`` script (``SCR_ENABLE 1``) adapts the firmware:

- it tells water from air by the altitude against the surface (``HYD_SURF``, m AMSL; 0 is the bridge's water level),
  with ``HYD_HYST`` of hysteresis;
- under water it drives the four water thrusters (``SERVO5..8``: front vertical, right forward, rear vertical, left
  forward) from the VTOL controllers' demands: heave from the throttle (the hover throttle is zero heave), pitch,
  yaw, and forward thrust. Each axis is scaled (``HYD_KH``, ``HYD_KP``, ``HYD_KY``, ``HYD_KF``) to about the
  acceleration it gives in the air, so the air-tuned loops keep their gain;
- under water it turns on QuadPlane's forward thrust instead of tilt (``Q_FWD_THR_USE 2``): position control and the
  pitch stick move the vehicle with the horizontal thrusters, level;
- under water it flags roll (no water actuator) and saturated axes as motor limits, so the integrators do not wind
  up. Wound up, they threw the vehicle as it broke the surface.

The firmware patch (``arduplane-vfwd-alt-off.patch``) lets ``Q_VFWD_ALT -1`` turn off QuadPlane's forward-thrust
ground cutoff, which kept the forward thrust at zero below home (all of the water).

Offline it holds depth and attitude under water, dives and moves forward at ~1 m/s, breaks out of the water at full
throttle (≤ 10° of tilt) and holds in the air; the air rate gains are 1/5 of the QuadPlane defaults (thrust-to-weight
19). Under water it has no roll actuator: it holds roll only through its hydrostatics (none: CG = CB), so keep it
moving gently.

Without the engine
==================

:class:`~biguasim.ardubridge.offline.OfflineWorld` steps one vehicle's dynamics model the way the engine does (no
collisions, a flat water surface, optionally a flat ground) and stands in for the environment, to try parameters or
missions without Unreal:

.. code-block:: bash

   python -m biguasim.ardubridge DjiMatrice --offline            # flat ground at z = 0
   python -m biguasim.ardubridge Hydrone --offline --location 0 0 -2

Without a ``geometry`` the models use their box hulls. The BlueBoat's box (one 0.17 m hull) is not stable in roll;
pass the hull exported by the engine: ``OfflineWorld("BlueBoat", geometry="BlueBoat.npz")`` with a file written by
:func:`~biguasim.ardubridge.offline.save_geometry` from ``env.agents[...].sensors["DynamicsSensor"].geometry``.

Testing
=======

- ``pytest tests/unit/test_ardubridge.py``: the channel mappings against ArduPilot's motor tables (with the dynamics
  models), PWM conversion, frames, the JSON protocol and the lockstep loop against a stand-in autopilot. No firmware.
- ``pytest tests/unit -m sitl``: each vehicle flown by its firmware against its dynamics model (OfflineWorld). Minutes
  each; skipped when the firmwares are not built.
- ``tests/live_ardupilot.py`` (repository root, with ``run_viewport_test.sh``): each vehicle against the editor build
  of the engine, ``VEHICLE=<name> TICKS_PER_SEC=<physics_hz> ENGINE_ARGS="-nullrhi"``.
