=========
Changelog
=========

.. Changelog Style Guide
  - Each release should have a New Features / Changes / Bug Fixes section.
  - Keep the first sentence of each point short and descriptive
  - The passive voice should be avoided
  - Try to make the first word a verb in past tense. Bug fixes should use
    "Fixed"
  - Add a link to the issue describing the change or the pull request that
    merged it at the end in parentheses
BiguaSim 1.1.0
===============
*-/-/2026*

New Features
------------
- Added Infrared Camera sensor
- Added Normals Camera sensor
- Added Buoy with physics in Pier-Harbor world
- Added the water surface block to the :ref:`dynamics-sensor` (``ExportWaterSurface``, on by default): the wave
  surface under each agent, every tick. All dynamics models use it as the water surface instead of ``z = 0``.
- Added the :ref:`water-collision-sensor`, with the wave surface under the agent (``ReportSurface``).
- Added the :ref:`coverage-sensor`, the share of a target's surface another sensor has seen.
- Added collision geometry export to the DynamicsSensor (``ExportGeometry``). The models build their drag panels and
  hydrostatics from it.
- Added the Fossen-based marine plant for BlueROV2, BlueROV2Heavy and BlueBoat, with ocean current and wind, and the
  ``controller`` choice (``original`` / ``marine``). See :ref:`vehicle-dynamics`.
- Added wind to the DjiMatrice (``fluid_velocity``) and ocean current to the TorpedoAUV and Hydrone.
- Added offline tests (``tests/unit``) that need no world binary. See :ref:`testing`.
- Added ArduPilot SITL support for every vehicle (:ref:`ardupilot`): DjiMatrice on ArduCopter, BlueBoat on Rover,
  BlueROV2 and BlueROV2Heavy on ArduSub, TorpedoAUV on ArduSub with a new torpedo frame, Hydrone on ArduPlane
  (QuadPlane) with a water-mode script. Includes the firmware parameters, patches and build script, a SITL launcher,
  a ground station helper, ``python -m biguasim.ardubridge``, and ``OfflineWorld`` to fly without the engine.

Changes
-------
- The ``water_level`` dynamics parameter is now passed from the scenario to the model.
- A world packaged before the water surface block no longer fails every scenario: with ``ExportWaterSurface`` left
  at its default, the client drops the block with a warning and the models use a flat water level.
- The marine and Hydrone plants take fewer substeps above 30 Hz (no substep shorter than at 30 Hz): about 10x faster
  at the 200-400 Hz ArduPilot runs at, unchanged at 30 Hz and below. A flat water surface skips the surface
  interpolation.
- The ArduPilot bridge builds the state from the DynamicsSensor alone (IMU included) and steps in lockstep with
  ArduPilot, one physics step per servo packet.

Bug Fixes
---------
- Fixed some problems with the docs
- Fixed ``BiguaSimEnvironment`` timing out on launch when no ``uuid`` was given: the engine read the next launch
  argument as the UUID.
- Fixed the Hydrone failing to build with ``batch_size`` other than 1 or 4, and mixing the robots' rotor torques
  with 4.
- Fixed the Hydrone's water velocity gains and added inertia with batches larger than one.
- Fixed the engine placing the center of mass of buoyant agents by their spawn location (a BlueROV2 spawned far from
  the origin had it hundreds of meters away).
- Fixed the PID derivative kick on the first step.
- Fixed the ArduPilot bridge sending the position as latitude/longitude, which ArduPilot reads as metres from home
  (the vehicle appeared ~120 m away with its altitude inverted).
- Fixed the ArduPilot channel mappings: the DjiMatrice's motors were in the wrong order for ArduCopter's quad X, and
  every BlueROV2 / BlueROV2Heavy thruster pushed against ArduSub's direction (the Heavy's vertical ones also in the
  wrong order).
- Fixed the ArduPilot runner stepping before ``reset``, and reading single-agent states by agent name.
- Fixed the Hydrone's hull voxels being one voxel short on one side: its center of buoyancy sat 0.2 mm off its center
  of gravity, which tilted it ~18 deg in 4 s under water with no thrust.

BiguaSim 1.0.0
===============
*-/-/2026*

First official release coming soon!