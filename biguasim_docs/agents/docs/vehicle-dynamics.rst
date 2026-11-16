.. _vehicle-dynamics:

================
Vehicle Dynamics
================

BiguaSim computes the dynamics of its vehicles in the Python client (PyTorch, CPU or GPU) instead of in Unreal
Engine. Every tick the engine reports each agent's state through its :ref:`dynamics-sensor`; the client's model
for that ``agent_type`` turns the command into accelerations, and the engine integrates them. The engine adds
nothing of its own: gravity, buoyancy, drag and thrust all come from the model.

.. list-table::
   :widths: 18 22 60
   :header-rows: 1

   * - Agent
     - Model
     - Plant
   * - ``DjiMatrice``
     - ``uav.QuadCopterX``
     - Quadrotor, DJI Matrice 300 RTK parameters. Drag on the hull panels of the exported geometry, relative to the
       wind. Rotors under the water surface give no thrust.
   * - ``BlueROV2``, ``BlueROV2Heavy``
     - ``marine.MarinePlant``
     - Fossen 6-DOF marine plant (below). BlueROV2 has no pitch control (5-DOF, like the real vehicle).
   * - ``BlueBoat``
     - ``marine.MarinePlant``
     - Same plant, as a surface vessel: its displacement is calibrated to the specified draft.
   * - ``Hydrone``
     - ``hauv.OctaCopterFourAirFourWaterCross``
     - Hybrid air/water vehicle: four air rotors and four water thrusters, each acting by its immersion.
   * - ``TorpedoAUV``
     - ``uuv.TorpedoAUV``
     - Fossen's REMUS 100 model, with buoyancy against the wave surface.

Requirements
============

Every agent needs a DynamicsSensor with ``"UseCOM": true`` and ``"UseRPY": false``, and a ``dynamics`` block with
at least the ``batch_size``:

.. code-block:: python

   {
       "agent_name": "rov", "agent_type": "BlueROV2",
       "sensors": [{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                    "configuration": {"UseCOM": True, "UseRPY": False,
                                      "ExportGeometry": True, "GeometrySource": "simple"}}],
       "dynamics": {"batch_size": 1},
       "control_abstraction": "cmd_pos_yaw",
       "location": [0, 0, -3], "rotation": [0, 0, 0],
   }

Keys of the ``dynamics`` block override the model's parameters of the same name (see ``_params`` in
``biguasim/dynamics/agents.py``), e.g. ``"dynamics": {"batch_size": 1, "mass": 12.0}``. Keys a model does not know
are ignored, except ``water_level`` (below).

Water surface
=============

All models take the water surface from the DynamicsSensor's water surface block (``ExportWaterSurface``, on by
default): a grid of wave surface heights under the agent, refreshed every tick. Between samples the height is
interpolated (inverse distance). So:

- the marine vehicles float on the actual surface and ride the waves;
- the DjiMatrice's rotors stop pushing once under the surface;
- the Hydrone's air rotors only push out of the water and its thrusters only in it;
- the TorpedoAUV surfaces and floats when stopped.

Without the block (``"ExportWaterSurface": false``, or a world packaged before it), the models use a flat surface at
``water_level``: 0 m for the marine vehicles, the TorpedoAUV and the Hydrone, no water at all for the DjiMatrice. Set it per agent
with ``"dynamics": {"batch_size": 1, "water_level": 0.6}``. A warning tells you when a model falls back to it.

After a ``step``, ``model.water_contact`` holds each robot's contact flag.

Marine plant
============

``BlueROV2``, ``BlueROV2Heavy`` and ``BlueBoat`` share Fossen's 6-DOF model about the engine's center of mass, with
velocities relative to the water:

- **Buoyancy**: Archimedes over a voxelized hull (2 cm voxels) compared with the water surface, so partial
  immersion, the righting moment and the waves follow from the geometry. The hull is the exported collision mesh
  (``ExportGeometry``), or a box from the ``length``/``width``/``height`` parameters without it.
- **Drag**: quadratic panel drag on the hull; panels under the surface see the water and the current, panels above
  see the air and the wind. Linear damping and added mass scale with the submerged fraction.
- **Thrust**: ``k_eta * w * |w|`` along each thruster, only while that thruster is under water.
- The plant is integrated in 10 substeps per engine tick.

.. note::
    Always export the geometry for the BlueBoat (``"ExportGeometry": true, "GeometrySource": "simple"``). The
    stand-in box is a single 0.17 m wide hull, which is not stable in roll: the boat heels over within seconds.

Controllers
-----------

The ``controller`` parameter picks the marine vehicles' controller for ``cmd_vel``, ``cmd_vel_yaw`` and
``cmd_pos_yaw``:

- ``"original"`` (default): the vehicle family's own cascade, adapted to the marine plant;
- ``"marine"``: ``ROVControl`` / ``SurfaceVesselControl``, written with the plant.

.. code-block:: python

   "dynamics": {"batch_size": 1, "controller": "marine"}

Any other value raises a ``ValueError`` on the first step. ``cmd_pos_yaw`` limits the horizontal speed to the
highest steady speed the plant can hold without saturating a thruster or tilting past 30°.

Wind and ocean current
======================

Wind and current are attributes of the dynamics models, one world-frame velocity (m/s) per robot of the batch.
They are zero by default and can change at any time:

.. code-block:: python

   import torch

   uav = env._dynamics_dict["uav"]       # the model of the agent named "uav"
   uav.fluid_velocity[:] = torch.tensor([5.0, 0.0, 0.0], dtype=torch.float64)       # 5 m/s wind along +x

   rov = env._dynamics_dict["rov"]
   rov.current_velocity[:] = torch.tensor([0.3, 0.0, 0.0], dtype=torch.float64)     # 0.3 m/s current

   boat = env._dynamics_dict["boat"]
   boat.current_velocity[:] = torch.tensor([0.0, 0.2, 0.0], dtype=torch.float64)    # water below the surface
   boat.wind_velocity[:] = torch.tensor([3.0, 0.0, 0.0], dtype=torch.float64)       # air above it

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Model
     - Attributes
   * - DjiMatrice
     - ``fluid_velocity`` (wind)
   * - BlueROV2, BlueROV2Heavy, BlueBoat
     - ``current_velocity`` (water), ``wind_velocity`` (air, on the parts above the surface)
   * - TorpedoAUV, Hydrone
     - ``current_velocity``

Batches
=======

``"dynamics": {"batch_size": N}`` spawns N copies of the agent (named ``<agent_name>-id0`` ... ``-id{N-1}``) driven
by one batched model. Pass one command per copy:

.. code-block:: python

   state = env.step({"hyd": [[0, 0, 3, 0], [0, 0, -2, 0]]})    # agent "hyd" with batch_size 2

Every robot of a batch evolves exactly as a model of its own would; the offline test suite checks this for every
vehicle and control abstraction.

Known limitations
=================

- **Hydrone under water**: its center of gravity coincides with its center of buoyancy and none of its four water
  thrusters gives a roll moment, so nothing holds its roll under water. It holds depth, but while it moves the roll
  drifts, the vertical thrust pushes sideways, and climbing out of the water from depth can tip it over at the
  surface. In the air it holds position well.
- The DjiMatrice's ``cmd_vel`` and ``cmd_vel_yaw`` have no integral term: in steady wind the velocity settles below
  the command.
