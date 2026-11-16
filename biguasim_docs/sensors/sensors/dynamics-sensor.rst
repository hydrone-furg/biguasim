.. _dynamics-sensor:

===============
Dynamics Sensor
===============

A sensor that returns ground truth values needed to implement custom dynamics. The dynamics sensor
returns linear acceleration, velocity, and position, followed by angular acceleration, velocity, and
orientation, in that order. Every agent driven by the built-in dynamics models (see :ref:`vehicle-dynamics`)
needs one, with ``"UseCOM": true`` and ``"UseRPY": false``.

.. note::
    This sensor always returns values at the **COM of the vehicle**, with values given in the **global
    frame**, regardless of the socket it is placed in.

See :py:class:`~biguasim.sensors.DynamicsSensor` for the Python API and more details.

Example sensor definition::

    {
        "sensor_type": "DynamicsSensor",
        "socket": "COM",
        "configuration": {
           "UseCOM": true,
           "UseRPY": false
        }
    }

.. note::
    "UseCOM" and "UseRPY" must be opposites. In other words, one must be true while the other must be false.

Configuration
=============

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Key
     - Default
     - Meaning
   * - ``UseCOM``
     - ``true``
     - Report the state at the center of mass instead of the socket.
   * - ``UseRPY``
     - ``true``
     - Orientation as roll, pitch, yaw (18 items) instead of a quaternion ``x, y, z, w`` (19 items).
   * - ``ExportWaterSurface``
     - ``true``
     - Also send, every tick, the wave surface under the agent (see `Water surface block`_).
   * - ``WaterSurfaceGrid``
     - ``3``
     - Samples per side of the water surface grid.
   * - ``ExportGeometry``
     - ``false``
     - Also send the collision geometry of the agent's body, once (see `Geometry block`_).
   * - ``GeometrySource``
     - ``"complex"``
     - ``"complex"`` for the exact collision mesh, ``"simple"`` for its convex hulls and boxes.
   * - ``GeometryMaxVertices`` / ``GeometryMaxTriangles``
     - ``16384`` / ``32768``
     - Capacity of the geometry block. A larger mesh is not sent and :attr:`geometry` raises with the sizes it
       needs.

Output layout
=============

``sensor_data`` is the dynamics vector followed by the water surface block (when exported):

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Items
     - Content
   * - ``0:3``
     - Linear acceleration (m/s²)
   * - ``3:6``
     - Linear velocity (m/s)
   * - ``6:9``
     - Position (m)
   * - ``9:12``
     - Angular acceleration
   * - ``12:15``
     - Angular velocity (world frame)
   * - ``15:18`` / ``15:19``
     - Roll, pitch, yaw (``UseRPY`` true) / quaternion ``x, y, z, w`` (``UseRPY`` false)
   * - after the dynamics
     - Water surface block: ``1 + 3 * WaterSurfaceGrid²`` items

The geometry block is not part of ``sensor_data``; read it with :py:attr:`~biguasim.sensors.DynamicsSensor.geometry`.

Water surface block
===================

With ``ExportWaterSurface`` (the default) the engine samples the Water plugin's wave surface under the agent every
tick, on a ``WaterSurfaceGrid`` x ``WaterSurfaceGrid`` grid spanning the agent's horizontal bounds. The block is:

- ``[0]``: water contact, 1 if the bottom of the agent's bounds is below the surface at any sample;
- then ``x, y, surface z`` per sample, in world coordinates (meters). ``z`` includes the waves, and is NaN where no
  water body covers the point.

:py:attr:`~biguasim.sensors.DynamicsSensor.water_surface` returns it as ``{"contact": bool, "samples": (N, 3)}``:

.. code-block:: python

   surface = env.agents["boat-id0"].sensors["DynamicsSensor"].water_surface
   print(surface["contact"], surface["samples"][:, 2])   # contact flag, surface heights

The block travels in the same row as the dynamics, so the dynamics models read it directly and use it as the water
surface: buoyancy, drag in water or air, and which rotors or thrusters are under water all follow the actual surface
(waves included) instead of a flat ``z = 0``. In Pier-Harbor, for instance, the surface sits around ``z = 0.6 m``
and moves by about half a meter with the waves.

.. note::
    Worlds packaged before this block did not size the buffer for it. If ``ExportWaterSurface`` is left at its
    default, the client then drops the block with a warning and the models use a flat water level (``water_level``,
    0 m for the marine vehicles; no water for the UAV). Set ``"ExportWaterSurface": true`` explicitly to make such a
    world an error instead, or ``false`` to never send the block.

The :ref:`water-collision-sensor` with ``ReportSurface`` returns the same samples, for agents that do not need a
DynamicsSensor.

Geometry block
==============

With ``ExportGeometry`` the engine writes the collision geometry of the agent's body once, on the first tick. At the
end of :py:meth:`~biguasim.environments.BiguaSimEnvironment.reset`, every model that supports it receives it
(``set_geometry``): the marine vehicles build their hydrostatics and drag panels from it, the DjiMatrice its drag
panels, the TorpedoAUV its hull. :py:attr:`~biguasim.sensors.DynamicsSensor.geometry` returns ``vertices`` and
``triangles`` in the body frame (meters), the engine's ``center_of_mass`` and ``mass``, and the displaced
``volume`` (also available alone from :py:attr:`~biguasim.sensors.DynamicsSensor.volume`).

Use ``"GeometrySource": "simple"`` for vehicles whose complex collision is a large render mesh (DjiMatrice,
BlueBoat, BlueROV2): the convex hulls are a few hundred triangles. The TorpedoAUV's complex collision is closed and
small; use ``"complex"``.

.. code-block:: python

   {
       "sensor_type": "DynamicsSensor",
       "socket": "IMUSocket",
       "configuration": {"UseCOM": True, "UseRPY": False,
                         "ExportGeometry": True, "GeometrySource": "simple"}
   }
