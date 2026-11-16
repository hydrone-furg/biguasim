.. _water-collision-sensor:

======================
Water Collision Sensor
======================

Returns true while the agent is in contact with water: touching the wave surface or under it. It is the water
counterpart of the :py:class:`~biguasim.sensors.CollisionSensor`: water bodies only overlap, so instead of a
one-tick pulse on impact the value holds for as long as the contact lasts.

The Water plugin's surface meshes have no collision, so the engine gives each water body an overlap-only volume
reaching the highest wave crest, and compares the agent with the actual wave surface on a grid under it. Contact
means that the bottom of the agent's bounds is below the surface at some grid sample (or that the agent hit a water
body).

See :py:class:`~biguasim.sensors.WaterCollisionSensor` for the Python API.

Configuration
=============

- ``ReportSurface``: Also return the wave surface under the agent. The output becomes a float array
  ``[contact (0/1), x0, y0, z0, x1, y1, z1, ...]``: one ``(x, y, surface z)`` per sample in world coordinates
  (meters), ``z`` NaN where no water body covers the point. Defaults to false (a single bool).
- ``SurfaceGrid``: Samples per side of the grid over the agent's horizontal bounds, used for contact and for the
  reported surface. Defaults to 3.

Example sensor definition::

    {
        "sensor_type": "WaterCollisionSensor",
        "configuration": {
            "ReportSurface": true,
            "SurfaceGrid": 3
        }
    }

.. note::
    The :ref:`dynamics-sensor` sends the same surface samples by default (``ExportWaterSurface``), and the dynamics
    models use them. For the same agent and grid, both sensors report identical values in the same tick; use this
    sensor when you only need the contact, or for agents without a DynamicsSensor.
