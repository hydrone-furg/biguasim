.. _coverage-sensor:

===============
Coverage Sensor
===============

Measures how much of a target's surface another sensor on the same agent has seen, for inspection and coverage
planning tasks. Everything runs in the engine.

The engine spreads ``NumPoints`` points over the target's collision surface (area-weighted, so the count is a share
of the area) and, every tick, marks a point as viewed once it is:

- inside the observing sensor's field of view: a camera's view frustum (``RGBCamera``, ``DepthCamera``,
  ``FOVDetection``, ...), a sonar's range, azimuth and elevation, or a lidar's range and field of view;
- facing the sensor (the side of the surface the point lies on looks towards it);
- in line of sight (nothing blocks the ray from the sensor).

Viewed points stay viewed until :py:meth:`~biguasim.environments.BiguaSimEnvironment.reset`. The sensor returns
``[viewed, not_viewed]`` point counts (uint32); :py:meth:`~biguasim.sensors.CoverageSensor.coverage` turns a reading
into the viewed fraction.

See :py:class:`~biguasim.sensors.CoverageSensor` for the Python API.

Configuration
=============

- ``Target`` (required): a prop spawned with that name, else the actors tagged with it, else the actor with that
  object name.
- ``Sensor`` (required): name of the observing sensor on this agent. Its field of view is read from its own
  configuration.
- ``NumPoints``: points spread over the surface. Defaults to 1000.
- ``MinRange`` / ``MaxRange``: extra range limits in meters on top of the sensor's own (cameras have none). ``MaxRange``
  0 means no extra limit. Default 0.
- ``SurfaceOffset``: the line of sight ends this far above the surface (meters), so it does not hit the point's own
  face; also the thinnest wall told apart. Defaults to 0.01.
- ``Seed``: seed of the point placement. Defaults to 0.
- ``Debug``: draw the points in the world, red not viewed yet and green viewed. Debug draws also show in camera
  images. Defaults to false.
- ``DebugPointSize``: size of the debug points in pixels. Defaults to 8.

Example
=======

A DjiMatrice whose forward camera inspects a spawned prop named ``wall``:

.. code-block:: python

   import biguasim
   from biguasim.sensors import CoverageSensor

   scenario = {
       "package_name": "SkyDive", "world": "Pier-Harbor", "main_agent": "uav",
       "agents": [{
           "agent_name": "uav", "agent_type": "DjiMatrice",
           "sensors": [
               {"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                "configuration": {"UseCOM": True, "UseRPY": False}},
               {"sensor_type": "RGBCamera", "location": [0.3, 0, 0],
                "configuration": {"CaptureWidth": 256, "CaptureHeight": 256}},
               {"sensor_type": "CoverageSensor",
                "configuration": {"Target": "wall", "Sensor": "RGBCamera", "NumPoints": 2000}},
           ],
           "dynamics": {"batch_size": 1}, "control_abstraction": "cmd_pos_yaw",
           "location": [38, -301, 15], "rotation": [0, 0, 0],
       }],
   }

   env = biguasim.make(scenario_cfg=scenario)
   env.reset()
   # The sensor finds the target by the prop's name once it exists
   env.spawn_prop("box", location=[48, -301, 15], scale=[0.2, 6, 4], name="wall")
   for _ in range(60):
       state = env.step([38, -301, 15, 0])
   print(f"{100 * CoverageSensor.coverage(state['CoverageSensor']):.1f} % of the wall seen")

With a wall 10 m ahead and level with the camera, only its front face can be seen: about 46 % of its 52 m² surface.
