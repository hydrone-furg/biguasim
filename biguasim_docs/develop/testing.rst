.. _testing:

=======
Testing
=======

BiguaSim has three kinds of tests. Run them from the root of the client repository, with the development copy
installed (``pip install --editable .``).

Offline tests (no world needed)
===============================

``tests/unit`` tests the client on its own, with the engine replaced by a stand-in that integrates the
accelerations the models return (gravity off, no contacts, a flat water surface at ``z = 0``). They need neither a
world binary nor a GPU:

.. code-block:: bash

   pytest tests/unit -m "not slow and not sitl"   # ~5 min: batching, water surface, PID, ArduPilot bridge
   pytest tests/unit -m slow                      # ~1.5 min: closed-loop position holds of every vehicle
   pytest tests/unit -m sitl                      # ArduPilot firmwares flying the models (minutes each)

What they check:

- **Batch = single robots**: for every vehicle and control abstraction, a batch of robots in different states gets,
  over several ticks, exactly what one model per robot gives.
- **Water surface**: a flat surface reported by the DynamicsSensor's water block gives the same accelerations as the
  flat ``water_level`` fallback; NaN samples (no water body) are skipped; a missing block warns once; a malformed
  block is rejected.
- **DynamicsSensor layout**: the water block sits between the dynamics and the geometry; against an engine that
  sized the buffer without it, the default block is dropped with a warning and an explicit one raises.
- **Controllers**: the PID derivative does not kick on its first call; an unknown marine ``controller`` raises.
- **Closed loop** (``slow``): DjiMatrice, BlueROV2 (both controllers), BlueROV2Heavy and Hydrone reach and hold a
  ``cmd_pos_yaw`` goal within 0.25 m and 5°, in batches of two.
- **ArduPilot bridge**: every vehicle's servo channels push it the way ArduPilot's motor tables mean (checked with
  the dynamics models), PWM conversion, frame conversions, the JSON protocol and the lockstep loop against a
  stand-in autopilot.
- **ArduPilot SITL** (``sitl``): each vehicle's firmware boots, arms and flies a short mission against its model
  (:ref:`ardupilot`). Skipped when the firmwares are not built; the BlueBoat needs its exported hull in
  ``tests/unit/data/geometry``.

Integration tests (installed world)
===================================

``tests/agents``, ``tests/sensors`` and ``tests/scenarios`` start a packaged world and drive it. They need the
package each test names installed (``biguasim.install(...)``), a GPU, and a display for the tests that open the
viewport.

.. code-block:: bash

   pytest tests/agents/test_djimatrice.py

Live checks against the editor build
====================================

Engine changes that are built but not yet packaged can be tested by running the editor build as the world and
attaching the client to it (``start_world=False`` with the engine's UUID). The repository's ``tests`` folder (next
to the client and engine) has ``run_viewport_test.sh``, which starts the editor on a map, waits for the server and
runs a script against it:

.. code-block:: bash

   ENGINE_ARGS="-RenderOffScreen" ./run_viewport_test.sh live_water_probe.py
   ENGINE_ARGS="-RenderOffScreen" ./run_viewport_test.sh coverage_test.py

``live_water_probe.py`` checks the DynamicsSensor water surface block (layout, contact, samples under the agent,
equality with the WaterCollisionSensor), that the center of mass does not depend on the spawn location, and position
holds of a BlueBoat, DjiMatrice, two BlueROV2 and a batch of two Hydrones on Pier-Harbor. ``coverage_test.py``
compares the CoverageSensor with the coverage expected from the geometry of a wall and an occluder.
