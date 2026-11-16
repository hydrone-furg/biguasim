==================
OpenWater-Hovering
==================

This scenario starts with a HoveringAUV near a submarine. Unless otherwise specified,
all sensors are named the same as their class name, ie IMUSensor is named "IMUSensor".

- ``auv0``: Main :ref:`HoveringAUV <hovering-auv-agent>` agent
    - :class:`~biguasim.sensors.IMUSensor` configured with noise, bias, and returns bias.
    - :class:`~biguasim.sensors.GPSSensor` gets measurements with N(1, 0.25) of the surface, actual measurement also has noise.
    - :class:`~biguasim.sensors.DVLSensor` configured with an elevation of 22.5 degrees, noise, and returns 4 range measurements.
    - :class:`~biguasim.sensors.DepthSensor` configured with noise.
    - :class:`~biguasim.sensors.PoseSensor` for ground truth.
    - :class:`~biguasim.sensors.VelocitySensor` for ground truth.

.. image:: starting.png
