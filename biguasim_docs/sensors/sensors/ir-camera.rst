====================
Infrared Camera (IR)
====================

An Infrared Camera sensor, which captures the agent's vision in thermal infrared.
The thermal physics are calculated automatically during initialization based on object semantic tags and Planck's Law, outputting a mapped thermal image.

See :py:class:`~biguasim.sensors.IRCamera` for the Python API.

Example sensor definition::

    {
        "sensor_type": "IRCamera",
        "sensor_name": "IRCamera",
        "configuration": {
            "CaptureWidth": 256,
            "CaptureHeight": 256
        }
    }