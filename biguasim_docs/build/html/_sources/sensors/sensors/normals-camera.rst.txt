======================
Surface Normals Camera
======================

A Surface Normals Camera sensor, which provides a geometric understanding of the environment. 
It captures the orientation of surfaces and returns a 3D vector (X, Y, Z) for each pixel, representing the direction the surface is facing relative to the camera.

See :py:class:`~biguasim.sensors.NormalsCamera` for the Python API.

Example sensor definition::

    {
        "sensor_type": "NormalsCamera",
        "sensor_name": "NormalsCamera",
        "configuration": {
            "CaptureWidth": 256,
            "CaptureHeight": 256
        }
    }