"""
Coordinate frame conversions: BiguaSim (NWU world, GLU body) → ArduPilot (NED world, FRD body).

BiguaSim world: NWU  — x=North, y=West,    z=Up
BiguaSim body:  GLU  — x=forward, y=left,  z=up
ArduPilot world: NED — x=North, y=East,    z=Down
ArduPilot body:  FRD — x=forward, y=right, z=down

Both world and body conversions are a 180° rotation about the x-axis: Rx(π).
Quaternion chain: R_frd2ned = Rx(π) * R_glu2nwu * Rx(π)
"""

import functools
import math

import numpy as np
from scipy.spatial.transform import Rotation as R

GRAVITY = 9.80665
_EARTH_RADIUS_M = 6_371_000.0


@functools.lru_cache(maxsize=None)
def _rx180() -> R:
    # Built on first use, not at import (the docs import the package with torch mocked, which scipy trips on)
    return R.from_euler('x', np.pi)


_FLIP_YZ = np.array([1.0, -1.0, -1.0])


def pos_nwu_to_ned(pos, origin=(0.0, 0.0, 0.0)) -> list:
    """
    BiguaSim position [x, y, z] (NWU, m) → ArduPilot JSON ``position`` [north, east, down] (m) from ``origin``.

    ArduPilot reads the JSON ``position`` as an offset from its home (the SITL ``--home`` location), so ``origin``
    is the world point that stands for home. For ArduSub its altitude is also the water surface: the simulated
    barometer turns the altitude below home into depth.
    """
    return ((np.asarray(pos, dtype=np.float64)[:3] - np.asarray(origin, dtype=np.float64)) * _FLIP_YZ).tolist()


def vel_nwu_to_ned(vel) -> list:
    """BiguaSim velocity [vn, vw, vu] → NED [vn, ve, vd]."""
    return (np.asarray(vel, dtype=np.float64)[:3] * _FLIP_YZ).tolist()


def quat_glu2nwu_to_frd2ned(q_xyzw) -> list:
    """
    BiguaSim DynamicsSensor quaternion [x,y,z,w] (GLU→NWU) → [w,x,y,z] (FRD→NED).
    R_frd2ned = Rx(π) * R_glu2nwu * Rx(π)
    scipy as_quat() returns [x,y,z,w]; ArduPilot JSON wants [w,x,y,z].
    Canonical form enforced: w >= 0 (q and -q represent the same rotation).
    """
    R_glu2nwu = R.from_quat(q_xyzw)  # scipy: [x,y,z,w]
    R_frd2ned = _rx180() * R_glu2nwu * _rx180()
    x, y, z, w = R_frd2ned.as_quat()
    if w < 0.0:  # enforce canonical form
        w, x, y, z = -w, -x, -y, -z
    return [float(w), float(x), float(y), float(z)]


def imu_from_dynamics(accel_nwu, angvel_nwu, q_xyzw) -> tuple:
    """
    IMU readings in the FRD body frame from the DynamicsSensor's world-frame kinematics.

    The accelerometer measures the specific force: the acceleration minus gravity, in the body frame, so a vehicle
    at rest reads [0, 0, -g] in FRD. The gyro is the angular velocity in the body frame.

    :param accel_nwu: Linear acceleration of the body, NWU world frame (m/s²).
    :param angvel_nwu: Angular velocity, NWU world frame (rad/s), as the DynamicsSensor reports it.
    :param q_xyzw: Body (GLU) → world (NWU) quaternion [x, y, z, w].
    :returns: ``(accel_body, gyro)`` as FRD lists.
    """
    world_to_body = R.from_quat(q_xyzw).inv()
    specific_force = np.asarray(accel_nwu, dtype=np.float64) + [0.0, 0.0, GRAVITY]
    accel_glu = world_to_body.apply(specific_force)
    gyro_glu = world_to_body.apply(np.asarray(angvel_nwu, dtype=np.float64))
    return (accel_glu * _FLIP_YZ).tolist(), (gyro_glu * _FLIP_YZ).tolist()


def depth_to_pressure(depth_z_up: float, rho: float = 1026.0) -> float:
    """
    BiguaSim DepthSensor z_up (metres, positive=up) → absolute pressure (Pa).
    Assumes seawater density rho=1026 kg/m³.
    """
    depth_m = max(0.0, -depth_z_up)  # z_up negative when submerged
    return 101325.0 + rho * GRAVITY * depth_m


def meters_to_gps(north_m: float, east_m: float,
                  lat0_deg: float, lon0_deg: float) -> tuple:
    """Local NED offset (metres) → GPS lat/lon (degrees)."""
    lat = lat0_deg + math.degrees(north_m / _EARTH_RADIUS_M)
    lon = lon0_deg + math.degrees(east_m / (_EARTH_RADIUS_M * math.cos(math.radians(lat0_deg))))
    return lat, lon
