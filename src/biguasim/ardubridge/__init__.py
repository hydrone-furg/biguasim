"""
biguasim.ardubridge — ArduPilot JSON SITL bridge for BiguaSim.

Each vehicle flies with its ArduPilot firmware: DjiMatrice on ArduCopter, BlueBoat on Rover (boat), BlueROV2 and
BlueROV2Heavy on ArduSub, TorpedoAUV on ArduSub with a torpedo frame, Hydrone on ArduHydrone (an ArduCopter +
ArduSub fusion, from the ardupilot submodule's hydrone branch).

Quick start::

    import biguasim
    from biguasim.ardubridge import ArduBiguaSimRunner, SITL, VEHICLE_REGISTRY

    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Pier-Harbor",
                                                 location=[38, -301, 0])   # ArduSub boots at the surface
    env = biguasim.make(scenario_cfg=scenario)
    with SITL(profile) as sitl:
        ArduBiguaSimRunner(profile, env, port=sitl.json_port).run(sitl=sitl)
"""

from .bridge import ArduPilotBridge
from .frame import (
    depth_to_pressure,
    imu_from_dynamics,
    meters_to_gps,
    pos_nwu_to_ned,
    quat_glu2nwu_to_frd2ned,
    vel_nwu_to_ned,
)
from .offline import OfflineWorld
from .runner import ArduBiguaSimRunner, agent_dynamics
from .sitl import SITL, ardupilot_home
from .vehicle import VEHICLE_REGISTRY, Actuator, VehicleProfile, hydrone_quadplane_profile

__all__ = [
    "ArduPilotBridge",
    "ArduBiguaSimRunner",
    "OfflineWorld",
    "SITL",
    "VehicleProfile",
    "Actuator",
    "VEHICLE_REGISTRY",
    "hydrone_quadplane_profile",
    "agent_dynamics",
    "ardupilot_home",
    # frame utilities
    "pos_nwu_to_ned",
    "vel_nwu_to_ned",
    "imu_from_dynamics",
    "quat_glu2nwu_to_frd2ned",
    "depth_to_pressure",
    "meters_to_gps",
]


def main() -> None:
    """Entry point: print available vehicle profiles."""
    print("biguasim.ardubridge — available vehicle profiles:")
    for name, profile in VEHICLE_REGISTRY.items():
        print(f"  {name:15s}  {profile.firmware:12s}  {profile.num_motors} actuators  {profile.control_abstraction}")
