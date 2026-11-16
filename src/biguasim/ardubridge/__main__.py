"""
Fly a BiguaSim vehicle with its ArduPilot firmware:

    python -m biguasim.ardubridge BlueROV2 --package SkyDive --world Pier-Harbor --location 0 0 0
    python -m biguasim.ardubridge DjiMatrice --offline              # no engine: the model alone, flat ground at z=0
    python -m biguasim.ardubridge --list

It starts the ArduPilot SITL firmware of the vehicle (with BiguaSim's parameters and scripts), the world, and the
bridge between them. Connect a ground station to UDP 14550 (QGroundControl, MAVProxy) or TCP 5760.
"""

import argparse

from . import VEHICLE_REGISTRY, main as list_profiles
from .offline import OfflineWorld
from .runner import ArduBiguaSimRunner
from .sitl import SITL


def _parse():
    parser = argparse.ArgumentParser(prog="python -m biguasim.ardubridge", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("vehicle", nargs="?", help=f"one of {list(VEHICLE_REGISTRY)}")
    parser.add_argument("--list", action="store_true", help="list the vehicle profiles")
    parser.add_argument("--package", default="SkyDive", help="world package (default SkyDive)")
    parser.add_argument("--world", default="Pier-Harbor", help="world (default Pier-Harbor)")
    parser.add_argument("--location", nargs=3, type=float, metavar=("X", "Y", "Z"), default=None,
                        help="spawn location, m (default 0 0 0: subs boot at the surface, see the docs)")
    parser.add_argument("--yaw", type=float, default=0.0, help="spawn heading, deg")
    parser.add_argument("--origin", nargs=3, type=float, metavar=("X", "Y", "Z"), default=None,
                        help="world point that stands for ArduPilot's home; its height is the water surface "
                             "(default: x = y = 0 at the water surface under the vehicle)")
    parser.add_argument("--offline", action="store_true", help="no engine: OfflineWorld")
    parser.add_argument("--ground", type=float, default=None,
                        help="offline: height of a flat ground (default 0 for DjiMatrice, none otherwise)")
    parser.add_argument("--editor", metavar="UUID", default=None,
                        help="attach to a running engine (editor build) with this UUID instead of starting the world")
    parser.add_argument("--instance", type=int, default=0, help="SITL instance: shifts every port by 10 x I")
    parser.add_argument("--param", action="append", default=[], metavar="NAME=VALUE",
                        help="extra ArduPilot parameter (repeatable)")
    parser.add_argument("--keep-params", action="store_true", help="keep the EEPROM of the run directory")
    parser.add_argument("--run-dir", default=None, help="SITL working directory (EEPROM, logs)")
    parser.add_argument("--viewport", action="store_true", help="show the engine's viewport")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def run() -> None:
    args = _parse()
    if args.list or not args.vehicle:
        list_profiles()
        return
    profile = VEHICLE_REGISTRY[args.vehicle]
    location = args.location or [0.0, 0.0, 0.0]

    if args.offline:
        ground = args.ground if args.ground is not None else (0.0 if profile.name == "DjiMatrice" else None)
        env = OfflineWorld(profile.name, location=location, yaw_deg=args.yaw, ticks_per_sec=profile.physics_hz,
                           ground=ground, water_level=args.origin[2] if args.origin else 0.0)
    else:
        import biguasim

        scenario = ArduBiguaSimRunner.build_scenario(profile, package_name=args.package, world=args.world,
                                                     location=list(location), rotation=[0.0, 0.0, args.yaw])
        if args.editor:
            from biguasim.environments import BiguaSimEnvironment

            env = BiguaSimEnvironment(scenario=scenario, start_world=False, uuid=args.editor, verbose=args.verbose)
        else:
            env = biguasim.make(scenario_cfg=scenario, show_viewport=args.viewport, verbose=args.verbose)

    params = dict(p.split("=", 1) for p in args.param)
    with SITL(profile, instance=args.instance, extra_params=params, wipe=not args.keep_params,
              run_dir=args.run_dir) as sitl:
        print(f"{profile.name} on {profile.firmware}: MAVLink on UDP 14550 and {sitl.mavlink_url}; "
              f"firmware log {sitl.log_file}")
        ArduBiguaSimRunner(profile, env, origin=tuple(args.origin) if args.origin else None, port=sitl.json_port,
                           verbose=args.verbose).run(sitl=sitl)


if __name__ == "__main__":
    run()
