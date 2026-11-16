"""
Vehicle profiles for ArduPilot JSON SITL integration with BiguaSim.

Each VehicleProfile says which ArduPilot firmware flies the vehicle, with which parameters and scripts, and how each
of its BiguaSim actuators (the ``cmd_motor_speeds`` / ``cmd_rudders_sterns_motor_speed`` command, in order) is driven
by an ArduPilot servo channel.

The channel and the sign of every actuator follow from ArduPilot's own motor tables: a positive output of motor ``k``
must push the vehicle the way the motor's roll/pitch/yaw/throttle/forward/lateral factors say
(``AP_MotorsMatrix`` for Copter and the QuadPlane, ``AP_Motors6DOF`` for Sub). ``tests/unit/test_ardubridge.py``
checks every mapping against those factors with the vehicle's dynamics model.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

FIRMWARE_DIR = Path(__file__).resolve().parent / "firmware"


# ---------------------------------------------------------------------------
# PWM → actuator command
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Actuator:
    """One BiguaSim actuator driven by one ArduPilot servo channel.

    ``bidirectional``: ``neutral ± span`` µs (ArduSub thrusters, Rover throttles, fins) maps to ``[low, high]``,
    neutral to 0; ``low`` is reached at ``neutral - span``, ``high`` at ``neutral + span``, so asymmetric limits keep
    0 at neutral. Otherwise ``pwm_min..pwm_max`` maps to ``[0, high]`` (Copter and QuadPlane motors).

    With ``thrust_linear`` the pulse sets the thrust instead of the speed (thrust grows with ``speed * |speed|`` in
    the models): the command is ``sqrt(|t|)`` of the range. This is how a T200 behaves (its published thrust is close
    to linear in the pulse outside the deadband), and what ArduSub's and Rover's controllers expect; with a speed
    linear in the pulse, small corrections give almost no thrust and the heading hunts.

    A pulse outside the channel's range (0: channel not configured or not sending) commands 0.
    """
    channel: int            # 0-based servo channel (SERVO<channel+1>)
    low: float              # command at the low end of the range (≤ 0)
    high: float             # command at the high end of the range
    sign: float = 1.0       # ±1: a positive ArduPilot output gives a positive (sign 1) or negative command
    bidirectional: bool = True
    neutral: float = 1500.0
    span: float = 400.0
    pwm_min: float = 1000.0
    pwm_max: float = 2000.0
    thrust_linear: bool = False

    def __call__(self, pwm: float) -> float:
        pwm = float(pwm)
        if self.bidirectional:
            if not (self.neutral - 2 * self.span <= pwm <= self.neutral + 2 * self.span) or pwm == 0:
                return 0.0
            t = float(np.clip((pwm - self.neutral) / self.span, -1.0, 1.0))
            if self.thrust_linear:
                t = float(np.sign(t) * np.sqrt(abs(t)))
            value = t * (self.high if t >= 0 else -self.low)
        else:
            if pwm < self.pwm_min - 100 or pwm == 0:
                return 0.0
            t = float(np.clip((pwm - self.pwm_min) / (self.pwm_max - self.pwm_min), 0.0, 1.0))
            if self.thrust_linear:
                t = float(np.sqrt(t))
            value = t * self.high
        return self.sign * value


def _limits(agent_type: str, index: int = None) -> tuple:
    """The (min, max) actuator speed of a dynamics model's parameters; ``index`` picks air (0) / water (1)."""
    from biguasim.dynamics.agents import ModelsFactory   # torch: only once a profile is built

    params = ModelsFactory._types[agent_type]._params
    low, high = np.asarray(params["rotor_speed_min"]), np.asarray(params["rotor_speed_max"])
    if index is not None:
        low, high = low[index], high[index]
    return float(low), float(high)


# ---------------------------------------------------------------------------
# VehicleProfile
# ---------------------------------------------------------------------------

@dataclass
class VehicleProfile:
    """All configuration needed to bridge one BiguaSim vehicle to ArduPilot SITL."""

    name: str
    """BiguaSim agent_type string."""

    actuators: list
    """One :class:`Actuator` per entry of the BiguaSim command, in order."""

    control_abstraction: str
    """BiguaSim control abstraction the actuators are in."""

    firmware: str
    """ArduPilot SITL binary: 'arducopter', 'ardusub', 'ardurover' or 'arduplane'."""

    params: list
    """Parameter files loaded as SITL defaults, in order (later files win): ``firmware/params/<name>``, or
    ``ardupilot:<name>`` for ArduPilot's own ``Tools/autotest/default_params/<name>``."""

    scripts: list = field(default_factory=list)
    """Lua scripts (``firmware/scripts``) the firmware runs (SCR_ENABLE in the params)."""

    ap_factors: dict = field(default_factory=dict)
    """What a positive ArduPilot output on each actuator's channel must do, as ArduPilot's motor tables say:
    actuator index → signs of its (roll, pitch, yaw, throttle, forward, lateral) effect (throttle: up). Documentation
    and test reference; actuators a script drives are not listed."""

    patches: list = field(default_factory=list)
    """Firmware patches (``firmware/patches``) the firmware needs (build with ``firmware/build.sh``)."""

    physics_hz: int = 200
    """Physics rate (the scenario's ``ticks_per_sec``). ArduPilot gets one IMU sample per physics step and its
    pre-arm checks want them at 1.8 x its loop rate or more, so the parameter files set SCHED_LOOP_RATE to at most
    ``physics_hz / 1.8``."""

    @property
    def num_motors(self) -> int:
        return len(self.actuators)

    def pwm_to_command(self, pwm) -> list:
        """The BiguaSim command for the 16 (or 32) PWM channels of one ArduPilot servo packet."""
        return [actuator(pwm[actuator.channel]) for actuator in self.actuators]

    def param_files(self, ardupilot_dir: Path = None) -> list:
        from .sitl import ardupilot_home

        files = []
        for name in self.params:
            if name.startswith("ardupilot:"):
                base = Path(ardupilot_dir) if ardupilot_dir else ardupilot_home()
                files.append(base / "Tools" / "autotest" / "default_params" / name[len("ardupilot:"):])
            else:
                files.append(FIRMWARE_DIR / "params" / name)
        return files

    def script_files(self) -> list:
        return [FIRMWARE_DIR / "scripts" / name for name in self.scripts]


# ---------------------------------------------------------------------------
# VEHICLE_REGISTRY
# ---------------------------------------------------------------------------

def _sub_thrusters(agent_type, channels):
    low, high = _limits(agent_type)
    # ArduSub's thrusters push against the BiguaSim thrusters' positive direction (its SIM_Submarine pushes the
    # vertical ones down and the horizontal ones as the vectored frame's factors say): all signs are -1.
    return [Actuator(c, low, high, sign=-1.0, thrust_linear=True) for c in channels]


def _djimatrice() -> VehicleProfile:
    _, high = _limits("DjiMatrice")
    # BiguaSim r1..r4: front-left (CW), front-right (CCW), rear-right (CW), rear-left (CCW).
    # ArduCopter quad X: MOT1 front-right CCW, MOT2 back-left CCW, MOT3 front-left CW, MOT4 back-right CW.
    return VehicleProfile(
        name="DjiMatrice",
        actuators=[Actuator(c, 0.0, high, bidirectional=False) for c in (2, 0, 3, 1)],
        control_abstraction="cmd_motor_speeds",
        firmware="arducopter",
        params=["ardupilot:copter.parm", "djimatrice.parm"],
        ap_factors={0: (0.5, 0.5, -1, 1, 0, 0), 1: (-0.5, 0.5, 1, 1, 0, 0),
                    2: (-0.5, -0.5, -1, 1, 0, 0), 3: (0.5, -0.5, 1, 1, 0, 0)},
    )


def _bluerov2() -> VehicleProfile:
    # BiguaSim r1, r2: left / right vertical; r3..r6: front-left, front-right, rear-left, rear-right horizontal.
    # ArduSub vectored: MOT1..4 front-right, front-left, rear-right, rear-left; MOT5 right, MOT6 left vertical.
    return VehicleProfile(
        name="BlueROV2",
        actuators=_sub_thrusters("BlueROV2", (5, 4, 1, 0, 3, 2)),
        control_abstraction="cmd_motor_speeds",
        firmware="ardusub",
        params=["ardupilot:sub.parm", "bluerov2.parm"],
        ap_factors={0: (-1, 0, 0, -1, 0, 0), 1: (1, 0, 0, -1, 0, 0),
                    2: (0, 0, -1, 0, -1, -1), 3: (0, 0, 1, 0, -1, 1),
                    4: (0, 0, 1, 0, 1, -1), 5: (0, 0, -1, 0, 1, 1)},
    )


def _bluerov2heavy() -> VehicleProfile:
    # BiguaSim r1..r4: front-left, front-right, rear-left, rear-right vertical; r5..r8 as the BlueROV2's r3..r6.
    # ArduSub vectored 6DOF: MOT5..8 front-right, front-left, rear-right, rear-left vertical.
    return VehicleProfile(
        name="BlueROV2Heavy",
        actuators=_sub_thrusters("BlueROV2Heavy", (5, 4, 7, 6, 1, 0, 3, 2)),
        control_abstraction="cmd_motor_speeds",
        firmware="ardusub",
        params=["ardupilot:sub-6dof.parm", "bluerov2heavy.parm"],
        ap_factors={0: (-1, -1, 0, -1, 0, 0), 1: (1, -1, 0, -1, 0, 0),
                    2: (-1, 1, 0, -1, 0, 0), 3: (1, 1, 0, -1, 0, 0),
                    4: (0, 0, -1, 0, -1, -1), 5: (0, 0, 1, 0, -1, 1),
                    6: (0, 0, 1, 0, 1, -1), 7: (0, 0, -1, 0, 1, 1)},
    )


def _blueboat() -> VehicleProfile:
    low, high = _limits("BlueBoat")
    # Skid steering: SERVO1 ThrottleLeft (port, r1), SERVO3 ThrottleRight (starboard, r2), as on the BlueBoat.
    return VehicleProfile(
        name="BlueBoat",
        actuators=[Actuator(0, low, high, thrust_linear=True), Actuator(2, low, high, thrust_linear=True)],
        control_abstraction="cmd_motor_speeds",
        firmware="ardurover",
        params=["ardupilot:rover.parm", "ardupilot:rover-skid.parm", "blueboat.parm"],
        ap_factors={0: (0, 0, 1, 0, 1, 0), 1: (0, 0, -1, 0, 1, 0)},
    )


def _torpedoauv() -> VehicleProfile:
    from biguasim.dynamics.agents import ModelsFactory

    params = ModelsFactory._types["TorpedoAUV"]._params
    fin = float(params["deltaMax_fin_deg"])
    rpm_low, rpm_high = _limits("TorpedoAUV")
    # ArduSub torpedo frame (firmware/patches): MOT1 propeller, MOT2/3 top/bottom rudder, MOT4/5 left/right stern
    # plane. The command is [rudder_top, rudder_bottom, stern_left, stern_right, rpm], fins in degrees.
    return VehicleProfile(
        name="TorpedoAUV",
        actuators=[Actuator(1, -fin, fin), Actuator(2, -fin, fin), Actuator(3, -fin, fin), Actuator(4, -fin, fin),
                   Actuator(0, rpm_low, rpm_high, thrust_linear=True)],
        control_abstraction="cmd_rudders_sterns_motor_speed",
        firmware="ardusub",
        params=["ardupilot:sub.parm", "torpedoauv.parm"],
        patches=["ardusub-torpedo-frame.patch"],
        ap_factors={0: (0, 0, 1, 0, 0, 0), 1: (0, 0, 1, 0, 0, 0),
                    2: (0, 1, 0, 0, 0, 0), 3: (0, 1, 0, 0, 0, 0), 4: (0, 0, 0, 0, 1, 0)},
    )


def _hydrone() -> VehicleProfile:
    _, air = _limits("Hydrone", 0)
    water_low, water_high = _limits("Hydrone", 1)
    # ArduHydrone (ArduCopter + ArduSub fusion, built from the ardupilot submodule's hydrone branch; see
    # firmware/HYDRONE_FIRMWARE_PLAN.md). Air: quad plus on SERVO1..4. BiguaSim r1..r4: front (CW), right (CCW),
    # rear (CW), left (CCW); ArduPilot plus: MOT1 right CCW, MOT2 left CCW, MOT3 front CW, MOT4 back CW.
    # Water: SERVO5..8 (MOT5..8 of AP_MotorsHydrone) as r5 front vertical, r6 right forward, r7 rear vertical, r8 left
    # forward, in the BiguaSim thrusters' own directions; 1500 +- 400 us sets the thrust (ArduSub-style).
    return VehicleProfile(
        name="Hydrone",
        actuators=[Actuator(c, 0.0, air, bidirectional=False) for c in (2, 0, 3, 1)]
                  + [Actuator(c, water_low, water_high, thrust_linear=True) for c in (4, 5, 6, 7)],
        control_abstraction="cmd_motor_speeds",
        firmware="arduhydrone",
        params=["ardupilot:copter.parm", "hydrone.parm"],
        physics_hz=400,
        ap_factors={0: (0, 1, -1, 1, 0, 0), 1: (-1, 0, 1, 1, 0, 0),
                    2: (0, -1, -1, 1, 0, 0), 3: (1, 0, 1, 1, 0, 0)},
    )


def hydrone_quadplane_profile() -> VehicleProfile:
    """The Hydrone on stock ArduPlane as a QuadPlane, with the hydrone_medium.lua water mixer (before ArduHydrone).

    Kept for its under-water tests until ArduHydrone flies under water; ``VEHICLE_REGISTRY["Hydrone"]`` is
    ArduHydrone."""
    profile = _hydrone()
    # the Lua mixer sets the water thrusters' speed (linear in the pulse), not their thrust
    profile.actuators = profile.actuators[:4] + [dataclasses.replace(a, thrust_linear=False)
                                                 for a in profile.actuators[4:]]
    profile.firmware = "arduplane"
    profile.params = ["ardupilot:quadplane.parm", "hydrone_quadplane.parm"]
    profile.scripts = ["hydrone_medium.lua"]
    profile.patches = ["arduplane-vfwd-alt-off.patch"]
    return profile


_BUILDERS = {
    "DjiMatrice": _djimatrice,
    "BlueROV2": _bluerov2,
    "BlueROV2Heavy": _bluerov2heavy,
    "BlueBoat": _blueboat,
    "TorpedoAUV": _torpedoauv,
    "Hydrone": _hydrone,
}


class _Registry(dict):
    """Builds each profile on first use (the actuator limits come from the dynamics models, which import torch)."""

    def __missing__(self, key):
        if key not in _BUILDERS:
            raise KeyError(f"No ArduPilot profile for {key!r}; available: {list(_BUILDERS)}")
        self[key] = profile = _BUILDERS[key]()
        return profile

    def __iter__(self):
        return iter(_BUILDERS)

    def __len__(self):
        return len(_BUILDERS)

    def keys(self):
        return _BUILDERS.keys()

    def items(self):
        return [(key, self[key]) for key in _BUILDERS]

    def values(self):
        return [self[key] for key in _BUILDERS]

    def __contains__(self, key):
        return key in _BUILDERS


VEHICLE_REGISTRY: dict = _Registry()
