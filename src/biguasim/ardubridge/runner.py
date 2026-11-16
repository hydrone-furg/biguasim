"""
ArduBiguaSimRunner — wires a BiguaSim environment to an ArduPilotBridge, in lockstep with ArduPilot.

Usage::

    import biguasim
    from biguasim.ardubridge import ArduBiguaSimRunner, SITL, VEHICLE_REGISTRY

    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Pier-Harbor",
                                                 location=[38, -301, 0])   # ArduSub boots at the surface
    env = biguasim.make(scenario_cfg=scenario)
    with SITL(profile) as sitl:             # the ArduSub firmware, with BiguaSim's BlueROV2 parameters
        ArduBiguaSimRunner(profile, env, port=sitl.json_port).run(sitl=sitl)

Then connect a ground station (QGroundControl, MAVProxy) to UDP 14550 or TCP 5760.
"""

from __future__ import annotations

import numpy as np

from .bridge import ArduPilotBridge
from .vehicle import VehicleProfile


def agent_dynamics(state: dict, agent_name: str | None = None) -> np.ndarray:
    """The DynamicsSensor vector of an agent (its first copy) in a state returned by ``step``/``tick``. A
    single-agent state is not keyed by agent name."""
    agent_state = state if "DynamicsSensor" in state else state[agent_name]
    data = agent_state["DynamicsSensor"]
    return np.asarray(data[0] if isinstance(data, list) else data, dtype=np.float64)


class ArduBiguaSimRunner:
    """Steps a BiguaSim environment (or an :class:`~biguasim.ardubridge.offline.OfflineWorld`) once per ArduPilot
    servo packet and answers each packet with the new state."""

    def __init__(
        self,
        profile: VehicleProfile,
        env,
        agent_name: str | None = None,
        origin: tuple | None = None,
        address: str = "127.0.0.1",
        port: int = 9002,
        other_commands: dict | None = None,
        verbose: bool = False,
    ):
        """
        :param profile: Vehicle profile of the agent ArduPilot flies.
        :param env: The environment, already made. It must hold the agent with a DynamicsSensor (UseRPY false) and
            the profile's control abstraction; when it holds several agents, the others take ``other_commands``.
        :param agent_name: The agent ArduPilot flies, when the environment has several (default: the scenario's
            ``main_agent``).
        :param origin: World point (m) that stands for ArduPilot's home (see :class:`ArduPilotBridge`). Default: the
            world's x = y = 0 at the height of the water surface under the vehicle when the run starts (from the
            DynamicsSensor's water surface block; 0 without water), so that ArduSub's depth is the depth below the
            actual surface.
        :param port: UDP port of the JSON physics backend: ``9002 + 10 * instance`` of the SITL.
        :param other_commands: The commands of the environment's other agents, by agent name (``env.step`` needs
            one per agent).
        """
        self._profile = profile
        self._env = env
        self._verbose = verbose
        self._origin = origin
        self._bridge = ArduPilotBridge(profile, address=address, port=port, origin=origin or (0.0, 0.0, 0.0))
        self._agent_name = agent_name
        self._multi_agent = False
        self._other_commands = dict(other_commands or {})
        self.sim_time = 0.0
        self.command = [0.0] * profile.num_motors

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def _step(self, command) -> dict:
        if self._multi_agent:
            return self._env.step({**self._other_commands, self._agent_name: list(command)})
        return self._env.step(list(command))

    def run(self, duration: float | None = None, on_step=None, sitl=None, reset: bool = True) -> None:
        """
        Lockstep loop. Returns after ``duration`` simulated seconds (``None``: until Ctrl-C).

        :param on_step: Called after every step as ``on_step(runner, dynamics)``.
        :param sitl: The :class:`~biguasim.ardubridge.sitl.SITL` feeding the bridge, to stop when it dies.
        :param reset: Reset the environment first. Pass False after resetting it yourself, e.g. to spawn props (they
            do not survive a reset).
        """
        bridge = self._bridge
        bridge.bind()

        state = self._env.reset() if reset else self._env.tick()
        if "DynamicsSensor" not in state:
            agents = [name for name in state if name != "t"]
            self._agent_name = self._agent_name or getattr(self._env, "_scenario", {}).get("main_agent")
            if self._agent_name not in agents:
                raise ValueError(f"Pick the agent ArduPilot flies with agent_name, one of {agents}")
            self._multi_agent = True
        dynamics = agent_dynamics(state, self._agent_name)
        if self._origin is None:
            surface = dynamics[19 + 1 + 2::3]
            level = float(np.nanmean(surface)) if surface.size and np.any(np.isfinite(surface)) else 0.0
            bridge._origin = (0.0, 0.0, level)
        print(f"ArduPilot home is the world point {np.round(bridge._origin, 3)}")
        last_frame, last_json = None, None
        start_time = state.get("t", 0.0)

        print(f"Running the {self._profile.name} bridge on UDP {bridge._port} (Ctrl-C to stop)...")
        try:
            while duration is None or self.sim_time < duration:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    if sitl is not None and sitl.poll() is not None:
                        raise RuntimeError(f"ArduPilot SITL exited with code {sitl.poll()}, see {sitl.log_file}")
                    continue
                if frame == last_frame and last_json is not None:
                    # ArduPilot resends the servos when a state got lost: answer again without stepping
                    bridge.send_state(last_json)
                    continue
                last_frame = frame

                self.command = bridge.pwm_to_motor_cmds(pwm)
                state = self._step(self.command)
                dynamics = agent_dynamics(state, self._agent_name)
                self.sim_time = state.get("t", self.sim_time) - start_time

                last_json = bridge.build_json_state(dynamics, self.sim_time)
                bridge.send_state(last_json)
                if on_step is not None:
                    on_step(self, dynamics)

                if self._verbose and frame % 400 == 0:
                    print(f"  t={self.sim_time:.2f}s frame={frame} pos={np.round(dynamics[6:9], 2)} "
                          f"cmd={[round(c, 1) for c in self.command]}")
        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()

    # ------------------------------------------------------------------
    # Scenario builder
    # ------------------------------------------------------------------

    @staticmethod
    def build_scenario(
        profile: VehicleProfile,
        package_name: str,
        world: str,
        agent_name: str = "",
        location: list | None = None,
        rotation: list | None = None,
        ticks_per_sec: int | None = None,
        **overrides,
    ) -> dict:
        """
        Build a minimal BiguaSim scenario dict with the sensor ArduPilot needs: a DynamicsSensor (position,
        velocity, attitude, and the accelerations the bridge turns into the IMU), exporting the collision geometry
        the dynamics models use.

        ``ticks_per_sec`` defaults to the profile's ``physics_hz``, which its parameter files are made for.

        Extra keyword args are merged into the top-level scenario dict.
        To override agent fields, pass them as overrides prefixed with ``agent_``, e.g.
        ``agent_location=[10, 0, 0]``.
        """
        if not agent_name:
            agent_name = profile.name.lower()

        # The TorpedoAUV's complex collision is its closed hull; the other vehicles' is a large render mesh.
        geometry = "complex" if profile.name == "TorpedoAUV" else "simple"
        sensors = [{
            "sensor_type": "DynamicsSensor",
            "socket": "IMUSocket",
            "configuration": {"UseCOM": True, "UseRPY": False, "ExportGeometry": True, "GeometrySource": geometry},
        }]

        agent_cfg = {
            "agent_name": agent_name,
            "agent_type": profile.name,
            "sensors": sensors,
            "dynamics": {"batch_size": 1},
            "control_abstraction": profile.control_abstraction,
            "location": location or [0, 0, 0],
            "rotation": rotation or [0.0, 0.0, 0.0],
        }

        # Allow per-agent overrides via agent_ prefix
        for key in list(overrides.keys()):
            if key.startswith("agent_"):
                agent_cfg[key[len("agent_"):]] = overrides.pop(key)

        scenario = {
            "package_name": package_name,
            "world": world,
            "main_agent": agent_name,
            "ticks_per_sec": ticks_per_sec or profile.physics_hz,
            "frames_per_sec": False,
            "agents": [agent_cfg],
        }
        scenario.update(overrides)
        return scenario

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "ArduBiguaSimRunner":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self._bridge.close()
