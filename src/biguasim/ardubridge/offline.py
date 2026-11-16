"""
OfflineWorld — one BiguaSim vehicle without the engine, for trying ArduPilot parameters and testing the bridge.

The vehicle's dynamics model runs exactly as in a BiguaSim environment; the world around it is reduced to what the
engine would add: integrating the accelerations the model returns (the engine adds no forces of its own), a flat
water surface, and optionally a flat ground. No collisions other than the ground, no waves. Without a ``geometry``
the models use their box hulls; pass one saved from the engine (:func:`save_geometry`) to use the exported hull.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


class OfflineWorld:
    """Steps like a single-agent :class:`~biguasim.environments.BiguaSimEnvironment` with client dynamics:
    ``step(command)`` returns ``{"DynamicsSensor": [vector], "t": time}`` and applies the command from the next step
    on, as the engine does."""

    def __init__(self, agent_type: str, location=(0.0, 0.0, 0.0), yaw_deg: float = 0.0, ticks_per_sec: int = 200,
                 control_abstraction: str = None, water_level: float = 0.0, ground: float = None,
                 water_grid: int = 3, params: dict = None, geometry=None):
        """
        :param agent_type: BiguaSim agent type, e.g. ``"BlueROV2"``.
        :param location: Spawn position (m, NWU).
        :param ticks_per_sec: Physics rate; one step lasts ``1 / ticks_per_sec``.
        :param control_abstraction: The model's control abstraction (default: the model's own default).
        :param water_level: Height of the flat water surface; ``None`` for no water.
        :param ground: Height of a flat ground the vehicle rests on; ``None`` for no ground.
        :param params: Overrides of the model's parameters.
        :param geometry: The DynamicsSensor geometry of the vehicle (``sensor.geometry``), or the ``.npz`` file
            :func:`save_geometry` wrote it to; the model builds its hull from it as in the environment.
        """
        from biguasim.dynamics.agents import ModelsFactory

        cls = ModelsFactory._types[agent_type]
        kwargs = {"batch_size": 1}
        if control_abstraction:
            kwargs["control_abstraction"] = control_abstraction
        if params:
            kwargs["params"] = {**cls._params, **params}
        self.model = cls(**kwargs)
        self.dt = 1.0 / ticks_per_sec
        self.model.dt = self.dt
        if geometry is not None:
            self.model.set_geometry([load_geometry(geometry) if not isinstance(geometry, dict) else geometry])
        self.water_level = water_level
        self.ground = ground
        self.grid = water_grid
        self.ticks = 0

        self.x = np.asarray(location, dtype=np.float64).copy()
        self.v = np.zeros(3)
        self.w = np.zeros(3)                        # world frame, as the engine reports it
        self.q = R.from_euler("z", yaw_deg, degrees=True).as_quat()   # body → world [x, y, z, w]
        self.a = np.zeros(3)                        # acceleration over the last step, as measured
        self._pending = np.zeros(6)                 # accelerations the model asked for, applied on the next step

    # ------------------------------------------------------------------

    def _water_block(self) -> np.ndarray:
        offsets = np.linspace(-0.3, 0.3, self.grid)
        if self.water_level is None:
            samples = [[self.x[0] + dx, self.x[1] + dy, np.nan] for dx in offsets for dy in offsets]
            contact = 0.0
        else:
            samples = [[self.x[0] + dx, self.x[1] + dy, self.water_level] for dx in offsets for dy in offsets]
            contact = float(self.x[2] - 0.3 <= self.water_level)
        return np.concatenate([[contact], np.ravel(samples)])

    def dynamics(self) -> np.ndarray:
        """The DynamicsSensor vector (UseRPY false) with its water surface block."""
        row = np.zeros(19)
        row[0:3], row[3:6], row[6:9] = self.a, self.v, self.x
        row[12:15], row[15:19] = self.w, self.q
        return np.concatenate([row, self._water_block()])

    def _integrate(self) -> None:
        v_old = self.v.copy()
        self.v = self.v + self._pending[:3] * self.dt
        self.w = self.w + self._pending[3:] * self.dt
        self.x = self.x + self.v * self.dt
        self.q = (R.from_rotvec(self.w * self.dt) * R.from_quat(self.q)).as_quat()
        if self.ground is not None and self.x[2] <= self.ground and self.v[2] <= 0:
            # Resting on the ground: no sinking, sliding or tipping, the heading kept
            self.x[2] = self.ground
            self.v[:] = 0.0
            self.w[:] = 0.0
            yaw = R.from_quat(self.q).as_euler("ZYX")[0]
            self.q = R.from_euler("z", yaw).as_quat()
        self.a = (self.v - v_old) / self.dt

    def step(self, command) -> dict:
        self._integrate()
        self.ticks += 1
        state = self.dynamics()
        accel = np.asarray(self.model.step([state], [list(command)]), dtype=np.float64)[0]
        if not np.all(np.isfinite(accel)):
            raise FloatingPointError(f"The model returned {accel}")
        self._pending = accel
        return {"DynamicsSensor": [state], "t": self.ticks * self.dt}

    def reset(self) -> dict:
        return {"DynamicsSensor": [self.dynamics()], "t": 0.0}

    def tick(self) -> dict:
        return self.reset()


def save_geometry(geometry: dict, path) -> None:
    """Saves a DynamicsSensor ``geometry`` (from a live environment) for :class:`OfflineWorld`."""
    np.savez_compressed(path, **{k: np.asarray(v) for k, v in geometry.items()})


def load_geometry(path) -> dict:
    with np.load(path) as data:
        return {k: (data[k].item() if data[k].ndim == 0 else data[k]) for k in data.files}
