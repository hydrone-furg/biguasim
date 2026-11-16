import warnings

import roma
import torch


from abc import ABC, abstractmethod
from torch import Tensor


from biguasim.dynamics.utils import BatchedParams


# Items of the DynamicsSensor vector (quaternion orientation, UseRPY false). Anything after them in a state row is
# the water surface block the sensor appends (ExportWaterSurface, on by default), read by every model.
DYNAMICS_ITEMS = 19
# Thickness of the band over which a point goes from out of the water to in it (m).
WATERLINE_BAND = 0.02


class VehicleModel(ABC):
    """
    Abstract base class for all batched rigid-body vehicle models in biguasim.

    Defines the interface and shared infrastructure for simulating a fleet of
    vehicles in parallel using PyTorch tensors. Concrete subclasses implement
    the vehicle-specific dynamics, control, and parameter construction by
    overriding the abstract methods below.

    State layout (per robot, as a flat row in the dynamics tensor):
        dynamics[:, 3:6]   — linear velocity v         (world frame, m/s)
        dynamics[:, 6:9]   — position x                (world frame, m)
        dynamics[:, 9:12]  — wq (auxiliary angular state, subclass-defined)
        dynamics[:, 12:15] — angular velocity w         (body frame, rad/s)
        dynamics[:, 15:19] — unit quaternion q          (body ← world, [x,y,z,w])

    The @map_states decorator handles conversion between this flat layout and
    the structured state dict used throughout the dynamics pipeline.

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : Key identifying the control input format.
    """

    # The DynamicsSensor reports angular velocity in the world frame, and the engine applies the
    # returned angular acceleration in the world frame. Models that set this convert ω to the body
    # frame before control and dynamics, and ω̇ back to the world frame; the others still treat the
    # world-frame value as body-frame, which they were tuned with.
    angular_velocity_in_world_frame = False

    # Flat surface height (m) used only when the DynamicsSensor sends no water surface block (ExportWaterSurface
    # false, or a world packaged before it); None means no water at all. A `water_level` param overrides it.
    DEFAULT_WATER_LEVEL = None

    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:
        self.batch_size = batch_size

        # Water surface under each robot, from the DynamicsSensor's ExportWaterSurface block (see
        # _read_water_surface): samples (B, N, 3) world (x, y, surface z), and the contact flag (B,).
        self.water_level = params.get("water_level", self.DEFAULT_WATER_LEVEL)
        self._water_surface = None
        self.water_contact = None
        self._warned_no_water_block = False

        # Shared parameter container — holds all batched physics tensors
        # (mass, inertia, motor coefficients, geometry, etc.)
        self.batched_params = BatchedParams(batch_size, params, device)

        self.device = device
        self.control_abstraction = control_abstraction

        # Integer index tensor [0, 1, …, B-1] used to index into batched state/parameter
        # tensors. Stored as int32 for compatibility with advanced indexing on all backends.
        self.idxs = Tensor(range(batch_size)).int()

        # Integration timestep (seconds); updated each call to step() via map_states
        self.dt = 0

    # =========================================================================
    # State Packing / Unpacking Utilities
    # =========================================================================

    @staticmethod
    def map_states(func):
        """
        Decorator that bridges the external simulator interface (flat tensor lists)
        and the internal structured-dict representation used by the dynamics methods.

        Responsibilities:
            1. Convert the raw flat state list → torch.Tensor on the correct device.
            2. Slice the flat tensor into a structured state dict {x, v, q, w, wq}.
            3. Convert the raw control list → structured control dict {cmd_ctrl}.
            4. Update self.dt from the simulator-provided absolute time.
            5. Call the wrapped function (typically step()) with the structured dicts.

        State tensor column layout expected from the simulator:
            [:, 3:6]   → v  (linear velocity, world frame)
            [:, 6:9]   → x  (position, world frame)
            [:, 9:12]  → wq (auxiliary angular state)
            [:, 12:15] → w  (angular velocity, body frame)
            [:, 15:19] → q  (unit quaternion)
            [:, 19:]   → extra (optional DynamicsSensor blocks, e.g. the water surface)

        Args:
            func: The wrapped method, expected signature func(self, state, control, dt).

        Returns:
            Callable: Wrapped function that accepts raw list inputs from the simulator.

        Raises:
            RuntimeError: If the dynamics tensor has zero rows (no agents in state).
        """
        def wrapper(self, state: list, control: list):
            # Convert raw simulator state to a double-precision tensor on the target device
            dynamics = torch.as_tensor(state).to(self.device).double()

            if dynamics.shape[0] == 0:
                raise RuntimeError("No DynamicsSensor found in state")

            # Convert raw control list to a batched (B, C) tensor
            control = torch.as_tensor(control, device=self.device).double()

            # Slice flat dynamics tensor into the structured state dict.
            # All slices are views (no copy) for efficiency.
            s = {
                'x':  dynamics[:, 6:9],    # Position         (B, 3) world frame
                'v':  dynamics[:, 3:6],    # Linear velocity  (B, 3) world frame
                'q':  dynamics[:, 15:19],  # Unit quaternion  (B, 4) body←world
                'w':  dynamics[:, 12:15],  # Angular velocity (B, 3) body frame
                'wq': dynamics[:, 9:12],   # Auxiliary state  (B, 3) subclass-defined
                # DynamicsSensor blocks after the dynamics items (B, K), K = 0 when none is exported
                'extra': dynamics[:, DYNAMICS_ITEMS:]
            }

            # Wrap control in the dict format expected by get_cmd_motor_speeds
            c = {'cmd_ctrl': control}

            # Every model sees the water surface the DynamicsSensor reports under the robot.
            self._read_water_surface(s['extra'])

            return func(self, s, c)

        return wrapper

    @staticmethod
    def pack_state(state: dict, batch_size: int, device: str) -> Tensor:
        """
        Packs a structured state dict into the compact internal 13-column tensor
        used by _s_dot_fn and the Runge-Kutta integrator.

        Internal layout:
            s[:, 0:3]  — position x          (world frame)
            s[:, 3:6]  — linear velocity v   (world frame)
            s[:, 6:10] — unit quaternion q   (body ← world)
            s[:, 10:13]— angular velocity w  (body frame)

        Args:
            state (dict) : Structured state dict with keys 'x', 'v', 'q', 'w'.
            batch_size (int): Number of agents (leading dimension of output).
            device (str): Target torch device.

        Returns:
            Tensor: Packed state tensor of shape (B, 13), dtype float64.
        """
        s = torch.zeros(batch_size, 13, device=device).double()
        s[..., 0:3]  = state['x']   # Position
        s[..., 3:6]  = state['v']   # Linear velocity
        s[..., 6:10] = state['q']   # Orientation quaternion
        s[..., 10:13] = state['w']  # Body angular rates
        return s

    @staticmethod
    def unpack_state(s: Tensor, idxs: Tensor, batch_size: int) -> dict:
        """
        Unpacks the compact internal 13-column state tensor back into a structured
        state dict, placing results at the correct batch indices.

        Robots not present in idxs are left as NaN (position, velocity, angular rate)
        or as the identity quaternion [0,0,0,1] (to ensure a valid rotation).

        Internal layout assumed:
            s[:, 0:3]  — position x
            s[:, 3:6]  — linear velocity v
            s[:, 6:10] — quaternion q
            s[:, 10:13]— angular velocity w

        Args:
            s (Tensor)   : Packed state tensor (len(idxs), 13).
            idxs (Tensor): Integer indices of the active robots within the full batch.
            batch_size (int): Total number of robots in the simulation.

        Returns:
            dict: State dict with keys 'x', 'v', 'q', 'w', each a (batch_size, ·) tensor.
                  Inactive robot slots contain NaN (or identity quaternion for 'q').
        """
        device = s.device

        # Pre-fill all slots with NaN; only active robots (idxs) will be overwritten
        state = {
            'x': torch.full((batch_size, 3), float("nan"), device=device).double(),
            'v': torch.full((batch_size, 3), float("nan"), device=device).double(),
            'q': torch.full((batch_size, 4), float("nan"), device=device).double(),
            'w': torch.full((batch_size, 3), float("nan"), device=device).double(),
        }

        # Default quaternion to identity [0,0,0,1] so inactive robots have a valid rotation
        state['q'][..., -1] = 1

        # Scatter packed values into the correct batch positions
        state['x'][idxs] = s[:, 0:3]
        state['v'][idxs] = s[:, 3:6]
        state['q'][idxs] = s[:, 6:10]
        state['w'][idxs] = s[:, 10:13]

        return state

    # =========================================================================
    # Water surface — the DynamicsSensor's ExportWaterSurface block, shared by every vehicle
    # =========================================================================

    def _read_water_surface(self, extra: Tensor) -> None:
        """Wave surface from the DynamicsSensor's ExportWaterSurface block, the state columns after the dynamics
        items (B, K): contact flag, then (x, y, surface z) world points, z NaN where there is no water. Without the
        block (K = 0) the surface is the flat water_level (no water if it is None)."""
        if extra.shape[-1] == 0:
            if not self._warned_no_water_block:
                where = "no water" if self.water_level is None else f"a flat water level at z = {self.water_level} m"
                warnings.warn(f"{type(self).__name__}: the DynamicsSensor sends no water surface (ExportWaterSurface "
                              f"false, or an older world); using {where}.")
                self._warned_no_water_block = True
            self._water_surface = None
            self.water_contact = None
            return
        if (extra.shape[-1] - 1) % 3 != 0:
            raise ValueError(f"DynamicsSensor rows carry {extra.shape[-1]} items after the {DYNAMICS_ITEMS} dynamics "
                             "items; expected only the ExportWaterSurface block (1 + 3 * WaterSurfaceGrid^2)")
        extra = extra.double()
        self.water_contact = extra[:, 0] > 0.5
        self._water_surface = extra[:, 1:].reshape(extra.shape[0], -1, 3)

    def _surface_height(self, xy: Tensor) -> Tensor:
        """Water surface height under each point (B, P, 2) -> (B, P); -inf where there is no water."""
        if self._water_surface is None:
            level = -torch.inf if self.water_level is None else float(self.water_level)
            return torch.full(xy.shape[:-1], level, dtype=torch.float64, device=self.device)
        samples = self._water_surface[self.idxs].to(self.device)                            # (B, N, 3)
        valid = torch.isfinite(samples[..., 2])
        if bool(valid.all()) and bool((samples[..., 2] == samples[:, :1, 2]).all()):
            # Flat surface (calm water): every inverse-distance average is that height; skip the (B, P, N) distances
            return samples[:, :1, 2].expand(xy.shape[:-1]).clone()
        d2 = torch.sum((xy.unsqueeze(2) - samples[..., :2].unsqueeze(1)) ** 2, dim=-1)       # (B, P, N)
        weights = torch.where(valid.unsqueeze(1), 1.0 / (d2 + 1e-4), torch.zeros_like(d2))
        heights = torch.nan_to_num(samples[..., 2], nan=0.0).unsqueeze(1)
        total = weights.sum(dim=-1)
        z = (weights * heights).sum(dim=-1) / total.clamp_min(1e-12)
        return torch.where(total > 0, z, torch.full_like(z, -torch.inf))

    def _height_above_water(self, points: Tensor) -> Tensor:
        """Height of world points (B, 3) or (B, P, 3) above the water surface under them (m); +inf over no water."""
        return points[..., 2] - self._surface_height(points[..., :2].unsqueeze(1) if points.dim() == 2
                                                      else points[..., :2]).reshape(points.shape[:-1])

    @staticmethod
    def _immersion(depth: Tensor) -> Tensor:
        """Fraction in the water (0..1) for a depth below the surface (m), linear across WATERLINE_BAND."""
        return torch.clamp(depth / WATERLINE_BAND + 0.5, 0.0, 1.0)

    # =========================================================================
    # Abstract Interface — must be implemented by every concrete vehicle model
    # =========================================================================

    @abstractmethod
    def _build_params(self, params: dict) -> None:
        """
        Parse the parameter dictionary and populate self.batched_params with all
        vehicle-specific physics quantities (geometry, inertia, motor coefficients,
        drag matrices, allocation matrices, PID controllers, etc.).

        Called once during __init__ before any simulation steps are taken.
        """
        pass

    def set_geometry(self, geometries: list[dict]) -> None:
        """
        Replace the primitive body approximation with the collision mesh the engine
        exports (DynamicsSensor with ExportGeometry). Called by the environment once
        the geometry is available, after each reset.

        Models that still use primitive approximations ignore it.

        Args:
            geometries (list[dict]): One DynamicsSensor.geometry dict per robot in the batch.
        """
        pass

    def _compute_external_forces(self, *args, **kwargs) -> Tensor:
        """
        Compute all external (non-thrust) forces and moments acting on the vehicle
        in the body frame: gravity, buoyancy, hydrodynamic/aerodynamic drag,
        Coriolis/centrifugal terms, and any added-mass contributions.

        Returns force and moment tensors (and optionally added-inertia increments)
        to be combined with the thruster wrench in _s_dot_fn.

        Optional: a helper of _s_dot_fn, which models with their own plant (MarinePlant) do not use.
        """
        raise NotImplementedError(f"{type(self).__name__} does not split its plant into _compute_external_forces")

    def _compute_body_wrench(self, *args, **kwargs) -> tuple:
        """
        Compute the net force and moment in the body frame produced by the
        vehicle's rotors or thrusters given their current speed commands.

        Returns a (FtotB, MtotB) tuple of (B, 3) tensors in the body frame.

        Optional: a helper of _s_dot_fn, which models with their own plant (MarinePlant) do not use.
        """
        raise NotImplementedError(f"{type(self).__name__} does not split its plant into _compute_body_wrench")

    @abstractmethod
    def _s_dot_fn(self, s: Tensor, cmd_ctrl: Tensor) -> Tensor:
        """
        Evaluate the continuous-time state derivative ṡ = f(s, u).

        Combines external forces (from _compute_external_forces) and thruster
        wrench (from _compute_body_wrench) to produce linear and angular
        accelerations, then assembles the full state-derivative vector.

        Args:
            s (Tensor)        : Packed state vector (B, 13).
            cmd_ctrl (Tensor) : Motor speed commands (B, n_motors).

        Returns:
            Tensor: State derivative ṡ (B, 13 + n_motors).
        """
        pass

    @abstractmethod
    def get_cmd_motor_speeds(self, state: dict, control: dict) -> tuple[Tensor, Tensor]:
        """
        Convert a high-level control command into motor speed commands using the
        vehicle's cascaded PID control architecture.

        The exact cascade (position → velocity → acceleration → attitude → wrench →
        motor speeds) and supported control abstraction keys are vehicle-specific.

        Args:
            state (dict)  : Current structured state dict {x, v, q, w, …}.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).

        Returns:
            Tensor: Motor speed commands (B, n_motors), split by domain where applicable.
        """
        pass

    # =========================================================================
    # Default Step — overridden by subclasses with domain-aware motor clipping
    # =========================================================================

    @map_states
    def step(self, state: dict, control: dict):
        """
        Default single-step dynamics integrator. Subclasses may override this to
        add domain-specific motor clipping, added-mass selection, or quaternion
        renormalization (as OctaCopterFourAirFourWaterCross does).

        Pipeline:
            1. Short-circuit for 'accel' abstraction (returns raw acceleration).
            2. Convert control commands → clipped motor speeds.
            3. Evaluate ṡ = f(s, u) via _s_dot_fn.
            4. Scatter v̇ and ω̇ into world-frame output tensors.
            5. Renormalize the quaternion to prevent integration drift.
            6. Return [v̇; ω̇] as a flat CPU list for the simulator.

        Args:
            state (dict)  : Structured state dict (populated by @map_states).
            control (dict): Structured control dict (populated by @map_states).
            dt (int)      : Absolute simulator time (elapsed dt computed in decorator).

        Returns:
            list: Concatenated [v̇, ω̇] (B, 6) as a CPU float list.
        """
        # 'accel' mode bypasses motor dynamics entirely — return raw command as acceleration
        if self.control_abstraction == 'accel':
            return control['cmd_ctrl'][self.idxs].cpu().tolist()

        # The engine reports ω and applies ω̇ in the world frame, while the dynamics work
        # in the body frame; models that account for it convert on the way in and out.
        if self.angular_velocity_in_world_frame:
            R_body_to_world = roma.unitquat_to_rotmat(state['q']).double()   # (B, 3, 3)
            state = {**state, 'w': torch.einsum('bji,bj->bi', R_body_to_world, state['w'])}

        # Compute motor speeds from the high-level command and clip to hardware limits
        cmd_ctrl = self.get_cmd_motor_speeds(state, control)
        cmd_ctrl = torch.clip(
            cmd_ctrl,
            self.batched_params.rotor_speed_min[self.idxs],
            self.batched_params.rotor_speed_max[self.idxs]
        )

        # Pack state dict → flat tensor, evaluate dynamics, scatter derivatives back
        s     = self.pack_state(state, self.batch_size, self.device)
        s_dot = self._s_dot_fn(s, cmd_ctrl)

        v_dot = torch.zeros_like(state["v"])
        w_dot = torch.zeros_like(state["w"])
        v_dot[self.idxs] = s_dot[..., 3:6].double()
        w_dot[self.idxs] = s_dot[..., 10:13].double()
        if self.angular_velocity_in_world_frame:
            w_dot = torch.einsum('bij,bj->bi', R_body_to_world, w_dot)

        # Renormalize quaternion to unit length after Euler integration
        state = self.unpack_state(s, self.idxs, self.batch_size)
        state['q'][self.idxs] = (
            state['q'][self.idxs]
            / torch.norm(state['q'][self.idxs], dim=-1).unsqueeze(-1)
        )

        return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()