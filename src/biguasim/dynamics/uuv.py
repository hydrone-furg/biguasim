import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.utils import Transform as tfrm
from biguasim.dynamics.utils import BatchedParams as bp
from biguasim.dynamics.marine import HYDROSTATIC_VOXEL, WaterHydrostatics, _box_geometry, _solid_voxel_centers


from biguasim.dynamics.control import BatchedPIDController

# Body directions the horizontal speed limit is checked along: the axes and the diagonals (the 45 deg thrusters give
# less force along a diagonal, where only two of them push).
HORIZONTAL_DIRECTIONS = [[1, 0, 0], [0, 1, 0], [0.7071067811865476, 0.7071067811865476, 0],
                         [0.7071067811865476, -0.7071067811865476, 0]]


class HexaCopterFiveDoF(VehicleModel):
    """
    Parameters, thrust allocation and legacy controllers of a 5-DOF underwater hexarotor (BlueROV2-style).

    Has no plant of its own: BlueROV2 combines it with dynamics.marine.MarinePlant (the Fossen model with voxel
    buoyancy against the water surface) and ROVControl. The former plant here (a depth-ramped buoyancy
    "dumping_hack", isotropic sphere drag, z thresholds for the surface) was removed. The frame carries 6
    thrusters arranged as follows:

        Thrusters 0–1 : vertical (body +Z), control heave and roll.
        Thrusters 2–5 : horizontal, angled ±45° in the body XY-plane,
                        control surge (X), sway (Y), and yaw (Mz).

    The vehicle is 5-DOF because pitch (My) is structurally unconstrained —
    the symmetric thruster layout produces no net pitch moment, and no pitch
    controller is included.

    Control abstractions supported:
        - 'cmd_motor_speeds' : Direct thruster speed pass-through.
        - 'cmd_vel'          : 3D velocity tracking    [vx, vy, vz].
        - 'cmd_vel_yaw'      : 3D velocity + yaw offset [vx, vy, vz, yaw_deg].
        - 'cmd_pos_yaw'      : 3D position + absolute yaw [x, y, z, yaw_deg].

    Control architecture (all non-passthrough abstractions):
        Velocity/Position P(D)  →  desired force F_des (body frame)
        Roll PID                →  desired roll moment M_roll
        Yaw PD                  →  desired yaw moment  M_yaw
        [F_des, M_roll, M_yaw]  →  TM_to_f (pseudo-inverse)  →  thruster forces
        Thruster forces         →  bidirectional motor speeds

    Frame conventions (inherited from VehicleModel):
        - World frame : Z-up, right-handed (NWU).
        - Body frame  : X-forward, Y-left, Z-up.

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device string, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : One of the supported control abstraction keys.
    """

    def __init__(self, batch_size: int, params: dict, device: str,
                 control_abstraction: str) -> None:
        # Initialize shared infrastructure (BatchedParams, idxs, dt, etc.)
        super().__init__(batch_size, params, device, control_abstraction)

        self._control_abstraction = control_abstraction

        # Populate vehicle-specific physics parameters and controllers
        self._build_params(params)


    # =========================================================================
    # Parameter Construction
    # =========================================================================

    def _build_params(self, params: dict):
        """
        Parses the parameter dictionary and populates self.batched_params with
        all geometry, hydrodynamics, allocation, and control gain tensors.

        Sections (in order):
            1. Geometry      — body dimensions, equivalent sphere radius.
            2. Inertia       — body inertia tensor, added mass, displaced volume,
                               buoyancy force magnitude.
            3. Drag          — isotropic translational (Cp) and rotational (Cr)
                               drag coefficient matrices (sphere approximation).
            4. Allocation    — 6×6 wrench-to-force pseudo-inverse allocation matrix.
            5. Control gains — velocity (k_vxy, k_vz), roll PID, yaw PD, position PD.

        Args:
            params (dict): Raw parameter dictionary from the vehicle config file.
        """

        # ---- 1. Geometry ----
        length = params['length']
        width  = params['width']
        self.batched_params.height = params['height']   # Vehicle height (m), used for buoyancy ramp

        # Equivalent sphere radius: average of length and width semi-axes.
        # Used for all added-mass and drag computations.
        rsphere = ((length + width) / 2) / 2
        self.batched_params.rsphere = rsphere

        # ---- 2. Inertia / mass / buoyancy ----
        self.batched_params.I = torch.from_numpy(
            np.array([params['I'] for _ in range(self.batch_size)])
        ).double().to(self.device)  # Body inertia tensor (B, 3, 3)

        # Displaced volume of the equivalent sphere (m³)
        self.batched_params.volume = (4/3) * np.pi * rsphere**3

        # Added mass = mass of fluid displaced by the equivalent sphere (kg).
        # rho is the single-fluid density set in BatchedParams from params['rho'].
        self.batched_params.added_mass = self.batched_params.volume * self.batched_params.rho

        # Static buoyancy force magnitude when fully submerged (N)
        self.batched_params.buoyancy = np.linalg.norm(
            self.batched_params.rho * self.batched_params.volume * self.batched_params.g
        )

        # Gravity acceleration vector in world frame, broadcast across the batch (B, 3)
        self.batched_params.gravity        = torch.zeros(self.batch_size, 3, device=self.device).double()
        self.batched_params.gravity[:, -1] = self.batched_params.g

        # ---- 3. Drag coefficient matrices ----
        # Isotropic sphere model: Cp = 0.47 π r² (pressure drag coefficient)
        # Same value on all three axes — the sphere has no directional preference.
        Cp  = 0.47 * np.pi * rsphere**2
        Cpx = Cpy = Cpz = Cp

        # Isotropic rotational drag: Cr = 0.890e-3 * 8π r³ (viscous torque coefficient)
        Cr  = 0.890e-3 * 8.0 * np.pi * rsphere**3
        Crx = Cry = Crz = Cr

        self.batched_params.Cp = torch.from_numpy(
            np.array([np.diag([Cpx, Cpy, Cpz]) for _ in range(self.batch_size)])
        ).double().to(self.device)  # (B, 3, 3)

        self.batched_params.Cr = torch.from_numpy(
            np.array([np.diag([Crx, Cry, Crz]) for _ in range(self.batch_size)])
        ).double().to(self.device)  # (B, 3, 3)

        # √2/2 ≈ 0.707 — projection factor for thrusters angled at 45° in the XY-plane
        self.batched_params.sqrt = np.sqrt(2) / 2

        # ---- 4. Allocation matrix ----
        # Rotor positions and thrust directions, replicated across the batch
        self.batched_params.rotor_pos    = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array(
            [params['rotor_directions'] for _ in range(self.batch_size)]
        )
        # Pre-compute position vectors and hat-map tensors (rotor_geometry, rotor_geometry_hat_maps)
        self.batched_params._extract_geometry()

        # Effective moment-arm length: Euclidean distance from CoM to rotor r1, scaled by 0.1.
        # Used as the lever arm L in the rotational drag formula.
        x, y, z = params["rotor_pos"]['r1']
        self.batched_params.rotor_distance = np.sqrt(x**2 + y**2 + z**2) * 0.1

        alloc_mats = []
        for i in range(self.batch_size):
            # Thruster positions in body frame: (n_rotors, 3)
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key]
                          for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(self.device)

            # Unit thrust direction vectors for each thruster: (n_rotors, 3)
            thrust_dirs = torch.from_numpy(
                self.batched_params.rotor_dir_np[i]
            ).double().to(self.device)

            # Moment arms via cross product: r × F̂ → (3, n_rotors)
            torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T


            # Assemble full 6×n_rotors allocation matrix:
            #   Rows 0–2: force  contributions (F̂ per thruster)
            #   Rows 3–5: moment contributions (r × F̂ per thruster)
            A = torch.vstack([
                thrust_dirs.T,   # (3, n_rotors) — force block
                torque_arms,     # (3, n_rotors) — moment block
            ])
            alloc_mats.append(A)

        # Compute Moore-Penrose pseudo-inverse: minimum-norm thrust solution for any wrench.
        # Pseudo-inverse is used (not exact inverse) because the system may be
        # over-actuated (more thrusters than DOF).
        TM_to_f = torch.stack(alloc_mats).to(self.device)
        self.batched_params.TM_to_f = torch.linalg.pinv(TM_to_f)  # (B, n_rotors, 6)

        # ---- 5. Control gains ----
        # Horizontal and vertical velocity P-gains (separate to allow independent tuning)
        self.batched_params.k_vxy = torch.tensor(
            [params['k_vxy'] for _ in range(self.batch_size)]
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        self.batched_params.k_vz = torch.tensor(
            [params['k_vz'] for _ in range(self.batch_size)]
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        # Roll PID — full integral action to reject persistent roll disturbances underwater
        self.pid_roll = BatchedPIDController(
            params['kp_roll'], params['ki_roll'], params['kd_roll'],
            self.batch_size, self.device
        )

        # Yaw PD gains — no integral to avoid yaw windup during slow turns
        self.batched_params.kp_yaw = torch.from_numpy(
            np.array([params['kp_yaw'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        self.batched_params.kd_yaw = torch.from_numpy(
            np.array([params['kd_yaw'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        # Position outer-loop PD gains
        self.batched_params.kp_pos = torch.from_numpy(
            np.array([params['kp_pos'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        self.batched_params.kd_pos = torch.from_numpy(
            np.array([params['kd_pos'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)




    # =========================================================================
    # Allocation (shared by the controllers)
    # =========================================================================

    @staticmethod
    def _yaw(q: Tensor) -> Tensor:
        """Heading (rad) of quaternions [x, y, z, w] (B, 4): atan2(R10, R00), valid at any roll and pitch."""
        R = roma.unitquat_to_rotmat(q.double())
        return torch.atan2(R[:, 1, 0], R[:, 0, 0])

    @staticmethod
    def _wrap_angle(angle: Tensor) -> Tensor:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _heading_setpoint(self, yaw: Tensor, offset: Tensor | None = None) -> Tensor:
        """Heading the velocity modes steer to (rad, B).

        cmd_vel (offset None) holds the heading the vehicle had at its first command; it used to take the current
        heading as the setpoint every tick, which only damped the yaw rate and let the heading drift. cmd_vel_yaw keeps
        its meaning: while an offset is commanded the setpoint leads the current heading by it (the vehicle turns),
        and the heading reached when the offset returns to 0 is held.
        """
        held = getattr(self, '_held_yaw', None)
        if held is None or held.shape != yaw.shape:
            held = yaw.clone()
        if offset is None:
            self._held_yaw = held
            return held
        turning = offset != 0
        self._held_yaw = torch.where(turning, yaw, held)
        return self._wrap_angle(torch.where(turning, yaw + offset, held))

    def _speed_limit(self, state: dict, directions: list, default: float) -> Tensor:
        """Speed (m/s, B) a position mode may command along the body directions: the plant's (MarinePlant overrides
        it with the speed its thrusters can hold against its drag); without a plant, the former fixed default."""
        return torch.full((len(self.idxs),), float(default), dtype=torch.float64, device=self.device)

    @staticmethod
    def _roll_pitch(q: Tensor) -> Tensor:
        """Roll and pitch (rad, (B, 2)) of quaternions [x, y, z, w], ZYX convention, from the attitude itself."""
        R = roma.unitquat_to_rotmat(q.double())
        roll = torch.atan2(R[:, 2, 1], R[:, 2, 2])
        pitch = torch.asin(torch.clamp(-R[:, 2, 0], -1.0, 1.0))
        return torch.stack([roll, pitch], dim=-1)

    def _level_moment(self, pid: BatchedPIDController, rate: Tensor, angle: Tensor) -> Tensor:
        """Roll or pitch moment of the attitude 'rate PID' (setpoint 10 w on the rate w, so its error is 9 w), in the
        controllers' former moment sign (converted in _allocate_thrust).

        Kp acts on 9 w (rate damping) and Kd on its change, as before. Ki acts on 9 x the actual angle, read from
        the attitude: the PID used to integrate the body rate, which equals the angle only while the vehicle does
        not yaw (after a 90 deg turn BlueROV2 settled rolled 44 deg).
        """
        idx = self.idxs
        error = 9.0 * rate.double()
        previous = pid.previous_error[idx].double()
        derivative = (error - previous) / self.dt if self.dt else torch.zeros_like(error)
        pid.previous_error = error.clone()
        out = pid.Kp * error + pid.Ki * 9.0 * angle.double() + pid.Kd * derivative
        return torch.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)

    @staticmethod
    def _limit_norm(v: Tensor, limit: Tensor) -> Tensor:
        """Scales vectors (B, n) down to a norm of at most limit (B,), keeping their direction."""
        norm = torch.linalg.norm(v, dim=-1)
        scale = torch.where(norm > limit, limit / norm.clamp_min(1e-12), torch.ones_like(norm))
        return v * scale.unsqueeze(-1)

    def _steady_force(self, state: dict, v_body: Tensor) -> Tensor:
        """Body-frame force (B, 3) that holds the body-frame velocity v_body steadily against the plant (drag,
        damping, net weight). No plant here: zero. MarinePlant overrides it with its own model."""
        return torch.zeros_like(v_body, dtype=torch.float64)

    def _feedforward_world(self, state: dict, v_des_world: Tensor) -> Tensor:
        """World-frame feed-forward force (B, 3) for a world-frame velocity setpoint (see _steady_force)."""
        R = roma.unitquat_to_rotmat(state['q'][self.idxs].double())
        v_body = (R.transpose(1, 2) @ v_des_world.double().unsqueeze(-1)).squeeze(-1)
        return (R @ self._steady_force(state, v_body).unsqueeze(-1)).squeeze(-1)

    def _allocate_thrust(self, F_world: Tensor, M_des: Tensor, q: Tensor) -> Tensor:
        """Desired force (world frame) and moment -> signed thruster speeds through the allocation TM_to_f.

        The force is rotated into the body frame, where the thrusters act (every abstraction computes it in the
        world frame). The moments are flipped: these controllers were tuned on the former plant, whose thruster
        moments had the opposite sign of r x F (and were built from k_m w^2, ~14x weaker); the marine plant now
        produces r x F, the same geometry TM_to_f inverts.
        """
        R_wb = roma.unitquat_to_rotmat(q.double()).transpose(1, 2)
        F_body = torch.bmm(R_wb, F_world.double().unsqueeze(-1)).squeeze(-1)
        TM = torch.cat([F_body, -M_des.double()], dim=-1)   # (B, 6)
        thrust = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)
        # Bidirectional speed conversion: w = sign(F) sqrt(|F| / k_eta)
        speeds = torch.sign(thrust) * torch.sqrt(torch.abs(thrust) / self.batched_params.k_eta[self.idxs])
        return torch.nan_to_num(speeds, nan=0.0, posinf=0.0, neginf=0.0)

    # =========================================================================
    # Control → Motor Speeds
    # =========================================================================

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        """
        Converts a high-level control command into bidirectional thruster speed
        commands using a cascaded P(D) velocity/position controller with
        independent roll PID and yaw PD inner loops.

        All three non-passthrough abstractions share the same inner loop:
            1. Compute desired force F_des (body frame) from velocity or position error.
            2. Compute desired roll moment  M_roll via PID on roll rate.
            3. Compute desired yaw  moment  M_yaw  via PD  on yaw error.
            4. Assemble wrench [F_des, M_roll, 0, M_yaw] and map through TM_to_f.
            5. Convert thruster forces to bidirectional motor speeds:
               ω = sign(F) √(|F| / k_eta).

        The pitch channel is always zero in the wrench vector, consistent with
        the 5-DOF design.

        Args:
            state (dict)  : Structured state dict {x, v, q, w} from VehicleModel.map_states.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).
                            D = 3 for cmd_vel, D = 4 for cmd_vel_yaw / cmd_pos_yaw.

        Returns:
            Tensor: Thruster speed commands (B, 6), signed rad/s.
        """

        # --- Pass-through: thruster speeds commanded directly ---
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]

        # =====================================================================
        # cmd_vel: Track desired 3D velocity [vx, vy, vz]
        # Yaw is held at current heading (zero yaw error commanded).
        # =====================================================================
        if self._control_abstraction == 'cmd_vel':
            v = state['v'][self.idxs]   # Linear velocity  (B, 3) world frame
            w = state['w'][self.idxs]   # Angular velocity (B, 3) body frame
            q = state['q'][self.idxs]   # Quaternion       (B, 4)

            goal = control['cmd_ctrl'][self.idxs]   # [vx, vy, vz]

            # Velocity error → desired acceleration (P-control, separate XY and Z gains)
            v_err       = goal[self.idxs] - v
            a_des       = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)   # Scale Z separately

            # Desired force in body frame: F = m * a
            F_des = self.batched_params.mass * a_des[self.idxs]   # (B, 3)
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, goal[:, :3])

            # ---- Roll stabilization ----
            # Rate damping + stiffness on the actual roll angle (see _level_moment)
            M_roll = self._level_moment(self.pid_roll, w[:, 0], self._roll_pitch(q)[:, 0])

            # ---- Yaw stabilization ----
            # Hold the heading of the first command (see _heading_setpoint)
            yaw     = self._yaw(q)
            yaw_des = self._heading_setpoint(yaw)   # heading held since the first command
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # ---- Assemble wrench (pitch channel left at zero — 5-DOF constraint) ----
            M_des       = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)

        # =====================================================================
        # cmd_vel_yaw: Track desired velocity + relative yaw offset [vx, vy, vz, yaw_deg]
        # Extends cmd_vel: yaw setpoint is offset from current heading by goal[:, 3].
        # =====================================================================
        if self._control_abstraction == 'cmd_vel_yaw':
            v = state['v'][self.idxs]
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal = control['cmd_ctrl'][self.idxs]   # [vx, vy, vz, yaw_delta_deg]

            # Velocity error → desired acceleration (same P-control as cmd_vel)
            v_err       = goal[:, :3] - v
            a_des       = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)
            F_des       = self.batched_params.mass * a_des[self.idxs]
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, goal[:, :3])

            # Roll stabilization (as in cmd_vel)
            M_roll = self._level_moment(self.pid_roll, w[:, 0], self._roll_pitch(q)[:, 0])

            # ---- Yaw control ----
            # Turn while an offset is commanded (degrees → radians), hold the heading when it is 0
            yaw       = self._yaw(q)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des   = self._heading_setpoint(yaw, yaw_delta)
            yaw_err   = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # Assemble wrench and allocate
            M_des       = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)

        # =====================================================================
        # cmd_pos_yaw: Track desired 3D position + absolute yaw [x, y, z, yaw_deg]
        # Outermost loop: position PD → desired velocity → same velocity loop above.
        # Desired force is rotated into body frame before allocation.
        # =====================================================================
        if self._control_abstraction == 'cmd_pos_yaw':
            x = state['x'][self.idxs]
            v = state['v'][self.idxs]
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal = control['cmd_ctrl'][self.idxs]   # [x_des, y_des, z_des, yaw_deg]

            # ---- Position PD outer loop ----
            # Desired velocity from proportional position error, clamped to safe limits
            pos_err    = goal[:, :3] - x
            v_des      = self.batched_params.kp_pos[self.idxs] * pos_err
            # Horizontal speed limited to what the thrusters can hold against the drag (was a fixed +-1.5 m/s per axis,
            # beyond the vehicle: the thrusters saturated and could no longer keep it level); vertical +-0.15 m/s.
            v_des[:, :2] = self._limit_norm(v_des[:, :2], self._speed_limit(state, HORIZONTAL_DIRECTIONS, 1.5))
            v_des[:, 2]  = torch.clamp(v_des[:, 2], -0.15, 0.15)

            # Velocity error → desired acceleration (same P-control structure as cmd_vel)
            v_err       = v_des - v
            a_des       = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)
            F_des       = self.batched_params.mass * a_des[self.idxs]
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, v_des)

            # Roll stabilization (as in cmd_vel)
            M_roll = self._level_moment(self.pid_roll, w[:, 0], self._roll_pitch(q)[:, 0])

            # ---- Yaw control ----
            # Desired yaw is absolute (not relative to current heading)
            yaw       = self._yaw(q)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des   = (yaw_delta + np.pi) % (2 * np.pi) - np.pi   # Wrap absolute yaw
            yaw_err   = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err


            # Assemble wrench and allocate
            M_des       = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)
            
class OctaCopterSixDoF(HexaCopterFiveDoF):
    """
    Fully actuated 6-DOF underwater vehicle model with 8 thrusters arranged
    in two functional groups:

        - 4 vertical thrusters (indices 0–3): produce Z-axis thrust and
          roll/pitch moments via differential thrust.
        - 4 horizontal thrusters (indices 4–7): oriented at ±45° in the XY
          body plane, producing surge (X), sway (Y), and yaw (Mz) forces.

    Extends HexaCopterFiveDoF by adding full roll and pitch moment authority
    through the vertical thruster group, upgrading the vehicle from 5-DOF
    to 6-DOF control.

    All thrusters are bidirectional — motor speed commands carry sign
    (positive = one spin direction, negative = the other), and thrust
    is proportional to ω|ω| = ω² sign(ω).

    Control abstractions supported:
        - 'cmd_motor_speeds' : Direct thruster speed pass-through.
        - 'cmd_vel'          : 3D velocity tracking [vx, vy, vz]
                               with heading-hold yaw stabilization.
        - 'cmd_vel_yaw'      : 3D velocity + relative yaw offset
                               [vx, vy, vz, yaw_deg].
        - 'cmd_pos_yaw'      : 3D position + absolute yaw
                               [x, y, z, yaw_deg].

    Control architecture:
        (optional) Position P  →  Velocity P  →  Desired force F_des
        F_des  →  Wrench [F, M]  →  TM_to_f (allocation)
               →  Thruster forces  →  Motor speeds

    Attitude stabilization:
        Roll and pitch are stabilized by independent PID controllers acting
        on angular rate (w[:, 0] and w[:, 1] respectively). Yaw is controlled
        by a PD law on the yaw angle error and yaw rate.

    Frame conventions (inherited from VehicleModel):
        - World frame : Z-up, right-handed.
        - Body frame  : X-forward, Y-left, Z-up.

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device string, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : One of the supported control abstraction keys above.
    """

    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:
        # Initialize HexaCopterFiveDoF infrastructure (geometry, allocation, shared PIDs, etc.)
        super().__init__(batch_size, params, device, control_abstraction)

        # Extend parameters with 6-DOF specific entries (adds pitch PID from params)
        self._build_params(params)


    # =========================================================================
    # Parameter Construction
    # =========================================================================

    def _build_params(self, params: dict):
        """
        Extends the parent _build_params with the pitch PID controller,
        which requires its gains to be specified in the parameter dictionary
        (unlike the roll PID, whose gains are hard-coded in __init__).

        Args:
            params (dict): Raw parameter dictionary from the vehicle config file.
                           Must contain keys: 'kp_pitch', 'ki_pitch', 'kd_pitch'.
        """
        # Inherit all geometry, inertia, drag, allocation, and shared control parameters
        super()._build_params(params)

        # Pitch stabilization PID — parameterized separately from roll to allow
        # independent tuning when pitch and roll dynamics differ (e.g., asymmetric frames)
        self.pid_pitch = BatchedPIDController(
            params['kp_pitch'], params['ki_pitch'], params['kd_pitch'],
            self.batch_size, self.device
        )


    # =========================================================================
    # Control → Motor Speeds
    # =========================================================================

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        """
        Converts a high-level control command into thruster speed commands using
        a cascaded P-control architecture with independent attitude stabilization.

        All three non-passthrough abstractions share the same inner structure:
            1. Compute desired force F_des from velocity or position error.
            2. Stabilize roll and pitch via PID on angular rates w[:, 0], w[:, 1].
            3. Control yaw via PD on yaw angle error and yaw rate.
            4. Assemble wrench [F_des, M_des] and map through TM_to_f.
            5. Convert thruster forces to signed motor speeds:
               ω = sign(F) * √(|F| / k_eta).

        Frames and sign (see _allocate_thrust):
            F_des is computed in the world frame and rotated into the body frame
            in every abstraction, and the moment demand is converted from the
            former plant's sign convention to r x F before allocation.

        Args:
            state (dict)  : Structured state dict {x, v, q, w} from VehicleModel.map_states.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).
                            D = 3 for cmd_vel, D = 4 for cmd_vel_yaw / cmd_pos_yaw.

        Returns:
            Tensor: Thruster speed commands (B, 8), signed rad/s.
        """

        # --- Pass-through: thruster speeds commanded directly ---
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]

        # =====================================================================
        # cmd_vel: Track desired 3D velocity [vx, vy, vz]
        # Yaw is held at current heading (zero yaw rate setpoint).
        # =====================================================================
        if self._control_abstraction == 'cmd_vel':

            v = state['v'][self.idxs]   # Linear velocity  (B, 3) world frame
            w = state['w'][self.idxs]   # Angular velocity (B, 3) body frame
            q = state['q'][self.idxs]   # Unit quaternion  (B, 4)

            goal = control['cmd_ctrl'][self.idxs]   # Desired velocity [vx, vy, vz]

            # Velocity error → desired acceleration (P-control) → desired force
            v_err = goal[self.idxs] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)   # Separate Z gain
            F_des = self.batched_params.mass * a_des[self.idxs]    # (B, 3) world frame
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, goal[:, :3])

            # ---- Roll and pitch stabilization ----
            # Rate damping + stiffness on the actual roll/pitch angles (see _level_moment)
            roll_pitch = self._roll_pitch(q)
            M_roll  = self._level_moment(self.pid_roll,  w[:, 0], roll_pitch[:, 0])
            M_pitch = self._level_moment(self.pid_pitch, w[:, 1], roll_pitch[:, 1])

            # ---- Yaw stabilization: hold current heading ----
            # Hold the heading of the first command (see _heading_setpoint)
            yaw     = self._yaw(q)
            yaw_des = self._heading_setpoint(yaw)   # heading held since the first command
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # ---- Assemble desired moment vector ----
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)

        # =====================================================================
        # cmd_vel_yaw: Track desired velocity + relative yaw offset [vx, vy, vz, yaw_deg]
        # Same velocity cascade as cmd_vel, with an active yaw tracking loop.
        # =====================================================================
        if self._control_abstraction == 'cmd_vel_yaw':

            v = state['v'][self.idxs]
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal = control['cmd_ctrl'][self.idxs]   # [vx, vy, vz, yaw_delta_deg]

            # Velocity error → desired acceleration → desired force (world frame)
            v_err = goal[:, :3] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]   # (B, 3)
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, goal[:, :3])

            # ---- Roll and pitch stabilization (as in cmd_vel) ----
            roll_pitch = self._roll_pitch(q)
            M_roll  = self._level_moment(self.pid_roll,  w[:, 0], roll_pitch[:, 0])
            M_pitch = self._level_moment(self.pid_pitch, w[:, 1], roll_pitch[:, 1])

            # ---- Yaw tracking: current heading + commanded offset ----
            yaw       = self._yaw(q)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des   = self._heading_setpoint(yaw, yaw_delta)
            yaw_err   = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # ---- Assemble desired moment vector ----
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)

        # =====================================================================
        # cmd_pos_yaw: Track desired 3D position + absolute yaw [x, y, z, yaw_deg]
        # Outer loop: position P → desired velocity, clamped to safety limits.
        # Inner loop: velocity P → F_des, then same attitude + allocation pipeline.
        # =====================================================================
        if self._control_abstraction == 'cmd_pos_yaw':

            x = state['x'][self.idxs]   # Position         (B, 3) world frame
            v = state['v'][self.idxs]   # Linear velocity  (B, 3) world frame
            w = state['w'][self.idxs]   # Angular velocity (B, 3) body frame
            q = state['q'][self.idxs]   # Unit quaternion  (B, 4)

            goal = control['cmd_ctrl'][self.idxs]   # [x_des, y_des, z_des, yaw_deg]

            # ---- Position outer loop: P-control → desired velocity ----
            pos_err = goal[:, :3] - x
            v_des   = self.batched_params.kp_pos[self.idxs] * pos_err

            # Horizontal speed limited to what the thrusters can hold against the drag (was a fixed +-1.5 m/s per axis,
            # beyond the vehicle: the thrusters saturated and could no longer keep it level); vertical +-0.15 m/s.
            v_des[:, :2] = self._limit_norm(v_des[:, :2], self._speed_limit(state, HORIZONTAL_DIRECTIONS, 1.5))
            v_des[:, 2]  = torch.clamp(v_des[:, 2], -0.15, 0.15)

            # ---- Velocity inner loop: P-control → desired acceleration → force ----
            v_err = v_des - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err
            a_des[:, 2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]   # (B, 3) world frame
            # Feed-forward: what the plant needs to hold the setpoint (drag, damping, net weight)
            F_des = F_des + self._feedforward_world(state, v_des)

            # ---- Roll and pitch stabilization ----
            roll_pitch = self._roll_pitch(q)
            M_roll  = self._level_moment(self.pid_roll,  w[:, 0], roll_pitch[:, 0])
            M_pitch = self._level_moment(self.pid_pitch, w[:, 1], roll_pitch[:, 1])

            # ---- Yaw tracking: absolute yaw setpoint (degrees → radians) ----
            # yaw_des is computed directly from goal[:, 3], not offset from current yaw,
            # making this an absolute heading command rather than a relative offset
            yaw       = self._yaw(q)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des   = (yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err   = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # ---- Assemble desired moment vector ----
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des, q)    

class TorpedoAUV(WaterHydrostatics, VehicleModel):
    """
    6-DOF torpedo-shaped Autonomous Underwater Vehicle (AUV) model.

    Models a slender, slightly positively buoyant underwater vehicle propelled by a
    single stern propeller and steered by two pairs of control surfaces:
        - Rudders  (δ_rt, δ_rb): port/starboard tail fins — yaw control.
        - Sternplanes (δ_sl, δ_sr): top/bottom tail fins  — depth/pitch control.

    All actuators are modeled with first-order lag dynamics (time constants
    T_delta for fins, T_n for the propeller), so commanded values are not
    applied instantaneously.

    Hydrodynamics:
        - Rigid-body and added-mass Coriolis/centripetal matrices via m2c.
        - Linear damping diagonal, with exponential velocity attenuation
          on surge and sway at low speeds.
        - Body lift/drag from the hull (flat-plate approximation).
        - Cross-flow drag via strip theory (BatchedParams.cross_flow_drag).
        - Gravity at the CG and Archimedes buoyancy on the voxelized hull against the water surface
          (WaterHydrostatics; the hull comes from the DynamicsSensor's exported "complex" collision mesh).
        - Propeller thrust and torque via an open-water diagram approximation
          (forward and reverse operating modes handled separately).

    Ocean current:
        `current_velocity` (B, 3), world frame (NWU), m/s, constant and irrotational: ν_r = ν − [Rᵀ v_c; 0]
        is used in all hydrodynamic forces, and ν̇ = ν̇_r − [ω × v_c; 0].

    Reference point:
        Fossen's CO (where r_bg, r_bb, the added mass and the damping are given) is the hull centroid, the
        center of buoyancy, so r_bb = 0. The engine reports the state and applies the accelerations at its own
        center of mass; `step` moves them between the two points.

    Frame conventions:
        - Internal dynamics use the NED (North-East-Down) body frame.
        - State input/output from VehicleModel uses NWU (North-West-Up).
        - Conversion between frames is handled by tfrm.convert_NWU_to_NED
          and tfrm.convert_NED_to_NWU at the entry and exit of each step; the
          body-frame derivative ν̇ is turned into the inertial acceleration
          v̇_b + ω × v_b before it is rotated to the world.

    Control abstractions supported:
        - 'cmd_rudders_sterns_motor_speed':
            Direct pass-through — [δ_rt, δ_rb, δ_sl, δ_sr, n] (rad, rad/s).
        - 'cmd_depth_heading_rpm_surge':
            High-level autopilot — [z_ref, ψ_ref_deg, n_rpm, u_ref].
            Depth is controlled by a successive-loop PI (z → θ_d → δ_s).
            Heading is controlled by an integral sliding-mode controller (SMC)
            with a 3rd-order reference model (z → ψ_d → δ_r).
            Surge speed is controlled by a PID on forward velocity, or a
            direct RPM command when surge control is disabled.

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device string, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : One of the supported control abstraction keys above.
    """

    # =========================================================================
    # Static Utilities
    # =========================================================================

    @staticmethod
    def ssa(angle: float) -> torch.Tensor:
        """
        Smallest Signed Angle — wraps an angle to the half-open interval [-π, π).

        Used throughout the control loops to compute minimal angular errors
        without sign discontinuities at ±π.

        Args:
            angle: Scalar or tensor angle in radians.

        Returns:
            Tensor: Angle wrapped to [-π, π).
        """
        return (angle + torch.pi) % (2 * torch.pi) - torch.pi

    @staticmethod
    def integralSMC(
        e_int:     torch.Tensor,
        e_x:       torch.Tensor,
        e_v:       torch.Tensor,
        x_d,
        v_d,
        a_d,
        T_nomoto:  torch.Tensor,
        K_nomoto:  torch.Tensor,
        wn_d:      torch.Tensor,
        zeta_d:    torch.Tensor,
        K_d:       torch.Tensor,
        K_sigma:   torch.Tensor,
        lam:       torch.Tensor,
        phi_b:     torch.Tensor,
        r,
        v_max:     torch.Tensor,
        sampleTime: float
    ):
        """
        Integral Sliding Mode Controller (ISMC) with a 3rd-order reference model.

        Designed for heading (yaw) autopilot of a vessel whose yaw dynamics
        are approximated by Nomoto's first-order model:
            T_nomoto * ṙ + r = K_nomoto * δ

        The controller drives the yaw angle error e_x = ψ − ψ_d to zero while
        using an integral term e_int to eliminate steady-state offset.

        Sliding surface:
            σ = ė_x + 2λ e_x + λ² e_int

        Control law (smooth inside boundary layer, sign outside):
            δ = [T_nomoto * v̇_r + v_r − K_d σ − K_σ sat(σ/φ_b)] / K_nomoto

            where  v_r  = v_d − 2λ e_x − λ² e_int    (reference velocity)
                   v̇_r  = a_d − 2λ ė_x − λ² e_x     (reference acceleration)
                   sat(·) = σ/φ_b  if |σ/φ_b| ≤ 1,   sign(σ) otherwise

        Reference model (3rd-order, inner function refModel3):
            Generates smooth desired trajectories (x_d, v_d, a_d) from a
            step reference r, with natural frequency wn_d and damping zeta_d.
            Desired velocity is clamped to ±v_max for safety.

        Args:
            e_int      : Integral of yaw error (B,).
            e_x        : Yaw angle error ψ − ψ_d (B,).
            e_v        : Yaw rate error r − r_d (B,).
            x_d        : Current desired yaw angle from reference model.
            v_d        : Current desired yaw rate from reference model.
            a_d        : Current desired yaw acceleration from reference model.
            T_nomoto   : Nomoto time constant (B,).
            K_nomoto   : Nomoto gain (B,).
            wn_d       : Reference model natural frequency (rad/s).
            zeta_d     : Reference model damping ratio.
            K_d        : SMC damping gain on σ.
            K_sigma    : SMC switching/smoothing gain on sat(σ/φ_b).
            lam        : Sliding surface pole location λ > 0.
            phi_b      : Boundary layer thickness φ_b > 0.
            r          : Step reference (desired heading) (B,).
            v_max      : Maximum allowed desired yaw rate (rad/s).
            sampleTime : Integration timestep (s).

        Returns:
            tuple:
                delta    : Rudder deflection command (B,) in radians.
                e_int    : Updated integral error state.
                x_d      : Updated desired position from reference model.
                v_d      : Updated desired velocity from reference model.
                a_d      : Updated desired acceleration from reference model.
        """

        def refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime):
            """
            3rd-order reference model that generates smooth desired trajectories
            from a step command r using a critically damped cascaded integrator.

            Jerk:  j_d = ωn³(r − x_d) − 3ωn²ζ v_d − 3ωn ζ a_d
            Then:  x_d ← x_d + dt * v_d
                   v_d ← v_d + dt * a_d     (clamped to ±v_max)
                   a_d ← a_d + dt * j_d

            Args:
                x_d, v_d, a_d : Current desired position, velocity, acceleration.
                r             : Step reference input.
                wn_d          : Natural frequency (rad/s).
                zeta_d        : Damping ratio.
                v_max         : Velocity saturation limit (rad/s).
                sampleTime    : Integration timestep (s).

            Returns:
                Updated (x_d, v_d, a_d).
            """
            j_d = (wn_d**3 * (r - x_d)
                   - 3 * wn_d**2 * zeta_d * v_d
                   - 3 * wn_d   * zeta_d  * a_d)

            x_d = x_d + sampleTime * v_d
            v_d = torch.clamp(v_d + sampleTime * a_d, min=-v_max, max=v_max)
            a_d = a_d + sampleTime * j_d

            return x_d, v_d, a_d

        device = e_x.device
        dtype  = e_x.dtype

        # Ensure all reference states are tensors on the correct device
        x_d = torch.as_tensor(x_d, device=device, dtype=dtype)
        v_d = torch.as_tensor(v_d, device=device, dtype=dtype)
        a_d = torch.as_tensor(a_d, device=device, dtype=dtype)
        r   = torch.as_tensor(r,   device=device, dtype=dtype)

        # ---- Sliding surface and reference signals ----
        v_r_dot = a_d - 2 * lam * e_v  - lam**2 * e_x    # Reference acceleration
        v_r     = v_d - 2 * lam * e_x  - lam**2 * e_int  # Reference velocity
        sigma   = e_v + 2 * lam * e_x  + lam**2 * e_int  # Sliding surface σ

        # ---- Control law: smooth inside boundary layer, switching outside ----
        # sat(σ/φ_b) = σ/φ_b when |σ/φ_b| ≤ 1, sign(σ) otherwise
        abs_ratio = torch.abs(sigma / phi_b)
        delta = torch.where(
            abs_ratio > 1.0,
            (T_nomoto * v_r_dot + v_r - K_d * sigma - K_sigma * torch.sign(sigma)) / K_nomoto,
            (T_nomoto * v_r_dot + v_r - K_d * sigma - K_sigma * (sigma / phi_b))   / K_nomoto
        )

        # ---- Integral update ----
        e_int = e_int + sampleTime * e_x

        # ---- Advance reference model by one timestep ----
        x_d, v_d, a_d = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)

        return delta, e_int, x_d, v_d, a_d

    # =========================================================================
    # Initialization
    # =========================================================================

    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:
        # Initialize shared VehicleModel infrastructure
        super().__init__(batch_size, params, device, control_abstraction)

        self._build_params(params)
        self._init_water(params)
        # Stand-in hull until the engine exports the collision mesh (DynamicsSensor ExportGeometry, "complex").
        self.set_geometry([_box_geometry(params['length'], params['diam'], params['diam'])] * batch_size)

        # Actual (physical) actuator states, shape (B, 5):
        # [δ_rt, δ_rb, δ_sl, δ_sr, n] — initialized to None and lazily
        # allocated as a zero tensor on the first call to get_cmd_motor_speeds,
        # ensuring the device is known at allocation time.
        self.u_actual = None

    # =========================================================================
    # Parameter Construction
    # =========================================================================

    def _build_params(self, params: dict):
        """
        Parses the parameter dictionary and populates self.batched_params with
        all geometry, hydrodynamic, actuator, and control parameters required
        by the AUV dynamics and autopilot.

        Sections (in order):
            1.  Geometry        — body length, diameter, CG/CB offsets.
            2.  Hydrodynamics   — drag, time constants, Nomoto gains.
            3.  Actuators       — fin area, deflection limits, RPM limit,
                                  fin/thruster time constants, lift coefficients.
            4.  Control gains   — depth PI, pitch PID, heading SMC, surge PID.
            5.  Controller state— integral accumulators and reference model states.
            6.  Derived geometry— wetted area, semi-axes, drag coefficient CD_0.
            7.  Mass matrices   — rigid-body MRB, added-mass MA, total M = MRB+MA.
            8.  Buoyancy        — weight W, buoyancy B, hydrostatic stiffness
                                  frequencies w_roll and w_pitch.
            9.  Actuator geometry— rudder and sternplane positions and areas.

        Args:
            params (dict): Raw parameter dictionary from the vehicle config file.
        """

        # ---- 1. Geometry ----
        self.batched_params.D2R    = np.pi / 180       # Degrees-to-radians conversion scalar
        self.batched_params.length = params['length']  # Body length (m)
        self.batched_params.diam   = params['diam']    # Body diameter (m)

        # CG (r_bg) and CB (r_bb) position vectors relative to the CO, in body frame (m)
        # Shape (B, 3) — replicated across the batch
        self.batched_params.r_bg = torch.from_numpy(
            np.array([params['r_bg'] for _ in range(self.batch_size)])
        ).to(self.device)

        self.batched_params.r_bb = torch.from_numpy(
            np.array([params['r_bb'] for _ in range(self.batch_size)])
        ).to(self.device)

        # ---- 2. Hydrodynamics ----
        self.batched_params.r44         = params['r44']           # Roll added-mass radius of gyration
        self.batched_params.Cd          = params['Cd']            # Axial drag coefficient
        self.batched_params.T_surge     = params['T_surge']       # Surge damping time constant (s)
        self.batched_params.T_sway      = params['T_sway']        # Sway damping time constant (s)
        self.batched_params.zeta_roll   = params['zeta_roll']     # Roll damping ratio
        self.batched_params.zeta_pitch  = params['zeta_pitch']    # Pitch damping ratio
        self.batched_params.T_yaw       = params['T_yaw']         # Yaw Nomoto time constant T (s)
        self.batched_params.K_nomoto    = params['K_nomoto']      # Yaw Nomoto gain K

        # ---- 3. Actuators ----
        self.batched_params.S_fin         = params['fin_area']         # Fin planform area (m²)
        self.batched_params.fin_center    = params['fin_center']       # Fin center of pressure (m, from CO)
        # Max fin deflection (rad). It used to keep the degrees (20) and clamp the fins at +-20 rad: they reached
        # -5.6 rad, whose ~33 N of fin drag held the vehicle at half speed on the surface, unable to dive.
        self.batched_params.deltaMax_fin  = params['deltaMax_fin_deg'] * self.batched_params.D2R
        self.batched_params.nMax          = params['nMax']             # Max propeller speed (RPM)
        self.batched_params.T_delta       = params['T_delta']          # Fin actuator time constant (s)
        self.batched_params.T_n           = params['T_n']              # Propeller actuator time constant (s)
        self.batched_params.CL_delta_r    = params['CL_delta_r']       # Rudder lift slope (1/rad)
        self.batched_params.CL_delta_s    = params['CL_delta_s']       # Sternplane lift slope (1/rad)

        # ---- 4. Control gains ----
        # Depth autopilot (successive loop closure: z → θ_d → δ_s)
        self.batched_params.wn_d_z    = params['wn_d_z']       # LP filter natural frequency for z_ref (rad/s)
        self.batched_params.Kp_z      = params['Kp_z']         # Proportional gain, z → θ_d
        self.batched_params.T_z       = params['T_z']          # Integral time constant, z → θ_d
        self.batched_params.Kp_theta  = params['Kp_theta']     # Proportional gain, θ → δ_s
        self.batched_params.Kd_theta  = params['Kd_theta']     # Derivative  gain,  θ → δ_s
        self.batched_params.Ki_theta  = params['Ki_theta']     # Integral    gain,  θ → δ_s
        self.batched_params.K_w       = params['K_w']          # Heave velocity damping gain
        self.batched_params.theta_max = params['theta_max_deg'] * self.batched_params.D2R  # Max pitch command (rad)

        # Heading SMC autopilot (ψ → δ_r)
        self.batched_params.wn_d    = params['wn_d']       # Reference model natural frequency (rad/s)
        self.batched_params.zeta_d  = params['zeta_d']     # Reference model damping ratio
        self.batched_params.r_max   = params['r_max']      # Max desired yaw rate (rad/s)
        self.batched_params.lam     = params['lam']        # SMC sliding surface pole λ
        self.batched_params.phi_b   = params['phi_b']      # SMC boundary layer thickness
        self.batched_params.K_d     = params['K_d']        # SMC damping gain
        self.batched_params.K_sigma = params['K_sigma']    # SMC switching gain

        # Surge PID (u → n)
        self.batched_params.Kp_surge = params['kp_surge']
        self.batched_params.Kd_surge = params['kd_surge']
        self.batched_params.Ki_surge = params['ki_surge']

        # ---- 5. Controller state (scalars, updated in-place each step) ----
        self.batched_params.r_z     = 0   # Depth reference setpoint
        self.batched_params.r_psi   = 0   # Heading reference setpoint
        self.batched_params.r_rpm   = 0   # RPM reference setpoint
        self.batched_params.r_surge = 0   # Surge speed reference setpoint

        # Body-frame 6-DOF velocity ν = [u, v, w, p, q, r]ᵀ, initialized to rest
        self.batched_params.nu = torch.zeros(self.batch_size, 6, device=self.device).double()

        # Surge PID state
        self.u_int        = 0   # Surge velocity error integral
        self._last_error  = 0   # Surge error from previous step (for derivative)

        # Depth autopilot state
        self.z_int   = 0   # Depth position integral
        self.z_d     = 0   # Low-pass filtered desired depth (LP filter state)

        # Pitch autopilot state
        self.theta_int = 0   # Pitch angle error integral

        # Heading SMC state
        self.psi_d     = 0   # Desired heading from reference model
        self.r_d       = 0   # Desired yaw rate from reference model
        self.a_d       = 0   # Desired yaw acceleration from reference model
        self.e_psi_int = 0   # Yaw angle error integral

        # Previous-step values for finite-difference derivatives
        self.prev_pitch = 0
        self.prev_yaw   = 0

        # Depth and heading references are seeded from the vehicle on the first autopilot call
        self._references_seeded = False

        # ---- 6. Derived geometry and drag ----
        # Wetted lateral area S = 0.7 * L * D (empirical approximation)
        self.batched_params.S = 0.7 * self.batched_params.length * self.batched_params.diam

        # Semi-major and semi-minor axes of the prolate spheroid hull model
        self.batched_params.a = self.batched_params.length * 0.5
        b                     = self.batched_params.diam   * 0.5

        # Zero-lift axial drag coefficient: CD_0 = Cd * π b² / S
        self.batched_params.CD_0 = self.batched_params.Cd * np.pi * b**2 / self.batched_params.S

        # ---- 7. Mass matrices ----
        m  = self.batched_params.mass
        Ix = (2/5) * m * b**2                                    # Roll inertia (sphere approx.)
        Iy = (1/5) * m * (self.batched_params.a**2 + b**2)       # Pitch inertia (prolate spheroid)
        Iz = Iy                                                   # Yaw inertia (symmetric)

        # Rigid-body mass matrix expressed at the CG
        MRB_CG = torch.from_numpy(
            np.array([np.diag([m, m, m, Ix, Iy, Iz]) for _ in range(self.batch_size)])
        ).to(self.device)

        # Transform MRB from CG to CO using the H-matrix:
        # MRB_CO = H(r_bg)ᵀ MRB_CG H(r_bg)
        H_rg = bp.Hmtrx(self.batched_params.r_bg)
        self.batched_params.MRB = H_rg.mT @ MRB_CG @ H_rg

        # Weight (N). Buoyancy comes from the hull voxels (see _restoring), not from W = B.
        self.batched_params.W = self.batched_params.mass * self.batched_params.g
        self.batched_params.volume = params['volume']   # Displaced volume when submerged (m³)

        # ---- Added-mass matrix (prolate spheroid, potential flow) ----
        # Lamb's k-factors k1, k2, k' from the eccentricity of the spheroid
        MA_44 = self.batched_params.r44 * Ix   # Roll added-moment-of-inertia

        e        = np.sqrt(1.0 - (b / self.batched_params.a)**2)   # Eccentricity
        log_term = np.log((1.0 + e) / (1.0 - e))

        # Prolate-spheroid coefficients α₀, β₀ (Lamb 1932)
        alpha_0 = ((2.0 * (1.0 - e**2) / e**3)
                   * (0.5 * log_term - e))

        beta_0  = (1.0 / e**2
                   - (1.0 - e**2) / (2.0 * e**3) * log_term)

        # Lamb k-factors for surge (k1), sway/heave (k2), and pitch/yaw (k')
        k1      = alpha_0 / (2.0 - alpha_0)
        k2      = beta_0  / (2.0 - beta_0)
        k_prime = (e**4 * (beta_0 - alpha_0)
                   / ((2.0 - e**2)
                      * (2.0 * e**2 - (2.0 - e**2) * (beta_0 - alpha_0))))

        # Diagonal added-mass matrix: [m k1, m k2, m k2, MA_44, k' Iy, k' Iy]
        self.batched_params.MA = torch.from_numpy(
            np.array([np.diag([m*k1, m*k2, m*k2, MA_44, k_prime*Iy, k_prime*Iy])
                      for _ in range(self.batch_size)])
        ).to(self.device)

        # Total mass matrix and its inverse (pre-computed for efficiency)
        self.batched_params.M    = self.batched_params.MRB + self.batched_params.MA
        self.batched_params.Minv = torch.linalg.inv(self.batched_params.M)

        # ---- 8. Hydrostatic stiffness natural frequencies ----
        # Roll and pitch restoring: ω² = (W * dz) / M_ii
        # where dz = z_G − z_B is the metacentric height (CG above CB → positive stability)
        dz           = self.batched_params.r_bg[:, 2] - self.batched_params.r_bb[:, 2]
        self.w_roll  = torch.sqrt(self.batched_params.W * dz / self.batched_params.M[:, 3, 3])
        self.w_pitch = torch.sqrt(self.batched_params.W * dz / self.batched_params.M[:, 4, 4])

        # Heave and yaw damping time constants aliased for consistency with
        # the D matrix assembly in _compute_external_forces
        self.batched_params.T_heave  = self.batched_params.T_sway    # Heave ≈ sway dynamics
        self.batched_params.T_nomoto = self.batched_params.T_yaw     # Nomoto T for heading SMC

        # ---- 9. Actuator geometry ----
        # Rudder (horizontal fins): located at the stern, x = −a
        self.A_r = self.batched_params.S_fin   # Rudder planform area (m²)
        self.x_r = -self.batched_params.a      # Rudder x-position in body frame (m)

        # Sternplane (vertical fins): same geometry as rudder for this vehicle
        self.A_s = self.batched_params.S_fin
        self.x_s = -self.batched_params.a      # Sternplane x-position in body frame (m)

        # Surge PID is disabled until a non-zero surge reference is received
        self.surge_control = False

    # =========================================================================
    # Hydrodynamic Sub-models
    # =========================================================================

    def _forceLiftDrag(self, alpha: Tensor, U_r: Tensor) -> torch.Tensor:
        """
        Computes the 6-DOF hydrodynamic lift and drag wrench on the AUV hull
        using a flat-plate / thin-wing approximation.

        Lift model:
            CL_α = π AR / (1 + √(1 + (AR/2)²))   (Prandtl finite-wing correction)
            CL   = CL_α sin α cos α                     (= CL_α α at small α; 0 at ±90°)
            CD   = CD_0 + CL² / (π e AR)           (induced + zero-lift drag)

        Force magnitudes:
            F_drag = ½ ρ S CD U_r²
            F_lift = ½ ρ S CL U_r²

        Projected into body frame (α measured in XZ-plane):
            Fx = −F_drag cos α + F_lift sin α
            Fz =  F_drag sin α − F_lift cos α       (sign: +Z is down in NED)

        Only surge (Fx) and heave (Fz) are non-zero; sway and all moments
        are zero for a symmetric hull at zero sideslip.

        Args:
            alpha (Tensor): Angle of attack α = atan2(w_r, u_r), shape (B,).
            U_r   (Tensor): Relative flow speed magnitude (m/s), shape (B,).

        Returns:
            Tensor: 6-DOF body-frame wrench [Fx,Fy,Fz,Mx,My,Mz], shape (6, B).
                    (Note: transposed relative to other force functions — callers
                    use .T to obtain shape (B, 6).)
        """
        e  = 0.7   # Oswald efficiency factor (empirical)
        AR = self.batched_params.diam**2 / self.batched_params.S   # Aspect ratio

        # Finite-wing lift slope (rad⁻¹)
        CL_alpha = np.pi * AR / (1 + np.sqrt(1 + (AR / 2)**2))
        # CL_α sin α cos α: CL_α α at small α, and no lift at ±90° (CL_α α, valid only for small α, gave full
        # lift when the hull rises or sinks with no forward speed, which projected into a forward surge force).
        CL       = CL_alpha * torch.sin(alpha) * torch.cos(alpha)
        CD       = self.batched_params.CD_0 + CL**2 / (torch.pi * e * AR)

        # Force magnitudes (N)
        F_drag = 0.5 * self.batched_params.rho * self.batched_params.S * CD * U_r**2
        F_lift = 0.5 * self.batched_params.rho * self.batched_params.S * CL * U_r**2

        # Project into body-frame XZ plane; sway and all moment components are zero
        tau_liftdrag = torch.stack([
            torch.cos(alpha) * (-F_drag) - torch.sin(alpha) * (-F_lift),  # Fx
            torch.zeros_like(alpha),                                        # Fy
            torch.sin(alpha) * (-F_drag) + torch.cos(alpha) * (-F_lift),  # Fz
            torch.zeros_like(alpha),                                        # Mx
            torch.zeros_like(alpha),                                        # My
            torch.zeros_like(alpha)                                         # Mz
        ])  # (6, B)

        return tau_liftdrag

    def set_geometry(self, geometries: list[dict]) -> None:
        """Hull voxels for the hydrostatics, and the CO, from each robot's collision mesh.

        Use the closed complex mesh (DynamicsSensor GeometrySource "complex"). The voxels are scaled to the
        specified displaced `volume`; the CO is put at their centroid (the center of buoyancy).

        Args:
            geometries: one DynamicsSensor.geometry dict per robot (vertices/triangles in the body frame).
        """
        if len(geometries) != self.batch_size:
            raise ValueError(f"Expected {self.batch_size} geometries, one per robot, got {len(geometries)}")
        voxels, volumes, offsets = [], [], []
        for geometry in geometries:
            vertices = np.asarray(geometry["vertices"], dtype=np.float64)
            centers = _solid_voxel_centers(vertices, np.asarray(geometry["triangles"], dtype=np.int64),
                                           HYDROSTATIC_VOXEL)
            centroid = centers.mean(axis=0)
            origin = (np.asarray(geometry["center_of_mass"], dtype=np.float64) if geometry["mass"] > 0
                      else centroid)
            voxels.append(centers - centroid)
            volumes.append(np.full(len(centers), self.batched_params.volume / len(centers)))
            offsets.append(centroid - origin)
        count = max(len(v) for v in voxels)
        pad = lambda a: np.concatenate([a, np.zeros((count - len(a),) + a.shape[1:])])   # zero-volume padding
        self._voxels = torch.from_numpy(np.stack([pad(v) for v in voxels])).double().to(self.device)
        self._voxel_volume = torch.from_numpy(np.stack([pad(v) for v in volumes])).double().to(self.device)
        # CO from the engine's center of mass, body frame NWU (B, 3).
        self._co_offset = torch.from_numpy(np.stack(offsets)).double().to(self.device)

    def _restoring(self, x: Tensor, q: Tensor) -> Tensor:
        """Gravity at the CG plus buoyancy on the wet hull voxels, about the CO, in the NED body frame (B, 6).

        Replaces Fossen's g(η) for a fully submerged, neutrally buoyant body: the net weight, the righting moment
        and partial immersion at the surface (waves included) all follow from the hull.

        Args:
            x: CO position, world frame NWU (B, 3).
            q: attitude quaternion [x, y, z, w], body -> world NWU (B, 4).
        """
        R = roma.unitquat_to_rotmat(q)
        F_b, M_b, _ = self._buoyancy(x, R)                                      # body NWU, about the CO
        flip = torch.tensor([1.0, -1.0, -1.0], dtype=torch.float64, device=self.device)   # NED <-> NWU
        weight = -self.batched_params.W * R[:, 2, :]                             # body NWU
        M_g = torch.cross(self.batched_params.r_bg * flip, weight, dim=-1)       # r_bg given in NED
        return torch.cat([(F_b + weight) * flip, (M_b + M_g) * flip], dim=-1)

    def _tick_dt(self) -> float:
        """Engine tick (s): the actuator lags and the autopilot integrate over it."""
        return float(self.dt) if self.dt else 1.0 / 20

    # =========================================================================
    # Actuator Wrench
    # =========================================================================

    def _compute_body_wrench(self, u_actual: Tensor, nu_r: Tensor) -> tuple:
        """
        Computes the hydrodynamic forces and moments produced by the four
        control surfaces (rudders and sternplanes) given their current
        deflection angles and the relative flow velocity.

        Fin layout:
            δ_rt, δ_rb : top and bottom rudder deflections (yaw control, sway force)
            δ_sl, δ_sr : left and right sternplane deflections (pitch/depth control)

        Fin force model (linearized lift, quadratic drag):
            Sway force  Y_r = −½ ρ U_rh² A_r CL_δr (δ_rt + δ_rb)
            Heave force Z_s = +½ ρ U_rv² A_s CL_δs (δ_sl + δ_sr)
            Axial drag  X   = −½ ρ U²   A   CL     δ²   (always resistive)

        Moments from fin forces via lever arm x_r / x_s (stern location):
            My (pitch) = −x_s * Z_s    (sternplane heave force × arm)
            Mz (yaw)   =  x_r * Y_r    (rudder sway force × arm)
            Mx (roll)  = 0              (symmetric fin pairs)

        Note: propeller thrust and torque are added in _s_dot_fn, not here.

        Args:
            u_actual (Tensor): Actual (lagged) actuator states (B, 5):
                               [δ_rt, δ_rb, δ_sl, δ_sr, n_rpm].
            nu_r (Tensor)    : Relative body-frame velocity ν − ν_c (B, 6).

        Returns:
            tuple:
                forces  (Tensor): Body-frame forces  [Fx, Fy, Fz] (B, 3).
                moments (Tensor): Body-frame moments [Mx, My, Mz] (B, 3).
        """
        delta_rt = u_actual[:, 0]   # Top    rudder deflection (rad)
        delta_rb = u_actual[:, 1]   # Bottom rudder deflection (rad)
        delta_sl = u_actual[:, 2]   # Left   sternplane deflection (rad)
        delta_sr = u_actual[:, 3]   # Right  sternplane deflection (rad)

        # Relative flow speeds in the horizontal and vertical planes
        U_rh = torch.sqrt(nu_r[:, 0]**2 + nu_r[:, 1]**2)   # Horizontal (surge + sway)
        U_rv = torch.sqrt(nu_r[:, 0]**2 + nu_r[:, 2]**2)   # Vertical   (surge + heave)

        # ---- Axial drag from fin deflections (always opposes motion) ----
        X_r = (-0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r
               * (delta_rt**2 + delta_rb**2))
        X_s = (-0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s
               * (delta_sl**2 + delta_sr**2))

        # ---- Sway force from rudders (linearized in δ) ----
        Y_r = (-0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r
               * (delta_rt + delta_rb))

        # ---- Heave force from sternplanes (linearized in δ) ----
        Z_s = (0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s
               * (delta_sl + delta_sr))

        forces  = torch.stack([X_r + X_s, Y_r, Z_s], dim=1)                           # (B, 3)
        moments = torch.stack([torch.zeros_like(X_r), -self.x_s * Z_s, self.x_r * Y_r], dim=1)  # (B, 3)

        return forces, moments

    # =========================================================================
    # External Forces (Coriolis, Damping, Lift/Drag, Cross-flow, Restoring)
    # =========================================================================

    def _compute_external_forces(self, alpha: Tensor, U_r: Tensor,
                                  nu_r: Tensor, tau: Tensor, restoring: Tensor) -> Tensor:
        """
        Assembles the total generalized force vector τ_sum for the equations
        of motion by combining all external hydrodynamic contributions:

            M ν̇ = τ_sum = τ − (C + D) ν_r + τ_lift + τ_cross + τ_restoring

        where:
            τ          : Control forces from propeller + control surfaces.
            C = CRB+CA : Rigid-body + added-mass Coriolis/centripetal matrix.
            D          : Linear damping matrix (frequency-dependent for surge/sway).
            τ_lift     : Hull lift/drag wrench from _forceLiftDrag.
            τ_cross    : Cross-flow drag from BatchedParams.cross_flow_drag, in the horizontal
                         (sway, yaw) and, by symmetry of revolution, the vertical (heave, pitch) plane.
            τ_restoring: Gravity + buoyancy wrench from _restoring.

        Damping model:
            Diagonal D matrix with entries M_ii / T_i for translational DOFs
            and 2 ζ ωn M_ii for rotational DOFs (roll and pitch).
            Surge and sway entries are attenuated by exp(−3 U_r) to reduce
            linear damping at high speeds where nonlinear drag dominates.

        Destabilizing CA terms zeroed:
            Several off-diagonal entries of the added-mass Coriolis matrix CA
            that produce unrealistic coupling are explicitly set to zero
            (common practice for slender underwater vehicles — see Fossen 2021).

        Args:
            alpha (Tensor): Angle of attack (B,), rad.
            U_r   (Tensor): Relative flow speed magnitude (B,), m/s.
            nu_r  (Tensor): Relative body-frame velocity ν − ν_c (B, 6).
            tau   (Tensor): Control + propeller wrench (B, 6).
            restoring (Tensor): Gravity + buoyancy wrench (B, 6), from _restoring.

        Returns:
            Tensor: Total generalized force τ_sum (B, 6) entering M ν̇ = τ_sum.
        """

        # ---- Coriolis / centripetal matrices ----
        CRB = self.batched_params.m2c(self.batched_params.MRB, nu_r)   # Rigid-body  (B, 6, 6)
        CA  = self.batched_params.m2c(self.batched_params.MA,  nu_r)   # Added-mass  (B, 6, 6)

        # Zero numerically destabilizing off-diagonal CA terms (Fossen 2021, Ch. 8)
        CA[:, 4, 0] = CA[:, 0, 4] = 0.0
        CA[:, 2, 4] = 0.0
        CA[:, 5, 0] = CA[:, 5, 1] = 0.0
        CA[:, 1, 5] = 0.0

        C = CRB + CA   # (B, 6, 6)

        # ---- Linear damping matrix ----
        # Translational DOFs: d_ii = M_ii / T_i
        # Rotational DOFs:    d_ii = 2 ζ_i ωn_i M_ii
        D = torch.diag_embed(torch.stack([
            self.batched_params.M[:, 0, 0] / self.batched_params.T_surge,
            self.batched_params.M[:, 1, 1] / self.batched_params.T_sway,
            self.batched_params.M[:, 2, 2] / self.batched_params.T_heave,
            self.batched_params.M[:, 3, 3] * 2.0 * self.batched_params.zeta_roll  * self.w_roll,
            self.batched_params.M[:, 4, 4] * 2.0 * self.batched_params.zeta_pitch * self.w_pitch,
            self.batched_params.M[:, 5, 5] / self.batched_params.T_yaw,
        ], dim=1))   # (B, 6, 6)

        # Attenuate surge and sway damping at high relative speeds: D * exp(−3 U_r)
        exp_term    = torch.exp(-3.0 * U_r)
        D[:, 0, 0] *= exp_term
        D[:, 1, 1] *= exp_term

        # ---- Hydrodynamic lift/drag and cross-flow drag ----
        tau_liftdrag  = self._forceLiftDrag(alpha, U_r)                              # (6, B)
        tau_crossflow = bp.cross_flow_drag(
            self.batched_params.rho,
            self.batched_params.length,
            self.batched_params.diam,
            self.batched_params.diam,
            nu_r
        )                                                                             # (B, 6)
        # Same strip theory in the vertical plane (body of revolution): the strip at x moves at w − x q, so it is
        # the sway case with (v, r) -> (w, −q), giving heave force Z = Y and pitch moment M = −N.
        nu_vertical = torch.zeros_like(nu_r)
        nu_vertical[:, 1] = nu_r[:, 2]
        nu_vertical[:, 5] = -nu_r[:, 4]
        vertical = bp.cross_flow_drag(self.batched_params.rho, self.batched_params.length,
                                      self.batched_params.diam, self.batched_params.diam, nu_vertical)
        tau_crossflow = tau_crossflow.clone()
        tau_crossflow[:, 2] += vertical[:, 1]
        tau_crossflow[:, 4] -= vertical[:, 5]

        # ---- Equations of motion: τ_sum = τ + τ_lift + τ_cross − (C+D) ν_r + τ_restoring ----
        tau_sum = (tau
                   + tau_liftdrag.T
                   + tau_crossflow.squeeze(-1)
                   - torch.einsum('bij,bj->bi', C + D, nu_r)
                   ).squeeze(-1) + restoring

        return tau_sum   # (B, 6)

    # =========================================================================
    # State Derivative
    # =========================================================================

    def _s_dot_fn(self, s: Tensor, cmd_ctrl: Tensor) -> Tensor:
        """
        Evaluates the continuous-time state derivative ṡ = f(s, u) for the
        AUV rigid-body dynamics, using NED body-frame conventions internally.

        Pipeline:
            1. Rotate the ocean current (current_velocity, world) into the body frame (ν_c).
            2. Compute relative velocity ν_r = ν − ν_c and kinematic quantities
               (angle of attack α, vehicle speed U, relative speed U_r).
            3. Compute propeller thrust and torque via open-water diagram.
            4. Assemble total control wrench τ = [T_prop, Y_r, Z_s, K_prop, My, Mz].
            5. Evaluate τ_sum via _compute_external_forces.
            6. Integrate actuator states (fins and propeller) with first-order lag.
            7. Convert NED ν̇ back to NWU and pack into s_dot.

        Propeller model:
            Forward:  X_prop = ρ D⁴ [KT_0 |n| n + (KT_max−KT_0)/Ja_max * (Va/D)|n|]
            Reverse:  X_prop = ρ D⁴   KT_0 |n| n     (no advance speed correction)
            Torque:   K_prop analogously with KQ coefficients.

        Actuator dynamics (first-order lag, exact over the engine tick dt):
            u ← u + (1 − exp(−dt/T)) (cmd − u),  T = T_delta (fins) or T_n (propeller)
            Then clamp to hardware limits (±δMax_fin, ±nMax).

        Args:
            s (Tensor)        : Packed state vector (B, 13), from VehicleModel.pack_state.
            cmd_ctrl (Tensor) : Desired actuator commands (B, 5):
                                [δ_rt_cmd, δ_rb_cmd, δ_sl_cmd, δ_sr_cmd, n_cmd].

        Returns:
            Tensor: State derivative ṡ (B, 13 + 5). Actuator derivatives are
                    integrated in-place here; the returned motor-speed slots are zero.
        """
        state = self.unpack_state(s, self.idxs, self.batch_size)
        x = state['x'][self.idxs].double()
        q = state['q'][self.idxs].double()
        flip = torch.tensor([1.0, -1.0, -1.0], dtype=torch.float64, device=self.device)   # NWU <-> NED

        # ---- Ocean current in the body frame (NED) ----
        R = roma.unitquat_to_rotmat(q)
        v_c = (self.current_velocity[self.idxs].unsqueeze(1) @ R).squeeze(1) * flip    # R^T v_c, then NED
        nu_c = torch.cat([v_c, torch.zeros_like(v_c)], dim=1)                          # (B, 6)

        # A current constant in the world turns in the body frame: d/dt(R^T v_c) = −ω × v_c
        Dnu_c = torch.cat([-torch.cross(self.nu[:, 3:6], v_c, dim=-1), torch.zeros_like(v_c)], dim=1)

        # Relative velocity and derived quantities
        nu_r  = self.nu - nu_c
        alpha = torch.arctan2(nu_r[:, 2], nu_r[:, 0])                            # Angle of attack (rad)
        U     = torch.sqrt(self.nu[:, 0]**2 + self.nu[:, 1]**2 + self.nu[:, 2]**2)  # Vehicle speed (m/s)
        U_r   = torch.sqrt(nu_r[:, 0]**2   + nu_r[:, 1]**2   + nu_r[:, 2]**2)       # Relative speed (m/s)

        # ---- Control surface forces ----
        forces, moments = self._compute_body_wrench(self.u_actual, nu_r)

        # ---- Propeller thrust and torque ----
        n      = self.u_actual[:, -1]   # Actual propeller speed (RPM)
        D_prop = torch.full_like(n, 0.14)   # Propeller diameter (m)
        t_prop = torch.full_like(n, 0.1)    # Thrust deduction fraction

        n_rps  = n / 60.0                   # RPM → rev/s
        Va     = 0.944 * U                  # Advance speed (Taylor wake fraction 0.056)

        # Open-water diagram coefficients
        Ja_max  = torch.full_like(n, 0.6632)   # Advance ratio at KT=0
        KT_0    = torch.full_like(n, 0.4566)   # Thrust coefficient at Ja=0
        KQ_0    = torch.full_like(n, 0.0700)   # Torque coefficient at Ja=0
        KT_max  = torch.full_like(n, 0.1798)   # Thrust coefficient at Ja=Ja_max
        KQ_max  = torch.full_like(n, 0.0312)   # Torque coefficient at Ja=Ja_max

        abs_n = torch.abs(n_rps)

        # Forward operating mode: linear KT/KQ interpolation with advance ratio
        X_fwd = self.batched_params.rho * D_prop**4 * (
            KT_0 * abs_n * n_rps
            + (KT_max - KT_0) / Ja_max * (Va / D_prop) * abs_n
        )
        K_fwd = self.batched_params.rho * D_prop**5 * (
            KQ_0 * abs_n * n_rps
            + (KQ_max - KQ_0) / Ja_max * (Va / D_prop) * abs_n
        )

        # Reverse operating mode: static thrust only (no advance-speed correction)
        X_rev = self.batched_params.rho * D_prop**4 * KT_0 * abs_n * n_rps
        K_rev = self.batched_params.rho * D_prop**5 * KQ_0 * abs_n * n_rps

        X_prop = torch.where(n_rps > 0, X_fwd, X_rev)   # (B,) net surge thrust
        K_prop = torch.where(n_rps > 0, K_fwd, K_rev)   # (B,) net roll torque

        # ---- Total control wrench τ (B, 6) ----
        # Surge: thrust minus deduction factor; other DOFs from control surfaces
        tau = torch.stack([
            (1.0 - t_prop) * X_prop + forces[:, 0],   # Surge: propeller + fin drag
            forces[:, 1],                               # Sway:  rudder lift
            forces[:, 2],                               # Heave: sternplane lift
            K_prop / 10.0 + moments[:, 0],              # Roll:  propeller torque (/10 as in Fossen's remus100, to match experiments) + fin
            moments[:, 1],                              # Pitch: sternplane moment
            moments[:, 2],                              # Yaw:   rudder moment
        ], dim=1)

        # ---- Equations of motion ----
        tau_sum = self._compute_external_forces(alpha, U_r, nu_r, tau, self._restoring(x, q))
        nu_dot  = Dnu_c + (self.batched_params.Minv @ tau_sum.unsqueeze(-1)).squeeze(-1)

        # ---- Actuator dynamics: first-order lag, exact over the engine tick ----
        dt = self._tick_dt()
        self.u_actual[:, :-1] += (1 - np.exp(-dt / self.batched_params.T_delta)) * (cmd_ctrl[:, :-1] - self.u_actual[:, :-1])
        self.u_actual[:, -1]  += (1 - np.exp(-dt / self.batched_params.T_n)) * (cmd_ctrl[:, -1] - self.u_actual[:, -1])

        # Clamp to hardware limits
        self.u_actual[:, :-1] = torch.clamp(
            self.u_actual[:, :-1],
            min=-self.batched_params.deltaMax_fin,
            max= self.batched_params.deltaMax_fin
        )
        self.u_actual[:, -1] = torch.clamp(
            self.u_actual[:, -1],
            min=-self.batched_params.nMax,
            max= self.batched_params.nMax
        )

        # ---- Inertial acceleration v̇_b + ω × v_b (ν̇ is the body-frame derivative), then NED → NWU world ----
        nu_dot = torch.cat([nu_dot[:, :3] + torch.cross(self.nu[:, 3:6], self.nu[:, :3], dim=-1), nu_dot[:, 3:]], dim=1)
        nu_dot = tfrm.convert_NED_to_NWU(q, nu_dot)

        s_dot = torch.zeros((len(self.idxs), 13 + cmd_ctrl.shape[1],), dtype=torch.float64, device=self.device)
        s_dot[:, 0:3]  = state['v'][self.idxs]     # ẋ = v  (world frame)
        s_dot[:, 3:6]  = nu_dot[:, 0:3]             # v̇ (NWU)
        s_dot[:, 6:10] = state['q'][self.idxs]      # q̇ (handled externally)
        s_dot[:, 10:13] = nu_dot[:, 3:6]            # ω̇ (NWU)

        return s_dot

    @VehicleModel.map_states
    def step(self, state: dict, control: dict):
        """One engine tick: autopilot and dynamics at the CO, accelerations returned for the engine's CoM.

        With r the CO from the CoM in the world frame and ω the world-frame angular velocity:
            x_CO = x + r,   v_CO = v + ω × r,   a_CoM = a_CO − ω̇ × r − ω × (ω × r)

        Args:
            state (dict)  : Structured state dict (populated by @map_states), at the engine's CoM.
            control (dict): Structured control dict (populated by @map_states).

        Returns:
            list: Concatenated [v̇, ω̇] (B, 6), world frame, as a CPU float list.
        """
        if self.control_abstraction == 'accel':
            return control['cmd_ctrl'][self.idxs].cpu().tolist()
        R = roma.unitquat_to_rotmat(state['q']).double()
        r = (R @ self._co_offset.unsqueeze(-1)).squeeze(-1)                     # (B, 3) world
        w = state['w']                                                         # world frame (engine)
        at_co = {**state, 'x': state['x'] + r, 'v': state['v'] + torch.cross(w, r, dim=-1)}

        cmd_ctrl = torch.clip(self.get_cmd_motor_speeds(at_co, control),
                              self.batched_params.rotor_speed_min[self.idxs],
                              self.batched_params.rotor_speed_max[self.idxs])
        s_dot = self._s_dot_fn(self.pack_state(at_co, self.batch_size, self.device), cmd_ctrl)

        accel = torch.zeros(self.batch_size, 6, dtype=torch.float64, device=self.device)
        alpha = s_dot[:, 10:13]
        idx = self.idxs
        accel[idx, :3] = (s_dot[:, 3:6] - torch.cross(alpha, r[idx], dim=-1)
                          - torch.cross(w[idx], torch.cross(w[idx], r[idx], dim=-1), dim=-1))
        accel[idx, 3:] = alpha
        return accel.cpu().tolist()

    # =========================================================================
    # Control → Actuator Commands
    # =========================================================================

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        """
        Converts a high-level control command into actuator commands
        [δ_rt, δ_rb, δ_sl, δ_sr, n] using the AUV's autopilot hierarchy.

        Entry point responsibilities:
            1. Convert NWU state to NED pose η and velocity ν for all
               autopilot computations (tfrm.convert_NWU_to_NED).
            2. Lazily initialize u_actual to zeros on first call.
            3. Dispatch to the appropriate control abstraction.

        Control abstractions:

        'cmd_rudders_sterns_motor_speed':
            Pass-through — returns raw [δ_rt, δ_rb, δ_sl, δ_sr, n] commands
            directly from control['cmd_ctrl'], bypassing all autopilot logic.

        'cmd_depth_heading_rpm_surge':
            Simultaneous depth, heading, and speed autopilot.
            Input: [z_ref (m), ψ_ref (deg), n_rpm, u_ref (m/s)]

            If both n_rpm and u_ref are zero, returns zero commands (vehicle stopped).

            Surge control (when u_ref > 0):
                PID on forward velocity error (u − u_ref).
                n = Kp_surge * e + Ki_surge * ∫e + Kd_surge * ė
                Integrated and clamped to ±nMax RPM.
            Surge open-loop (when u_ref = 0):
                n = n_rpm directly.

            Depth autopilot (successive loop closure):
                Step 1: LP filter z_ref → z_d with bandwidth wn_d_z.
                Step 2: PI on depth error → desired pitch θ_d:
                    θ_d = Kp_z * [(z − z_d) + (1/T_z) * ∫(z − z_d)]
                    Saturated to ±theta_max.
                Step 3: PID on pitch error → sternplane deflection δ_s:
                    δ_s = −Kp_θ ssa(θ−θ_d) − Kd_θ q − Ki_θ ∫(θ−θ_d) − K_w w

            Heading autopilot (integral SMC):
                Integral SMC with 3rd-order reference model (see integralSMC).
                Drives ψ → ψ_ref via rudder deflection δ_r.
                Reference model state (ψ_d, ṙ_d, r̈_d) persists between steps.

            Final command assembly:
                δ_rt = δ_rb = δ_r   (symmetric rudder pair)
                δ_sl = δ_sr = δ_s   (symmetric sternplane pair)
                Output: [δ_rt, δ_rb, δ_sl, δ_sr, n]

        Args:
            state (dict)  : Structured state dict {x, v, q, w} from VehicleModel.map_states.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).

        Returns:
            Tensor: Actuator commands (B, 5): [δ_rt, δ_rb, δ_sl, δ_sr, n].
        """

        # Convert NWU state to NED pose η and body-frame velocity ν
        self.eta, self.nu = tfrm.convert_NWU_to_NED(
            state['x'][self.idxs],
            state['q'][self.idxs],
            state['v'][self.idxs],
            state['w'][self.idxs]
        )

        # Lazy initialization of actual actuator state on first call
        if not isinstance(self.u_actual, Tensor):
            self.u_actual = torch.zeros(self.batch_size, 5, device=self.device).double()

        # ---- Pass-through ----
        if self.control_abstraction == 'cmd_rudders_sterns_motor_speed':
            return control['cmd_ctrl'][self.idxs]

        # =====================================================================
        # cmd_depth_heading_rpm_surge: simultaneous depth + heading + speed autopilot
        # =====================================================================
        if self.control_abstraction == 'cmd_depth_heading_rpm_surge':

            self.ref_z   = control['cmd_ctrl'][:, 0]   # Depth setpoint (m, positive down NED)
            self.ref_psi = control['cmd_ctrl'][:, 1]   # Heading setpoint (deg)
            self.ref_n   = control['cmd_ctrl'][:, 2]   # Propeller RPM command
            self.ref_u   = control['cmd_ctrl'][:, 3]   # Surge speed setpoint (m/s)

            # Enable surge PID only when a positive speed reference is given
            self.surge_control = True if self.ref_u > 0 else False

            # Extract relevant state quantities from NED ν and η
            z     = self.eta[:, 2]   # Depth (m)
            theta = self.eta[:, 4]   # Pitch angle (rad)
            psi   = self.eta[:, 5]   # Yaw angle   (rad)
            w     = self.nu[:, 2]    # Heave velocity (m/s)
            q     = self.nu[:, 4]    # Pitch rate (rad/s)
            r     = self.nu[:, 5]    # Yaw rate   (rad/s)

            # Seed the reference filters with the current depth and heading: starting them at 0 (the surface,
            # north) made a vehicle spawned at depth climb toward the surface for the first ~20 s.
            if not self._references_seeded:
                self.z_d, self.psi_d = z.clone(), psi.clone()
                self.r_d, self.a_d = torch.zeros_like(r), torch.zeros_like(r)
                self._references_seeded = True

            # Heading tracking errors for SMC (wrapped: ψ is in [−π, π), ψ_d is continuous)
            e_psi = self.ssa(psi - self.psi_d)   # Yaw angle error
            e_r   = r   - self.r_d               # Yaw rate  error

            # Heading reference: the equivalent of the command nearest the reference model's heading, so a
            # command across ±180° turns the short way.
            psi_ref = self.ref_psi * self.batched_params.D2R   # degrees → radians
            psi_ref = self.psi_d + self.ssa(psi_ref - self.psi_d)

            dt = self._tick_dt()

            # Only run autopilot when a non-zero thrust command is present
            if self.ref_n > 0 or self.ref_u > 0:

                # ---- Surge controller ----
                if self.surge_control:
                    u          = self.nu[:, 0]
                    error      = self.ref_u - u
                    derivative = (error - self._last_error) / dt

                    n = (self.batched_params.Kp_surge * error
                         + self.batched_params.Ki_surge * self.u_int
                         + self.batched_params.Kd_surge * derivative)

                    self.u_int    += dt * error
                    self._last_error = error

                    n = torch.clamp(n, min=-self.batched_params.nMax, max=self.batched_params.nMax)
                else:
                    n = self.ref_n   # Open-loop RPM command

                # ---- Depth autopilot (successive loop closure) ----
                # Step 1: first-order LP filter on depth reference
                self.z_d = self.z_d   # Hold current z_d on first call
                self.z_d = (np.exp(-dt * self.batched_params.wn_d_z) * self.z_d
                            + (1 - np.exp(-dt * self.batched_params.wn_d_z)) * self.ref_z)

                # Step 2: PI on depth error → desired pitch angle θ_d
                theta_d = self.batched_params.Kp_z * (
                    (z - self.z_d) + (1.0 / self.batched_params.T_z) * self.z_int
                )
                # Saturate pitch command to prevent aggressive dive/climb
                if abs(theta_d) > self.batched_params.theta_max:
                    theta_d = torch.sign(theta_d) * self.batched_params.theta_max

                # Step 3: PID on pitch error → sternplane deflection δ_s
                delta_s = (- self.batched_params.Kp_theta * self.ssa(theta - theta_d)
                           - self.batched_params.Kd_theta * q
                           - self.batched_params.Ki_theta * self.theta_int
                           - self.batched_params.K_w      * w)

                # Euler integration of depth and pitch integral states
                self.z_int     += dt * (z - self.z_d)
                self.theta_int += dt * self.ssa(theta - theta_d)

                # ---- Heading autopilot (integral SMC with 3rd-order reference model) ----
                [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
                    self.integralSMC(
                        self.e_psi_int,
                        e_psi, e_r,
                        self.psi_d, self.r_d, self.a_d,
                        self.batched_params.T_nomoto,
                        self.batched_params.K_nomoto,
                        self.batched_params.wn_d,
                        self.batched_params.zeta_d,
                        self.batched_params.K_d,
                        self.batched_params.K_sigma,
                        self.batched_params.lam,
                        self.batched_params.phi_b,
                        psi_ref,
                        self.batched_params.r_max,
                        dt
                    )

                # Euler integration of heading error integral
                self.e_psi_int += dt * self.ssa(psi - self.psi_d)

                u_control = torch.hstack((delta_r, delta_s, n)).unsqueeze(0)   # (1, 3)

            else:
                # Vehicle stopped — zero all actuator commands
                u_control = torch.zeros(self.batch_size, 3, device=self.device).double()

            # ---- Expand symmetric fin pairs to full 5-channel command ----
            # Both rudder fins and both sternplane fins receive the same command
            delta_rt = delta_rb = u_control[:, 0]   # Rudder pair
            delta_sl = delta_sr = u_control[:, 1]   # Sternplane pair
            n                   = u_control[:, 2]   # Propeller RPM

            return torch.stack([delta_rt, delta_rb, delta_sl, delta_sr, n], dim=1)















































