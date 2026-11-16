import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.utils import Transform as tfrm
from biguasim.dynamics.utils import BatchedParams as bp


class Catamaran(VehicleModel):
    """
    Parameters, thrust allocation and legacy controllers of a catamaran with 2 differential thrusters.

    Has no plant of its own: BlueBoat combines it with dynamics.marine.MarinePlant (the Fossen model with voxel
    buoyancy against the water surface) and SurfaceVesselControl. The former plant here (a depth-ramped buoyancy
    "dumping_hack", halved while both thrusters run, z thresholds for the surface, sphere drag) was removed.

    Models a twin-hull surface vessel that operates at the water surface.
    Propulsion is provided by two bidirectional thrusters mounted along the
    surge axis; differential thrust between them produces yaw moments,
    making this a 2-DOF actuated vehicle (surge + yaw).

    The vehicle is constrained to the water surface — sway, heave, roll, and
    pitch are not actively controlled, though their dynamics are modeled via
    hydrodynamic drag and buoyancy restoring forces.

    Thruster layout (body frame):
        Thruster 0 (r1): port   — positive speed = forward thrust
        Thruster 1 (r2): starboard — positive speed = forward thrust
        Differential (T0 − T1) → yaw moment Mz

    Frame conventions (inherited from VehicleModel):
        - World frame : Z-up, right-handed.
        - Body frame  : X-forward, Y-left, Z-up.

    Control abstractions supported:
        - 'cmd_motor_speeds' : Direct thruster speed pass-through.
        - 'cmd_vel'          : Surge velocity tracking [vx, vy, vz]
                               (only vx is controlled; vy, vz are zeroed).
        - 'cmd_vel_yaw'      : Surge velocity + relative yaw offset [vx, _, _, yaw_deg].
        - 'cmd_pos_yaw'      : 2D position tracking; yaw is steered toward
                               the bearing to the goal [x, y, z, _].

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device string, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : One of the supported control abstraction keys above.
    """

    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:
        # Initialize shared VehicleModel infrastructure (BatchedParams, idxs, dt, etc.)
        super().__init__(batch_size, params, device, control_abstraction)

        self._control_abstraction = control_abstraction

        # Parse and store all vehicle-specific physics parameters and controllers
        self._build_params(params)


    # =========================================================================
    # Parameter Construction
    # =========================================================================

    def _build_params(self, params: dict):
        """
        Parses the parameter dictionary and populates self.batched_params with
        all geometry, hydrodynamic, allocation, and control gain tensors.

        Sections (in order):
            1. Geometry       — hull dimensions and displaced volume.
            2. Inertia        — body inertia tensor, added mass, buoyancy.
            3. Drag           — translational (Cp) and rotational (Cr) drag matrices.
            4. Allocation     — 6×2 wrench-to-thruster pseudo-inverse.
            5. Control gains  — surge velocity gain, yaw PD, position PD.

        Args:
            params (dict): Raw parameter dictionary from the vehicle config file.
        """

        # ---- 1. Geometry ----
        length = params['length']   # Hull length (m)
        width  = params['width']    # Hull width  (m)  — includes both hulls
        height = params['height']   # Hull height (m)  — draft + freeboard
        self.batched_params.height = height   # Stored for submersion threshold in _compute_external_forces

        vessel = width * length * height   # Bounding-box volume, used for rotational drag

        # ---- 2. Inertia / mass / buoyancy ----
        self.batched_params.I = torch.from_numpy(
            np.array([params['I'] for _ in range(self.batch_size)])
        ).double().to(self.device)   # Body inertia tensor (B, 3, 3)

        # Displaced volume modeled as a prolate ellipsoid: V = (4/3)π a b c
        # with semi-axes (2w, l, h/2) — gives a generous displaced volume for a
        # wide catamaran hull
        self.batched_params.volume = (4/3) * np.pi * (2 * width) * length * (height / 2)

        # Added mass = ρ × displaced volume (potential flow, full submersion)
        self.batched_params.added_mass = self.batched_params.volume * self.batched_params.rho

        # Static buoyancy force magnitude when fully submerged (N)
        self.batched_params.buoyancy = np.linalg.norm(
            self.batched_params.rho * self.batched_params.volume * self.batched_params.g
        )

        # Gravity acceleration vector in world frame (Z-up): [0, 0, g]
        # Shape (B, 3) for direct broadcast against force tensors
        self.batched_params.gravity        = torch.zeros(self.batch_size, 3, device=self.device).double()
        self.batched_params.gravity[:, -1] = self.batched_params.g

        # ---- 3. Drag coefficient matrices ----
        # Hoerner (1965), pg. 60: bluff body drag coefficient Cd = 0.38
        # Translational drag areas: projected areas of the hull on each body plane
        Cd  = 0.38
        Cpz = Cd * np.pi * width  * length    # Heave drag area  (top/bottom projection)
        Cpx = Cd * np.pi * width  * height    # Surge drag area  (front/rear projection)
        Cpy = Cd * np.pi * length * height    # Sway drag area   (side projection)

        # Rotational drag: scaled by bounding-box volume and arm length
        Crx = Cd * np.pi * width  * length    # Roll  drag
        Cry = Cd * np.pi * vessel             # Pitch drag
        Crz = Cry                             # Yaw   drag (symmetric)

        self.batched_params.Cp = torch.from_numpy(
            np.array([np.diag([Cpx, Cpy, Cpz]) for _ in range(self.batch_size)])
        ).double().to(self.device)   # (B, 3, 3)

        self.batched_params.Cr = torch.from_numpy(
            np.array([np.diag([Crx, Cry, Crz]) for _ in range(self.batch_size)])
        ).double().to(self.device)   # (B, 3, 3)

        # √2/2 projection factor for thrusters oriented at 45° (not used for this
        # vehicle's inline thrusters, but stored for allocation consistency)
        self.batched_params.sqrt = np.sqrt(2) / 2

        # ---- 4. Thruster allocation matrix ----
        # Rotor positions and directions replicated across the batch
        self.batched_params.rotor_pos    = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array(
            [params['rotor_directions'] for _ in range(self.batch_size)]
        )
        # Pre-compute rotor geometry and hat-maps for moment-arm calculations
        self.batched_params._extract_geometry()

        # Effective moment arm distance from CoM to thruster (scaled by 0.1 for unit consistency)
        x, y, z = params["rotor_pos"]['r1']
        self.batched_params.rotor_distance = np.sqrt(x**2 + y**2 + z**2) * 0.1

        alloc_mats = []
        for i in range(self.batch_size):
            # Thruster positions in body frame: (n_thrusters, 3)
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key]
                          for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(self.device)

            # Unit thrust direction vectors for each thruster: (n_thrusters, 3)
            thrust_dirs = torch.from_numpy(
                self.batched_params.rotor_dir_np[i]
            ).double().to(self.device)

            # Moment arms via cross product: r × F̂ → (3, n_thrusters)
            torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T

            # 6×n_thrusters allocation matrix A:
            #   Rows 0–2: force contributions [Fx, Fy, Fz]
            #   Rows 3–5: moment contributions [Mx, My, Mz]
            A = torch.vstack([
                thrust_dirs.T,   # Force block
                torque_arms,     # Moment block
            ])
            alloc_mats.append(A)

        # Pseudo-inverse of the stacked allocation matrix.
        # For a 2-thruster vessel, A is 6×2 (over-determined in wrench space),
        # so the pseudo-inverse gives the minimum-norm thrust solution.
        TM_to_f = torch.stack(alloc_mats).to(self.device)
        self.batched_params.TM_to_f = torch.linalg.pinv(TM_to_f)   # (B, 2, 6)

        # ---- 5. Control gains ----
        # Surge P-gain for velocity tracking
        self.batched_params.k_vx = torch.tensor(
            [params['k_vx'] for _ in range(self.batch_size)]
        ).unsqueeze(-1).to(self.device)   # (B, 1)

        # Yaw PD gains
        self.batched_params.kp_yaw = torch.from_numpy(
            np.array([params['kp_yaw'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)   # (B, 1)

        self.batched_params.kd_yaw = torch.from_numpy(
            np.array([params['kd_yaw'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)   # (B, 1)

        # Position outer-loop PD gains (used in cmd_pos_yaw)
        self.batched_params.kp_pos = torch.from_numpy(
            np.array([params['kp_pos'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)   # (B, 1)

        self.batched_params.kd_pos = torch.from_numpy(
            np.array([params['kd_pos'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)   # (B, 1)




    # =========================================================================
    # Allocation (shared by the controllers)
    # =========================================================================

    @staticmethod
    def _yaw(q: Tensor) -> Tensor:
        """Heading (rad) of quaternions [x, y, z, w] (B, 4): atan2(R10, R00), valid at any roll and pitch."""
        R = roma.unitquat_to_rotmat(q.double())
        return torch.atan2(R[:, 1, 0], R[:, 0, 0])

    def _body_velocity(self, state: dict) -> Tensor:
        """Velocity in the body frame (B, 3): the controllers track surge, the only actuated axis, so they compare
        the command with the body-frame velocity (the world-frame one turned into positive feedback once the
        heading moved away from the x axis)."""
        R = roma.unitquat_to_rotmat(state['q'][self.idxs].double())
        return (R.transpose(1, 2) @ state['v'][self.idxs].double().unsqueeze(-1)).squeeze(-1)

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

    def _steady_force(self, state: dict, v_body: Tensor) -> Tensor:
        """Body-frame force (B, 3) that holds the body-frame velocity v_body steadily against the plant (drag,
        damping, net weight). No plant here: zero. MarinePlant overrides it with its own model."""
        return torch.zeros_like(v_body, dtype=torch.float64)

    def _surge_feedforward(self, state: dict, v_des_body: Tensor) -> Tensor:
        """Surge-only feed-forward force (B, 3) for a body-frame surge setpoint: the thrusters act on surge only."""
        v = torch.zeros_like(v_des_body, dtype=torch.float64)
        v[:, 0] = v_des_body[:, 0]
        force = torch.zeros_like(v)
        force[:, 0] = self._steady_force(state, v)[:, 0]
        return force

    def _allocate_thrust(self, F_body: Tensor, M_des: Tensor) -> Tensor:
        """Desired force and moment (body frame) -> signed thruster speeds through the allocation TM_to_f.

        The moments are flipped: these controllers were tuned on the former plant, whose thruster yaw moment had the
        opposite sign of r x F (and was built from k_m w^2, ~9x weaker); the marine plant now produces r x F, the same
        geometry TM_to_f inverts.
        """
        TM = torch.cat([F_body.double(), -M_des.double()], dim=-1)   # (B, 6)
        thrust = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)
        # Bidirectional speed conversion: w = sign(F) sqrt(|F| / k_eta)
        speeds = torch.sign(thrust) * torch.sqrt(torch.abs(thrust) / self.batched_params.k_eta[self.idxs])
        return torch.nan_to_num(speeds, nan=0.0, posinf=0.0, neginf=0.0)

    # =========================================================================
    # Control → Motor Speeds
    # =========================================================================

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        """
        Converts a high-level control command into thruster speed commands
        using a cascaded P-control architecture for surge and yaw.

        All three non-passthrough abstractions share the same inner structure:
            1. Compute desired surge force F_des from a velocity error
               (optionally preceded by a position → velocity outer loop).
            2. Compute desired yaw moment M_yaw from a yaw angle PD law.
            3. Assemble wrench [F_des, M_des] and map through TM_to_f.
            4. Convert thruster forces to signed motor speeds:
               ω = sign(F) * √(|F| / k_eta).

        Surge-only constraint:
            This vehicle can only actuate surge (X) and yaw (Mz).
            Sway, heave, roll and pitch commands are zeroed before
            force computation to prevent spurious allocation outputs.

        cmd_pos_yaw bearing computation:
            The desired yaw is not taken from the goal vector but is
            computed as the geometric bearing to the goal position:
            ψ_des = atan2(y_goal − y, x_goal − x).
            This steers the vehicle toward the target rather than
            commanding an absolute world heading.

        Args:
            state (dict)  : Structured state dict {x, v, q, w} from VehicleModel.map_states.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).
                            D = 3 for cmd_vel, D = 4 for cmd_vel_yaw / cmd_pos_yaw.

        Returns:
            Tensor: Thruster speed commands (B, 2), signed rad/s.
        """

        # --- Pass-through: thruster speeds commanded directly ---
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]

        # =====================================================================
        # cmd_vel: Track desired surge velocity [vx, _, _]
        # Sway and heave goals are zeroed — only surge is actuated.
        # Yaw is held at current heading (zero yaw error setpoint).
        # =====================================================================
        if self._control_abstraction == 'cmd_vel':

            v = self._body_velocity(state)
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal        = control['cmd_ctrl'][self.idxs]
            goal[:, 1:] = 0   # Zero sway and heave — vehicle cannot actuate them

            # Surge velocity error → desired acceleration (P-control) → desired force
            v_err       = goal[self.idxs] - v
            a_des       = self.batched_params.k_vx[self.idxs] * v_err
            a_des[:, 1:] = 0   # Enforce surge-only constraint before force computation
            F_des       = self.batched_params.mass * a_des[self.idxs]   # (B, 3)
            # Feed-forward: surge force that holds the setpoint against the plant's drag
            F_des = F_des + self._surge_feedforward(state, goal[:, :3])

            # ---- Yaw hold: heading of the first command (see _heading_setpoint) ----
            yaw     = self._yaw(q)
            yaw_des = self._heading_setpoint(yaw)   # heading held since the first command
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # Assemble wrench: only surge force and yaw moment are non-zero
            M_des       = torch.zeros_like(F_des)
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des)

        # =====================================================================
        # cmd_vel_yaw: Track surge velocity + relative yaw offset [vx, _, _, yaw_deg]
        # Identical to cmd_vel but yaw setpoint is offset by the commanded delta.
        # =====================================================================
        if self._control_abstraction == 'cmd_vel_yaw':

            v = self._body_velocity(state)
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal       = control['cmd_ctrl'][self.idxs]
            goal[:, 1] = goal[:, 2] = 0   # Zero sway and heave inputs

            # Surge velocity error → force (P-control, surge-only)
            v_err        = goal[:, :3] - v
            a_des        = self.batched_params.k_vx[self.idxs] * v_err
            a_des[:, 1:] = 0
            F_des        = self.batched_params.mass * a_des[self.idxs]
            # Feed-forward: surge force that holds the setpoint against the plant's drag
            F_des = F_des + self._surge_feedforward(state, goal[:, :3])

            # ---- Yaw tracking: current heading + commanded relative offset ----
            yaw       = self._yaw(q)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des   = self._heading_setpoint(yaw, yaw_delta)
            yaw_err   = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            M_des       = torch.zeros_like(F_des)
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des)

        # =====================================================================
        # cmd_pos_yaw: Steer toward a 2D goal position [x_goal, y_goal, _, _]
        # Outer loop: Euclidean distance to goal → desired surge speed.
        # Yaw: bearing to goal (atan2) replaces the goal yaw field entirely.
        # =====================================================================
        if self._control_abstraction == 'cmd_pos_yaw':

            x = state['x'][self.idxs]
            v = self._body_velocity(state)
            w = state['w'][self.idxs]
            q = state['q'][self.idxs]

            goal = control['cmd_ctrl'][self.idxs]   # [x_goal, y_goal, z_goal, _]

            # ---- Position outer loop ----
            # pos_err[:, 0] = Euclidean distance to goal in XY plane
            # pos_err[:, 1:] = 0 (no lateral or vertical control)
            pos_err       = torch.zeros(self.batch_size, 3, device=self.device).double()
            pos_err[:, 0] = torch.sqrt(
                (goal[:, 0] - x[:, 0])**2 + (goal[:, 1] - x[:, 1])**2
            )

            # Desired yaw = bearing from current position to goal (geometric, not from goal[3])
            yaw_delta = torch.atan2(goal[:, 1] - x[:, 1], goal[:, 0] - x[:, 0])

            # Desired surge speed: P-control on distance, limited below
            v_des       = self.batched_params.kp_pos[self.idxs] * pos_err
            # Surge speed limited to what the thrusters can hold against the drag (was a fixed -2.5..3 m/s: full thrust
            # leaves nothing for steering)
            limit = self._speed_limit(state, [[1, 0, 0]], 3.0)
            v_des[:, 0] = torch.maximum(torch.minimum(v_des[:, 0], limit), -limit)

            # ---- Velocity inner loop → desired force ----
            v_err = v_des - v
            a_des = self.batched_params.k_vx[self.idxs] * v_err
            F_des = self.batched_params.mass * a_des[self.idxs]
            # Feed-forward: surge force that holds the setpoint against the plant's drag
            F_des = F_des + self._surge_feedforward(state, v_des)

            # ---- Yaw tracking: steer toward goal bearing ----
            yaw     = self._yaw(q)
            yaw_des = (yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate     = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err  = yaw_rate_des - yaw_rate
            M_yaw        = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            M_des       = torch.zeros_like(F_des)
            M_des[:, 2] = M_yaw

            return self._allocate_thrust(F_des, M_des)
        


# class Differential(VehicleModel):
#     def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
#         super().__init__(batch_size, params, device, control_abstraction)   
#         self._control_abstraction = control_abstraction
#         self._build_params(params)

#         self.surface_mask = torch.zeros(self.batch_size, 1, device=self.device).double()

#     def _build_params(self, params):
        
#         self.batched_params.L = 1.195    # length (m)
#         self.batched_params.B = .19   # beam (m)

#         self.batched_params.V_c = 0
#         self.batched_params.beta_c = 0 * (np.pi / 180 ) #TODO set spawn angle from config file (deg) instead zero.

#         self.batched_params.r_bg = torch.from_numpy(np.array([params['r_bg'] 
#                                                              for _ in range(self.batch_size)])).to(self.device)
        
#         self.batched_params.S_rg = bp.hat_map(self.batched_params.r_bg).permute(2, 0, 1)
#         self.batched_params.H_rg = bp.Hmtrx(self.batched_params.r_bg)
#         self.batched_params.zeta_d = params['zeta_d']  # desired relative damping ratio

#         R44 = 0.2 * self.batched_params.B  # radii of gyration (m)
#         R55 = 0.25 * self.batched_params.L
#         R66 = 0.25 * self.batched_params.L

#         T_sway = params['T_sway']      # time constant in sway (s)
#         T_yaw = params['T_yaw']      # time constant in yaw (s)
#         Umax = self.batched_params.rotor_speed_max   # max forward speed (m/s)

#         # Data for one pontoon
#         self.batched_params.B_pont = 0.299  # beam of one pontoon (m)
#         y_pont = params['y_pont']      # distance from centerline to waterline centroid (m)
#         Cw_pont = params['Cw_pont']      # waterline area coefficient (-)
#         Cb_pont = params['Cb_pont']       # block coefficient

#         # Inertia dyadic, volume displacement and draft
#         nabla = self.batched_params.mass / self.batched_params.rho  # volume
#         self.batched_params.T = nabla / (2 * Cb_pont * self.batched_params.B_pont * self.batched_params.L)  # draft
#         Ig_CG = self.batched_params.mass * torch.from_numpy(np.array([np.diag(np.array([R44 ** 2, R55 ** 2, R66 ** 2])) 
#                                                                       for _ in range(self.batch_size)])).to(self.device)
#         self.batched_params.Ig = Ig_CG - self.batched_params.mass * self.batched_params.S_rg @ self.batched_params.S_rg

#         # Experimental propeller data including lever arms
#         self.batched_params.l1 = -y_pont  # lever arm, left propeller (m)
#         self.batched_params.l2 = y_pont  # lever arm, right propeller (m)
#         self.batched_params.k_pos = 0.02216 / 2  # Positive Bollard, one propeller
#         self.batched_params.k_neg = 0.01289 / 2  # Negative Bollard, one propeller
#         self.batched_params.n_max = np.sqrt((0.5 * 24.4 * self.batched_params.g) / self.batched_params.k_pos)  # max. prop. rev.
#         self.batched_params.n_min = -np.sqrt((0.5 * 13.6 * self.batched_params.g) / self.batched_params.k_neg) # min. prop. rev.

#         # MRB_CG = [ (m+mp) * I3  O3      (Fossen 2021, Chapter 3)
#         #               O3       Ig ]
#         MRB_CG = torch.zeros((self.batch_size, 6, 6)).double().to(self.device)
#         MRB_CG[:, 0:3, 0:3] = self.batched_params.mass * torch.eye(3)
#         MRB_CG[:, 3:6, 3:6] = self.batched_params.Ig
#         MRB = self.batched_params.H_rg.mT @ MRB_CG @ self.batched_params.H_rg

#         # Hydrodynamic added mass (best practice)
#         Xudot = -0.1 * torch.from_numpy(np.array([ self.batched_params.mass 
#                                                  for _ in range(self.batch_size)])).to(self.device)
#         Yvdot = -1.5 * torch.from_numpy(np.array([ self.batched_params.mass 
#                                                  for _ in range(self.batch_size)])).to(self.device)
#         Zwdot = -1.0 * torch.from_numpy(np.array([ self.batched_params.mass 
#                                                  for _ in range(self.batch_size)])).to(self.device)
#         Kpdot = -0.2 * self.batched_params.Ig[:, 0, 0]
#         Mqdot = -0.8 * self.batched_params.Ig[:, 1, 1]
#         Nrdot = -1.7 * self.batched_params.Ig[:, 2, 2]

#         self.batched_params.MA = -torch.diag_embed(torch.stack([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot], dim=1))

#         # System mass matrix
#         self.batched_params.M = MRB + self.batched_params.MA
#         self.batched_params.Minv = torch.linalg.inv(self.batched_params.M)

#         # Hydrostatic quantities (Fossen 2021, Chapter 4)
#         Aw_pont = Cw_pont * self.batched_params.L * self.batched_params.B_pont  # waterline area, one pontoon
#         I_T = (
#             2
#             * (1 / 12)
#             * self.batched_params.L
#             * self.batched_params.B_pont ** 3
#             * (6 * Cw_pont ** 3 / ((1 + Cw_pont) * (1 + 2 * Cw_pont)))
#             + 2 * Aw_pont * y_pont ** 2
#         )
#         I_L = 0.8 * 2 * (1 / 12) * self.batched_params.B_pont * self.batched_params.L ** 3
#         KB = (1 / 3) * (5 * self.batched_params.T / 2 - 0.5 * nabla / (self.batched_params.L * self.batched_params.B_pont))
#         BM_T = I_T / nabla  # BM values
#         BM_L = I_L / nabla
#         KM_T = KB + BM_T    # KM values
#         KM_L = KB + BM_L
#         KG = self.batched_params.T - self.batched_params.r_bg[:, 2]
#         GM_T = KM_T - KG    # GM values
#         GM_L = KM_L - KG

#         G33 =  torch.from_numpy(np.array([ self.batched_params.mass 
#                                                  for _ in range(self.batch_size)])).to(self.device)
#         G44 = self.batched_params.rho * self.batched_params.g * nabla * GM_T
#         G55 = self.batched_params.rho * self.batched_params.g * nabla * GM_L
#         G_CF =torch.diag_embed(torch.stack([torch.zeros_like(G55), torch.zeros_like(G55), G33, 
#                                       G44, G55, torch.zeros_like(G55)], dim=1))

#         LCF = -0.2
#         H = bp.Hmtrx(torch.from_numpy(np.array([np.array([LCF, 0.0, 0.0])
#                                                                 for _ in range(self.batch_size)])).to(self.device))  # transform G_CF from CF to CO
#         self.batched_params.G = H.mT @ G_CF @ H

#         # Natural frequencies
#         w3 = torch.sqrt(G33 / self.batched_params.M[:, 2, 2])
#         w4 = torch.sqrt(G44 / self.batched_params.M[:, 3, 3])
#         w5 = torch.sqrt(G55 / self.batched_params.M[:, 4, 4])

#         # Linear damping terms (hydrodynamic derivatives)
#         Xu = -24.4 * self.batched_params.g / Umax.squeeze(-1)   # specified using the maximum speed
#         Yv = -self.batched_params.M[:,1, 1]  / T_sway # specified using the time constant in sway
#         Zw = -2 * 0.03 * w3 * self.batched_params.M[:, 2, 2]  # specified using relative damping
#         Kp = -2 * 0.02 * w4 * self.batched_params.M[:, 3, 3]
#         Mq = -2 * 0.04 * w5 * self.batched_params.M[:, 4, 4]
#         Nr = -self.batched_params.M[:, 5, 5] / T_yaw  # specified by the time constant T_yaw

#         self.batched_params.D = -torch.diag_embed(torch.stack([Xu, Yv, Zw, Kp, Mq, Nr], dim=1))

#         # Propeller configuration/input matrix
#         # B = self.batched_params.k_pos * torch.from_numpy(np.array([np.array([[1, 1], [-self.batched_params.l1, -self.batched_params.l2]])
#         #                                                            for _ in range(self.batch_size) ])).double().to(self.device)
#         # self.batched_params.TM_to_f = torch.linalg.inv(B)

#         #--------------------- Control
#         self.batched_params.rotor_pos = [params["rotor_pos"] for _ in range(self.batch_size)]
#         self.batched_params.rotor_dir_np = np.array([params['rotor_directions'] for _ in range(self.batch_size)]) #air
#         self.batched_params._extract_geometry()

#         x,y,z = params["rotor_pos"]['r1']
#         self.batched_params.rotor_distance = np.sqrt(x**2 + y**2 + z**2) *.1

#         alloc_mats = []
#         for i in range(self.batch_size):
#             # Approximate BlueROV2 thruster positions [m] (body frame)
#             pos = torch.tensor(
#                 np.stack([self.batched_params.rotor_pos[i][key] for key in self.batched_params.rotor_pos[i]], axis=0),
#                 dtype=torch.float64
#             )  # shape (num_rotors, 3)
#             # Unit thrust directions for each thruster
#             thrust_dirs = torch.from_numpy(self.batched_params.rotor_dir_np[i]).double().to(self.device)

#             # Compute moment arms (r × F)
#             torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T  # (3,2)

#             # Build full 6x2 allocation matrix
#             A = torch.vstack([
#                 thrust_dirs.T,   # Fx,Fy,Fz rows
#                 torque_arms,     # Mx,My,Mz rows
#             ])

#             alloc_mats.append(A)

#         # Use pseudo-inverse for generality
#         TM_to_f = torch.stack(alloc_mats).to(self.device)
#         self.batched_params.TM_to_f = torch.linalg.pinv(TM_to_f)

#         # Low-Level Control Gains
#         self.batched_params.k_vx = torch.tensor([params['k_vx'] 
#                                                 for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        
#         self.batched_params.kp_yaw = torch.from_numpy(np.array([params['kp_yaw'] 
#                                                                 for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
#         self.batched_params.kd_yaw = torch.from_numpy(np.array([params['kd_yaw'] 
#                                                                 for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
    

    
#     def _compute_external_forces(self, s : Tensor, rpy : Tensor):
#         nu = torch.cat([s['v'][self.idxs], s['w'][self.idxs]], dim=1).double().to(self.device)
#         eta = torch.cat([s['x'][self.idxs], rpy], dim=1).double().to(self.device)


#         self.surface_mask = torch.ones(self.batch_size, 1, device=self.device).double()
#         z = s['x'][:,2].unsqueeze(-1)
#         self.surface_mask[s['x'][:,2] > 0] = 0

#         # Current velocities
#         u_c = (self.batched_params.V_c * 
#                torch.cos(self.batched_params.beta_c - rpy[..., 1])).unsqueeze(-1)  # current surge velocity
#         v_c = (self.batched_params.V_c * 
#                torch.sin(self.batched_params.beta_c- rpy[..., 1])).unsqueeze(-1)   # current sway velocity

#         nu_c = torch.cat([u_c, v_c, 
#                           torch.zeros((self.batch_size,4))], dim=1).double().to(self.device) # current velocity 
#         nu_r = nu - nu_c                               # relative velocity    
#         Dnu_c = torch.cat([nu[:, 5:6] * v_c, -nu[:, 5:6] * u_c, 
#                            torch.zeros((self.batch_size,4))], dim=1).double().to(self.device) # current velocity 

#         # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
#         top= torch.cat([self.batched_params.mass * self.batched_params.hat_map(nu[:, 3:6]).permute(2, 0, 1), 
#                        torch.zeros(self.batch_size, 3, 3, dtype=torch.float64)], dim=2)
#         bottom = torch.cat([torch.zeros(self.batch_size, 3, 3, dtype=torch.float64), 
#                        self.batched_params.hat_map( torch.einsum('bij,bj->bi',self.batched_params.Ig, 
#                                                                  nu[:, 3:6])).permute(2, 0, 1)], dim=2)
        
#         CRB_CG = torch.cat([top, bottom], dim=1)
#         CRB = self.batched_params.H_rg.mT @ CRB_CG @ self.batched_params.H_rg
#         CA  = bp.m2c(self.batched_params.MA, nu_r)
        
#         C = CRB + CA

#         tau_damp = torch.einsum('bij,bj->bi',self.batched_params.D, nu_r) 
#         tau_damp[:, 5] = tau_damp[:, 5] - 10 * self.batched_params.D[:, 5, 5] * torch.abs(nu_r[:, 5]) * nu_r[:, 5]
#         tau_crossflow = bp.cross_flow_drag(self.batched_params.rho, self.batched_params.L, self.batched_params.B_pont, nu_r, self.batched_params.T, self.batched_params.rho)
#         tau = (
#                tau_damp 
#                + tau_crossflow 
#                - torch.einsum('bij,bj->bi',C, nu_r)  
#                - torch.einsum('bij,bj->bi',self.batched_params.G, eta) 
#                ) * self.surface_mask
        
#         # Hack damp
#         # tau *= 0.1
#         tau[:, 0] *= 0
#         return (tau, Dnu_c) 
    
#     def _compute_body_wrench(self, rotor_speeds : Tensor):
#         """
#         Computes the wrench acting on the rigid body based on the rotor speeds for thrust and airspeed
#         for aerodynamic forces.
#         The airspeed is represented in the body frame.
#         The net force Ftot is represented in the body frame.
#         The net moment Mtot is represented in the body frame.
#         """

#         # thrust magnitudes per rotor
#         T = torch.zeros(self.batch_size, 3, rotor_speeds.shape[-1], 
#                         device=self.device).double().to(self.device)
#         T[:, 0, :] = self.batched_params.k_eta[self.idxs] * rotor_speeds **2 *torch.sign(rotor_speeds) #polinomy to get thrust
#         FtotB = torch.sum(T, dim=2)

#         # ----- Moments (mt) -----
#         l1 = self.batched_params.l1       
#         l2 = self.batched_params.l2   
#         M = self.batched_params.k_m[self.idxs] * rotor_speeds**2 *torch.sign(rotor_speeds) # (B,4)

#         mt_x = torch.zeros((self.batch_size, 1))               # roll
#         mt_y = torch.zeros((self.batch_size, 1))               # pitch
#         # mt_z = -l1 * M[:, 0] +l2 * M[:, 1] # yaw
#         mt_z =  -(-M[:, 0] + M[:, 1]) # yaw
#         MtotB = torch.stack([mt_x, mt_y, mt_z.unsqueeze(-1)], dim=1)  # (B,3)


#         Fx = FtotB[:, 0].unsqueeze(-1)
#         tau = torch.stack([
#             Fx,
#             torch.zeros_like(Fx),
#             torch.zeros_like(Fx),
#             torch.zeros_like(Fx),
#             torch.zeros_like(Fx),
#             MtotB[:, 2],
#         ], dim=1)

#         return tau
    
    
#     def _s_dot_fn(self, s, cmd_rotor_speeds : Tensor):
#         state = self.unpack_state(s, self.idxs, self.batch_size)

#         R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()
#         rpy = roma.rotmat_to_euler('xyz', R)

#         x_dot = state['v'][self.idxs]
#         q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])

#         F, Dnu_c = self._compute_external_forces(state, rpy)
#         F = F.unsqueeze(-1)

#         FMtotB = self._compute_body_wrench(cmd_rotor_speeds)
#         FMtotB[:, 1] = FMtotB[:, 0] * torch.cos(rpy[..., -1]).unsqueeze(-1)

#         # FMtotB[:, :3] = R @ FMtotB[:, :3]
#         # FMtotB[:, 1] = FMtotB[:, 0] * torch.cos(rpy[...,-1]).unsqueeze(-1)

#         Ftot = FMtotB
#         # self.batched_params.Minv[:, :3, :3] = self.batched_params.Minv[:, :3, :3]
#         nu_dot = Dnu_c + (self.batched_params.Minv[self.idxs] @ Ftot[self.idxs]).squeeze(-1)

#         v_dot = nu_dot[:, :3]
#         w_dot = nu_dot[:, 3:]

#         # Pack into vector of derivatives.
#         s_dot = torch.zeros((len(self.idxs), 13 + cmd_rotor_speeds.shape[1],), device=self.device)
#         s_dot[:, 0:3] = x_dot
#         s_dot[:, 3:6] = v_dot
#         s_dot[:, 6:10] = q_dot
#         s_dot[:, 10:13] = w_dot.squeeze(-1)

#         return s_dot
        
    
#     def get_cmd_motor_speeds(self, state : dict,  control : dict):  #control needs to be a tensor
#         if self._control_abstraction == 'cmd_motor_speeds':
#             return control['cmd_ctrl'][self.idxs]
    
#         if self._control_abstraction == 'cmd_vel':
#             # --- Extract state ---
#             v = state['v'][self.idxs]          # linear velocity (B,3)
#             w = state['w'][self.idxs]          # angular velocity (B,3)
#             q = state['q'][self.idxs]          # quaternion orientation (B,4)

#             # --- Control input ---
#             goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz]
#             goal[:, 1:] = 0 # only vx is available

#             # --- Velocity control (P controller on velocity error) ---
#             v_err = goal[self.idxs] - v
#             a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
#             F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


#             # --- Yaw stabilization ---
#             yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
#             yaw_des = (yaw + np.pi) % (2 * np.pi) - np.pi
#             yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

#             yaw_rate = w[:, 2]
#             yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
#             yaw_dot_err = yaw_rate_des - yaw_rate
#             M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

#             # --- Combine moments (no pitch for 5-DoF) ---
#             M_des = torch.zeros_like(F_des)
#             M_des[:, 2] = M_yaw

#             # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
#             TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

#             # --- Map to thruster forces ---
#             cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

#             # --- Convert to motor speeds (bidirectional) ---
#             cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
#             cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

#             # Safety: remove NaNs / infs
#             cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

#             return cmd_motor_speeds
        
#         if self._control_abstraction == 'cmd_vel_yaw':
#             # --- Extract state ---
#             v = state['v'][self.idxs]          # linear velocity (B,3)
#             w = state['w'][self.idxs]          # angular velocity (B,3)
#             q = state['q'][self.idxs]          # quaternion orientation (B,4)

#             # --- Control input ---
#             goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw]
#             goal[:, 1:3] = 0 # only vx is available

#             # --- Velocity control (P controller on velocity error) ---
#             v_err = goal[:, :3] - v
#             a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
#             F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


#             # --- Yaw stabilization ---
#             yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
#             yaw_delta = torch.deg2rad(goal[:, 3].double())
#             yaw_des = (yaw + yaw_delta + np.pi) % (2 * np.pi) - np.pi
#             yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

#             yaw_rate = w[:, 2]
#             yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
#             yaw_dot_err = yaw_rate_des - yaw_rate
#             M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

#             # --- Combine moments (no pitch for 5-DoF) ---
#             M_des = torch.zeros_like(F_des)
#             M_des[:, 2] = M_yaw

#             # # # --- Apply yaw-aligned rotation to F_des ---
#             # R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
#             # F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)


#             # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
#             TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

#             # --- Map to thruster forces ---
#             cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

#             # --- Convert to motor speeds (bidirectional) ---
#             cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
#             cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

#             # Safety: remove NaNs / infs
#             cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

#             return cmd_motor_speeds

#     # @map_states
#     # def step(self, state : dict, control : dict):
#     #     """
#     #     Integrate dynamics forward from state given constant control for time t_step.
#     #     Params:
#     #         - state: dictionary containing keys ['x', 'v', 'q', 'w', rotor_speeds'], and values which
#     #           are pytorch tensors with dtype double and which have a batch dimension
#     #         - control: dictionary with keys depending on the chosen control mode. Values are torch 
#     #         tensors, again with dtype double and with a batch dimension equal to the number of agents.
#     #     """
#     #     cmd_rotor_speeds = self.get_cmd_motor_speeds(control)
#     #     cmd_rotor_speeds = torch.clip(
#     #         cmd_rotor_speeds,
#     #         self.batched_params.rotor_speed_min[self.idxs],
#     #         self.batched_params.rotor_speed_max[self.idxs]
#     #     )

#     #     s = self.pack_state(state, self.batch_size, self.device)
#     #     s_dot = self._s_dot_fn(s, cmd_rotor_speeds)
#     #     v_dot = torch.zeros_like(state["v"])
#     #     w_dot = torch.zeros_like(state["w"])
#     #     v_dot[self.idxs] = s_dot[..., 3:6].double()
#     #     w_dot[self.idxs] = s_dot[..., 10:13].double()

#     #     state = self.unpack_state(s, self.idxs, self.batch_size)
#     #     state['q'][self.idxs] = state['q'][self.idxs] / \
#     #                             torch.norm(state['q'][self.idxs], dim=-1).unsqueeze(-1)

#     #     return Transform.convert_NED_to_NWU(state['q'][self.idxs], v_dot, w_dot).cpu().tolist()    
#     #     return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()