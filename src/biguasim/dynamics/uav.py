import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.control import BatchedPIDController
from biguasim.dynamics.geometry import analyze_mesh, drag_panels, panel_drag
from biguasim.dynamics.utils import Transform as tfrm



class QuadCopterX(VehicleModel):
    """
    Concrete vehicle model for a standard X-frame quadrotor operating in air only.

    Extends VehicleModel with a full rigid-body dynamics pipeline and a
    cascaded SE(3) control architecture. Operates in a single domain (air)
    with a square 4×4 allocation matrix mapping [thrust, Mx, My, Mz] to
    4 rotor speeds exactly.

    Control abstractions supported:
        - 'cmd_motor_speeds' : Direct rotor speed pass-through.
        - 'cmd_vel'          : 3D velocity tracking using SE(3) attitude control.
        - 'cmd_vel_yaw'      : 3D velocity + relative yaw offset [vx, vy, vz, yaw_deg].
        - 'cmd_pos_yaw'      : 3D position + absolute yaw     [x,  y,  z,  yaw_deg].

    Control architecture (cmd_vel / cmd_vel_yaw / cmd_pos_yaw):
        Position PD  →  Velocity P  →  Desired force F_des
        F_des  →  Desired orientation R_des  (SE(3) geometric controller)
        R_des  →  Attitude error  →  Desired moment M_des
        [T, M_des]  →  TM_to_f (allocation inverse)  →  Rotor thrusts  →  Motor speeds

    Frame conventions (inherited from VehicleModel):
        - World frame : Z-up, right-handed.
        - Body frame  : X-forward, Y-left, Z-up.

    Args:
        batch_size (int)          : Number of vehicles simulated in parallel.
        params (dict)             : Vehicle parameter dictionary.
        device (str)              : Torch device string, e.g. 'cpu' or 'cuda:0'.
        control_abstraction (str) : One of the supported control abstraction keys above.
    """

    # The engine reports ω and applies ω̇ in the world frame (see VehicleModel); without the
    # conversion, a Matrice not at yaw 0 damps roll/pitch on the wrong axes and flies off course.
    angular_velocity_in_world_frame = True

    # Panel exposure is blended over the table directions within this angle of the flow (about twice
    # the ~13° spacing of geometry.DRAG_DIRECTION_COUNT directions, so every flow has neighbours).
    EXPOSURE_BLEND_RAD = np.radians(25)

    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:
        # Initialize shared infrastructure (BatchedParams, idxs, dt, etc.)
        super().__init__(batch_size, params, device, control_abstraction)

        # Populate all vehicle-specific physics parameters and controllers
        self._build_params(params)
        self._params_for_geometry = params

        # Velocity of the air in the world frame (B, 3), i.e. the wind; drag acts on the
        # velocity relative to it. Set it on the model: model.fluid_velocity[:] = ...
        self.fluid_velocity = torch.zeros(batch_size, 3, device=device).double()

        # Drag panels from the collision mesh (set_geometry); None keeps the primitive drag.
        self._drag_panels = None

        # cmd_pos_yaw: integral gain, its anti-windup limits and the flight envelope.
        self._pos_control = {
            'ki':                   float(params.get('ki_pos', 0.0)),
            'integral_accel_limit': float(params.get('pos_integral_accel_limit', 0.0)),
            'integral_radius':      float(params.get('pos_integral_radius', np.inf)),
            'max_tilt':             np.radians(params.get('max_tilt_deg', 89.0)),
            'max_vertical_accel':   float(params.get('max_vertical_accel', np.inf)),
        }
        self.position_integral = torch.zeros(batch_size, 3, device=device).double()  # (B, 3), m·s

        # Optional high-level PID controllers — reserved for future outer-loop experiments,
        # not consumed by the current SE(3) inner loop
        self.linear_pid  = BatchedPIDController(Kp=10, Ki=1, Kd=0.1,
                                                batch_shape=(batch_size, 3), device=device)
        self.angular_pid = BatchedPIDController(Kp=10, Ki=1, Kd=0.1,
                                                batch_shape=(batch_size, 3), device=device)

    # =========================================================================
    # Parameter Construction
    # =========================================================================

    def _build_params(self, params: dict):
        """
        Parses the parameter dictionary and populates self.batched_params with
        all geometry, inertia, drag, allocation, and control gain tensors.

        Sections (in order):
            1. Geometry      — rotor positions, spin directions, hat-map cache.
            2. Drag          — 3×3 diagonal aerodynamic drag coefficient matrix.
            3. Inertia       — body inertia tensor and its pre-computed inverse.
            4. Allocation    — 4×4 thrust-moment → rotor-force matrix (exact inverse).
            5. Control gains — attitude (roll/pitch, yaw) and position PD gains.

        Args:
            params (dict): Raw parameter dictionary from the vehicle config file.
        """

        # ---- 1. Geometry ----
        self.batched_params.num_rotors = len(params["rotor_pos"].keys())

        # Rotor positions and spin directions replicated across the batch
        self.batched_params.rotor_pos    = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array(
            [params['rotor_directions'] for _ in range(self.batch_size)]
        )
        # Pre-compute rotor position vectors and their skew-symmetric hat-maps
        # (stored in batched_params.rotor_geometry and rotor_geometry_hat_maps)
        self.batched_params._extract_geometry()

        # Spin direction tensor (+1 CW / -1 CCW) — used in yaw torque row of allocation
        self.batched_params.rotor_dir = torch.from_numpy(
            self.batched_params.rotor_dir_np
        ).to(self.device)  # (B, n_rotors)

        # ---- 2. Aerodynamic drag ----
        # Diagonal drag matrix C_D ∈ R^{3×3} in the body frame.
        # Drag force: D = -|v_body| * C_D * v_body  (quadratic, velocity-aligned)
        self.batched_params.drag_matrix = torch.tensor(
            [[[params["c_Dx"], 0,             0            ],
              [0,              params["c_Dy"], 0            ],
              [0,              0,             params["c_Dz"]]]
             for _ in range(self.batch_size)],
            device=self.device
        ).double()  # (B, 3, 3)

        # ---- 3. Inertia ----
        self.batched_params.I = torch.from_numpy(
            np.array([params['I'] for _ in range(self.batch_size)])
        ).double().to(self.device)  # Body inertia tensor (B, 3, 3)

        # Pre-invert I once to avoid a matrix solve at every integration step.
        # Valid under the rigid-body assumption (I is constant in the body frame).
        self.batched_params.invI = torch.linalg.inv(self.batched_params.I).double()  # (B, 3, 3)

        # ---- 4. Allocation matrix ----
        # Velocity gain for the P-controller used in cmd_vel abstraction
        self.batched_params.k_v = torch.tensor(
            [params['k_v'] for _ in range(self.batch_size)]
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        # Torque-to-thrust ratio κ = k_m / k_eta.
        # Populates the yaw row of the allocation matrix:
        #   M_z_i = κ * dir_i * F_i   (reaction torque from each rotor)
        k = self.batched_params.k_m / self.batched_params.k_eta  # (B,) scalar ratio

        alloc_mats = []
        for i in range(self.batch_size):
            # Rotor positions in body frame: (n_rotors, 3)
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key]
                          for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(self.device)

            # Moment arms for roll and pitch: r × ẑ gives the XY components
            # of the moment produced by a unit upward thrust at each rotor location
            z_axis   = torch.tensor([0., 0., 1.], dtype=torch.float64).to(self.device)
            cross    = torch.cross(pos, z_axis.expand_as(pos)).to(self.device)
            cross_xy = cross[:, :2].T  # (2, n_rotors) — roll and pitch rows

            # Assemble 4×n_rotors allocation matrix A:
            #   Row 0: thrust     — all rotors contribute equally (+1)
            #   Row 1: roll  Mx   — cross_xy[0]
            #   Row 2: pitch My   — cross_xy[1]
            #   Row 3: yaw   Mz   — κ * dir_i (reaction torque, alternating sign CW/CCW)
            mat = torch.vstack([
                torch.ones((1, len(self.batched_params.rotor_pos[i])),
                           dtype=torch.float64).to(self.device),
                cross_xy,
                (k[i] * self.batched_params.rotor_dir[i]).reshape(1, -1)
            ]).to(self.device)  # (4, n_rotors)
            alloc_mats.append(mat)

        # Exact matrix inverse: the X-frame quadrotor is fully actuated in
        # [T, Mx, My, Mz], so the square 4×4 system has a unique solution.
        self.batched_params.TM_to_f = torch.linalg.inv(
            torch.stack(alloc_mats).to(self.device)
        )  # (B, 4, 4)

        # ---- 5. Control gains ----
        # Attitude inner-loop PD gains (roll & pitch)
        self.batched_params.kp_att = torch.from_numpy(
            np.array([params['kp_att'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        self.batched_params.kd_att = torch.from_numpy(
            np.array([params['kd_att'] for _ in range(self.batch_size)])
        ).unsqueeze(-1).to(self.device)  # (B, 1)

        # Yaw inner-loop PD gains — tuned independently from roll/pitch
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
    # Collision Geometry and External Forces (aerodynamic drag only)
    # =========================================================================

    def set_geometry(self, geometries: list[dict]) -> None:
        """
        Replaces the primitive drag with drag on the panels of the collision mesh
        exported by each robot's DynamicsSensor (use GeometrySource "simple", the
        convex hulls). See _compute_external_forces for the drag model.

        Only drag changes: rotor positions, mass and inertia stay the parameters, so
        the controllers see the same model.

        Args:
            geometries (list[dict]): One DynamicsSensor.geometry dict per robot in the batch.

        Raises:
            ValueError: If the number of geometries does not match the batch size.
        """
        if len(geometries) != self.batch_size:
            raise ValueError(f"Expected {self.batch_size} geometries, one per robot, got {len(geometries)}")
        params = self._params_for_geometry

        panels, arms = [], []
        for geometry in geometries:
            panel_set = drag_panels(geometry['vertices'], geometry['triangles'], device=self.device)
            # Lever arms from the point the engine reports the state at (its CoM); the
            # solid's centroid if the engine does not simulate the body (mass 0).
            com = (geometry['center_of_mass'] if geometry['mass'] > 0
                   else analyze_mesh(geometry['vertices'], geometry['triangles']).centroid)
            panels.append(panel_set)
            arms.append(panel_set.centroids - com)

        # Robots of one model share a mesh in practice; pad with zero-area panels otherwise.
        panel_count = max(len(p.areas) for p in panels)

        def stacked(values):
            padded = [np.concatenate([v, np.zeros((panel_count - len(v),) + v.shape[1:])]) for v in values]
            return torch.from_numpy(np.stack(padded)).double().to(self.device)

        self._drag_panels = {
            'areas':        stacked([p.areas for p in panels]),      # (B, F)
            'normals':      stacked([p.normals for p in panels]),    # (B, F, 3)
            'arms':         stacked(arms),                           # (B, F, 3) from the CoM
            'exposure':     stacked([p.exposure for p in panels]),   # (B, F, K)
            'directions':   torch.from_numpy(panels[0].directions).double().to(self.device),  # (K, 3)
            'coefficients': torch.tensor([params["C_Dx"], params["C_Dy"], params["C_Dz"]],
                                         dtype=torch.float64, device=self.device),
            'with_moments': bool(params.get("drag_moments", False)),
        }

    def _compute_external_forces(self, R: Tensor, inertial_velocity: Tensor, w: Tensor) -> tuple:
        """
        Computes the aerodynamic drag force and moment in the body frame, from the
        velocity relative to the air: inertial velocity minus self.fluid_velocity (wind).

        Without geometry (primitive model), a single quadratic drag at the CoM:
            D = -|v_body| * C_D * v_body,  v_body = Rᵀ v_rel,  no moment.

        With geometry (set_geometry), drag is summed over the panels of the mesh:
            u_i = v_body + ω × r_i                         relative flow at panel i
            F_i = -½ ρ A_i |n_i·û_i| e_i(û_i) |u_i| (C ⊙ u_i)
            D   = Σ F_i,   M = Σ r_i × F_i   (M only with the drag_moments parameter)
        with r_i the panel centroid from the CoM, A_i its area, n_i its normal, e_i the
        fraction of its facing area the flow reaches (not hidden behind other panels, see
        geometry.drag_panels; blended over nearby tabulated directions) and C = [C_Dx, C_Dy, C_Dz]. Along a body axis Σ A_i|n_i·û|e_i
        is the silhouette, so axial drag is ½ ρ C A u².

        Note: gravity is handled separately in _s_dot_fn via batched_params.weight,
        which stores the constant [0, 0, −mg] world-frame gravity force vector
        as defined in VehicleModel / BatchedParams.

        Args:
            R (Tensor)                : Rotation matrices body→world, shape (B, 3, 3).
            inertial_velocity (Tensor): Linear velocity in the world frame, shape (B, 3).
            w (Tensor)                : Angular velocity in the body frame, shape (B, 3).

        Returns:
            tuple:
                D (Tensor): Drag force  in the body frame, shape (B, 3).
                M (Tensor): Drag moment in the body frame, shape (B, 3).
        """
        # Velocity relative to the air, projected into the body frame: v_body = Rᵀ v_rel
        relative_velocity = inertial_velocity.double() - self.fluid_velocity[self.idxs]
        body_airspeed_vector = (
            R.transpose(1, 2) @ relative_velocity.unsqueeze(-1)
        ).squeeze(-1)  # (B, 3)

        if self._drag_panels is None:
            # Quadratic drag: D = -|v_body| * C_D * v_body
            tmp = self.batched_params.drag_matrix[self.idxs] @ body_airspeed_vector.unsqueeze(-1)
            D   = -torch.linalg.norm(body_airspeed_vector, dim=-1).unsqueeze(-1) * tmp.squeeze()
            return D, torch.zeros_like(body_airspeed_vector)  # (B, 3) body frame

        F_i = self._panel_drag(body_airspeed_vector, w)                          # (B, F, 3)
        D = F_i.sum(dim=1)
        if not self._drag_panels['with_moments']:
            return D, torch.zeros_like(D)
        M = torch.cross(self._drag_panels['arms'][self.idxs], F_i, dim=-1).sum(dim=1)
        return D, M  # (B, 3) body frame

    def panel_drag_forces(self, R: Tensor, inertial_velocity: Tensor, w: Tensor) -> tuple:
        """
        Drag on each panel of the collision mesh, as _compute_external_forces sums it.
        Meant for inspecting where the wind acts on the body.

        Args:
            R (Tensor)                : Rotation matrices body→world, shape (B, 3, 3).
            inertial_velocity (Tensor): Linear velocity in the world frame, shape (B, 3).
            w (Tensor)                : Angular velocity in the body frame, shape (B, 3).

        Returns:
            tuple:
                F_i  (Tensor): Drag force on each panel, body frame, shape (B, F, 3) (N).
                arms (Tensor): Panel centroids from the CoM, body frame, shape (B, F, 3) (m).

        Raises:
            RuntimeError: If set_geometry has not been called (no exported geometry).
        """
        if self._drag_panels is None:
            raise RuntimeError("No drag panels: the DynamicsSensor must export the geometry (ExportGeometry)")
        relative_velocity = inertial_velocity.double() - self.fluid_velocity[self.idxs]
        body_velocity = (R.transpose(1, 2) @ relative_velocity.unsqueeze(-1)).squeeze(-1)
        return self._panel_drag(body_velocity, w), self._drag_panels['arms'][self.idxs]

    def _panel_drag(self, body_velocity: Tensor, w: Tensor) -> Tensor:
        """Per-panel drag (B, F, 3) in the body frame for a body-frame relative velocity (B, 3)."""
        panels = self._drag_panels
        arms = panels['arms'][self.idxs]                                       # (B, F, 3)
        u = body_velocity.unsqueeze(1) + torch.cross(
            w.double().unsqueeze(1).expand_as(arms), arms, dim=-1)             # (B, F, 3)
        batch = {key: panels[key][self.idxs] for key in ('areas', 'normals', 'exposure')}
        batch['directions'] = panels['directions']
        return panel_drag(batch, u, self.batched_params.rho, panels['coefficients'],
                          self.EXPOSURE_BLEND_RAD)                              # (B, F, 3)

    # =========================================================================
    # Body Wrench from Rotors
    # =========================================================================

    def _compute_body_wrench(self, rotor_speeds: Tensor, rotor_in_air: Tensor | None = None) -> tuple:
        """
        Computes the net force and moment in the body frame produced by the
        4 rotors given their current speed commands.

        A rotor under the water surface produces neither thrust nor reaction torque: both are scaled by
        rotor_in_air (0 under water, 1 in air, linear across the waterline band).

        Thrust model: T_i = k_eta * ω_i²   (all rotors point in body +Z)
        Torque model: M_i = k_m   * ω_i²   (reaction torque, sign from rotor_dir)

        Force:
            Each rotor produces [0, 0, T_i] in the body frame.
            FtotB = Σ T_i

        Moment:
            M_force : thrust moment via pre-cached hat-maps: −Σ S(r_i) T_i
                      (uses rotor_geometry_hat_maps from BatchedParams._extract_geometry)
            M_yaw   : reaction torque Σ dir_i * k_m * ω_i²
            MtotB   = M_force + M_yaw

        Args:
            rotor_speeds (Tensor): Motor speed commands (B, n_rotors), rad/s.
            rotor_in_air (Tensor): Fraction of each rotor out of the water (B, n_rotors); None = all in air.

        Returns:
            tuple:
                FtotB (Tensor): Net body-frame force  (B, 3).
                MtotB (Tensor): Net body-frame moment (B, 3).
        """
        if rotor_in_air is not None:
            # Thrust and reaction torque both go with ω², so scaling ω² removes a submerged rotor's action.
            rotor_speeds = rotor_speeds * torch.sqrt(rotor_in_air)

        # Thrust forces: zero in X,Y; T_z = k_eta * ω² per rotor
        T = torch.zeros(self.batch_size, 3, self.batched_params.num_rotors,
                        device=self.device).double()
        T[..., -1, :] = self.batched_params.k_eta[self.idxs] * rotor_speeds**2  # (B, n_rotors)

        # Moment from rotor thrust via pre-cached skew-symmetric position matrices:
        # M_force = -Σ S(r_i) T_i   across all rotors
        M_force = -torch.einsum(
            'bijk, bik->bj', self.batched_params.rotor_geometry_hat_maps[self.idxs], T
        )

        # Reaction yaw torque: M_z_i = dir_i * k_m * ω_i²
        M_yaw = torch.zeros(self.batch_size, 3, 4, device=self.device)
        M_yaw[..., -1, :] = (self.batched_params.rotor_dir[self.idxs]
                              * self.batched_params.k_m[self.idxs]
                              * rotor_speeds**2)

        # Sum contributions across all rotors
        FtotB = torch.sum(T, dim=2)                # (B, 3)
        MtotB = M_force + torch.sum(M_yaw, dim=2)  # (B, 3)

        return FtotB, MtotB

    # =========================================================================
    # State Derivative
    # =========================================================================

    def _s_dot_fn(self, s: Tensor, cmd_ctrl: Tensor) -> Tensor:
        """
        Evaluates the continuous-time state derivative ṡ = f(s, u) for the
        quadrotor rigid-body dynamics in the world frame.

        State layout (13 + n_rotors components, from VehicleModel.pack_state):
            s[:, 0:3]   — position x          (world frame)
            s[:, 3:6]   — linear velocity v   (world frame)
            s[:, 6:10]  — unit quaternion q   (body ← world)
            s[:, 10:13] — angular velocity w  (body frame)

        Dynamics:
            ẋ  = v
            v̇  = (W + R F_body) / m        W = batched_params.weight = [0,0,−mg]
            q̇  = ½ q ⊗ [0, ω]
            ω̇  = I⁻¹ (M_body − ω × (Iω))  (Euler's rotation equation)

        Note: I⁻¹ is pre-computed in _build_params (batched_params.invI) to avoid
        a linear solve at every integration step.

        Args:
            s (Tensor)        : Packed state vector (B, 13), from VehicleModel.pack_state.
            cmd_ctrl (Tensor) : Rotor speed commands (B, n_rotors), rad/s.

        Returns:
            Tensor: State derivative ṡ (B, 13 + n_rotors). Motor speed derivatives
                    are left as zero — rotors are commanded, not integrated as states.
        """
        state             = self.unpack_state(s, self.idxs, self.batch_size)
        inertial_velocity = state['v'][self.idxs]
        R                 = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()

        # Kinematic derivatives
        x_dot = state['v'][self.idxs]                                                # ẋ = v
        q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])   # q̇ = ½ q ⊗ ω

        # Aerodynamic drag in the body frame
        F, M_drag = self._compute_external_forces(R, inertial_velocity, state['w'][self.idxs])

        # Rotor thrust force and moment in the body frame; rotors under the water surface (DynamicsSensor water
        # surface, not z = 0) do not act.
        rotor_world = (state['x'][self.idxs].unsqueeze(1)
                       + self.batched_params.rotor_geometry[self.idxs].double() @ R.transpose(1, 2))  # (B, n, 3)
        rotor_in_air = 1.0 - self._immersion(-self._height_above_water(rotor_world))
        FtotB, MtotB = self._compute_body_wrench(cmd_ctrl, rotor_in_air)

        # Add drag to rotor thrust, then rotate total body force into world frame
        FtotB += F
        MtotB  = MtotB + M_drag
        Ftot   = R @ FtotB.unsqueeze(-1)  # (B, 3, 1) world frame

        # Linear acceleration: Newton's second law with world-frame gravity (batched_params.weight)
        v_dot = (self.batched_params.weight[self.idxs] + Ftot.squeeze(-1)) / self.batched_params.mass

        # Angular acceleration: Euler's rotation equation ω̇ = I⁻¹(M − ω × Iω)
        w     = state['w'][self.idxs].double()
        w_hat = self.batched_params.hat_map(w).permute(2, 0, 1)  # S(ω) via BatchedParams.hat_map
        w_dot = self.batched_params.invI[self.idxs] @ (
            MtotB - (w_hat.double() @ (
                self.batched_params.I[self.idxs] @ w.unsqueeze(-1)
            )).squeeze(-1)
        ).unsqueeze(-1)  # (B, 3, 1)

        # Pack all derivatives into flat state-derivative vector
        s_dot = torch.zeros(
            (len(self.idxs), 13 + self.batched_params.num_rotors,), device=self.device
        )
        s_dot[:, 0:3]   = x_dot
        s_dot[:, 3:6]   = v_dot
        s_dot[:, 6:10]  = q_dot
        s_dot[:, 10:13] = w_dot.squeeze(-1)
        # Motor speed derivatives left as zero — rotors are direct commands, not states

        return s_dot

    # =========================================================================
    # Control → Motor Speeds
    # =========================================================================

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        """
        Converts a high-level control command into rotor speed commands using a
        geometric SE(3) controller cascaded with optional position and yaw loops.

        All three non-passthrough abstractions share the same inner loop:
            1. Compute desired force F_des from velocity or position error.
            2. Derive desired orientation R_des that aligns body +Z with F_des
               and body +X with the desired yaw heading (Gram-Schmidt).
            3. Compute orientation error via the skew-symmetric error matrix:
                   S_err   = ½ (R_des^T R − R^T R_des)
                   att_err = vee(S_err)    (extracts the Lie algebra error vector)
            4. Compute desired moment via PD law on att_err and angular velocity,
               with gyroscopic feedforward: M = I(−kp e_R − kd ω) + ω × Iω
            5. Map [T, Mx, My, Mz] through TM_to_f to rotor thrusts.
            6. Convert rotor thrusts to signed motor speeds: ω = sign(F)√(|F|/k_eta).

        cmd_pos_yaw: PID on position (ki_pos, with anti-windup: integrates only within
        pos_integral_radius of the target and while unsaturated, contribution capped at
        pos_integral_accel_limit), desired acceleration limited to max_tilt_deg and
        max_vertical_accel, heading from the commanded yaw, and yaw controlled inside the
        same inertia-scaled attitude loop as roll and pitch.

        Args:
            state (dict)  : Structured state dict {x, v, q, w} from VehicleModel.map_states.
            control (dict): Control command dict with key 'cmd_ctrl' (B, D).
                            D = 3 for cmd_vel, D = 4 for cmd_vel_yaw / cmd_pos_yaw.

        Returns:
            Tensor: Rotor speed commands (B, n_rotors), signed rad/s.
                    Sign encodes spin direction for yaw torque allocation.
        """

        # --- Pass-through: rotor speeds commanded directly ---
        if self.control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]

        # =====================================================================
        # cmd_vel: Track desired 3D velocity [vx, vy, vz]
        # Yaw is held at zero — body +X points along world +X at all times.
        # =====================================================================
        if self.control_abstraction == 'cmd_vel':

            # Velocity error → desired acceleration (P-control)
            v_err = state['v'][self.idxs] - control['cmd_ctrl'][self.idxs]
            a_cmd = -self.batched_params.k_v[self.idxs] * v_err

            # Desired force = m * (a_cmd + g ẑ), with gravity feedforward in world frame
            F_des = self.batched_params.mass * (
                a_cmd + torch.tensor([0, 0, self.batched_params.g], device=self.device).double()
            )

            # Current body +Z axis in world frame: b3 = R ẑ
            R  = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()
            b3 = R @ torch.tensor([0.0, 0.0, 1.0], device=self.device).double()

            # Scalar thrust = projection of F_des onto current body +Z
            cmd_thrust = torch.sum(F_des * b3, dim=-1).double().unsqueeze(-1)

            # ---- Desired orientation R_des ----
            # b3_des: normalize F_des to get the desired body +Z direction
            tmp    = torch.norm(F_des, dim=-1, keepdim=True)
            mask   = (tmp.squeeze(-1) > 0)
            b3_des = torch.zeros_like(F_des)
            b3_des[mask] = F_des[mask] / torch.norm(F_des[mask], dim=-1, keepdim=True)

            # c1_des: reference heading along world +X (zero yaw command)
            c1_des = torch.tensor([1.0, 0.0, 0.0], device=self.device).repeat(
                self.batch_size, 1).double()

            # Gram-Schmidt: b2 = normalize(b3_des × c1_des), b1 = b2 × b3_des
            cross  = torch.cross(b3_des, c1_des, dim=-1)
            tmp    = torch.norm(cross, dim=-1, keepdim=True)
            mask   = (tmp.squeeze(-1) > 0)
            b2_des = torch.zeros_like(cross)
            b2_des[mask] = cross[mask] / torch.norm(cross[mask], dim=-1, keepdim=True)
            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des  = torch.stack([b1_des, b2_des, b3_des], dim=-1)  # (B, 3, 3)

            # ---- Orientation error (SO(3) Lie algebra) ----
            S_err   = 0.5 * (R_des.transpose(-1, -2) @ R - R.transpose(-1, -2) @ R_des)
            att_err = torch.stack([-S_err[:, 1, 2], S_err[:, 0, 2], -S_err[:, 0, 1]], dim=-1)

            # ---- Angular control: PD on att_err + gyroscopic feedforward ----
            # M = I(−kp e_R − kd ω) + ω × Iω
            Iw        = self.batched_params.I[self.idxs] @ state['w'][self.idxs].unsqueeze(-1)
            tmp       = (-self.batched_params.kp_att[self.idxs] * att_err
                         - self.batched_params.kd_att[self.idxs] * state['w'][self.idxs])
            cmd_moment = ((self.batched_params.I[self.idxs] @ tmp.unsqueeze(-1)).squeeze(-1)
                          + torch.cross(state['w'][self.idxs], Iw.squeeze(-1), dim=-1))

            # ---- Allocation: [T, Mx, My, Mz] → rotor thrusts → motor speeds ----
            TM                = torch.cat([cmd_thrust, cmd_moment.squeeze(-1)], dim=-1)
            cmd_rotor_thrusts = (self.batched_params.TM_to_f[self.idxs]
                                 @ TM.unsqueeze(1).transpose(-1, -2)).squeeze(-1)
            cmd_motor_speeds  = cmd_rotor_thrusts / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds  = torch.sign(cmd_motor_speeds) * torch.sqrt(
                torch.abs(cmd_motor_speeds))

            return cmd_motor_speeds

        # =====================================================================
        # cmd_vel_yaw: Track desired velocity + relative yaw offset [vx, vy, vz, yaw_deg]
        # Extends cmd_vel with an independent yaw PD loop and a yaw-aware R_des.
        # =====================================================================
        if self.control_abstraction == 'cmd_vel_yaw':

            # Velocity error → desired acceleration (same P-control as cmd_vel)
            v_err = state['v'][self.idxs] - control['cmd_ctrl'][self.idxs][:, :3]
            a_cmd = -self.batched_params.k_v[self.idxs] * v_err

            # Current yaw extracted from quaternion via rotation vector representation
            yaw = roma.unitquat_to_rotvec(state['q'][self.idxs])[:, 2]  # ψ (rad)

            # Desired yaw = current yaw + commanded relative offset (degrees → radians)
            yaw_delta_deg = control['cmd_ctrl'][:, 3]
            yaw_delta     = torch.deg2rad(yaw_delta_deg.double())
            yaw_des       = (yaw + yaw_delta + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]

            # Yaw tracking error (wrapped) and current yaw rate
            yaw_rate = state['w'][self.idxs][:, 2]
            yaw_err  = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            # Desired force with gravity feedforward
            F_des = self.batched_params.mass * (
                a_cmd + torch.tensor([0, 0, self.batched_params.g], device=self.device).double()
            )

            # Current attitude. (A former "large-yaw guard" replaced it by q ⊗ Rz(yaw_des) when
            # |yaw_des| > 90°, which turned the vehicle the wrong way; c1_des from yaw_des is valid
            # at any heading.)
            R    = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()

            # Scalar thrust projected onto the selected body +Z axis
            b3        = R @ torch.tensor([0.0, 0.0, 1.0], device=self.device).double()
            cmd_thrust = torch.sum(F_des * b3, dim=-1).double().unsqueeze(-1)

            # Yaw moment: desired yaw rate from P-gain, then D-gain on rate error
            yaw_dot_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_dot_des - yaw_rate
            M_yaw       = self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # ---- Desired orientation R_des ----
            # b3_des aligned with F_des; c1_des points along yaw_des in world XY-plane
            tmp    = torch.norm(F_des, dim=-1, keepdim=True)
            mask   = (tmp.squeeze(-1) > 0)
            b3_des = torch.zeros_like(F_des)
            b3_des[mask] = F_des[mask] / torch.norm(F_des[mask], dim=-1, keepdim=True)
            c1_des = torch.stack(
                [torch.cos(yaw_des), torch.sin(yaw_des), torch.zeros_like(yaw_des)], dim=-1
            )

            cross  = torch.cross(b3_des, c1_des, dim=-1)
            tmp    = torch.norm(cross, dim=-1, keepdim=True)
            mask   = (tmp.squeeze(-1) > 0)
            b2_des = torch.zeros_like(cross)
            b2_des[mask] = cross[mask] / torch.norm(cross[mask], dim=-1, keepdim=True)
            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des  = torch.stack([b1_des, b2_des, b3_des], dim=-1)

            # ---- Orientation error and angular control ----
            S_err   = 0.5 * (R_des.transpose(-1, -2) @ R - R.transpose(-1, -2) @ R_des)
            att_err = torch.stack([-S_err[:, 1, 2], S_err[:, 0, 2], -S_err[:, 0, 1]], dim=-1)

            Iw        = self.batched_params.I[self.idxs] @ state['w'][self.idxs].unsqueeze(-1)
            tmp       = (-self.batched_params.kp_att[self.idxs] * att_err
                         - self.batched_params.kd_att[self.idxs] * state['w'][self.idxs])
            cmd_moment = ((self.batched_params.I[self.idxs] @ tmp.unsqueeze(-1)).squeeze(-1)
                          + torch.cross(state['w'][self.idxs], Iw.squeeze(-1), dim=-1))

            # Inject yaw moment correction into the Z-axis moment channel
            cmd_moment[:, 2] += M_yaw[self.idxs]

            # ---- Allocation ----
            TM                = torch.cat([cmd_thrust, cmd_moment.squeeze(-1)], dim=-1)
            cmd_rotor_thrusts = (self.batched_params.TM_to_f[self.idxs]
                                 @ TM.unsqueeze(1).transpose(-1, -2)).squeeze(-1)
            cmd_motor_speeds  = cmd_rotor_thrusts / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds  = torch.sign(cmd_motor_speeds) * torch.sqrt(
                torch.abs(cmd_motor_speeds))

            return cmd_motor_speeds

        # =====================================================================
        # cmd_pos_yaw: Track desired 3D position + absolute yaw [x, y, z, yaw_deg]
        # Outermost loop: position PD → F_des, then same SE(3) inner loop as above.
        # =====================================================================
        if self.control_abstraction == 'cmd_pos_yaw':
            bp   = self.batched_params
            idxs = self.idxs
            limits = self._pos_control

            # Parse goal: desired position and absolute yaw (degrees → radians)
            pos_des = control['cmd_ctrl'][idxs, :3].double()
            yaw_des = torch.deg2rad(control['cmd_ctrl'][idxs, 3].double())

            x = state['x'][idxs].double()
            v = state['v'][idxs].double()
            w = state['w'][idxs].double()   # body frame

            # ---- Position PID outer loop: desired acceleration (world frame) ----
            # Desired velocity is zero (hold-position), so e_vel = 0 − v = −v. The integral
            # cancels steady disturbances such as wind drag.
            e_pos = pos_des - x
            a_des = (bp.kp_pos[idxs] * e_pos - bp.kd_pos[idxs] * v
                     + limits['ki'] * self.position_integral[idxs])

            # ---- Flight envelope: tilt (horizontal acceleration) and vertical acceleration ----
            horizontal = torch.linalg.norm(a_des[:, :2], dim=-1, keepdim=True)
            horizontal_max = bp.g * np.tan(limits['max_tilt'])
            saturated = ((horizontal > horizontal_max)
                         | (a_des[:, 2:3].abs() > limits['max_vertical_accel']))
            a_des[:, :2] = a_des[:, :2] * torch.clamp(horizontal_max / horizontal.clamp_min(1e-9), max=1.0)
            a_des[:, 2] = torch.clamp(a_des[:, 2], -limits['max_vertical_accel'], limits['max_vertical_accel'])

            # ---- Integral with anti-windup ----
            # Integrates only near the target and while the acceleration is not saturated, and
            # its contribution is capped, so a long approach does not wind it up into an overshoot.
            frozen = saturated | (torch.linalg.norm(e_pos, dim=-1, keepdim=True) > limits['integral_radius'])
            integral_max = limits['integral_accel_limit'] / limits['ki'] if limits['ki'] > 0 else 0.0
            updated = (self.position_integral[idxs] + e_pos * self.dt).clamp(-integral_max, integral_max)
            self.position_integral[idxs] = torch.where(frozen, self.position_integral[idxs], updated)

            # Desired force with gravity feedforward
            F_des = bp.mass * (a_des + torch.tensor([0, 0, bp.g], device=self.device).double())  # (B, 3)

            # Scalar thrust projected onto the current body +Z axis
            R = roma.unitquat_to_rotmat(state['q'][idxs]).double()
            cmd_thrust = torch.sum(F_des * R[:, :, 2], dim=-1).unsqueeze(-1)

            # ---- Desired orientation R_des ----
            # b3_des along F_des; heading from the commanded yaw (SE(3) geometric controller)
            b3_des = F_des / torch.norm(F_des, dim=-1, keepdim=True)
            c1_des = torch.stack(
                [torch.cos(yaw_des), torch.sin(yaw_des), torch.zeros_like(yaw_des)], dim=-1
            )
            b2_des = torch.cross(b3_des, c1_des, dim=-1)
            b2_des = b2_des / torch.norm(b2_des, dim=-1, keepdim=True)
            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des  = torch.stack([b1_des, b2_des, b3_des], dim=-1)

            # ---- Orientation error (SO(3)); yaw is part of it ----
            S_err   = 0.5 * (torch.matmul(R_des.transpose(1, 2), R)
                             - torch.matmul(R.transpose(1, 2), R_des))
            att_err = torch.stack([S_err[:, 2, 1], S_err[:, 0, 2], S_err[:, 1, 0]], dim=-1)

            # ---- Angular control: M = I(−kp e_R − kd ω) + ω × Iω ----
            I   = bp.I[idxs].double()
            Iw  = torch.einsum("bij,bj->bi", I, w)
            tmp = -bp.kp_att[idxs] * att_err - bp.kd_att[idxs] * w
            cmd_moment = torch.einsum("bij,bj->bi", I, tmp) + torch.cross(w, Iw, dim=-1)

            # ---- Allocation ----
            TM                = torch.cat([cmd_thrust, cmd_moment], dim=-1)  # (B, 4)
            cmd_rotor_thrusts = torch.einsum("bij,bj->bi", bp.TM_to_f[idxs], TM)
            cmd_motor_speeds  = (torch.sign(cmd_rotor_thrusts)
                                 * torch.sqrt(torch.abs(cmd_rotor_thrusts / bp.k_eta[idxs])))

            return cmd_motor_speeds  # (B, 4) rad/s