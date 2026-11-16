import torch
import roma

import numpy as np

from torch import Tensor
    
class BatchedParams:
    """
    Parameter container for a batch of identical multirotor vehicles.

    Holds all physics quantities required by the dynamics pipeline as either
    plain scalars/numpy arrays (shared across the batch) or batched PyTorch
    tensors of leading dimension B (one entry per vehicle). Tensor quantities
    are placed on the specified device at construction time.

    Subclasses (or vehicle __init__ methods) are expected to attach additional
    attributes after construction (e.g. inertia tensors, drag matrices,
    allocation matrices) by directly assigning to the instance.

    Args:
        batch_size (int): Number of vehicles in the batch.
        params (dict)   : Raw parameter dict from the vehicle config file.
        device (str)    : Torch device string, e.g. 'cpu' or 'cuda:0'.
    """

    def __init__(self, batch_size: int, params: dict, device: str):

        self.batch_size = batch_size
        self.device = device

        # Fluid density (kg/m³) — optional; single-fluid models may omit this.
        # Multi-fluid models (air/water) override self.rho after construction.
        if 'rho' in params.keys():
            self.rho = params['rho']

        # Gravitational acceleration (m/s²) — standard value used throughout dynamics
        self.g = 9.80665

        # --- Mass / weight ---
        self.mass = params['mass']  # Vehicle mass (kg), scalar shared across batch

        # Gravity force vector in world frame (Z-up convention): F_g = [0, 0, -mg]
        # Shape (B, 3) so it broadcasts directly against per-robot force tensors.
        self.weight = torch.zeros(batch_size, 3, device=device).double()
        self.weight[:, -1] = -self.mass * self.g

        # --- Motor speed limits ---
        # Both tensors are shape (B, 1) with a trailing singleton for broadcast
        # against per-rotor speed tensors of shape (B, n_rotors).
        # Note: for multi-domain vehicles these are overridden with (B, 2) tensors
        # where column 0 = air limit and column 1 = water limit.
        self.rotor_speed_min = torch.tensor(
            [params['rotor_speed_min'] for _ in range(batch_size)]
        ).unsqueeze(-1).to(device)   # Minimum rotor speed (rad/s)

        self.rotor_speed_max = torch.tensor(
            [params['rotor_speed_max'] for _ in range(batch_size)]
        ).unsqueeze(-1).to(device)   # Maximum rotor speed (rad/s)

        # --- Motor coefficients (optional — absent for simplified models) ---
        # k_eta : thrust coefficient  F = k_eta * ω²      (N / (rad/s)²), shape (B, n_domains, 1)
        # k_m   : drag-torque coefficient M = k_m * ω²    (N·m / (rad/s)²), shape (B, n_domains, 1)
        # The extra domain dimension allows separate coefficients for air and water rotors.
        if 'k_eta' in params.keys() and 'k_m' in params.keys():
            self.k_eta = torch.from_numpy(
                np.array([params['k_eta'] for _ in range(batch_size)])
            ).unsqueeze(-1).to(device)

            self.k_m = torch.from_numpy(
                np.array([params['k_m'] for _ in range(batch_size)])
            ).unsqueeze(-1).to(device)

    # =========================================================================
    # Static Geometric / Hydrodynamic Utilities
    # =========================================================================

    @staticmethod
    def hat_map(s: Tensor) -> Tensor:
        """
        Computes the skew-symmetric (hat) matrix S(s) ∈ R^{3×3} for a vector s ∈ R^3,
        satisfying S(s) v = s × v for all v.

        Supports both single-vector and batched inputs:
            - Single:    s shape (3,)    → output shape (3, 3)
            - Batched:   s shape (N, 3)  → output shape (3, 3, N)
              (note the unconventional (3, 3, N) layout; callers use .permute(2,0,1)
               to convert to the standard (N, 3, 3) batch convention)

        Args:
            s (Tensor): Vector(s) in R^3, shape (3,) or (N, 3).

        Returns:
            Tensor: Skew-symmetric matrix/matrices.
                    Shape (3, 3) for single input, (3, 3, N) for batched input.
        """
        device = s.device
        if len(s.shape) > 1:
            # Batched path: build the 9 elements of the skew-symmetric matrix
            # and reshape into (3, 3, N). Callers permute to (N, 3, 3) as needed.
            s = s.unsqueeze(-1)  # (N, 3, 1)
            hat = torch.cat([
                torch.zeros(s.shape[0], 1, device=device), -s[:, 2],  s[:, 1],
                 s[:, 2], torch.zeros(s.shape[0], 1, device=device),  -s[:, 0],
                -s[:, 1],  s[:, 0], torch.zeros(s.shape[0], 1, device=device)
            ], dim=0).view(3, 3, s.shape[0]).double()
            return hat
        else:
            # Single-vector path: explicit 3×3 construction
            return torch.tensor([
                [ 0,     -s[2],  s[1]],
                [ s[2],   0,    -s[0]],
                [-s[1],   s[0],  0   ]
            ], device=device)

    @staticmethod
    def Hmtrx(r: Tensor) -> Tensor:
        """
        Computes the 6×6 system transformation matrix H(r) used to transform
        generalized forces and inertia matrices between different reference points
        on the vehicle body (e.g. center of origin ↔ center of gravity).

            H(r) = [ I₃    S(r)ᵀ ]
                   [ 0₃    I₃    ]

        Key property: H(r)⁻¹ = H(−r).

        If r = r_bg is the vector from the CO (center of origin) to the CG
        (center of gravity), then mass matrices and generalized forces in the
        two frames are related by:
            M_CO  = H(r_bg)ᵀ M_CG H(r_bg)
            τ_CO  = H(r_bg)ᵀ τ_CG
            η_CO  = H(r_bg)ᵀ η_CG

        Reference: Fossen (2021), Handbook of Marine Craft Hydrodynamics and
                   Motion Control, Chapter 3.

        Args:
            r (Tensor): Position vectors (B, 3) from CO to CG in the body frame.

        Returns:
            Tensor: Transformation matrices of shape (B, 6, 6), dtype float64.
        """
        # Start from batched identity and fill the upper-right 3×3 block with S(r)ᵀ
        H = torch.eye(6).unsqueeze(0).repeat(r.shape[0], 1, 1).double().to(r.device)
        H[:, 0:3, 3:6] = BatchedParams.hat_map(r).mT.permute(1, 0, 2)
        return H

    @staticmethod
    def m2c(M: Tensor, nu: Tensor) -> Tensor:
        """
        Computes the Coriolis and centripetal matrix C(M, ν) from the
        (possibly non-symmetric) mass matrix M and the generalized velocity ν,
        using the parameterization of Fossen (2021), Chapter 3.

        The result satisfies the skew-symmetry property: C + Cᵀ = 0,
        which is exploited by energy-based stability proofs.

        Two model variants are supported based on the velocity dimension:
            - 6-DOF (full): ν ∈ R^6  → C ∈ R^{6×6}
            - 3-DOF (surge/sway/yaw): ν[:, [0,1,5]] → simplified C ∈ R^{6×6}

        Reference: Fossen (2021), Eq. (3.x).

        Args:
            M  (Tensor): Mass/inertia matrix, shape (B, 6, 6). Need not be symmetric;
                         symmetrized internally via M ← ½(M + Mᵀ).
            nu (Tensor): Generalized velocity [v; ω], shape (B, 6) for 6-DOF
                         or (B, 3) for 3-DOF (surge, sway, yaw).

        Returns:
            Tensor: Coriolis matrix C(ν), shape (B, 6, 6).
        """
        # Symmetrize M to remove any numerical asymmetry before parameterization
        M = 0.5 * (M + M.mT)
        batch_size = M.shape[0]
        device     = M.device

        v_dot = nu[:, :3]
        w_dot = nu[:, 3:]

        if v_dot.shape[-1] == 3:
            # ---- 6-DOF Coriolis ----
            # Partition M into 3×3 blocks
            M11 = M[:, 0:3, 0:3]
            M12 = M[:, 0:3, 3:6]
            M21 = M12.mT
            M22 = M[:, 3:6, 3:6]

            # Intermediate vectors: ∂T/∂v and ∂T/∂ω (Fossen 2021, Eq. 3.46)
            dt_dv_dot = (torch.einsum('bij,bj->bi', M11, v_dot)
                         + torch.einsum('bij,bj->bi', M12, w_dot))
            dt_dw_dot = (torch.einsum('bij,bj->bi', M21, v_dot)
                         + torch.einsum('bij,bj->bi', M22, w_dot))

            # Assemble 6×6 Coriolis matrix from skew-symmetric blocks
            C = torch.cat([
                torch.cat([
                    torch.zeros((batch_size, 3, 3), device=device),
                    -BatchedParams.hat_map(dt_dv_dot).permute(2, 0, 1)
                ], dim=-1),
                torch.cat([
                    -BatchedParams.hat_map(dt_dv_dot).permute(2, 0, 1),
                    -BatchedParams.hat_map(dt_dw_dot).permute(2, 0, 1)
                ], dim=-1)
            ], dim=-2)

        else:
            # ---- 3-DOF Coriolis (surge, sway, yaw) ----
            # Only the sway and yaw coupling terms are non-zero
            C = torch.zeros((batch_size, 6, 6)).to(device)
            C[:, 0, 2] = -M[:, 1, 1] * v_dot[:, 1] - M[:, 1, 2] * w_dot[:, 2]
            C[:, 1, 2] =  M[:, 0, 0] * v_dot[:, 0]
            C[:, 2, 0] = -C[:, 0, 2]
            C[:, 2, 1] = -C[:, 1, 2]

        return C

    @staticmethod
    def cross_flow_drag(rho: float, L: float, B: torch.Tensor, T: torch.Tensor,
                        nu_r: torch.Tensor) -> torch.Tensor:
        """
        Computes the cross-flow drag force and yaw moment on a slender body using
        strip theory (Fossen 2021, Chapter 6).

        The body is discretized into n = 20 strips along its length. For each strip,
        the local transverse velocity (sway + yaw rate × lever arm) is used to
        compute a quadratic drag force via the 2D Hoerner drag coefficient C_Y(B/2T).

        Only sway force (Y) and yaw moment (N) are non-zero; surge, heave, roll,
        and pitch contributions are set to zero (slender-body approximation).

        Reference: Hoerner (1965), Fluid-Dynamic Drag; Fossen (2021), Ch. 6.

        Args:
            rho  (float)   : Fluid density (kg/m³).
            L    (float)   : Body length (m).
            B    (Tensor)  : Body beam/width (m), shape (B,).
            T    (Tensor)  : Body draft/height (m), shape (B,).
            nu_r (Tensor)  : Relative generalized velocity [v; ω], shape (B, 6).
                             nu_r[:, 1] = sway velocity; nu_r[:, 5] = yaw rate.

        Returns:
            Tensor: Cross-flow drag wrench [Fx, Fy, Fz, Mx, My, Mz] of shape (B, 6).
                    Only Fy (index 1) and Mz (index 5) are non-zero.
        """

        def Hoerner(B, T):
            """
            Interpolates the 2D Hoerner cross-flow drag coefficient C_Y^{2D}
            as a function of the beam-to-draft ratio B/(2T).

            Data digitized from Hoerner (1965), Figure 3-23. The coefficient
            decreases monotonically as B/(2T) increases (flatter cross-sections
            produce less cross-flow resistance per unit area).

            Args:
                B (float): Body beam (m).
                T (float): Body draft (m).

            Returns:
                float: Interpolated 2D drag coefficient C_Y^{2D}.
            """
            DATA1 = np.array([
                0.0109, 0.1766, 0.3530, 0.4519, 0.4728, 0.4929, 0.4933, 0.5585,
                0.6464, 0.8336, 0.9880, 1.3081, 1.6392, 1.8600, 2.3129, 2.6000,
                3.0088, 3.4508, 3.7379, 4.0031
            ])  # B/(2T) lookup table
            DATA2 = np.array([
                1.9661, 1.9657, 1.8976, 1.7872, 1.5837, 1.2786, 1.2108, 1.0836,
                0.9986, 0.8796, 0.8284, 0.7599, 0.6914, 0.6571, 0.6307, 0.5962,
                0.5868, 0.5859, 0.5599, 0.5593
            ])  # Corresponding C_Y^{2D} values
            return np.interp(B / (2 * T), DATA1, DATA2)

        batch_size = nu_r.shape[0]
        device     = nu_r.device
        n  = 20          # Number of strips along the body length
        dx = L / n       # Strip width (m)

        # 2D drag coefficient for each robot (scalar per batch entry after Hoerner lookup)
        Cd_2D = torch.from_numpy(
            np.array([Hoerner(B, T) for _ in range(batch_size)])
        ).to(device)

        # Accumulate sway force Yh and yaw moment Nh by integrating along the body
        Yh = 0
        Nh = 0
        xL = -L / 2  # Strip position: starts at the bow, steps sternward

        for _ in range(n + 1):
            v_r = nu_r[:, 1]   # Sway velocity component (m/s)
            r   = nu_r[:, 5]   # Yaw rate (rad/s)

            # Local transverse velocity at strip position xL: U_cf = v_r + xL * r
            # Quadratic drag: dY = -½ρ T C_Y^{2D} U_cf|U_cf| dx
            Ucf  = (v_r + xL * r).abs() * (v_r + xL * r)
            Yh  -= 0.5 * rho * T * Cd_2D * Ucf * dx
            Nh  -= 0.5 * rho * T * Cd_2D * xL * Ucf * dx  # Lever arm contribution to yaw
            xL  += dx

        # Assemble 6-DOF wrench; only sway (index 1) and yaw (index 5) are non-zero
        tau_crossflow = torch.hstack([
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),  # Fx
            Yh.unsqueeze(1),                                                    # Fy (sway)
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),  # Fz
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),  # Mx
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),  # My
            Nh.unsqueeze(1),                                                    # Mz (yaw)
        ])

        return tau_crossflow

    # =========================================================================
    # Geometry Extraction
    # =========================================================================

    def _extract_geometry(self):
        """
        Builds batched rotor geometry tensors from the per-robot rotor position
        dictionaries stored in self.rotor_pos.

        Populates two new attributes:
            self.rotor_geometry          (Tensor, shape (B, n_rotors, 3)):
                Position of each rotor relative to the CoM in the body frame.
                Used to compute moment arms r × F in the allocation matrix.

            self.rotor_geometry_hat_maps (Tensor, shape (B, 3, 3, n_rotors) or similar):
                Pre-computed skew-symmetric matrices S(r_i) for each rotor position,
                enabling vectorized cross-product moment-arm computations without
                recomputing hat maps at every dynamics step.

        Requires:
            self.rotor_pos must be set before calling (asserted).
            Each self.rotor_pos[i] is a dict mapping rotor name → (3,) position array.

        Raises:
            AssertionError: If self.rotor_pos has not been set prior to calling.
        """
        assert hasattr(self, "rotor_pos"), "rotor_pos must be defined before calling _extract_geometry."

        geoms         = []
        geom_hat_maps = []

        for i in range(self.batch_size):
            # Stack all rotor positions for robot i into an (n_rotors, 3) array
            rotor_geometry = np.array([]).reshape(0, 3)
            for rotor in self.rotor_pos[i]:
                rp = self.rotor_pos[i][rotor]
                rotor_geometry = np.vstack([rotor_geometry, rp])

            geoms.append(rotor_geometry)
            # Pre-compute the skew-symmetric matrix for each rotor position
            geom_hat_maps.append(
                self.hat_map(torch.from_numpy(rotor_geometry.squeeze())).numpy()
            )

        # Convert lists to batched tensors and move to the target device
        self.rotor_geometry          = torch.from_numpy(np.array(geoms)).to(self.device)
        self.rotor_geometry_hat_maps = torch.from_numpy(np.array(geom_hat_maps)).to(self.device)

    


class Transform:
    """
    Static utility class for coordinate-frame transformations used throughout
    the biguasim dynamics pipeline.

    All vehicles in biguasim operate internally in the NWU (North-West-Up)
    right-handed coordinate system with Z-up. This class provides the tools
    to convert between NWU and NED (North-East-Down), to integrate quaternion
    kinematics, and to rotate accelerations between body and world frames.

    Coordinate system conventions:
        NWU : X-North, Y-West,  Z-Up   (internal simulation frame)
        NED : X-North, Y-East,  Z-Down (external / sensor output frame)

    The conversion matrix T maps NWU ↔ NED by flipping Y and Z:
        T = diag(1, -1, -1)

    Note: all methods are static or class methods — this class is never
    instantiated.
    """

    # Coordinate system conversion matrix: NWU ↔ NED.
    # Flips Y (West → East) and Z (Up → Down) while leaving X (North) unchanged.
    # Property: T² = I  (self-inverse — the same matrix converts in both directions)
    T_coord_sys = torch.tensor([
        [ 1,  0,  0],
        [ 0, -1,  0],
        [ 0,  0, -1]
    ]).double()

    # =========================================================================
    # Quaternion Kinematics
    # =========================================================================

    @staticmethod
    def quat_dot_torch(quat: Tensor, omega: Tensor) -> Tensor:
        """
        Computes the quaternion time derivative q̇ given the current unit quaternion
        and the body-frame angular velocity, using the kinematic relation:

            q̇ = ½ G(q)ᵀ ω

        where G(q) is the 3×4 matrix derived from the quaternion components such
        that G(q)ᵀ ω gives the rate of change of all four quaternion components.

        A soft normalization correction is applied to prevent quaternion drift
        during numerical integration:
            q̇ ← q̇ − (|q|² − 1) * 2q

        This keeps |q| ≈ 1 without a hard renormalization step.

        Reference: Basile Graf, "Quaternions And Dynamics" (2008).

        Convention: quaternion stored as [i, j, k, w] = [x, y, z, w].

        Args:
            quat  (Tensor): Unit quaternion (B, 4), components [x, y, z, w].
            omega (Tensor): Angular velocity in the body frame (B, 3), rad/s.

        Returns:
            Tensor: Quaternion time derivative q̇ (B, 4), same layout as quat.
        """
        b = quat.shape[0]

        # Unpack quaternion components: [x, y, z, w] → scalar w = q3, vector = (q0,q1,q2)
        (q0, q1, q2, q3) = (quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3])

        # Build G(q) ∈ R^{3×4}: the matrix satisfying q̇ = ½ Gᵀ ω
        # Rows correspond to ω_x, ω_y, ω_z; columns to q_x, q_y, q_z, q_w
        G = torch.stack([
             q3,  q2, -q1, -q0,
            -q2,  q3,  q0, -q1,
             q1, -q0,  q3, -q2
        ], dim=1).view((b, 3, 4))

        # q̇ = ½ Gᵀ ω
        quat_dot = 0.5 * torch.transpose(G, 1, 2) @ omega.unsqueeze(-1)

        # Soft normalization: subtract the gradient of ½(|q|²−1)² to keep |q| ≈ 1
        quat_err      = torch.sum(quat**2, dim=-1) - 1   # Scalar unit-norm violation
        quat_err_grad = 2 * quat                          # Gradient of |q|² w.r.t. q
        quat_dot      = quat_dot.squeeze(-1) - quat_err.unsqueeze(-1) * quat_err_grad

        return quat_dot  # (B, 4)

    # =========================================================================
    # Full State Conversion: NWU → NED
    # =========================================================================

    @classmethod
    def _convert_NWU_to_NED(cls, x: torch.Tensor) -> torch.Tensor:
        """
        Converts a full BiguaSim dynamic sensor output tensor from NWU to NED.

        The 19-column state layout is:
            x[:, 0:3]   — linear acceleration  (world frame)
            x[:, 3:6]   — linear velocity      (world frame)
            x[:, 6:9]   — position             (world frame)
            x[:, 9:12]  — angular acceleration (world frame)
            x[:, 12:15] — angular velocity     (world frame)
            x[:, 15:19] — unit quaternion      [x, y, z, w]

        All 3-vectors are converted by left-multiplying with T = diag(1,−1,−1).
        The quaternion is converted by conjugating the rotation matrix:
            R_NED = T R_NWU T

        Args:
            x (Tensor): Full state tensor (B, 19) in NWU frame.

        Returns:
            Tensor: Full state tensor (B, 19) in NED frame.
        """
        B   = x.shape[0]
        T   = cls.T_coord_sys   # (3, 3)
        x_ned = x.clone()

        # Convert all five 3-vector blocks with the same linear transform T
        for start, end in [(0, 3), (3, 6), (6, 9), (9, 12), (12, 15)]:
            vec     = x[:, start:end]                      # (B, 3)
            vec_ned = torch.matmul(T, vec.T).T             # (B, 3)
            x_ned[:, start:end] = vec_ned

        # Convert quaternion: R_NED = T R_NWU T  (T is its own inverse)
        quat_nwu  = x[:, 15:]                                          # (B, 4)
        R_nwu     = roma.unitquat_to_rotmat(quat_nwu)                  # (B, 3, 3)
        T_expand  = T.unsqueeze(0).expand(B, -1, -1)                   # (B, 3, 3)
        R_ned     = T_expand @ R_nwu @ T_expand                        # (B, 3, 3)
        quat_ned  = roma.rotmat_to_unitquat(R_ned)                     # (B, 4)
        x_ned[:, 15:] = quat_ned

        return x_ned  # (B, 19)

    # =========================================================================
    # Acceleration Conversion: NED Body → NWU World
    # =========================================================================

    @classmethod
    def _convert_NED_to_NWU(cls, q: torch.Tensor, v_dot: torch.Tensor,
                             w_dot: torch.Tensor) -> torch.Tensor:
        """
        Converts batched linear and angular accelerations from the body frame
        in NED to the world frame in NWU, in two steps:

            1. NED body → NWU body : a_NWU_body = T * a_NED_body
            2. NWU body → NWU world: a_NWU_world = R_body_to_world * a_NWU_body

        where T = diag(1,−1,−1) and R_body_to_world is derived from the quaternion q.

        Args:
            q     (Tensor): Unit quaternions (B, 4), [x, y, z, w], body←world rotation.
            v_dot (Tensor): Linear  acceleration in body frame NED (B, 3), m/s².
            w_dot (Tensor): Angular acceleration in body frame NED (B, 3), rad/s².

        Returns:
            Tensor: Concatenated [v̈, ẇ] in world frame NWU (B, 6).
        """
        B        = q.shape[0]
        T        = cls.T_coord_sys                              # (3, 3)
        T_expand = T.unsqueeze(0).expand(B, -1, -1)            # (B, 3, 3)

        # Rotation matrix from body to world frame
        R_body_to_world = roma.unitquat_to_rotmat(q)           # (B, 3, 3)

        # Step 1: flip NED → NWU in body frame
        v_nwu_body = torch.matmul(T_expand, v_dot.unsqueeze(-1)).squeeze(-1)  # (B, 3)
        w_nwu_body = torch.matmul(T_expand, w_dot.unsqueeze(-1)).squeeze(-1)  # (B, 3)

        # Step 2: rotate body → world frame
        v_nwu_world = torch.bmm(R_body_to_world, v_nwu_body.unsqueeze(-1)).squeeze(-1)  # (B, 3)
        w_nwu_world = torch.bmm(R_body_to_world, w_nwu_body.unsqueeze(-1)).squeeze(-1)  # (B, 3)

        return torch.cat([v_nwu_world, w_nwu_world], dim=1)  # (B, 6)

    # =========================================================================
    # Full State Conversion: NWU World → NED Body  (η, ν)
    # =========================================================================

    @classmethod
    def convert_NWU_to_NED(cls, pos: Tensor, quat: Tensor,
                            vel: Tensor, omega: Tensor) -> tuple:
        """
        Converts a vehicle state from the NWU world frame into the NED body frame,
        producing the standard marine vehicle state representation (η, ν) used by
        Fossen-style hydrodynamic models.

        Output:
            η = [x, y, z, φ, θ, ψ]  — position and ZYX Euler angles in NED
            ν = [u, v, w, p, q, r]  — linear and angular velocity in the NED body frame

        Conversion steps:
            Position  : η_pos = T * pos_NWU
            Orientation: R_NED = T R_NWU T  → ZYX Euler angles (φ, θ, ψ)
            Velocity  : ν_lin = T * R_NWU^{-1} * vel_NWU  (world → body, NWU → NED)
            Ang. vel. : ν_ang = T * R_NWU^{-1} * ω_NWU

        Args:
            pos   (Tensor): Position in NWU world frame             (B, 3), m.
            quat  (Tensor): Unit quaternion [x, y, z, w]            (B, 4), body←world NWU.
            vel   (Tensor): Linear velocity in NWU world frame      (B, 3), m/s.
            omega (Tensor): Angular velocity in NWU world frame     (B, 3), rad/s.

        Returns:
            tuple:
                eta (Tensor): Pose vector [x, y, z, φ, θ, ψ] in NED  (B, 6).
                nu  (Tensor): Velocity vector [u, v, w, p, q, r] in NED body frame (B, 6).
        """

        def R2euler(R: torch.Tensor) -> tuple:
            """
            Extracts ZYX Euler angles (roll φ, pitch θ, yaw ψ) from a rotation
            matrix using the standard atan2-based formulas.

            Supports batched input: R shape (B, 3, 3) or single (3, 3).

            Convention:
                φ (roll)  = atan2(R[2,1], R[2,2])
                θ (pitch) = −asin(R[2,0])
                ψ (yaw)   = atan2(R[1,0], R[0,0])

            Args:
                R (Tensor): Rotation matrix/matrices, shape (B, 3, 3) or (3, 3).

            Returns:
                tuple: (phi, theta, psi) — each shape (B,) for batched input.
            """
            phi   = torch.atan2(R[..., 2, 1], R[..., 2, 2])   # Roll
            theta = -torch.asin(R[..., 2, 0])                  # Pitch
            psi   = torch.atan2(R[..., 1, 0], R[..., 0, 0])   # Yaw
            return phi, theta, psi

        T_coord_sys = cls.T_coord_sys.to(pos.device)   # (3, 3)

        R     = roma.unitquat_to_rotmat(quat)           # R_body←world in NWU  (B, 3, 3)
        R_inv = R.transpose(-1, -2)                     # R_world←body = Rᵀ    (B, 3, 3)

        # ---- Position: NWU world → NED world ----
        eta_pos = torch.matmul(T_coord_sys, pos.unsqueeze(-1)).squeeze(-1)   # (B, 3)

        # ---- Orientation: NWU rotation → NED ZYX Euler angles ----
        # Conjugate the rotation matrix into the NED frame: R_NED = T R_NWU T
        R_Fos          = torch.matmul(torch.matmul(T_coord_sys, R), T_coord_sys)
        phi, theta, psi = R2euler(R_Fos)
        eta = torch.cat([
            eta_pos,
            phi.unsqueeze(-1),
            theta.unsqueeze(-1),
            psi.unsqueeze(-1)
        ], dim=-1)  # (B, 6)

        # ---- Velocity: NWU world → NED body ----
        # Step 1: rotate world → body (R_inv = Rᵀ)
        # Step 2: flip NWU → NED (multiply by T)
        vel_body_NED   = torch.matmul(T_coord_sys,
                             torch.matmul(R_inv, vel.unsqueeze(-1))).squeeze(-1)    # (B, 3)
        omega_body_NED = torch.matmul(T_coord_sys,
                             torch.matmul(R_inv, omega.unsqueeze(-1))).squeeze(-1)  # (B, 3)
        nu = torch.cat([vel_body_NED, omega_body_NED], dim=-1)  # (B, 6)

        return eta, nu

    # =========================================================================
    # Acceleration Conversion: NED Body → NWU World  (public interface)
    # =========================================================================

    @classmethod
    def convert_NED_to_NWU(cls, quat_xyzw: Tensor, accel: Tensor) -> Tensor:
        """
        Converts linear and angular accelerations from the body frame in NED
        to the world frame in NWU.

        This is the public-facing counterpart to _convert_NED_to_NWU. It accepts
        accelerations as a single 6-vector (linear + angular concatenated) and
        applies the same two-step transform:

            1. NED body → NWU body : a_NWU = T * a_NED
            2. NWU body → NWU world: a_world = R_body_to_world * a_NWU

        Typical caller: VehicleModel.step() — converts the s_dot output of
        _s_dot_fn (body frame, NED) into the world-frame NWU accelerations
        expected by the simulator.

        Args:
            quat_xyzw (Tensor): Unit quaternion [x, y, z, w], body←world (B, 4).
            accel     (Tensor): Concatenated accelerations in body frame NED (B, 6).
                                accel[:, :3] = linear  acceleration (m/s²)
                                accel[:, 3:] = angular acceleration (rad/s²)

        Returns:
            Tensor: Concatenated [v̈, ẇ] in world frame NWU (B, 6).
        """
        T_coord_sys = cls.T_coord_sys.to(quat_xyzw.device)   # (3, 3)

        # Rotation matrix from body to world frame
        R_body_to_world = roma.unitquat_to_rotmat(quat_xyzw)  # (B, 3, 3)

        lin_accel = accel[..., :3].unsqueeze(-1)   # (B, 3, 1) linear  acceleration NED body
        ang_accel = accel[..., 3:6].unsqueeze(-1)  # (B, 3, 1) angular acceleration NED body

        # Step 1: NED → NWU in body frame; Step 2: body → world frame
        lin_accel_world = torch.matmul(
            R_body_to_world, torch.matmul(T_coord_sys, lin_accel)
        ).squeeze(-1)  # (B, 3)

        ang_accel_world = torch.matmul(
            R_body_to_world, torch.matmul(T_coord_sys, ang_accel)
        ).squeeze(-1)  # (B, 3)

        return torch.cat([lin_accel_world, ang_accel_world], dim=-1)  # (B, 6)



    




















