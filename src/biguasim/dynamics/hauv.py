import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.control import BatchedPIDController
from biguasim.dynamics.marine import WaterHydrostatics
from biguasim.dynamics.utils import Transform as tfrm

# Voxel edge of the primitive hull used for the hydrostatics: the arms are only 2 cm thick.
HULL_VOXEL = 0.005


def _primitive_hull_voxels(rc: float, dd: float, length: float, arm_directions: np.ndarray, size: float) -> np.ndarray:
    """Voxel centers (N, 3), from the center of mass, of the hull the model's volume describes: a sphere of radius rc
    at the center and one cylinder of diameter dd per arm, from the sphere (rc) to the arm length, along each of
    arm_directions (horizontal unit vectors)."""
    # Grids symmetric about the center of mass (integer multiples of size): a float arange ended one voxel short on
    # the + side, which put the center of buoyancy 0.2 mm off the CG; with CG = CB and 30 N of buoyancy that tilted
    # the vehicle ~18 deg in 4 s under water with no thrust.
    n = int(np.ceil((length + size) / size))
    nz = int(np.ceil((max(rc, dd / 2) + size) / size))
    axis = np.arange(-n, n + 1) * size
    zaxis = np.arange(-nz, nz + 1) * size
    points = np.stack(np.meshgrid(axis, axis, zaxis, indexing="ij"), axis=-1).reshape(-1, 3)
    inside = np.linalg.norm(points, axis=1) <= rc
    for direction in arm_directions:
        along = points @ direction
        radial = np.linalg.norm(points - along[:, None] * direction, axis=1)
        inside |= (along >= rc) & (along <= length) & (radial <= dd / 2)
    return points[inside]



class OctaCopterFourAirFourWaterCross(WaterHydrostatics, VehicleModel):
    # Flat water at z = 0 only when the DynamicsSensor sends no water surface (see VehicleModel._read_water_surface).
    DEFAULT_WATER_LEVEL = 0.0
    # Plant substeps per engine tick (see step): 20 at 30 Hz is 1.7 ms.
    SUBSTEPS = 20
    # cmd_pos_yaw underwater: below this horizontal force need (N) the heading is held (its direction is noise).
    DIVE_HORIZONTAL_FORCE_MIN = 0.05

    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   
        self._control_abstraction = control_abstraction
        self._build_params(params)
        
        self.domain = torch.zeros(batch_size, 1, device=device)
        self.action_domain = torch.zeros(batch_size, 1, device=device)
        self.last_domain = torch.zeros(batch_size, 1, device=device)
        self.domain_rho = torch.zeros(batch_size, 1, device=device)
        self.sat_angle = np.deg2rad(25)
        # Submerged share of the hull volume (B,), from the hydrostatics.
        self.submerged_fraction = torch.zeros(self.batch_size, device=self.device).double()

    def _build_params(self, params : dict):
        self.batched_params.L = params['l']
        self.batched_params.Diam = params['d']
        self.batched_params.rc = params['rc']
        self.batched_params.dd = params['dd']
        self.batched_params.dc = self.batched_params.L - self.batched_params.rc


        self.batched_params.rotor_pos = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array([params['rotor_directions'] for _ in range(self.batch_size)]) #air
        self.batched_params._extract_geometry()
        # Rotor positions and unit directions (body frame) for the plant, the same the allocation below inverts
        self._rotor_positions = torch.tensor(np.stack([np.asarray(r, dtype=np.float64) for r in params["rotor_pos"].values()]),
                                             dtype=torch.float64, device=self.device)                   # (8, 3)
        directions = np.asarray(params['rotor_directions'], dtype=np.float64)
        self._rotor_directions = torch.tensor(directions / np.linalg.norm(directions, axis=1, keepdims=True),
                                              dtype=torch.float64, device=self.device)                 # (8, 3)
        # Air rotors' spin: their reaction torques about body z, as in the allocation's yaw row
        self._air_spins = torch.tensor([1.0, -1.0, 1.0, -1.0], dtype=torch.float64, device=self.device)

        self.batched_params.I = torch.from_numpy(np.array([params['I'] 
                                                           for _ in range(self.batch_size)])).double().to(device=self.device)
        
        self.batched_params.diag = torch.arange(self.batched_params.I.size(-1), device=self.device)

        self.batched_params.rho = np.array([params['rho_air'], params['rho_water']])

        added_air = (4/6) * np.pi * (self.batched_params.rc**3.0) *self.batched_params.rho[0]
        added_water = (4/6) * np.pi * (self.batched_params.rc**3.0) *self.batched_params.rho[1]
        
        self.batched_params.added_mass = np.array([added_air, added_water])
        
        self.batched_params.volume = (4/3) * np.pi * self.batched_params.rc**3 + 4 * np.pi * \
                                        (self.batched_params.dd/2)**2 * self.batched_params.dc
        self.batched_params.buoyancy = np.linalg.norm(self.batched_params.rho[1] * \
                                                      self.batched_params.volume * self.batched_params.g)

        # Hydrostatics (Archimedes on the voxelized hull against the water surface, see WaterHydrostatics): the hull
        # is the primitive the volume above describes, its voxels scaled to exactly that volume.
        self.rho_water = float(params['rho_water'])
        self.current_velocity = torch.zeros(self.batch_size, 3, device=self.device).double()
        rotors = np.stack([np.asarray(r, dtype=np.float64) for r in params['rotor_pos'].values()])[:4]
        arm_directions = rotors[:, :2] / np.linalg.norm(rotors[:, :2], axis=1, keepdims=True)
        arm_directions = np.hstack([arm_directions, np.zeros((len(arm_directions), 1))])
        hull = _primitive_hull_voxels(self.batched_params.rc, self.batched_params.dd, self.batched_params.L,
                                      arm_directions, HULL_VOXEL)
        self._voxels = torch.from_numpy(hull).double().to(self.device).expand(self.batch_size, -1, -1).contiguous()
        self._voxel_volume = torch.full((self.batch_size, len(hull)), self.batched_params.volume / len(hull),
                                        dtype=torch.float64, device=self.device)

        self.batched_params.gravity = torch.zeros(self.batch_size, 3, device=self.device).double()
        self.batched_params.gravity[:,-1] = self.batched_params.g

        self.batched_params.domain = torch.zeros(self.batch_size,3, device=self.device).double()



        Cpz = (0.47 * np.pi * self.batched_params.rc**2 + 4 * 1.17 * 
               (self.batched_params.L-self.batched_params.rc) * self.batched_params.dd)
        Cpx = Cpz + 1.17*(self.batched_params.dd/2)**2
        Cpy = Cpx
        Crx = 1.17 * (self.batched_params.L-self.batched_params.rc) * self.batched_params.dd
        Cry = Crx
        Crz = 2 * Crx

        self.batched_params.Cp = torch.from_numpy(np.array([np.diag([Cpx, Cpy, Cpz]) 
                                                for _ in range(self.batch_size)])).double().to(self.device)
        self.batched_params.Cr = torch.from_numpy(np.array([np.diag([Crx, Cry, Crz]) 
                                                for _ in range(self.batch_size)])).double().to(self.device)
        
        k = self.batched_params.k_m[:,0]/self.batched_params.k_eta[:,0]
        alloc_mats = []
        for i in range(self.batch_size):
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key] for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(device=self.device) # shape (num_rotors, 3)
            # Unit thrust directions for each thruster
            thrust_dirs = torch.from_numpy(self.batched_params.rotor_dir_np[i]).double().to(self.device)

            # Compute moment arms (r × F)
            torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T  # (3,8)
            # This robot's reaction-torque ratio (k[:, 0] gave every robot's, mixing a batch's rotors)
            torque_arms[-1, :4] = k[i, 0] * torch.tensor([1, -1, 1, -1], device=self.device)


            # Build full 6x8 allocation matrix
            A = torch.vstack([
                thrust_dirs.T,   # Fx,Fy,Fz rows
                torque_arms,     # Mx,My,Mz rows
            ])


            alloc_mats.append(A)

        # Use pseudo-inverse for generality
        # One allocation per rotor set: flight() drives only the air rotors and dive() only the water thrusters, so each
        # inverts its own 6x4 block. (The pseudo-inverse of the whole 6x8 matrix, split afterwards, shared every
        # demanded force between both sets: each set alone produced only its share, e.g. a third of the dive force,
        # which left the Hydrone held at the surface by its net buoyancy.)
        A = torch.stack(alloc_mats).to(self.device)                                         # (B, 6, 8)
        self.batched_params.TM_to_f_air = torch.linalg.pinv(A[:, :, :4])                      # (B, 4, 6)
        self.batched_params.TM_to_f_water = torch.linalg.pinv(A[:, :, 4:])                    # (B, 4, 6)


        #Control - air
        self.pid_air_roll = BatchedPIDController(params['kp_air_roll'], 
                                                 params['ki_air_roll'], 
                                                 params['kd_air_roll'], self.batch_size, self.device)
        
        self.pid_air_pitch = BatchedPIDController(params['kp_air_pitch'], 
                                                  params['ki_air_pitch'], 
                                                  params['kd_air_pitch'], self.batch_size, self.device)
        
        self.pid_air_yaw = BatchedPIDController(params['kp_air_yaw'], 
                                                params['ki_air_yaw'], 
                                                params['kd_air_yaw'], self.batch_size, self.device)
        
        self.pid_air_vx = BatchedPIDController(params['kp_air_vx'], 
                                               params['ki_air_vx'], 
                                               params['kd_air_vx'], self.batch_size, self.device)
        
        self.pid_air_vy = BatchedPIDController(params['kp_air_vy'], 
                                               params['ki_air_vy'], 
                                               params['kd_air_vy'], self.batch_size, self.device)
        
        self.pid_air_vz = BatchedPIDController(params['kp_air_vz'], 
                                               params['ki_air_vz'], 
                                               params['kd_air_vz'], self.batch_size, self.device)
        
        self.pid_air_x = BatchedPIDController(params['kp_air_x'], 
                                             params['ki_air_x'], 
                                             params['kd_air_x'], self.batch_size, self.device)
        
        self.pid_air_y = BatchedPIDController(params['kp_air_y'], 
                                             params['ki_air_y'], 
                                             params['kd_air_y'], self.batch_size, self.device)
        
        self.pid_air_z = BatchedPIDController(params['kp_air_z'], 
                                             params['ki_air_z'], 
                                             params['kd_air_z'], self.batch_size, self.device)

        #Control - water
        self.pid_water_pitch = BatchedPIDController(params['kp_water_pitch'], 
                                                  params['ki_water_pitch'], 
                                                  params['kd_water_pitch'], self.batch_size, self.device)
        
        self.pid_water_yaw = BatchedPIDController(params['kp_water_yaw'], 
                                                params['ki_water_yaw'], 
                                                params['kd_water_yaw'], self.batch_size, self.device)
        
        self.batched_params.kp_water_vx = torch.from_numpy(np.array([params['kp_water_vx'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kp_water_vz = torch.from_numpy(np.array([params['kp_water_vz'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kp_water_pos = torch.from_numpy(np.array([params['kp_water_pos'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)

    def _setup_domain(self, h : Tensor) -> Tensor:
        """
        Computes the vehicles domain from its height above the water surface h (B,).
        1  - air
        0  - transition
        -1 - underwater
        """
        self.last_domain = self.domain.clone()
        self.action_domain = torch.where(h > 0.5 * np.sqrt(self.batched_params.Diam), 
                                  1, 
                                  torch.where(h < -0.5 * np.sqrt(self.batched_params.Diam),
                                 -1, 0)).unsqueeze(1)
        self.domain  = torch.where(h >=-0.028397707, 1, -1).unsqueeze(1)
        return self.action_domain
         
    def _apply_domain_constraints(self, air : Tensor, trans : Tensor, water : Tensor) -> Tensor:
        return torch.where(self.action_domain.squeeze(1) > 0, 
                           air, 
                           torch.where(self.action_domain.squeeze(1) < 0, 
                           water, trans))
    
    def _transition(self) -> Tensor:
        ntrans = self.action_domain.clone()
        ntrans[(self.domain == 1) & (self.action_domain == 0)] = 1
        ntrans[(self.domain == -1) & (self.action_domain == 0)] = -1
        return ntrans

    def _mask_domain(self) -> tuple[Tensor, Tensor]:
        trans = self._transition()
        tair = (trans.squeeze(-1) == 1)
        twater = (trans.squeeze(-1) == -1)
        air = torch.logical_or((self.action_domain.squeeze(-1) == 1), tair) # (B,)
        water = torch.logical_or((self.action_domain.squeeze(-1) == -1), twater)

        return air, water

    def _compute_external_forces(self, s):
        x = s['x'][self.idxs]
        v = s['v'][self.idxs]
        w = s['w'][self.idxs]

        # Height above the water surface under the vehicle (DynamicsSensor water surface); +inf over no water.
        h = self._height_above_water(x)
        self._setup_domain(h)

        self.domain_rho = self._apply_domain_constraints(self.batched_params.rho[0], 
                                             1001.5 / (1 + torch.exp(90 * h)), 
                                             self.batched_params.rho[1]).view(-1,
                                                                                 1,1).permute(1, 0, 2).squeeze(0)
        
        Cp = self.batched_params.Cp
        Cr = self.batched_params.Cr

        # Buoyancy (Archimedes): rho g V_submerged upward at the submerged centroid, from the hull voxels under the
        # water surface; body frame, about the center of mass.
        R = roma.unitquat_to_rotmat(s['q'][self.idxs]).double()
        F_buoyancy, M_buoyancy, submerged = self._buoyancy(x.double(), R)
        self.submerged_fraction = submerged / self.batched_params.volume

        #Forces
        W = self.batched_params.weight
        D = (R @ F_buoyancy.unsqueeze(-1)).squeeze(-1)  # world frame, like the other forces summed here
        C = -0.5 * self.domain_rho.view(-1,1) * torch.einsum('bij,bj->bi', Cp, v *  torch.abs(v))
        F = torch.cross(w, self.batched_params.mass * v, dim=1)
        FtotB = W + D + C + F
        
        #Moments
        M1 = -torch.cross(w, torch.einsum('bij,bj->bi', self.batched_params.I, w), dim=1)
        M2 = -0.5 * self.domain_rho.view(-1,1) * torch.einsum('bij,bj->bi', Cr, 
                                                  w *  torch.abs(w) * self.batched_params.L)
        MtotB = M1 + M2 + M_buoyancy

        #Added inertia
        # self.batched_params.I = self.batched_params.I + \
        #                         ((2/5) * added_mass * self.batched_params.rc**2.0).view(-1,1,1) 

        added_inertia = (((2/5) * self.added_mass * self.batched_params.rc**2.0).view(-1,1,1)).squeeze(-1).squeeze(-1)
        # self.batched_params.I[..., self.batched_params.diag, self.batched_params.diag] += added_inertia

        # added_inertia = ((2/5) * added_mass * self.batched_params.rc**2.0).view(-1,1,1)
       
        return (FtotB, MtotB, added_inertia)

    def _compute_body_wrench(self, rotor_speeds: Tensor, x: Tensor, R: Tensor) -> tuple:
        """Body-frame force and moment (B, 3) of the 8 rotors, on the geometry the allocation (TM_to_f_air /
        TM_to_f_water) inverts: each rotor pushes T_i along its direction at its position (moment r_i x F_i), and the
        air rotors add their reaction torque about body z (k_m w^2, spins [1, -1, 1, -1]).

        Air rotors: T = k_eta_air w^2; water thrusters: T = k_eta_water w |w|. Each acts by its immersion against the
        water surface (DynamicsSensor): the air rotors only out of the water, the thrusters only in it.

        Replaces hand-written moments that had the opposite roll sign, used L = 0.33 m for the 0.25 m rotor arms and,
        underwater, k_m w^2 (reaction-torque constant) as a lever moment, ~70x too weak.
        """
        idxs = self.idxs
        speeds = rotor_speeds.double()
        k_eta = self.batched_params.k_eta[idxs].double().reshape(len(idxs), 2)   # [air, water] per robot
        k_m = self.batched_params.k_m[idxs].double().reshape(len(idxs), 2)
        thrust = torch.cat([k_eta[:, 0:1] * speeds[:, :4] ** 2,
                            k_eta[:, 1:2] * speeds[:, 4:] * torch.abs(speeds[:, 4:])], dim=-1)          # (B, 8)
        rotor_world = x.double().unsqueeze(1) + self._rotor_positions @ R.transpose(1, 2)          # (B, 8, 3)
        in_water = self._immersion(-self._height_above_water(rotor_world))                         # (B, 8)
        active = torch.cat([1.0 - in_water[:, :4], in_water[:, 4:]], dim=-1)
        thrust = thrust * active
        forces = thrust.unsqueeze(-1) * self._rotor_directions                                      # (B, 8, 3)
        FtotB = forces.sum(dim=1)
        MtotB = torch.cross(self._rotor_positions.expand_as(forces), forces, dim=-1).sum(dim=1)
        reaction = k_m[:, 0:1] * speeds[:, :4] ** 2 * self._air_spins * active[:, :4]
        MtotB = MtotB + torch.stack([torch.zeros_like(FtotB[:, 0]), torch.zeros_like(FtotB[:, 0]),
                                     reaction.sum(dim=-1)], dim=-1)
        return FtotB, MtotB

    def _s_dot_fn(self, s, cmd_rotor_speeds):
        state = self.unpack_state(s, self.idxs, self.batch_size)

        R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()
        rpy = roma.rotmat_to_euler('xyz', R)

        x_dot = state['v'][self.idxs]
        q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])

        F, M, added_inertia = self._compute_external_forces(state)
        
        #compute body wrench
        (FtotB, MtotB) = self._compute_body_wrench(cmd_rotor_speeds, state['x'][self.idxs], R)

        Ftot = (R @ FtotB.unsqueeze(-1)).squeeze(-1) + F[self.idxs]
        v_dot = Ftot /  (self.batched_params.mass + self.added_mass).squeeze(0)
        # w_dot = torch.linalg.solve(self.batched_params.I[self.idxs] + added_inertia, MtotB[self.idxs] + M[self.idxs])

        I = self.batched_params.I.clone()
        # One added inertia per robot (B, 1), the same on the three axes; (B,) broadcast over the axes instead.
        I[..., self.batched_params.diag, self.batched_params.diag] += added_inertia.reshape(-1, 1)
        w_dot = torch.linalg.solve(I, MtotB[self.idxs] + M[self.idxs])
    
        # Pack into vector of derivatives.
        s_dot = torch.zeros((len(self.idxs), 13 + cmd_rotor_speeds.shape[1],), device=self.device)
        s_dot[:, 0:3] = x_dot
        s_dot[:, 3:6] = v_dot
        s_dot[:, 6:10] = q_dot
        s_dot[:, 10:13] = w_dot.squeeze(-1)

        return s_dot
    
    def _hydrostatic_feedforward(self, x: Tensor, R: Tensor) -> Tensor:
        """Body-frame force (B, 3) cancelling gravity and buoyancy at the current pose (the dive controllers'
        thrusters hold it; without it their P loops settled ~1 m/s off the vertical setpoint, +3.9 N net lift)."""
        F_buoyancy, _, _ = self._buoyancy(x.double(), R.double())
        weight_body = (R.double().transpose(1, 2) @ self.batched_params.weight[self.idxs].double().unsqueeze(-1)).squeeze(-1)
        return -(F_buoyancy + weight_body)

    @staticmethod
    def _heading_to_body(vector: Tensor, R: Tensor, yaw: Tensor) -> Tensor:
        """Vectors (B, 3) in the heading frame (x along the heading, z up) into the body frame (R: body -> world)."""
        c, s = torch.cos(yaw), torch.sin(yaw)
        world = torch.stack([c * vector[:, 0] - s * vector[:, 1], s * vector[:, 0] + c * vector[:, 1], vector[:, 2]], dim=-1)
        return (R.double().transpose(1, 2) @ world.double().unsqueeze(-1)).squeeze(-1)

    @staticmethod
    def _to_heading_frame(xy: Tensor, yaw: Tensor) -> Tensor:
        """World-frame horizontal vectors (B, 2) expressed in the frame of the heading yaw (rad, B)."""
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]], dim=-1)

    def get_cmd_motor_speeds(self, state: dict, control : dict):  #control needs to be a tensor
        """
        Converts high-level control commands into motor speed commands for a hybrid
        air/water vehicle (e.g., aquatic drone), supporting multiple control abstraction levels.

        The method implements a cascaded control architecture:
        - Position → Velocity → Acceleration → Attitude → Forces/Moments → Motor Speeds

        Supported control abstractions (self._control_abstraction):
        - 'cmd_motor_speeds' : Pass-through; motor speeds are commanded directly.
        - 'cmd_vel'          : Track a desired 3D velocity [vx, vy, vz] in both air and water.
        - 'cmd_vel_yaw'      : Track a desired 3D velocity + yaw offset [vx, vy, vz, yaw_deg].
        - 'cmd_pos_yaw'      : Track a desired 3D position + yaw [x, y, z, yaw_deg].

        For air ('flight') and water ('dive') modes, separate inner-loop PID controllers
        handle attitude stabilization, while outer-loop PIDs handle velocity or position tracking.
        The resulting desired forces and moments are mapped through the vehicle's thrust-moment
        allocation matrix (TM_to_f_air / TM_to_f_water) to individual thruster forces, which
        are then converted to motor speeds via the motor thrust coefficient k_eta.

        Args:
            state (dict): Current vehicle state for the active batch, containing:
                            - 'x' : positions         (N, 3)
                            - 'v' : linear velocities  (N, 3)
                            - 'q' : unit quaternions   (N, 4)
            control (dict): High-level control commands, containing:
                            - 'cmd_ctrl': command tensor (N, D), where D depends on the
                            abstraction (3 for cmd_vel, 4 for cmd_vel_yaw / cmd_pos_yaw).

        Returns:
            torch.Tensor: Concatenated air and water motor speed commands of shape (N, M),
                        where M is the total number of air + water motors/thrusters.
                        For 'cmd_motor_speeds', returns the raw command directly.
        """

        # --- Extract batched state for the active robot indices ---
        x = state['x'][self.idxs]       # Position       (B, 3)
        v = state['v'][self.idxs]       # Linear velocity (B, 3)
        q = state['q'][self.idxs]       # Quaternion      (B, 4)
        goal = control['cmd_ctrl'][self.idxs]  # High-level command (B, D)

        # Convert quaternion → rotation matrix → roll/pitch/yaw (intrinsic XYZ Euler angles)
        R = roma.unitquat_to_rotmat(q).double()
        rpy = roma.rotmat_to_euler('xyz', R)  # (B, 3): [roll, pitch, yaw]

        # --- Pass-through: motor speeds are commanded directly, no conversion needed ---
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]
       
        # =========================================================================
        # cmd_vel: Track a desired 3D body velocity [vx, vy, vz]
        # =========================================================================
        if self._control_abstraction == 'cmd_vel':

            def flight():
                """
                Air-mode velocity controller.
                Outer loop: velocity PID → desired acceleration.
                Inner loop: acceleration → desired roll/pitch, then attitude PID → moments.
                Yaw setpoint tracks the vehicle's current heading (no yaw change commanded).
                """
                # Set velocity setpoints from goal [vx, vy, vz]
                self.pid_air_vx.update_setpoints(goal[:, 0])
                self.pid_air_vy.update_setpoints(goal[:, 1])
                self.pid_air_vz.update_setpoints(goal[:, 2])

                # Compute desired acceleration from velocity error; add gravity feedforward on Z
                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, 0] = self.pid_air_vx.compute(v[:, 0], self.dt, self.idxs)
                a_des[:, 1] = self.pid_air_vy.compute(v[:, 1], self.dt, self.idxs)
                a_des[:, 2] = self.pid_air_vz.compute(v[:, 2], self.dt, self.idxs) + self.batched_params.g

                # Normalize lateral accelerations by g to obtain tilt angles
                acc = a_des.clone()
                acc[:, :2] = acc[:, :2] / self.batched_params.g
                # World-frame horizontal acceleration into the heading frame: pitch acts along the nose, roll across it
                # (used directly, x -> pitch and y -> roll only held at zero yaw; any heading mixed them up)
                acc[:, :2] = self._to_heading_frame(acc[:, :2], rpy[:, 2])

                # Desired roll and pitch from acceleration direction, clamped to safe angles
                roll  = torch.clamp(torch.atan2(-acc[:, 1], acc[:, 2]),   # +y acceleration needs a negative roll
                                    -self.sat_angle, self.sat_angle)
                pitch = torch.clamp(torch.atan2(acc[:, 0], torch.sqrt(acc[:, 1]**2 + acc[:, 2]**2)),
                                    -self.sat_angle, self.sat_angle)

                # Correct vertical thrust for vehicle tilt (project onto body Z-axis)
                a_des[:, 2] = a_des[:, 2] / (torch.cos(roll) * torch.cos(pitch))

                # Desired thrust force along body Z
                F_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                F_des[:, -1] = self.batched_params.mass * a_des[:, 2]

                # Yaw setpoint: maintain current heading (wrap to [-π, π])
                yaw = (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi

                # Update attitude setpoints and compute attitude PID outputs (desired moments)
                self.pid_air_roll.update_setpoints(roll)
                self.pid_air_pitch.update_setpoints(pitch)
                self.pid_air_yaw.update_setpoints(yaw)

                att_theta = self.pid_air_roll.compute(rpy[:, 0],  self.dt, self.idxs)  # Mx
                att_phi   = self.pid_air_pitch.compute(rpy[:, 1], self.dt, self.idxs)  # My
                att_psi   = self.pid_air_yaw.compute(
                                (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi, self.dt, self.idxs)  # Mz

                # Assemble desired moment vector
                M_des = torch.zeros_like(F_des)
                M_des[:, 0] = att_theta
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                # Concatenate force and moment into a wrench vector [Fx, Fy, Fz, Mx, My, Mz]
                TM = torch.cat([F_des, M_des], dim=-1)  # (B, 6)

                # Map wrench to individual thruster forces via allocation matrix
                cmd_thruster_forces = (self.batched_params.TM_to_f_air[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                # Convert thruster forces to motor speeds: ω = sqrt(F / k_eta)
                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 0]
                cmd_motor_speeds = torch.sqrt(torch.abs(cmd_motor_speeds))

                # Sanitize: replace NaN / ±Inf with 0
                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            def dive():
                """
                Water-mode velocity controller.
                Simplified 2-DOF control: forward (X) and vertical (Z) velocity tracking.
                Pitch is derived from the desired acceleration direction; yaw tracks current heading.
                Motors are bidirectional, so signed sqrt is used for speed conversion.
                """
                # Velocity error in all 3 axes
                # Forward (x) and lateral in the heading frame: the forward thrusters push along the heading, not world x
                v_h = v.clone(); v_h[:, :2] = self._to_heading_frame(v[:, :2], rpy[:, 2])
                goal_h = goal[:, :3].clone(); goal_h[:, :2] = self._to_heading_frame(goal[:, :2], rpy[:, 2])
                v_err = goal_h - v_h

                # Proportional acceleration commands for forward (X) and vertical (Z) axes only
                # (lateral Y motion is not actuated underwater in this model)
                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, 0] =  self.batched_params.kp_water_vx[self.idxs, 0] * v_err[:, 0]   # was negated to match the former plant's inverted surge
                a_des[:, 2] =  self.batched_params.kp_water_vz[self.idxs, 0] * v_err[:, 2]

                # Desired force from Newton's second law
                F_des = self.batched_params.mass * a_des
                # a_des is forward/lateral in the heading frame and vertical in the world: into the body frame, where
                # the thrusters act (used as is, a tilted or inverted vehicle pushed the wrong way)
                F_des = self._heading_to_body(F_des, R, rpy[:, 2])
                # Net buoyancy fed forward (the plant's own hydrostatics at the current pose)
                F_des = F_des + self._hydrostatic_feedforward(x, R)

                # Desired pitch derived from forward/vertical acceleration ratio
                # Level underwater: the forward thrusters push along the heading directly. (The target used to be
                # atan2(a_x, |a_z|), up to 90 deg for a purely forward demand: with no roll control and CG at CB the
                # Hydrone tumbled.)
                pitch = torch.zeros_like(a_des[:, 0])

                # Yaw setpoint: maintain current heading (wrap to [-π, π])
                self.pid_water_pitch.update_setpoints(pitch)
                self.pid_water_yaw.update_setpoints((rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi)

                # Attitude PID → desired pitch and yaw moments
                att_phi = self.pid_water_pitch.compute(rpy[:, 1], self.dt, self.idxs)
                att_psi = self.pid_water_yaw.compute(
                            (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi, self.dt, self.idxs)

                # Assemble desired moment vector (only pitch and yaw are controlled underwater)
                M_des = torch.zeros_like(F_des)
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                # Concatenate force and moment into wrench vector [Fx, Fy, Fz, Mx, My, Mz]
                TM = torch.cat([F_des, M_des], dim=-1)  # (B, 6)

                # Map wrench to individual thruster forces via water allocation matrix
                cmd_thruster_forces = (self.batched_params.TM_to_f_water[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                # Bidirectional motor speed conversion: ω = sign(F) * sqrt(|F| / k_eta)
                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 1]
                cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

                # Sanitize: replace NaN / ±Inf with 0
                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            air   = flight()
            water = dive()

            # Concatenate air and water motor commands into a single output tensor
            return torch.cat((air, water), dim=1)
        

        # =========================================================================
        # cmd_vel_yaw: Track desired 3D velocity + yaw rate offset [vx, vy, vz, yaw_deg]
        # Same cascade as cmd_vel but yaw setpoint is offset by the commanded yaw angle.
        # =========================================================================
        if self._control_abstraction == 'cmd_vel_yaw':

            def flight():
                """
                Air-mode velocity + yaw controller.
                Identical to cmd_vel flight(), except the yaw setpoint is shifted by
                the commanded yaw offset (goal[:, 3], in degrees).
                """
                self.pid_air_vx.update_setpoints(goal[:, 0])
                self.pid_air_vy.update_setpoints(goal[:, 1])
                self.pid_air_vz.update_setpoints(goal[:, 2])

                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, 0] = self.pid_air_vx.compute(v[:, 0], self.dt, self.idxs)
                a_des[:, 1] = self.pid_air_vy.compute(v[:, 1], self.dt, self.idxs)
                a_des[:, 2] = self.pid_air_vz.compute(v[:, 2], self.dt, self.idxs) + self.batched_params.g

                acc = a_des.clone()
                acc[:, :2] = acc[:, :2] / self.batched_params.g
                # World-frame horizontal acceleration into the heading frame: pitch acts along the nose, roll across it
                # (used directly, x -> pitch and y -> roll only held at zero yaw; any heading mixed them up)
                acc[:, :2] = self._to_heading_frame(acc[:, :2], rpy[:, 2])

                roll  = torch.clamp(torch.atan2(-acc[:, 1], acc[:, 2]),   # +y acceleration needs a negative roll
                                    -self.sat_angle, self.sat_angle)
                pitch = torch.clamp(torch.atan2(acc[:, 0], torch.sqrt(acc[:, 1]**2 + acc[:, 2]**2)),
                                    -self.sat_angle, self.sat_angle)

                a_des[:, 2] = a_des[:, 2] / (torch.cos(roll) * torch.cos(pitch))

                F_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                F_des[:, -1] = self.batched_params.mass * a_des[:, 2]

                # Yaw setpoint: current heading offset by commanded yaw (degrees → radians)
                yaw = (rpy[:, 2] + torch.deg2rad(goal[:, 3].double()) + np.pi) % (2 * np.pi) - np.pi

                self.pid_air_roll.update_setpoints(roll)
                self.pid_air_pitch.update_setpoints(pitch)
                self.pid_air_yaw.update_setpoints(yaw)

                att_theta = self.pid_air_roll.compute(rpy[:, 0],  self.dt, self.idxs)
                att_phi   = self.pid_air_pitch.compute(rpy[:, 1], self.dt, self.idxs)
                att_psi   = self.pid_air_yaw.compute(
                                (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi, self.dt, self.idxs)

                M_des = torch.zeros_like(F_des)
                M_des[:, 0] = att_theta
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                TM = torch.cat([F_des, M_des], dim=-1)
                cmd_thruster_forces = (self.batched_params.TM_to_f_air[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 0]
                cmd_motor_speeds = torch.sqrt(torch.abs(cmd_motor_speeds))

                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            def dive():
                """
                Water-mode velocity + yaw controller.
                Same as cmd_vel dive(), except the yaw setpoint is offset by the
                commanded yaw angle (goal[:, 3], in degrees).
                """
                # Forward (x) and lateral in the heading frame: the forward thrusters push along the heading, not world x
                v_h = v.clone(); v_h[:, :2] = self._to_heading_frame(v[:, :2], rpy[:, 2])
                goal_h = goal[:, :3].clone(); goal_h[:, :2] = self._to_heading_frame(goal[:, :2], rpy[:, 2])
                v_err = goal_h - v_h

                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, 0] =  self.batched_params.kp_water_vx[self.idxs, 0] * v_err[:, 0]   # was negated to match the former plant's inverted surge
                a_des[:, 2] =  self.batched_params.kp_water_vz[self.idxs, 0] * v_err[:, 2]

                F_des = self.batched_params.mass * a_des
                # a_des is forward/lateral in the heading frame and vertical in the world: into the body frame, where
                # the thrusters act (used as is, a tilted or inverted vehicle pushed the wrong way)
                F_des = self._heading_to_body(F_des, R, rpy[:, 2])
                # Net buoyancy fed forward (the plant's own hydrostatics at the current pose)
                F_des = F_des + self._hydrostatic_feedforward(x, R)

                # Level underwater: the forward thrusters push along the heading directly. (The target used to be
                # atan2(a_x, |a_z|), up to 90 deg for a purely forward demand: with no roll control and CG at CB the
                # Hydrone tumbled.)
                pitch = torch.zeros_like(a_des[:, 0])

                # Yaw setpoint: current heading offset by commanded yaw (degrees → radians)
                yaw = torch.deg2rad(goal[:, 3].double())
                self.pid_water_pitch.update_setpoints(pitch)
                self.pid_water_yaw.update_setpoints((rpy[:, 2] + yaw + np.pi) % (2 * np.pi) - np.pi)

                att_phi = self.pid_water_pitch.compute(rpy[:, 1], self.dt, self.idxs)
                att_psi = self.pid_water_yaw.compute(
                            (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi, self.dt, self.idxs)

                M_des = torch.zeros_like(F_des)
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                TM = torch.cat([F_des, M_des], dim=-1)
                cmd_thruster_forces = (self.batched_params.TM_to_f_water[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 1]
                cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            air   = flight()
            water = dive()
            return torch.cat((air, water), dim=1)
        

        # =========================================================================
        # cmd_pos_yaw: Track desired 3D position + absolute yaw [x, y, z, yaw_deg]
        # Outermost loop: position PID → velocity setpoints, then same cascade as cmd_vel_yaw.
        # =========================================================================
        if self._control_abstraction == 'cmd_pos_yaw':

            def flight():
                """
                Air-mode position + yaw controller.
                Three-stage cascade: position PID → velocity PID → acceleration →
                attitude PID → forces/moments → motor speeds.
                Velocity outputs are clamped to safe limits before feeding the inner loop.
                """
                # Outer position loop: position error → desired velocity
                self.pid_air_x.update_setpoints(goal[:, 0])
                self.pid_air_y.update_setpoints(goal[:, 1])
                self.pid_air_z.update_setpoints(goal[:, 2])

                v_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                v_des[:, 0] = self.pid_air_x.compute(x[:, 0], self.dt, self.idxs)
                v_des[:, 1] = self.pid_air_y.compute(x[:, 1], self.dt, self.idxs)
                v_des[:, 2] = self.pid_air_z.compute(x[:, 2], self.dt, self.idxs)

                # Clamp desired velocities to physical limits before passing to velocity loop
                v_des[:, :2] = torch.clamp(v_des[:, :2], -2.5, 2.5)  # Horizontal max ±2.5 m/s
                v_des[:, 2]  = torch.clamp(v_des[:, 2],  -1.5, 1.5)  # Vertical max   ±1.5 m/s

                # Inner velocity loop: velocity error → desired acceleration
                self.pid_air_vx.update_setpoints(v_des[:, 0])
                self.pid_air_vy.update_setpoints(v_des[:, 1])
                self.pid_air_vz.update_setpoints(v_des[:, 2])

                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, 0] = self.pid_air_vx.compute(v[:, 0], self.dt, self.idxs)
                a_des[:, 1] = self.pid_air_vy.compute(v[:, 1], self.dt, self.idxs)
                a_des[:, 2] = self.pid_air_vz.compute(v[:, 2], self.dt, self.idxs) + self.batched_params.g

                acc = a_des.clone()
                acc[:, :2] = acc[:, :2] / self.batched_params.g
                # World-frame horizontal acceleration into the heading frame: pitch acts along the nose, roll across it
                # (used directly, x -> pitch and y -> roll only held at zero yaw; any heading mixed them up)
                acc[:, :2] = self._to_heading_frame(acc[:, :2], rpy[:, 2])

                roll  = torch.clamp(torch.atan2(-acc[:, 1], acc[:, 2]),   # +y acceleration needs a negative roll
                                    -self.sat_angle, self.sat_angle)
                pitch = torch.clamp(torch.atan2(acc[:, 0], torch.sqrt(acc[:, 1]**2 + acc[:, 2]**2)),
                                    -self.sat_angle, self.sat_angle)

                a_des[:, 2] = a_des[:, 2] / (torch.cos(roll) * torch.cos(pitch))

                F_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                F_des[:, -1] = self.batched_params.mass * a_des[:, 2]

                # Absolute yaw setpoint from goal (degrees → radians, wrapped to [-π, π])
                yaw = (torch.deg2rad(goal[:, 3].double()) + np.pi) % (2 * np.pi) - np.pi

                self.pid_air_roll.update_setpoints(roll)
                self.pid_air_pitch.update_setpoints(pitch)
                self.pid_air_yaw.update_setpoints(yaw)

                att_theta = self.pid_air_roll.compute(rpy[:, 0],  self.dt, self.idxs)
                att_phi   = self.pid_air_pitch.compute(rpy[:, 1], self.dt, self.idxs)
                att_psi   = self.pid_air_yaw.compute(
                                (rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi, self.dt, self.idxs)

                M_des = torch.zeros_like(F_des)
                M_des[:, 0] = att_theta
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                TM = torch.cat([F_des, M_des], dim=-1)
                cmd_thruster_forces = (self.batched_params.TM_to_f_air[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 0]
                cmd_motor_speeds = torch.sqrt(torch.abs(cmd_motor_speeds))

                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            def dive():
                """
                Water-mode position controller, with the yaw used to compensate the uncontrolled roll.

                Underwater the thrusters only push along body x (forward pair) and body z (vertical pair): there is
                no sway thruster and none of them gives a roll moment, and with the CG at the CB nothing rights the
                roll. Once rolled, the vertical pair (holding depth and the net buoyancy) also pushes along the
                missing body-y direction and the Hydrone drifted (~0.4 m/s). So the yaw is chosen to put the force
                the position loop needs, F (world), in the body x-z plane, where the thrusters can produce all of it:
                with roll phi at level pitch, body y = (-sin psi cos phi, cos psi cos phi, sin phi) and F . y_b = 0
                gives  psi = beta + asin(F_z tan(phi) / |F_h|),  beta the direction of F's horizontal part. When the
                horizontal need is too small for that (asin saturates), the rolled vertical push is turned straight
                onto it. The commanded yaw offset (goal[:, 3]) is not used underwater: the yaw is taken by this.

                Position P -> world velocity (horizontal norm <= 1.5 m/s, vertical -1.5..3 m/s) -> P on the world
                velocity -> world force + net-buoyancy feed-forward -> body frame -> allocation; pitch held level.
                """
                err = goal[:, :3] - x
                v_des = self.batched_params.kp_water_pos[self.idxs] * err
                speed = torch.linalg.norm(v_des[:, :2], dim=-1, keepdim=True)
                v_des[:, :2] = v_des[:, :2] * torch.clamp(1.5 / speed.clamp_min(1e-9), max=1.0)
                v_des[:, 2] = torch.clamp(v_des[:, 2], -1.5, 3)

                v_err = v_des - v
                a_des = torch.zeros(self.batch_size, 3, device=self.device).double()
                a_des[:, :2] = self.batched_params.kp_water_vx[self.idxs] * v_err[:, :2]
                a_des[:, 2] = self.batched_params.kp_water_vz[self.idxs, 0] * v_err[:, 2]

                # Force needed (world): the loop's plus the net buoyancy (the plant's hydrostatics at this pose)
                F_world = self.batched_params.mass * a_des + \
                    (R.double() @ self._hydrostatic_feedforward(x, R).unsqueeze(-1)).squeeze(-1)

                # Yaw putting F in the body x-z plane
                roll = torch.atan2(R[:, 2, 1], R[:, 2, 2])
                F_h = torch.linalg.norm(F_world[:, :2], dim=-1)
                beta = torch.atan2(F_world[:, 1], F_world[:, 0])
                ratio = F_world[:, 2] * torch.tan(torch.clamp(roll, -1.3, 1.3)) / F_h.clamp_min(1e-9)
                yaw_target = beta + torch.asin(torch.clamp(ratio, -1.0, 1.0))
                # No horizontal need at all: keep the heading
                held = getattr(self, '_dive_held_yaw', None)
                if held is None or held.shape != yaw_target.shape:
                    held = rpy[:, 2].clone()
                needs = F_h > self.DIVE_HORIZONTAL_FORCE_MIN
                yaw = torch.where(needs, yaw_target, held)
                self._dive_held_yaw = torch.where(needs, rpy[:, 2], held)

                F_des = (R.double().transpose(1, 2) @ F_world.unsqueeze(-1)).squeeze(-1)
                # The body-y part cannot be produced (no sway thruster). Depth first: the body-z push is set so the
                # world-vertical force is exact with the body-x push the allocation will give (dropping body y alone
                # lost the cos(roll) share of it: the Hydrone held -4.4 m for -5).
                F_des[:, 1] = 0.0
                Rd = R.double()
                F_des[:, 2] = (F_world[:, 2] - Rd[:, 2, 0] * F_des[:, 0]) / Rd[:, 2, 2].clamp(min=0.2)

                # Level underwater: the forward thrusters push along the heading directly. (The target used to be
                # atan2(a_x, |a_z|), up to 90 deg for a purely forward demand, and the Hydrone tumbled.)
                pitch = torch.zeros_like(a_des[:, 0])

                self.pid_water_pitch.update_setpoints(pitch)
                self.pid_water_yaw.update_setpoints((yaw + np.pi) % (2 * np.pi) - np.pi)

                att_phi = self.pid_water_pitch.compute(rpy[:, 1], self.dt, self.idxs)
                # Yaw error wrapped to [-pi, pi]: the PID sees the error to the target, not the raw angles
                yaw_error = (yaw - rpy[:, 2] + np.pi) % (2 * np.pi) - np.pi
                self.pid_water_yaw.update_setpoints(yaw_error)
                att_psi = self.pid_water_yaw.compute(torch.zeros_like(yaw_error), self.dt, self.idxs)

                M_des = torch.zeros_like(F_des)
                M_des[:, 1] = att_phi
                M_des[:, 2] = att_psi

                TM = torch.cat([F_des, M_des], dim=-1)
                cmd_thruster_forces = (self.batched_params.TM_to_f_water[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

                cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[:, 1]
                cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

                return torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            air   = flight()
            water = dive()
            return torch.cat((air, water), dim=1)


    

    @VehicleModel.map_states   
    def step(self, state : dict, control : dict):
        """
        Integrate dynamics forward from state given constant control for time t_step.
        Params:
            - state: dictionary containing keys ['x', 'v', 'q', 'w', rotor_speeds'], and values which
              are pytorch tensors with dtype double and which have a batch dimension
            - control: dictionary with keys depending on the chosen control mode. Values are torch 
            tensors, again with dtype double and with a batch dimension equal to the number of agents.
        """

        if self.control_abstraction == 'accel':
            return control['cmd_ctrl'][self.idxs].cpu().tolist()
        self.added_mass = self._apply_domain_constraints(self.batched_params.added_mass[0], 
                                                    self.batched_params.added_mass[0], 
                                                    self.batched_params.added_mass[1]).unsqueeze(-1)
        # The DynamicsSensor reports w in the world frame and the engine applies w_dot in the world frame; the model
        # works in the body frame (as VehicleModel.step does for angular_velocity_in_world_frame models).
        R_body_to_world = roma.unitquat_to_rotmat(state['q']).double()
        state = {**state, 'w': torch.einsum('bji,bj->bi', R_body_to_world, state['w'].double())}
        cmd_ctrl = self.get_cmd_motor_speeds(state, control)
        cmd_ctrl[:, :4] = torch.clip(
            cmd_ctrl[:, :4],
            self.batched_params.rotor_speed_min[:, 0],
            self.batched_params.rotor_speed_max[:, 0]
        )

        cmd_ctrl[:, 4:8] = torch.clip(
            cmd_ctrl[:, 4:8],
            self.batched_params.rotor_speed_min[:, 1],
            self.batched_params.rotor_speed_max[:, 1]
        )

        # Integrate the plant over the engine tick in SUBSTEPS (semi-implicit Euler, body-frame rotation) and return the
        # constant accelerations that reproduce the integrated velocities, as MarinePlant does: one explicit step of the
        # instantaneous acceleration over the 33 ms tick is unstable for the stiff rotational water drag (time constant
        # ~3 ms at a few rad/s; the yaw rate went 6 -> -57 rad/s in one tick).
        dt = float(self.dt) if self.dt else 1 / 30
        substeps = self._substeps(dt)
        h = dt / substeps
        xs, vs = state['x'].double().clone(), state['v'].double().clone()
        qs, ws = state['q'].double().clone(), state['w'].double().clone()
        for _ in range(substeps):
            s = self.pack_state({'x': xs, 'v': vs, 'q': qs, 'w': ws}, self.batch_size, self.device)
            s_dot = self._s_dot_fn(s, cmd_ctrl).double()
            vs = vs + s_dot[:, 3:6] * h
            ws = ws + s_dot[:, 10:13] * h
            xs = xs + vs * h
            qs = roma.quat_product(qs, roma.rotvec_to_unitquat(ws * h))
            qs = qs / torch.linalg.norm(qs, dim=-1, keepdim=True)
        w_world_start = torch.einsum('bij,bj->bi', R_body_to_world, state['w'].double())
        w_world_end = torch.einsum('bij,bj->bi', roma.unitquat_to_rotmat(qs).double(), ws)
        v_dot = (vs - state['v'].double()) / dt
        w_dot = (w_world_end - w_world_start) / dt
        return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()
        















