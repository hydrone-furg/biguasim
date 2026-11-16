"""Plant shared by the thruster-driven marine vehicles (BlueROV2, BlueROV2Heavy, BlueBoat).

Fossen's 6-DOF model about the reference point O, the engine's center of mass (where the DynamicsSensor reports the
state and where the engine applies the accelerations this model returns), with velocities relative to the water:

    M_RB nu_dot + C_RB(nu) nu + M_A nu_r_dot + C_A(nu_r) nu_r = tau_gravity + tau_buoyancy + tau_drag + tau_damping
                                                                + tau_thrust,     nu_r = nu - [R^T v_current; 0]

  - Rigid body: mass m at the center of gravity r_g (from O); inertia given about the CG.
  - Buoyancy (Archimedes): rho g V_sub upward, at the submerged centroid. The hull is voxelized (2 cm) once;
    every step each voxel is compared with the water surface (the DynamicsSensor's ExportWaterSurface block, waves
    included, or a flat water_level),
    so partial immersion, the righting moment and waves come out of the geometry. The voxels are scaled to the
    vehicle's specified displaced volume (`volume`), or for a surface vessel calibrated to its specified draft.
  - Drag: quadratic panel drag on the collision hull (geometry.panel_drag); panels below the surface see water
    and the current, panels above see air and the wind.
  - Linear damping and added mass: diagonal (`linear_damping`, `added_mass`) at the hull centroid (Fossen's
    vehicle center), moved to O with H(r_c), scaled by the submerged fraction.
  - Thrust: T_i = k_eta w_i |w_i| along each thruster's direction at its position (r x F), only while the
    thruster is under water.

The engine adds nothing (no gravity, damping or buoyancy): this model computes every force.
"""
import math
import numpy as np
import roma
import torch
from scipy import ndimage
from torch import Tensor

from biguasim.dynamics import geometry as geo
from biguasim.dynamics.base_model import DYNAMICS_ITEMS, WATERLINE_BAND

# Hydrostatics voxel edge: 2 cm keeps a 0.1 m^3 hull to ~12k voxels per step.
HYDROSTATIC_VOXEL = 0.02


def _skew(v: Tensor) -> Tensor:
    """S(v), shape (..., 3, 3), with S(v) w = v x w."""
    zero = torch.zeros_like(v[..., 0])
    return torch.stack([torch.stack([zero, -v[..., 2], v[..., 1]], -1),
                        torch.stack([v[..., 2], zero, -v[..., 0]], -1),
                        torch.stack([-v[..., 1], v[..., 0], zero], -1)], -2)


def _box_geometry(length: float, width: float, height: float) -> dict:
    """A closed box centered on the origin, as a stand-in until the engine exports the collision mesh."""
    corners = np.array([[x, y, z] for x in (-length / 2, length / 2) for y in (-width / 2, width / 2)
                        for z in (-height / 2, height / 2)])
    faces = [(0, 2, 6, 4), (1, 5, 7, 3), (0, 4, 5, 1), (2, 3, 7, 6), (0, 1, 3, 2), (4, 6, 7, 5)]
    triangles = np.array([t for a, b, c, d in faces for t in ((a, b, c), (a, c, d))])
    return {"vertices": corners, "triangles": triangles, "center_of_mass": np.zeros(3), "mass": 0.0}


def _solid_voxel_centers(vertices: np.ndarray, triangles: np.ndarray, size: float) -> np.ndarray:
    """Centers (N, 3) of the voxels of a mesh's solid (surface voxels, gaps closed, interior filled)."""
    origin = vertices.min(axis=0) - 2 * size
    shape = np.ceil((vertices.max(axis=0) + 2 * size - origin) / size).astype(int) + 1
    shell = np.zeros(shape, dtype=bool)
    points = geo._surface_points(vertices[triangles[:, 0]], vertices[triangles[:, 1]], vertices[triangles[:, 2]],
                                 spacing=size / 2)
    index = np.floor((points - origin) / size).astype(int)
    shell[index[:, 0], index[:, 1], index[:, 2]] = True
    solid = ndimage.binary_fill_holes(ndimage.binary_closing(shell) | shell)
    return (np.argwhere(solid) + 0.5) * size + origin


class WaterHydrostatics:
    """Archimedes buoyancy of a voxelized hull against the water surface (VehicleModel._surface_height), shared by
    the marine models.

    Call ``_init_water`` in ``__init__``. The model sets ``self._voxels`` (B, P, 3), the hull voxel centers from its
    reference point in the body frame, and ``self._voxel_volume`` (B, P), the volume each voxel stands for.
    Parameters read: rho (water), optional water_level (flat surface height, only when the DynamicsSensor sends no
    water surface; defaults to 0 for these water-only vehicles).
    """

    DEFAULT_WATER_LEVEL = 0.0

    def _init_water(self, params: dict) -> None:
        self.rho_water = float(params["rho"])
        # Water velocity in the world frame (B, 3), m/s: set it directly on the model.
        self.current_velocity = torch.zeros(self.batch_size, 3, device=self.device).double()

    def _substeps(self, dt: float) -> int:
        """Substeps for a tick of dt seconds: SUBSTEPS up to 30 Hz, fewer at higher rates (ArduPilot runs the
        physics at 200-400 Hz), as many as keep each substep no longer than at 30 Hz. Same count as before at 30 Hz
        and below; at 200 Hz 2 instead of 10, each still shorter than at 30 Hz."""
        return min(self.SUBSTEPS, max(1, math.ceil(dt * 30 * self.SUBSTEPS - 1e-9)))

    def _buoyancy(self, x: Tensor, R: Tensor) -> tuple:
        """Archimedes force on the wet hull voxels, for reference-point positions x (B, 3) and attitudes R
        (body -> world): (force (B, 3) and moment about the reference point (B, 3), both in the body frame,
        submerged volume (B,))."""
        voxels = self._voxels[self.idxs]
        world = x.unsqueeze(1) + voxels @ R.transpose(1, 2)                                  # (B, P, 3)
        wet = self._immersion(self._surface_height(world[..., :2]) - world[..., 2])
        wet_volume = wet * self._voxel_volume[self.idxs]                                     # (B, P)
        up_body = R[:, 2, :]                                                                 # R^T z, (B, 3)
        lift = self.rho_water * self.batched_params.g * up_body
        force = wet_volume.sum(dim=1, keepdim=True) * lift
        moment = torch.cross(torch.einsum("bp,bpk->bk", wet_volume, voxels), lift, dim=-1)
        return force, moment, wet_volume.sum(dim=1)


class MarinePlant(WaterHydrostatics):
    """Mixin replacing a marine vehicle model's plant (_s_dot_fn) with the Fossen model above.

    Use as the first base: ``class BlueROV2(MarinePlant, uuv.HexaCopterFiveDoF)``, and call ``_init_marine`` at the
    end of ``__init__``. The vehicle's controllers are left as they are.

    Parameters read (vehicle `_params`): mass, I (about the CG), rho (water), volume (displaced, m^3),
    rotor_pos (thruster positions from the CG, m), rotor_directions, k_eta, added_mass (6,), linear_damping (6,),
    drag_coefficients (3,), length/width/height (stand-in box), and the CG rule: cg_below_cb (m) or
    cg_above_keel (m). Optional: rho_air, water_level, draft_reference = (mass_kg, draft_m) to calibrate the
    displaced volume of a surface vessel instead of `volume`.
    """

    angular_velocity_in_world_frame = True
    EXPOSURE_BLEND_RAD = np.radians(25)
    # Plant substeps per engine tick (see step): 10 at 20 Hz is 5 ms, ~0.1 rad per step for the stiffest mode.
    SUBSTEPS = 10

    def _init_marine(self, params: dict) -> None:
        self._marine = params
        self._init_water(params)
        self.rho_air = float(params.get("rho_air", 1.225))
        # Air velocity in the world frame (B, 3), m/s: set it directly on the model.
        self.wind_velocity = torch.zeros(self.batch_size, 3, device=self.device).double()
        box = _box_geometry(params["length"], params["width"], params["height"])
        self.set_geometry([box] * self.batch_size)

    # ------------------------------------------------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------------------------------------------------

    def set_geometry(self, geometries: list[dict]) -> None:
        """Builds hydrostatics, drag panels and the rigid body about the engine's CoM from each collision mesh.

        Args:
            geometries: one DynamicsSensor.geometry dict per robot (vertices/triangles in the body frame).
        """
        if len(geometries) != self.batch_size:
            raise ValueError(f"Expected {self.batch_size} geometries, one per robot, got {len(geometries)}")
        self._speed_limit_cache = None   # depends on the drag panels
        p = self._marine
        device = self.device
        voxels, voxel_volumes, cgs, origins, panel_sets, centroids = [], [], [], [], [], []
        for geometry in geometries:
            vertices = np.asarray(geometry["vertices"], dtype=np.float64)
            triangles = np.asarray(geometry["triangles"], dtype=np.int64)
            centers = _solid_voxel_centers(vertices, triangles, HYDROSTATIC_VOXEL)
            origin = (np.asarray(geometry["center_of_mass"], dtype=np.float64) if geometry["mass"] > 0
                      else centers.mean(axis=0))
            volume_each = self._volume_per_voxel(centers, vertices)
            centroid = centers.mean(axis=0)
            if "cg_below_cb" in p:
                cg = centroid - np.array([0.0, 0.0, p["cg_below_cb"]])
            else:
                cg = np.array([centroid[0], centroid[1], vertices[:, 2].min() + p["cg_above_keel"]])
            voxels.append(centers - origin)
            centroids.append(centroid - origin)
            voxel_volumes.append(volume_each)
            cgs.append(cg - origin)
            origins.append(origin)
            panel_sets.append(geo.drag_panels(vertices, triangles, device=device))
            panel_sets[-1] = (panel_sets[-1], panel_sets[-1].centroids - origin)

        def stacked(values, pad=0.0):
            count = max(len(v) for v in values)
            padded = [np.concatenate([v, np.full((count - len(v),) + v.shape[1:], pad)]) for v in values]
            return torch.from_numpy(np.stack(padded)).double().to(device)

        self._voxels = stacked(voxels)                                                       # (B, P, 3) from O
        self._voxel_volume = stacked([np.full(len(v), dv) for v, dv in zip(voxels, voxel_volumes)])  # (B, P)
        self._displaced_volume = self._voxel_volume.sum(dim=1)                                # (B,)
        self._r_g = torch.from_numpy(np.stack(cgs)).double().to(device)                      # (B, 3) CG from O
        # Hydrodynamic center: the hull centroid, where the linear damping and added mass are defined (Fossen).
        self._r_c = torch.from_numpy(np.stack(centroids)).double().to(device)                # (B, 3) from O
        S_c = _skew(self._r_c)
        eye3 = torch.eye(3, dtype=torch.float64, device=device).expand(self.batch_size, 3, 3)
        # H(r_c): velocity at the hull centroid from the velocity at O, nu_c = H nu (Fossen 2011, eq. 3.29).
        self._H_c = torch.cat([torch.cat([eye3, -S_c], dim=-1),
                               torch.cat([torch.zeros_like(eye3), eye3], dim=-1)], dim=-2)
        self._drag_panels = {
            "areas": stacked([s.areas for s, _ in panel_sets]),
            "normals": stacked([s.normals for s, _ in panel_sets]),
            "exposure": stacked([s.exposure for s, _ in panel_sets]),
            "arms": stacked([arms for _, arms in panel_sets]),
            "directions": torch.from_numpy(panel_sets[0][0].directions).double().to(device),
        }
        self._drag_coefficients = torch.tensor(p["drag_coefficients"], dtype=torch.float64, device=device)

        # Thrusters: positions given from the CG, moved to O; unit directions.
        positions = np.stack([np.asarray(r, dtype=np.float64) for r in p["rotor_pos"].values()])
        directions = np.asarray(p["rotor_directions"], dtype=np.float64)
        directions = directions / np.linalg.norm(directions, axis=1, keepdims=True)
        self._thruster_arms = (torch.from_numpy(positions).double().to(device).unsqueeze(0)
                               + self._r_g.unsqueeze(1))                                     # (B, n, 3)
        self._thruster_directions = torch.from_numpy(directions).double().to(device)          # (n, 3)

        # Rigid-body mass matrix about O (Fossen): [[m I, -m S(r_g)], [m S(r_g), I_g - m S(r_g)^2]].
        m = float(p["mass"])
        S = _skew(self._r_g)
        I_g = torch.from_numpy(np.asarray(p["I"], dtype=np.float64)).to(device).expand(self.batch_size, 3, 3)
        eye = torch.eye(3, dtype=torch.float64, device=device).expand(self.batch_size, 3, 3)
        top = torch.cat([m * eye, -m * S], dim=-1)
        bottom = torch.cat([m * S, I_g - m * S @ S], dim=-1)
        self._M_RB = torch.cat([top, bottom], dim=-2)                                           # (B, 6, 6)
        self._added_mass = torch.tensor(p["added_mass"], dtype=torch.float64, device=device)   # (6,)
        self._linear_damping = torch.tensor(p["linear_damping"], dtype=torch.float64, device=device)
        self._mass = m

    def _volume_per_voxel(self, centers: np.ndarray, vertices: np.ndarray) -> float:
        """Volume each hydrostatic voxel stands for, so the hull displaces the specified volume."""
        p = self._marine
        if "draft_reference" in p:
            # Surface vessel: the hull below the reference draft displaces the reference mass.
            mass, draft = p["draft_reference"]
            keel = vertices[:, 2].min()
            below = np.count_nonzero(centers[:, 2] < keel + draft)
            return mass / self.rho_water / max(below, 1)
        return float(p["volume"]) / len(centers)

    # ------------------------------------------------------------------------------------------------------------
    # Plant
    # ------------------------------------------------------------------------------------------------------------

    def _plant_accel(self, x: Tensor, v: Tensor, q: Tensor, w: Tensor, cmd_ctrl: Tensor) -> tuple:
        """Accelerations of O for a state (world position/velocity, quaternion body->world, body angular velocity)
        and thruster speeds: (v_dot in the world frame (B, 3), w_dot in the body frame (B, 3))."""
        idxs = self.idxs
        R = roma.unitquat_to_rotmat(q)
        Rt = R.transpose(1, 2)
        v_b = (Rt @ v.unsqueeze(-1)).squeeze(-1)
        z_up = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64, device=self.device)
        g = self.batched_params.g

        # ---- Hydrostatics: submerged voxels ----
        F_buoyancy, M_buoyancy, submerged = self._buoyancy(x, R)
        fraction = (submerged / self._displaced_volume[idxs]).clamp(0.0, 1.0).unsqueeze(-1)  # (B, 1)
        up_body = Rt @ z_up                                                                  # (B, 3)

        # ---- Gravity at the CG ----
        r_g = self._r_g[idxs]
        F_gravity = -self._mass * g * up_body
        M_gravity = torch.cross(r_g, F_gravity, dim=-1)

        # ---- Panel drag: water below the surface (current), air above (wind) ----
        panels = self._drag_panels
        arms = panels["arms"][idxs]
        panel_world = x.unsqueeze(1) + arms @ Rt
        in_water = self._immersion(self._surface_height(panel_world[..., :2]) - panel_world[..., 2])  # (B, F)
        fluid_world = (in_water.unsqueeze(-1) * self.current_velocity[idxs].unsqueeze(1)
                       + (1 - in_water).unsqueeze(-1) * self.wind_velocity[idxs].unsqueeze(1))
        u = (v_b.unsqueeze(1) + torch.cross(w.unsqueeze(1).expand_as(arms), arms, dim=-1)
             - fluid_world @ R)                                                              # (B, F, 3)
        density = in_water * self.rho_water + (1 - in_water) * self.rho_air
        batch = {key: panels[key][idxs] for key in ("areas", "normals", "exposure")}
        batch["directions"] = panels["directions"]
        F_panels = geo.panel_drag(batch, u, density, self._drag_coefficients, self.EXPOSURE_BLEND_RAD)
        F_drag = F_panels.sum(dim=1)
        M_drag = torch.cross(arms, F_panels, dim=-1).sum(dim=1)

        # ---- Velocity relative to the water at O; linear damping (water only) ----
        current_b = (Rt @ self.current_velocity[idxs].unsqueeze(-1)).squeeze(-1)
        nu = torch.cat([v_b, w], dim=-1)
        nu_r = torch.cat([v_b - current_b, w], dim=-1)
        H = self._H_c[idxs]
        Ht = H.transpose(1, 2)
        nu_r_c = (H @ nu_r.unsqueeze(-1)).squeeze(-1)
        tau_damping = (Ht @ (-fraction * self._linear_damping * nu_r_c).unsqueeze(-1)).squeeze(-1)

        # ---- Thrust: only thrusters under water push ----
        thrust = self.batched_params.k_eta[idxs] * cmd_ctrl * torch.abs(cmd_ctrl)           # (B, n)
        thruster_world = x.unsqueeze(1) + self._thruster_arms[idxs] @ Rt
        thrust = thrust * self._immersion(
            self._surface_height(thruster_world[..., :2]) - thruster_world[..., 2])
        F_thruster = thrust.unsqueeze(-1) * self._thruster_directions                       # (B, n, 3)
        F_thrust = F_thruster.sum(dim=1)
        M_thrust = torch.cross(self._thruster_arms[idxs], F_thruster, dim=-1).sum(dim=1)

        tau = torch.cat([F_buoyancy + F_gravity + F_drag + F_thrust,
                         M_buoyancy + M_gravity + M_drag + M_thrust], dim=-1) + tau_damping

        # ---- Fossen: (M_RB + M_A) nu_dot = tau - C_RB(nu) nu - C_A(nu_r) nu_r - M_A [w x v_c; 0] ----
        M_RB = self._M_RB[idxs]
        # Added mass defined at the hull centroid, moved to O: M_A(O) = H^T M_A(c) H.
        M_A = Ht @ torch.diag_embed(fraction * self._added_mass) @ H                         # (B, 6, 6)
        current_term = torch.cat([torch.cross(w, current_b, dim=-1), torch.zeros_like(w)], dim=-1)
        tau = (tau - self._coriolis(M_RB, nu) - self._coriolis(M_A, nu_r)
               - (M_A @ current_term.unsqueeze(-1)).squeeze(-1))
        nu_dot = torch.linalg.solve(M_RB + M_A, tau.unsqueeze(-1)).squeeze(-1)

        # Acceleration of O in the world frame for the engine: a = R (v_b_dot + w x v_b)
        v_dot = (R @ (nu_dot[:, :3] + torch.cross(w, v_b, dim=-1)).unsqueeze(-1)).squeeze(-1)

        self._last_hydro = {"submerged_fraction": fraction.squeeze(-1), "buoyancy": F_buoyancy,
                            "drag": F_drag, "thrust": F_thrust}
        return v_dot, nu_dot[:, 3:]

    def _steady_wrench(self, state: dict, v_body: Tensor) -> tuple:
        """Body-frame force and moment about O ((B, 3) each) that hold the body-frame velocity v_body (no rotation) at
        the current pose against this plant: panel drag (water/air by immersion, relative to the current/wind), linear
        damping, gravity and buoyancy."""
        idxs = self.idxs
        x = state["x"][idxs].double()
        R = roma.unitquat_to_rotmat(state["q"][idxs].double())
        Rt = R.transpose(1, 2)
        v_body = v_body.double()

        F_buoyancy, M_buoyancy, submerged = self._buoyancy(x, R)
        fraction = (submerged / self._displaced_volume[idxs]).clamp(0.0, 1.0).unsqueeze(-1)
        up_body = Rt @ torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64, device=self.device)
        F_gravity = -self._mass * self.batched_params.g * up_body
        M_gravity = torch.cross(self._r_g[idxs], F_gravity, dim=-1)

        panels = self._drag_panels
        arms = panels["arms"][idxs]
        panel_world = x.unsqueeze(1) + arms @ Rt
        in_water = self._immersion(self._surface_height(panel_world[..., :2]) - panel_world[..., 2])
        fluid_world = (in_water.unsqueeze(-1) * self.current_velocity[idxs].unsqueeze(1)
                       + (1 - in_water).unsqueeze(-1) * self.wind_velocity[idxs].unsqueeze(1))
        u = v_body.unsqueeze(1).expand_as(arms) - fluid_world @ R
        density = in_water * self.rho_water + (1 - in_water) * self.rho_air
        batch = {key: panels[key][idxs] for key in ("areas", "normals", "exposure")}
        batch["directions"] = panels["directions"]
        F_panels = geo.panel_drag(batch, u, density, self._drag_coefficients, self.EXPOSURE_BLEND_RAD)

        current_b = (Rt @ self.current_velocity[idxs].unsqueeze(-1)).squeeze(-1)
        nu_r = torch.cat([v_body - current_b, torch.zeros_like(v_body)], dim=-1)
        H = self._H_c[idxs]
        tau_damping = (H.transpose(1, 2) @ (-fraction * self._linear_damping
                                            * (H @ nu_r.unsqueeze(-1)).squeeze(-1)).unsqueeze(-1)).squeeze(-1)

        force = F_panels.sum(dim=1) + tau_damping[:, :3] + F_buoyancy + F_gravity
        moment = (torch.cross(arms, F_panels, dim=-1).sum(dim=1) + tau_damping[:, 3:] + M_buoyancy + M_gravity)
        return -force, -moment

    def _steady_force(self, state: dict, v_body: Tensor) -> Tensor:
        """Body-frame force (B, 3) that holds the body-frame velocity v_body at the current pose (see _steady_wrench).
        The vehicles' controllers feed it forward at their velocity setpoint."""
        return self._steady_wrench(state, v_body)[0]

    # Largest thruster force the steady speed may take, as a share of its maximum; the rest stays for steering and
    # attitude (at full thrust the allocation can no longer hold the moments and the vehicle tilts).
    SPEED_LIMIT_THRUST_SHARE = 0.8
    # For a submerged vehicle (cg_below_cb), the roll/pitch moment the thrusters cannot cancel at the steady speed
    # (e.g. BlueROV2's pitch, which it does not control) tilts it until W*BG*sin(tilt) balances it; the speed is
    # limited so that this tilt stays below this angle.
    SPEED_LIMIT_MAX_TILT = np.radians(30)

    def _allocation_residual(self, pose: dict, v_body: Tensor, A: Tensor) -> tuple:
        """Thruster forces (B, n) for the steady wrench at a pose and body velocity (through TM_to_f, about the CG),
        and the part of that wrench (B, 6) they cannot produce."""
        idxs = self.idxs
        force, moment_O = self._steady_wrench(pose, v_body)
        moment_cg = moment_O - torch.cross(self._r_g[idxs], force, dim=-1)
        wrench = torch.cat([force, moment_cg], dim=-1)
        thrust = (self.batched_params.TM_to_f[idxs].double() @ wrench.unsqueeze(-1)).squeeze(-1)
        return thrust, wrench - (A @ thrust.unsqueeze(-1)).squeeze(-1)

    def _speed_feasible(self, x: Tensor, direction: Tensor, speed: Tensor, t_max: Tensor, A: Tensor) -> Tensor:
        """(B,) bool: the vehicle can move at `speed` along the horizontal `direction` (world = body at zero yaw)
        with every thruster at most SPEED_LIMIT_THRUST_SHARE of its maximum and, for a submerged vehicle
        (cg_below_cb), at a steady tilt below SPEED_LIMIT_MAX_TILT.

        The tilt is where the roll/pitch moment the thrusters cannot cancel is balanced by the hydrostatics: Newton
        iterations on the tilt (rotation vector about x and y), the steady wrench re-evaluated at each tilted pose
        with the same world velocity, so the tilted thrusters and hull are accounted for.
        """
        B = len(self.idxs)
        tilt = torch.zeros(B, 3, dtype=torch.float64, device=self.device)
        v_world = speed.unsqueeze(-1) * direction

        def evaluate(tilt):
            q = roma.rotvec_to_unitquat(tilt)
            R = roma.unitquat_to_rotmat(q)
            v_body = (R.transpose(1, 2) @ v_world.unsqueeze(-1)).squeeze(-1)
            return self._allocation_residual({"x": x, "q": q}, v_body, A)

        thrust, residual = evaluate(tilt)
        submerged = "cg_below_cb" in self._marine
        if submerged:
            step = 1e-3
            for _ in range(8):
                # Newton step per axis on the unbalanced roll/pitch moment, with a numerical derivative.
                for axis in (0, 1):
                    bumped = tilt.clone()
                    bumped[:, axis] += step
                    _, residual_bumped = evaluate(bumped)
                    slope = (residual_bumped[:, 3 + axis] - residual[:, 3 + axis]) / step
                    update = torch.where(slope.abs() > 1e-9, -residual[:, 3 + axis] / slope, torch.zeros_like(slope))
                    tilt[:, axis] = torch.clamp(tilt[:, axis] + update, -np.pi / 2, np.pi / 2)
                    thrust, residual = evaluate(tilt)
        ok = torch.all(torch.abs(thrust) <= self.SPEED_LIMIT_THRUST_SHARE * t_max, dim=-1)
        if submerged:
            balanced = torch.linalg.norm(residual[:, 3:5], dim=-1) < 0.05 * self._mass * self.batched_params.g * \
                float(self._marine["cg_below_cb"])
            ok &= balanced & (torch.linalg.norm(tilt[:, :2], dim=-1) <= self.SPEED_LIMIT_MAX_TILT)
        return ok

    def _speed_limit(self, state: dict, directions: list, default: float) -> Tensor:
        """Highest steady speed (m/s, B) along the given directions (the slowest of them) the vehicle can hold at its
        current position (see _speed_feasible). Bisection, cached until set_geometry.

        Args:
            state: controller state dict (x, q, ...), full batch.
            directions: unit body directions, e.g. [[1, 0, 0], [0, 1, 0]].
            default: unused here (the value of models without a plant).
        """
        cache = getattr(self, "_speed_limit_cache", None)
        if cache is None:
            cache = self._speed_limit_cache = {}
        key = tuple(tuple(float(c) for c in d) for d in directions)
        if key in cache:
            return cache[key]
        idxs = self.idxs
        speed = torch.minimum(self.batched_params.rotor_speed_max[idxs].abs(),
                              self.batched_params.rotor_speed_min[idxs].abs()).double().reshape(len(idxs), -1)
        t_max = self.batched_params.k_eta[idxs].double().reshape(len(idxs), -1) * speed ** 2      # (B, 1 or n)
        A = torch.linalg.pinv(self.batched_params.TM_to_f[idxs].double())                      # (B, 6, n)
        x = state["x"][idxs].double()
        limits = []
        for d in directions:
            d = torch.tensor(d, dtype=torch.float64, device=self.device)
            lo = torch.zeros(len(idxs), dtype=torch.float64, device=self.device)
            hi = torch.full_like(lo, 10.0)
            for _ in range(25):
                mid = (lo + hi) / 2
                ok = self._speed_feasible(x, d, mid, t_max, A)
                lo, hi = torch.where(ok, mid, lo), torch.where(ok, hi, mid)
            limits.append(lo)
        cache[key] = torch.stack(limits).min(dim=0).values
        return cache[key]

    def _s_dot_fn(self, s: Tensor, cmd_ctrl: Tensor) -> Tensor:
        state = self.unpack_state(s, self.idxs, self.batch_size)
        idxs = self.idxs
        v_dot, w_dot = self._plant_accel(state["x"][idxs].double(), state["v"][idxs].double(),
                                         state["q"][idxs].double(), state["w"][idxs].double(), cmd_ctrl)
        s_dot = torch.zeros((len(idxs), 13 + cmd_ctrl.shape[1]), dtype=torch.float64, device=self.device)
        s_dot[:, 0:3] = state["v"][idxs]
        s_dot[:, 3:6] = v_dot
        s_dot[:, 10:13] = w_dot
        return s_dot

    # ------------------------------------------------------------------------------------------------------------
    # Integration over the engine tick
    # ------------------------------------------------------------------------------------------------------------

    def step(self, state: list, control: list) -> list:
        """One engine tick: controller on the reported state, then the plant integrated in the client.

        The engine applies the returned acceleration during its next tick, and the state it reports after that tick
        already includes it (measured: v[k+1] = v[k] + a[k] dt exactly), so the controller acts on the reported
        state with no loop delay. Its 20 Hz step cannot integrate the stiff hydrostatics (a surface vessel's roll
        reaches ~15 rad/s), so the client integrates its own dynamics over the tick in SUBSTEPS substeps and returns
        the constant acceleration that takes the engine to the integrated velocities. The engine substeps its
        physics (bSubstepping, ~2.8 ms), so its pose then advances with the mean of the start and end velocities.

        Args:
            state: DynamicsSensor vectors (B, 19 + K): v world [3:6], x [6:9], w world [12:15], q [15:19], then the
                ExportWaterSurface block (see VehicleModel._read_water_surface).
            control: commands (B, D) for the control abstraction.

        Returns:
            (B, 6) list: linear acceleration and angular acceleration, both in the world frame.
        """
        dynamics = torch.as_tensor(np.asarray(state, dtype=np.float64), device=self.device)
        command = torch.as_tensor(np.asarray(control, dtype=np.float64), device=self.device)
        if self.control_abstraction == "accel":
            return command.cpu().tolist()
        self._read_water_surface(dynamics[:, DYNAMICS_ITEMS:])
        x, v = dynamics[:, 6:9], dynamics[:, 3:6]
        q, w_world = dynamics[:, 15:19], dynamics[:, 12:15]
        w_body = (roma.unitquat_to_rotmat(q).transpose(1, 2) @ w_world.unsqueeze(-1)).squeeze(-1)

        # Controller on the reported state (body-frame angular velocity, as the controllers expect).
        reported = {"x": x, "v": v, "q": q, "w": w_body, "wq": dynamics[:, 9:12]}
        cmd = torch.clip(self.get_cmd_motor_speeds(reported, {"cmd_ctrl": command}),
                         self.batched_params.rotor_speed_min[self.idxs],
                         self.batched_params.rotor_speed_max[self.idxs]).double()

        # Integrate the plant over one tick (semi-implicit Euler, body-frame rotation).
        dt = float(self.dt) if self.dt else 1 / 20
        substeps = self._substeps(dt)
        h = dt / substeps
        xs, vs, qs, ws = x.clone(), v.clone(), q.clone(), w_body.clone()
        for _ in range(substeps):
            v_dot, w_dot = self._plant_accel(xs, vs, qs, ws, cmd)
            vs = vs + v_dot * h
            ws = ws + w_dot * h
            xs = xs + vs * h
            qs = roma.quat_product(qs, roma.rotvec_to_unitquat(ws * h))
            qs = qs / torch.linalg.norm(qs, dim=-1, keepdim=True)

        # Constant accelerations reproducing the integrated velocities over the engine tick.
        w_end_world = (roma.unitquat_to_rotmat(qs) @ ws.unsqueeze(-1)).squeeze(-1)
        accel = torch.cat([(vs - v) / dt, (w_end_world - w_world) / dt], dim=-1)
        return accel.cpu().tolist()

    @staticmethod
    def _coriolis(M: Tensor, nu: Tensor) -> Tensor:
        """C(nu) nu for a 6x6 mass matrix M (Fossen, Kirchhoff form): [w x a; v x a + w x b], (a; b) = M nu."""
        ab = (M @ nu.unsqueeze(-1)).squeeze(-1)
        a, b = ab[:, :3], ab[:, 3:]
        v, w = nu[:, :3], nu[:, 3:]
        return torch.cat([torch.cross(w, a, dim=-1),
                          torch.cross(v, a, dim=-1) + torch.cross(w, b, dim=-1)], dim=-1)


# ================================================================================================================
# Controllers
# ================================================================================================================

def _wrap(angle: Tensor) -> Tensor:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _yaw_of(R: Tensor) -> Tensor:
    return torch.atan2(R[:, 1, 0], R[:, 0, 0])


class _MarineControlBase:
    """Shared pieces of the marine controllers: thrust allocation and heading hold."""

    def _thruster_speeds(self, wrench_body: Tensor) -> Tensor:
        """Body wrench about the CG (B, 6) -> signed thruster speeds, through the vehicle's allocation TM_to_f."""
        thrust = (self.batched_params.TM_to_f[self.idxs] @ wrench_body.unsqueeze(-1)).squeeze(-1)
        k_eta = self.batched_params.k_eta[self.idxs]
        return torch.sign(thrust) * torch.sqrt(torch.abs(thrust) / k_eta)

    def _held_heading(self, yaw: Tensor, command_yaw_rate: Tensor | None = None) -> Tensor:
        """Heading to hold: captured on the first call, moved by a commanded yaw offset when given."""
        if getattr(self, "_heading_hold", None) is None or self._heading_hold.shape != yaw.shape:
            self._heading_hold = yaw.clone()
        if command_yaw_rate is not None:
            moving = command_yaw_rate != 0
            self._heading_hold = torch.where(moving, _wrap(yaw + command_yaw_rate), self._heading_hold)
        return self._heading_hold

    def _gain(self, key: str, default):
        value = self._marine.get(key, default)
        return torch.as_tensor(value, dtype=torch.float64, device=self.device)


class ROVControl(_MarineControlBase):
    """Controllers of the ROVs (BlueROV2 5-DOF, BlueROV2Heavy 6-DOF), on the marine plant.

    - cmd_vel     [vx, vy, vz]          world velocity, heading held.
    - cmd_vel_yaw [vx, vy, vz, dyaw]    world velocity; a nonzero dyaw (deg) moves the held heading to yaw + dyaw.
    - cmd_pos_yaw [x, y, z, yaw]        position (world) and absolute heading (deg).

    Position PID (integral cancels currents) -> velocity setpoint limited to ctrl_v_max -> velocity P loop ->
    desired force (world) with the net weight fed forward, rotated into the body frame. Attitude PD keeps roll and
    pitch level and tracks the heading. The wrench is about the CG and goes through the thruster allocation; the
    5-DOF BlueROV2 cannot act on pitch (its CB above the CG keeps it level).
    """

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        idxs = self.idxs
        if self.control_abstraction == "cmd_motor_speeds":
            return control["cmd_ctrl"][idxs]
        x, v = state["x"].double(), state["v"].double()
        w = state["w"].double()
        R = roma.unitquat_to_rotmat(state["q"].double())
        yaw = _yaw_of(R)
        cmd = control["cmd_ctrl"][idxs].double()

        if self.control_abstraction == "cmd_pos_yaw":
            e_pos = cmd[:, :3] - x
            self._pos_integral = getattr(self, "_pos_integral", torch.zeros_like(e_pos))
            v_max = self._gain("ctrl_v_max", [1.0, 1.0, 0.5])
            v_des = self._gain("ctrl_kp_pos", 0.6) * e_pos + self._gain("ctrl_ki_pos", 0.1) * self._pos_integral
            saturated = torch.any(torch.abs(v_des) > v_max, dim=-1, keepdim=True)
            v_des = torch.maximum(torch.minimum(v_des, v_max), -v_max)
            near = torch.linalg.norm(e_pos, dim=-1, keepdim=True) < self._marine.get("ctrl_integral_radius", 2.0)
            self._pos_integral = torch.where(near & ~saturated, self._pos_integral + e_pos * self.dt,
                                             self._pos_integral)
            yaw_des = torch.deg2rad(cmd[:, 3])
        else:
            v_des = cmd[:, :3]
            offset = torch.deg2rad(cmd[:, 3]) if self.control_abstraction == "cmd_vel_yaw" else None
            yaw_des = self._held_heading(yaw, offset)

        # Velocity loop -> desired force (world), net weight (gravity minus buoyancy) fed forward.
        a_des = self._gain("ctrl_k_v", [1.5, 1.5, 1.5]) * (v_des - v)
        m = self._marine["mass"]
        m_eff = m + self._added_mass[:3]
        net_weight = (m - self.rho_water * float(self._displaced_volume[0])) * self.batched_params.g
        F_world = m_eff * a_des + torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64, device=self.device) * net_weight
        F_body = (R.transpose(1, 2) @ F_world.unsqueeze(-1)).squeeze(-1)
        # Steady hydrodynamic drag at the velocity setpoint (body axes), fed forward: linear + quadratic damping.
        v_des_body = (R.transpose(1, 2) @ v_des.unsqueeze(-1)).squeeze(-1)
        F_body = F_body + (self._linear_damping[:3] * v_des_body
                           + self._gain("ctrl_drag_ff", [0.0, 0.0, 0.0]) * v_des_body * torch.abs(v_des_body))

        # Attitude: level roll/pitch, heading to yaw_des (body-frame PD, inertia-scaled).
        euler = roma.rotmat_to_euler("xyz", R)
        att_err = torch.stack([euler[:, 0], euler[:, 1], _wrap(yaw - yaw_des)], dim=-1)
        I = torch.as_tensor(np.diag(self._marine["I"]), dtype=torch.float64, device=self.device) + self._added_mass[3:]
        M_body = I * (-self._gain("ctrl_kp_att", [25.0, 25.0, 9.0]) * att_err - self._gain("ctrl_kd_att", [8.0, 8.0, 6.0]) * w)

        return self._thruster_speeds(torch.cat([F_body, M_body], dim=-1))


class SurfaceVesselControl(_MarineControlBase):
    """Controllers of a differential-thrust surface vessel (BlueBoat), on the marine plant.

    - cmd_vel     [u, -, -]           surge speed (m/s, body x), heading held.
    - cmd_vel_yaw [u, -, -, dyaw]     surge speed; a nonzero dyaw (deg) moves the held heading to yaw + dyaw.
    - cmd_pos_yaw [x, y, -, -]        go to a point: steer to its bearing, surge speed proportional to the distance
                                      (limited to ctrl_u_max), slowing while the heading error is large.

    Surge P loop with the drag at the setpoint fed forward; heading PD (inertia-scaled). The two thrusters give
    surge force and yaw moment only.
    """

    def get_cmd_motor_speeds(self, state: dict, control: dict) -> Tensor:
        idxs = self.idxs
        if self.control_abstraction == "cmd_motor_speeds":
            return control["cmd_ctrl"][idxs]
        x, v = state["x"].double(), state["v"].double()
        w = state["w"].double()
        R = roma.unitquat_to_rotmat(state["q"].double())
        yaw = _yaw_of(R)
        cmd = control["cmd_ctrl"][idxs].double()
        u = (R.transpose(1, 2) @ v.unsqueeze(-1)).squeeze(-1)[:, 0]          # surge speed

        u_max = float(self._marine.get("ctrl_u_max", 2.5))
        if self.control_abstraction == "cmd_pos_yaw":
            e = cmd[:, :2] - x[:, :2]
            distance = torch.linalg.norm(e, dim=-1)
            bearing = torch.atan2(e[:, 1], e[:, 0])
            arrived = distance < self._marine.get("ctrl_arrival_radius", 1.0)
            yaw_des = torch.where(arrived, self._held_heading(yaw), bearing)
            self._heading_hold = torch.where(arrived, self._heading_hold, bearing) if hasattr(self, "_heading_hold") \
                else bearing.clone()
            heading_factor = torch.clamp(torch.cos(_wrap(bearing - yaw)), min=0.0)
            u_des = torch.clamp(self._gain("ctrl_kp_pos", 0.3) * distance, max=u_max) * heading_factor
            u_des = torch.where(arrived, torch.zeros_like(u_des), u_des)
        else:
            u_des = torch.clamp(cmd[:, 0], -u_max, u_max)
            offset = torch.deg2rad(cmd[:, 3]) if self.control_abstraction == "cmd_vel_yaw" else None
            yaw_des = self._held_heading(yaw, offset)

        # Surge: P loop plus the steady drag at the setpoint (quadratic, from the 3 m/s spec calibration).
        m_eff = self._marine["mass"] + float(self._added_mass[0])
        drag_at = self._gain("ctrl_surge_drag", 6.5)                           # N per (m/s)^2
        F_x = m_eff * self._gain("ctrl_k_u", 1.0) * (u_des - u) + drag_at * u_des * torch.abs(u_des)
        # Heading PD, inertia-scaled.
        I_z = float(np.asarray(self._marine["I"])[2, 2]) + float(self._added_mass[5])
        M_z = I_z * (-self._gain("ctrl_kp_yaw", 1.0) * _wrap(yaw - yaw_des) - self._gain("ctrl_kd_yaw", 2.0) * w[:, 2])

        zeros = torch.zeros_like(F_x)
        wrench = torch.stack([F_x, zeros, zeros, zeros, zeros, M_z], dim=-1)
        return self._thruster_speeds(wrench)
