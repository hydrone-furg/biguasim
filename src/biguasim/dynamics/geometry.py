"""Body properties derived from the collision mesh the engine exports through the DynamicsSensor.

The models used to approximate each vehicle with primitives (spheres, boxes, cylinders) and hand-set
arm lengths. With ``ExportGeometry`` the engine sends the mesh it actually collides with, so volume,
rotor positions and drag panels can be taken from it instead.

Exported meshes are often render meshes: open edges, loose parts, internal faces. Integrating over such
a surface (divergence theorem) gives unreliable volumes, so volume and centroid come from the mesh voxelized
into a solid: surface voxels are marked, gaps up to one voxel wide are closed, and the interior is filled.
Drag panels are the mesh triangles themselves, with occlusion from depth rasters (``drag_panels``).
"""
import hashlib
from dataclasses import dataclass

import numpy as np
import torch
from scipy import ndimage

# Voxel edge for volume and centroid; 5 mm resolves the parts of a 0.5-1 m vehicle.
DEFAULT_VOXEL_SIZE = 0.005

# Surface points are sampled in chunks of triangles to bound memory on large meshes.
_TRIANGLES_PER_CHUNK = 20000


@dataclass(frozen=True)
class BodyGeometry:
    """Solid properties of a body mesh, in the body frame of the mesh (meters, right-handed).

    Attributes:
        surface_points: ``(P, 3)`` centers of the voxels the surface crosses: a dense, evenly spaced sampling
            of the surface, used to locate parts such as rotor hubs.
        volume: Volume of the voxelized solid (m^3).
        centroid: ``(3,)`` centroid of the solid, i.e. its center of mass at uniform density.
        voxel_size: Edge of the voxels the properties were computed from (m).
    """

    surface_points: np.ndarray
    volume: float
    centroid: np.ndarray
    voxel_size: float


_cache: dict[tuple[str, float], BodyGeometry] = {}


def analyze_mesh(vertices: np.ndarray, triangles: np.ndarray, *,
                 voxel_size: float = DEFAULT_VOXEL_SIZE) -> BodyGeometry:
    """Voxelizes a mesh into a solid and computes its volume, centroid and dense surface points.

    Results are cached by mesh contents: models are rebuilt on every reset while the mesh stays the same.

    Args:
        vertices: ``(N, 3)`` vertex positions (m).
        triangles: ``(M, 3)`` vertex indices.
        voxel_size: Voxel edge (m). Gaps in the mesh up to this size are closed.

    Returns:
        The solid's properties.

    Raises:
        ValueError: If the mesh is empty.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError(f"Cannot analyze an empty mesh ({len(vertices)} vertices, {len(triangles)} triangles)")

    digest = hashlib.sha1(vertices.tobytes() + triangles.tobytes()).hexdigest()
    key = (digest, voxel_size)
    if key not in _cache:
        _cache[key] = _analyze(vertices, triangles, voxel_size)
    return _cache[key]


def _analyze(vertices: np.ndarray, triangles: np.ndarray, voxel_size: float) -> BodyGeometry:
    # Two empty voxels of margin so the closing and the outside flood fill never touch the border.
    origin = vertices.min(axis=0) - 2 * voxel_size
    shape = np.ceil((vertices.max(axis=0) + 2 * voxel_size - origin) / voxel_size).astype(int) + 1
    shell = np.zeros(shape, dtype=bool)

    for start in range(0, len(triangles), _TRIANGLES_PER_CHUNK):
        chunk = triangles[start:start + _TRIANGLES_PER_CHUNK]
        points = _surface_points(vertices[chunk[:, 0]], vertices[chunk[:, 1]], vertices[chunk[:, 2]],
                                 spacing=voxel_size / 2)
        index = np.floor((points - origin) / voxel_size).astype(int)
        shell[index[:, 0], index[:, 1], index[:, 2]] = True

    solid = ndimage.binary_fill_holes(ndimage.binary_closing(shell) | shell)
    centers = (np.argwhere(solid) + 0.5) * voxel_size + origin
    return BodyGeometry(
        surface_points=(np.argwhere(shell) + 0.5) * voxel_size + origin,
        volume=len(centers) * voxel_size**3,
        centroid=centers.mean(axis=0),
        voxel_size=voxel_size,
    )


def _surface_points(a: np.ndarray, b: np.ndarray, c: np.ndarray, *, spacing: float) -> np.ndarray:
    """Points covering each triangle ``(a, b, c)`` no farther apart than ``spacing``."""
    return _surface_samples(a, b, c, spacing=spacing)[0]


def _surface_samples(a: np.ndarray, b: np.ndarray, c: np.ndarray, *,
                     spacing: float) -> tuple[np.ndarray, np.ndarray]:
    """Points covering each triangle ``(a, b, c)`` no farther apart than ``spacing``, and their triangle's index."""
    longest_edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                           np.linalg.norm(a - c, axis=1)], axis=0)
    divisions = np.maximum(1, np.ceil(longest_edge / spacing)).astype(int)
    everyone = np.arange(len(a))
    points, triangles = [a, b, c], [everyone, everyone, everyone]
    for count in np.unique(divisions[divisions > 1]):
        selected = np.flatnonzero(divisions == count)
        i, j = np.meshgrid(np.arange(count + 1), np.arange(count + 1), indexing="ij")
        inside = i + j <= count
        u, w = (i[inside] / count)[None, :, None], (j[inside] / count)[None, :, None]
        points.append((a[selected, None] * (1 - u - w) + b[selected, None] * u
                       + c[selected, None] * w).reshape(-1, 3))
        triangles.append(np.repeat(selected, inside.sum()))
    return np.concatenate(points), np.concatenate(triangles)


def locate_rotor_hubs(points: np.ndarray, nominal_positions: np.ndarray, *,
                      hub_radius: float = 0.08) -> np.ndarray:
    """Finds each rotor hub as the outermost part of the mesh in the direction of its nominal position.

    Meant for multirotors whose motors sit at the tips of the arms. For each nominal rotor, the surface
    points within its angular sector (around the body z axis) are searched for the one farthest from
    the axis; the hub is the part of the mesh within ``hub_radius`` of it. Its center is the middle of
    that part's horizontal extent, at the height of its top, where the propeller spins.

    Args:
        points: ``(N, 3)`` points evenly covering the mesh surface, in the body frame
            (``BodyGeometry.surface_points``); raw vertices are too sparse on coarse meshes.
        nominal_positions: ``(R, 3)`` approximate rotor positions; they set each rotor's direction and
            the order of the result.
        hub_radius: Horizontal distance from the outermost vertex that still belongs to the hub (m).

    Returns:
        ``(R, 3)`` hub positions in the frame of ``points``.

    Raises:
        ValueError: If a rotor's sector holds no points.
    """
    nominal_positions = np.asarray(nominal_positions, dtype=np.float64)
    azimuths = np.arctan2(points[:, 1], points[:, 0])
    radii = np.hypot(points[:, 0], points[:, 1])
    half_sector = np.pi / len(nominal_positions)

    hubs = []
    for nominal in nominal_positions:
        heading = np.arctan2(nominal[1], nominal[0])
        in_sector = np.abs(np.angle(np.exp(1j * (azimuths - heading)))) < half_sector
        if not in_sector.any():
            raise ValueError(f"No surface points in the sector of the rotor at {np.round(nominal, 3).tolist()}")
        sector = points[in_sector]
        tip = sector[np.argmax(radii[in_sector]), :2]
        hub = sector[np.hypot(*(sector[:, :2] - tip).T) < hub_radius]
        lo, hi = hub[:, :2].min(axis=0), hub[:, :2].max(axis=0)
        hubs.append([*(lo + hi) / 2, hub[:, 2].max()])
    return np.array(hubs)


@dataclass(frozen=True)
class DragPanels:
    """The triangles of a body mesh as drag panels, and how much of each one a flow reaches.

    Panels are two-sided, so the mesh's winding does not matter: a panel's area facing a flow direction ``d``
    is ``area * |normal . d|``. Of that, only the part not hidden behind other panels, seen from upstream,
    meets the flow; ``exposure`` holds that fraction for a set of directions. Summed over the panels, the
    exposed areas give the silhouette of the body along the direction, as rasterized at 2 mm.

    Attributes:
        areas: ``(F,)`` panel areas (m^2).
        normals: ``(F, 3)`` unit panel normals, body frame.
        centroids: ``(F, 3)`` panel centroids, body frame of the mesh (m).
        directions: ``(K, 3)`` unit directions of motion through the fluid, body frame.
        exposure: ``(F, K)`` fraction (0 to 1) of each panel's facing area that meets the flow when the body
            moves along each direction.
    """

    areas: np.ndarray
    normals: np.ndarray
    centroids: np.ndarray
    directions: np.ndarray
    exposure: np.ndarray


# Directions of motion sampled for the exposure table (plus the six body axes). 256 directions are
# ~13 degrees apart; the table is looked up at the nearest one.
DRAG_DIRECTION_COUNT = 256

# Pixel edge of the depth rasters that decide which panel is in front. Owned pixels overstate a panel's
# area by about half a pixel along the silhouette's outline, a few percent at 2 mm.
_EXPOSURE_PIXEL_SIZE = 0.002

_panel_cache: dict[str, DragPanels] = {}


def drag_panels(vertices: np.ndarray, triangles: np.ndarray, *, device: str | torch.device = "cpu") -> DragPanels:
    """Builds the drag panels of a mesh and their exposure to the flow along each sampled direction.

    For each direction, the surface is sampled and projected on the plane across the flow; in every pixel,
    the sample farthest along the direction of motion (the leading surface) owns the pixel. A panel's
    exposed area is the area of the pixels it owns, and its exposure that area over its facing area
    (capped at 1: panels seen edge-on own a sliver of pixels larger than their facing area).
    Cached by mesh contents. For the Matrice's 182 hull panels (2.5M samples, 262 directions) this takes
    ~11 s on the CPU and well under a second on a GPU.

    Args:
        vertices: ``(N, 3)`` vertex positions (m).
        triangles: ``(M, 3)`` vertex indices.
        device: Torch device for the rasterization.

    Returns:
        The panels, with an exposure table over ``DRAG_DIRECTION_COUNT`` directions and the six axes.

    Raises:
        ValueError: If the mesh has no triangle with a nonzero area.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    digest = hashlib.sha1(vertices.tobytes() + triangles.tobytes()).hexdigest()
    if digest not in _panel_cache:
        _panel_cache[digest] = _build_panels(vertices, triangles, torch.device(device))
    return _panel_cache[digest]


def _build_panels(vertices: np.ndarray, triangles: np.ndarray, device: torch.device) -> DragPanels:
    a, b, c = (vertices[triangles[:, i]] for i in range(3))
    cross = np.cross(b - a, c - a)
    doubled_areas = np.linalg.norm(cross, axis=1)
    keep = doubled_areas > 1e-12
    if not keep.any():
        raise ValueError(f"The mesh has no triangle with a nonzero area ({len(triangles)} triangles)")
    a, b, c, cross, doubled_areas = a[keep], b[keep], c[keep], cross[keep], doubled_areas[keep]

    areas = doubled_areas / 2
    normals = cross / doubled_areas[:, None]
    samples, owners = _surface_samples(a, b, c, spacing=_EXPOSURE_PIXEL_SIZE / 2)
    samples, owners = torch.from_numpy(samples).to(device), torch.from_numpy(owners).to(device)

    directions = np.concatenate([np.vstack([np.eye(3), -np.eye(3)]), _sphere_directions(DRAG_DIRECTION_COUNT)])
    exposed = np.stack([_exposed_areas(samples, owners, direction, panel_count=len(a)) for direction in directions],
                       axis=1)
    facing = areas[:, None] * np.abs(normals @ directions.T)
    exposure = np.clip(np.divide(exposed, facing, out=np.zeros_like(exposed), where=facing > 0), 0, 1)

    return DragPanels(areas=areas, normals=normals, centroids=(a + b + c) / 3,
                      directions=directions, exposure=exposure)


def _exposed_areas(samples: torch.Tensor, owners: torch.Tensor, direction: np.ndarray, *,
                   panel_count: int) -> np.ndarray:
    """Area of the pixels each panel owns, seen from ahead of a body moving along ``direction`` (m^2)."""
    # Orthonormal basis of the plane across the flow.
    helper = np.array([1.0, 0.0, 0.0]) if abs(direction[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    across_1 = np.cross(direction, helper)
    across_1 /= np.linalg.norm(across_1)
    across_2 = np.cross(direction, across_1)
    basis = torch.from_numpy(np.stack([across_1, across_2, direction], axis=1)).to(samples.device)

    projected = samples @ basis
    cells = torch.floor(projected[:, :2] / _EXPOSURE_PIXEL_SIZE).long()
    cells -= cells.min(dim=0).values
    pixels = cells[:, 0] * (int(cells[:, 1].max()) + 1) + cells[:, 1]
    depth = projected[:, 2]

    # The leading surface in each pixel is the one farthest along the motion. Adjacent panels share their
    # edge samples at equal depth; the higher panel index takes those pixels, so the result is deterministic.
    leading = torch.full((int(pixels.max()) + 1,), -torch.inf, dtype=depth.dtype, device=depth.device)
    leading = leading.scatter_reduce(0, pixels, depth, reduce="amax")
    in_front = depth == leading[pixels]
    owner_of_pixel = torch.full_like(leading, -1, dtype=torch.long)
    owner_of_pixel = owner_of_pixel.scatter_reduce(0, pixels[in_front], owners[in_front], reduce="amax")
    owner_of_pixel = owner_of_pixel[owner_of_pixel >= 0]
    return torch.bincount(owner_of_pixel, minlength=panel_count).cpu().numpy() * _EXPOSURE_PIXEL_SIZE**2


def _sphere_directions(count: int) -> np.ndarray:
    """``count`` unit vectors spread evenly over the sphere (Fibonacci lattice)."""
    index = np.arange(count) + 0.5
    polar = np.arccos(1 - 2 * index / count)
    azimuth = np.pi * (1 + 5**0.5) * index
    return np.stack([np.sin(polar) * np.cos(azimuth), np.sin(polar) * np.sin(azimuth), np.cos(polar)], axis=1)


def panel_drag(panels: dict, flow_velocity: torch.Tensor, density, coefficients: torch.Tensor,
               blend_rad: float) -> torch.Tensor:
    """Quadratic drag on each drag panel, in the body frame.

    F_i = -1/2 rho_i A_i |n_i . u_i_hat| e_i(u_i_hat) |u_i| (C . u_i), with u_i the velocity of panel i
    relative to the fluid, e_i its exposure (the share of its facing area the flow reaches, see
    drag_panels) blended over the tabulated directions within blend_rad of u_i (weights falling to
    zero at that angle, so the drag is continuous in the attitude), and C the per-axis coefficients.

    Args:
        panels: tensors from DragPanels, batched: 'areas' (B, F), 'normals' (B, F, 3),
            'exposure' (B, F, K) and 'directions' (K, 3).
        flow_velocity: (B, F, 3) velocity of each panel relative to the fluid, body frame (m/s).
        density: fluid density (kg/m^3), scalar or (B, F).
        coefficients: (3,) dimensionless drag coefficients along the body axes.
        blend_rad: angular radius of the exposure blend (rad).

    Returns:
        (B, F, 3) drag force on each panel, body frame (N).
    """
    speed = torch.linalg.norm(flow_velocity, dim=-1, keepdim=True)                # (B, F, 1)
    flow = flow_velocity / speed.clamp_min(1e-9)
    cos_angle = flow @ panels['directions'].T                                      # (B, F, K)
    weights = torch.clamp(cos_angle - np.cos(blend_rad), min=0) ** 2
    exposure = (torch.sum(weights * panels['exposure'], dim=-1)
                / torch.sum(weights, dim=-1).clamp_min(1e-12))                     # (B, F)
    facing = panels['areas'] * torch.abs(torch.sum(panels['normals'] * flow, dim=-1))
    if torch.is_tensor(density):
        density = density.unsqueeze(-1)
    return -0.5 * density * (facing * exposure).unsqueeze(-1) * speed * coefficients * flow_velocity
