"""Convert Rapier :class:`SharedShape` instances into Panda3D ``Geom`` nodes.

Each public converter returns a list of ``(local_pose, Geom)`` pairs, where
``local_pose`` is the shape's local offset inside its parent (identity for
non-compound shapes). The list always has at least one entry — for shapes
we don't yet have a proper triangulation for we fall back to an AABB box.

Vertex format: ``GeomVertexFormat.getV3n3c4()`` — interleaved position
(3 floats), normal (3 floats), color (4 uint8). Vertices, normals and
colors are built as contiguous NumPy arrays and uploaded in one
``copyDataFrom`` call per Geom; no per-triangle Python loops in the
upload path.

All Panda3D imports happen inside the converter functions so the module
can be imported (and unit-tested) in environments where panda3d isn't
installed — useful for headless CI runs that touch this module
indirectly.
"""

from __future__ import annotations

import math
from typing import Any, List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

_SPHERE_LAT: int = 14
_SPHERE_LON: int = 22
_CAPSULE_HEMI_LAT: int = 8
_CAPSULE_LON: int = 18
_CYLINDER_SEGMENTS: int = 22
_CONE_SEGMENTS: int = 22

# Numeric ShapeType codes (must match parry's ``ShapeType`` enum). The 2D
# and 3D bindings expose separate ``ShapeType`` enum classes that share
# these values; we compare via ``int(...)`` to stay dim-agnostic.
_ST_BALL: int = 0
_ST_CUBOID: int = 1
_ST_CAPSULE: int = 2
_ST_SEGMENT: int = 3
_ST_TRIANGLE: int = 4
_ST_TRIMESH: int = 6
_ST_POLYLINE: int = 7
_ST_HEIGHTFIELD: int = 9
_ST_COMPOUND: int = 10
_ST_CONVEX_POLYGON: int = 11
_ST_CONVEX_POLYHEDRON: int = 12
_ST_CYLINDER: int = 13
_ST_CONE: int = 14
_ST_ROUND_CUBOID: int = 15
_ST_ROUND_TRIANGLE: int = 16
_ST_ROUND_CYLINDER: int = 17
_ST_ROUND_CONE: int = 18
_ST_ROUND_CONVEX_POLYHEDRON: int = 19
_ST_ROUND_CONVEX_POLYGON: int = 20


# ---------------------------------------------------------------------------
# NumPy primitive builders (no Panda3D imports — fast + testable)
# ---------------------------------------------------------------------------


def _sphere_mesh(radius: float, lat: int = _SPHERE_LAT, lon: int = _SPHERE_LON) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Smooth-shaded UV sphere. Returns (positions, normals, indices)."""
    lat = max(3, int(lat))
    lon = max(3, int(lon))
    # +1 because the seam vertex is duplicated for clean wrap-around in UV
    # but here we don't need UVs, so we still close the loop with index
    # arithmetic: the sphere has (lat+1) * (lon+1) vertices.
    i = np.arange(lat + 1, dtype=np.float32)
    j = np.arange(lon + 1, dtype=np.float32)
    theta = (i * (math.pi / lat))[:, None]      # 0..pi, (lat+1, 1)
    phi = (j * (2 * math.pi / lon))[None, :]    # 0..2pi, (1, lon+1)
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    sin_p = np.sin(phi)
    cos_p = np.cos(phi)
    x = sin_t * cos_p
    y = cos_t * np.ones_like(phi)
    z = sin_t * sin_p
    normals = np.stack([x, y, z], axis=-1).reshape(-1, 3).astype(np.float32)
    positions = normals * float(radius)
    # Indices: two triangles per quad in the (lat × lon) grid.
    a, b = np.meshgrid(np.arange(lat, dtype=np.uint32),
                       np.arange(lon, dtype=np.uint32),
                       indexing="ij")
    v00 = a * (lon + 1) + b
    v01 = v00 + 1
    v10 = v00 + (lon + 1)
    v11 = v10 + 1
    tris = np.stack([
        np.stack([v00, v10, v11], axis=-1),
        np.stack([v00, v11, v01], axis=-1),
    ], axis=2).reshape(-1, 3).astype(np.uint32)
    return positions, normals, tris


def _cuboid_mesh(hx: float, hy: float, hz: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Box with per-face normals (vertex-duplicated for flat shading)."""
    hx, hy, hz = float(hx), float(hy), float(hz)
    # 6 faces × 4 vertices each = 24 vertices.
    # Each face: 4 corners with a single outward normal.
    faces = [
        # (+x face)
        ([( hx, -hy, -hz), ( hx,  hy, -hz), ( hx,  hy,  hz), ( hx, -hy,  hz)], ( 1, 0, 0)),
        # (-x face)
        ([(-hx,  hy, -hz), (-hx, -hy, -hz), (-hx, -hy,  hz), (-hx,  hy,  hz)], (-1, 0, 0)),
        # (+y face)
        ([(-hx,  hy, -hz), (-hx,  hy,  hz), ( hx,  hy,  hz), ( hx,  hy, -hz)], ( 0, 1, 0)),
        # (-y face)
        ([(-hx, -hy,  hz), (-hx, -hy, -hz), ( hx, -hy, -hz), ( hx, -hy,  hz)], ( 0,-1, 0)),
        # (+z face)
        ([(-hx, -hy,  hz), ( hx, -hy,  hz), ( hx,  hy,  hz), (-hx,  hy,  hz)], ( 0, 0, 1)),
        # (-z face)
        ([( hx, -hy, -hz), (-hx, -hy, -hz), (-hx,  hy, -hz), ( hx,  hy, -hz)], ( 0, 0,-1)),
    ]
    pos = np.empty((24, 3), dtype=np.float32)
    nrm = np.empty((24, 3), dtype=np.float32)
    idx = np.empty((12, 3), dtype=np.uint32)
    for fi, (corners, normal) in enumerate(faces):
        base = fi * 4
        for ci, c in enumerate(corners):
            pos[base + ci] = c
            nrm[base + ci] = normal
        idx[fi * 2 + 0] = (base + 0, base + 1, base + 2)
        idx[fi * 2 + 1] = (base + 0, base + 2, base + 3)
    return pos, nrm, idx


def _cylinder_mesh(half_height: float, radius: float, segments: int = _CYLINDER_SEGMENTS) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Cylinder along the Y axis, ``2*half_height`` tall, with end caps.

    Side vertices use radial normals (smooth around the side); cap
    vertices use ±Y normals (flat caps). The two groups are kept separate
    so caps look crisp.
    """
    n = max(3, int(segments))
    h = float(half_height)
    r = float(radius)
    # Side: 2*(n+1) vertices (loop top + loop bottom, vertex duplicated for wrap).
    j = np.arange(n + 1, dtype=np.float32)
    phi = j * (2 * math.pi / n)
    cx = np.cos(phi).astype(np.float32) * r
    cz = np.sin(phi).astype(np.float32) * r
    nx = np.cos(phi).astype(np.float32)
    nz = np.sin(phi).astype(np.float32)
    side_pos_bot = np.stack([cx, np.full_like(cx, -h), cz], axis=-1)
    side_pos_top = np.stack([cx, np.full_like(cx,  h), cz], axis=-1)
    side_pos = np.concatenate([side_pos_bot, side_pos_top], axis=0)
    side_nrm = np.tile(np.stack([nx, np.zeros_like(nx), nz], axis=-1), (2, 1))
    # Side indices.
    base = 0
    bottom = base + np.arange(n, dtype=np.uint32)
    top = bottom + (n + 1)
    bottom_next = bottom + 1
    top_next = top + 1
    side_idx = np.stack([
        np.stack([bottom, bottom_next, top_next], axis=-1),
        np.stack([bottom, top_next, top], axis=-1),
    ], axis=1).reshape(-1, 3).astype(np.uint32)

    # Caps: center + ring (duplicated vertices for flat top/bottom normals).
    cap_phi = (np.arange(n, dtype=np.float32)) * (2 * math.pi / n)
    cap_cx = (np.cos(cap_phi) * r).astype(np.float32)
    cap_cz = (np.sin(cap_phi) * r).astype(np.float32)
    # Bottom cap.
    bot_center = np.array([[0.0, -h, 0.0]], dtype=np.float32)
    bot_ring = np.stack([cap_cx, np.full_like(cap_cx, -h), cap_cz], axis=-1)
    bot_pos = np.concatenate([bot_center, bot_ring], axis=0)
    bot_nrm = np.tile(np.array([[0.0, -1.0, 0.0]], dtype=np.float32), (bot_pos.shape[0], 1))
    # Top cap.
    top_center = np.array([[0.0, h, 0.0]], dtype=np.float32)
    top_ring = np.stack([cap_cx, np.full_like(cap_cx, h), cap_cz], axis=-1)
    top_pos = np.concatenate([top_center, top_ring], axis=0)
    top_nrm = np.tile(np.array([[0.0, 1.0, 0.0]], dtype=np.float32), (top_pos.shape[0], 1))

    side_count = side_pos.shape[0]
    bot_offset = side_count
    top_offset = bot_offset + bot_pos.shape[0]

    # Bottom triangle fan (CW so normal points down).
    bot_i = np.arange(n, dtype=np.uint32)
    bot_tris = np.stack([
        np.full(n, bot_offset, dtype=np.uint32),                # center
        bot_offset + 1 + (bot_i + 1) % n,                       # next ring vertex
        bot_offset + 1 + bot_i,                                 # current ring vertex
    ], axis=-1)
    # Top triangle fan (CCW so normal points up).
    top_tris = np.stack([
        np.full(n, top_offset, dtype=np.uint32),
        top_offset + 1 + bot_i,
        top_offset + 1 + (bot_i + 1) % n,
    ], axis=-1)

    positions = np.concatenate([side_pos, bot_pos, top_pos], axis=0).astype(np.float32, copy=False)
    normals = np.concatenate([side_nrm, bot_nrm, top_nrm], axis=0).astype(np.float32, copy=False)
    indices = np.concatenate([side_idx, bot_tris, top_tris], axis=0).astype(np.uint32, copy=False)
    return positions, normals, indices


def _cone_mesh(half_height: float, radius: float, segments: int = _CONE_SEGMENTS) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Cone with tip at ``(0, +half_height, 0)`` and base at ``(0, -half_height, 0)``.

    Side uses smooth normals around the cone; the base cap has a flat
    downward normal.
    """
    n = max(3, int(segments))
    h = float(half_height)
    r = float(radius)
    # Side: tip + base ring (n+1 ring vertices for seamed UV, but we use
    # only positions/normals so we duplicate the tip per-segment to give
    # each side triangle its own per-vertex normal.
    j = np.arange(n + 1, dtype=np.float32)
    phi = j * (2 * math.pi / n)
    cx = (np.cos(phi) * r).astype(np.float32)
    cz = (np.sin(phi) * r).astype(np.float32)
    # Smooth side normal: cone slant in (cos(phi), slope_y, sin(phi))
    # where slope_y = r / sqrt(r^2 + (2h)^2) — points slightly up.
    slant_len = math.hypot(r, 2.0 * h)
    side_y = (r / slant_len) if slant_len > 0 else 0.0
    side_xz_scale = ((2.0 * h) / slant_len) if slant_len > 0 else 1.0
    ring_pos = np.stack([cx, np.full_like(cx, -h), cz], axis=-1)
    tip_pos = np.tile(np.array([[0.0, h, 0.0]], dtype=np.float32), (n + 1, 1))
    ring_nrm = np.stack([
        np.cos(phi).astype(np.float32) * side_xz_scale,
        np.full(n + 1, side_y, dtype=np.float32),
        np.sin(phi).astype(np.float32) * side_xz_scale,
    ], axis=-1)
    # Tip normal: average of two adjacent ring normals (smooth but the
    # tip apex itself is a singularity; this approximation is fine).
    side_pos = np.concatenate([ring_pos, tip_pos], axis=0)
    side_nrm = np.concatenate([ring_nrm, ring_nrm], axis=0)
    side_idx = np.stack([
        np.arange(n, dtype=np.uint32),
        np.arange(n, dtype=np.uint32) + 1,
        np.arange(n, dtype=np.uint32) + (n + 1),
    ], axis=-1)

    # Base cap (flat normal down).
    cap_phi = np.arange(n, dtype=np.float32) * (2 * math.pi / n)
    cap_cx = (np.cos(cap_phi) * r).astype(np.float32)
    cap_cz = (np.sin(cap_phi) * r).astype(np.float32)
    cap_center = np.array([[0.0, -h, 0.0]], dtype=np.float32)
    cap_ring = np.stack([cap_cx, np.full_like(cap_cx, -h), cap_cz], axis=-1)
    cap_pos = np.concatenate([cap_center, cap_ring], axis=0)
    cap_nrm = np.tile(np.array([[0.0, -1.0, 0.0]], dtype=np.float32), (cap_pos.shape[0], 1))

    side_count = side_pos.shape[0]
    cap_offset = side_count
    cap_i = np.arange(n, dtype=np.uint32)
    cap_tris = np.stack([
        np.full(n, cap_offset, dtype=np.uint32),
        cap_offset + 1 + (cap_i + 1) % n,
        cap_offset + 1 + cap_i,
    ], axis=-1)

    positions = np.concatenate([side_pos, cap_pos], axis=0).astype(np.float32, copy=False)
    normals = np.concatenate([side_nrm, cap_nrm], axis=0).astype(np.float32, copy=False)
    indices = np.concatenate([side_idx, cap_tris], axis=0).astype(np.uint32, copy=False)
    return positions, normals, indices


def _capsule_mesh(a: Tuple[float, float, float], b: Tuple[float, float, float], radius: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Capsule = cylinder + two hemispheres, with the axis along (b - a).

    The mesh is built in a local frame where the cylinder runs along
    ``Y``, then rotated to align ``+Y`` with ``b - a`` and translated so
    its midpoint is ``(a + b) / 2``. We bake the rotation directly into
    the vertex positions so callers don't need to apply an extra
    transform.
    """
    ax = float(a[0]); ay = float(a[1]); az = float(a[2])
    bx = float(b[0]); by = float(b[1]); bz = float(b[2])
    dx, dy, dz = bx - ax, by - ay, bz - az
    seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
    half_seg = seg_len / 2.0

    # Cylinder along Y, then two hemispheres flipped to point ±Y.
    cy_pos, cy_nrm, cy_idx = _cylinder_mesh(half_seg, radius, segments=_CAPSULE_LON)
    # Drop the cylinder's flat caps — they live inside the hemispheres.
    n_cyl = _CAPSULE_LON
    # _cylinder_mesh: side verts first ((n+1)*2 = 2n+2), then bot cap (n+1), then top cap (n+1).
    side_v = (n_cyl + 1) * 2
    cy_pos = cy_pos[:side_v]
    cy_nrm = cy_nrm[:side_v]
    cy_idx = cy_idx[: 2 * n_cyl]  # only the side quads.

    top_hemi_pos, top_hemi_nrm, top_hemi_idx = _hemisphere_mesh(radius, upward=True, segments=_CAPSULE_LON, lat=_CAPSULE_HEMI_LAT)
    bot_hemi_pos, bot_hemi_nrm, bot_hemi_idx = _hemisphere_mesh(radius, upward=False, segments=_CAPSULE_LON, lat=_CAPSULE_HEMI_LAT)
    # Shift hemispheres to the capsule ends.
    top_hemi_pos = top_hemi_pos + np.array([0.0, half_seg, 0.0], dtype=np.float32)
    bot_hemi_pos = bot_hemi_pos + np.array([0.0, -half_seg, 0.0], dtype=np.float32)

    offset_cy = 0
    offset_top = cy_pos.shape[0]
    offset_bot = offset_top + top_hemi_pos.shape[0]
    positions = np.concatenate([cy_pos, top_hemi_pos, bot_hemi_pos], axis=0)
    normals = np.concatenate([cy_nrm, top_hemi_nrm, bot_hemi_nrm], axis=0)
    indices = np.concatenate([
        cy_idx + offset_cy,
        top_hemi_idx + offset_top,
        bot_hemi_idx + offset_bot,
    ], axis=0)

    # Now rotate from +Y to (b - a) direction and translate to midpoint.
    if seg_len > 1e-9:
        axis = np.array([dx, dy, dz], dtype=np.float32) / seg_len
        up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        cosang = float(np.dot(up, axis))
        if cosang < 0.999999:
            if cosang > -0.999999:
                rot_axis = np.cross(up, axis)
                rot_axis /= np.linalg.norm(rot_axis)
                angle = math.acos(max(-1.0, min(1.0, cosang)))
                R = _axis_angle_matrix(rot_axis, angle).astype(np.float32)
            else:
                # 180° flip: rotate about X.
                R = np.diag([1.0, -1.0, -1.0]).astype(np.float32)
            positions = positions @ R.T
            normals = normals @ R.T
    mid = np.array([(ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5], dtype=np.float32)
    positions = positions + mid
    return positions.astype(np.float32, copy=False), normals.astype(np.float32, copy=False), indices.astype(np.uint32, copy=False)


def _hemisphere_mesh(radius: float, *, upward: bool, segments: int, lat: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Upper (``upward=True``) or lower hemisphere centered at origin."""
    lat = max(2, int(lat))
    lon = max(3, int(segments))
    i = np.arange(lat + 1, dtype=np.float32)
    j = np.arange(lon + 1, dtype=np.float32)
    if upward:
        theta = (i * (math.pi * 0.5 / lat))[:, None]  # 0..pi/2
    else:
        theta = (math.pi * 0.5 + i * (math.pi * 0.5 / lat))[:, None]  # pi/2..pi
    phi = (j * (2 * math.pi / lon))[None, :]
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    sin_p = np.sin(phi)
    cos_p = np.cos(phi)
    x = sin_t * cos_p
    y = cos_t * np.ones_like(phi)
    z = sin_t * sin_p
    normals = np.stack([x, y, z], axis=-1).reshape(-1, 3).astype(np.float32)
    positions = normals * float(radius)
    a, b = np.meshgrid(np.arange(lat, dtype=np.uint32),
                       np.arange(lon, dtype=np.uint32),
                       indexing="ij")
    v00 = a * (lon + 1) + b
    v01 = v00 + 1
    v10 = v00 + (lon + 1)
    v11 = v10 + 1
    tris = np.stack([
        np.stack([v00, v10, v11], axis=-1),
        np.stack([v00, v11, v01], axis=-1),
    ], axis=2).reshape(-1, 3).astype(np.uint32)
    return positions, normals, tris


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """3×3 rotation matrix from axis-angle (Rodrigues)."""
    x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
    c = math.cos(angle); s = math.sin(angle); C = 1.0 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ], dtype=np.float64)


def _triangle_mesh(a: Tuple[float, float, float], b: Tuple[float, float, float], c: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Double-sided triangle: 6 vertices (3 per side), with opposite normals."""
    va = np.array(a, dtype=np.float32)
    vb = np.array(b, dtype=np.float32)
    vc = np.array(c, dtype=np.float32)
    edge1 = vb - va
    edge2 = vc - va
    n = np.cross(edge1, edge2)
    n_len = float(np.linalg.norm(n))
    if n_len > 1e-12:
        n /= n_len
    pos = np.stack([va, vb, vc, va, vc, vb], axis=0).astype(np.float32, copy=False)
    normals = np.empty((6, 3), dtype=np.float32)
    normals[0:3] = n
    normals[3:6] = -n
    idx = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.uint32)
    return pos, normals, idx


def _trimesh_mesh(vertices: np.ndarray, indices: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangle mesh with per-face (flat) normals.

    Expands each triangle into 3 unique vertices so adjacent faces don't
    share normals — gives nice faceted look matching the Rust testbed.
    """
    v = np.asarray(vertices, dtype=np.float32)
    idx = np.asarray(indices, dtype=np.uint32).reshape(-1, 3)
    if idx.size == 0:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    tri_v = v[idx.reshape(-1)].reshape(-1, 3, 3)        # (T, 3, 3)
    e1 = tri_v[:, 1] - tri_v[:, 0]
    e2 = tri_v[:, 2] - tri_v[:, 0]
    n = np.cross(e1, e2)
    n_len = np.linalg.norm(n, axis=1, keepdims=True)
    n_len = np.where(n_len > 1e-12, n_len, 1.0)
    n = (n / n_len).astype(np.float32)
    positions = tri_v.reshape(-1, 3).astype(np.float32, copy=False)
    normals = np.repeat(n, 3, axis=0)
    out_idx = np.arange(positions.shape[0], dtype=np.uint32).reshape(-1, 3)
    return positions, normals, out_idx


def _heightfield_mesh_3d(heights: np.ndarray, scale: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangulate a Rapier 3D heightfield.

    Mirrors :func:`rapier_testbed.examples3.trimesh3._heightfield_to_trimesh`:
    rows ``nrows``, cols ``ncols``, with ``(x, y, z) = ((c - 0.5) * sx,
    h * sy, (r - 0.5) * sz)``. Two triangles per cell.
    """
    h = np.asarray(heights, dtype=np.float32)
    if h.ndim != 2 or h.shape[0] < 2 or h.shape[1] < 2:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    nrows, ncols = h.shape
    sx, sy, sz = float(scale[0]), float(scale[1]), float(scale[2])
    rs = (np.arange(nrows, dtype=np.float32) / (nrows - 1) - 0.5) * sz
    cs = (np.arange(ncols, dtype=np.float32) / (ncols - 1) - 0.5) * sx
    cc, rr = np.meshgrid(cs, rs)
    yy = h * sy
    pts = np.stack([cc, yy, rr], axis=-1).reshape(-1, 3).astype(np.float32)
    # Two triangles per quad.
    r = np.arange(nrows - 1, dtype=np.uint32)
    c = np.arange(ncols - 1, dtype=np.uint32)
    rg, cg = np.meshgrid(r, c, indexing="ij")
    v00 = rg * ncols + cg
    v01 = v00 + 1
    v10 = v00 + ncols
    v11 = v10 + 1
    idx = np.stack([
        np.stack([v00, v10, v11], axis=-1),
        np.stack([v00, v11, v01], axis=-1),
    ], axis=2).reshape(-1, 3).astype(np.uint32)
    return _trimesh_mesh(pts, idx)


def _heightfield_mesh_2d(heights: np.ndarray, scale: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2D heightfield = polyline; render as a strip of vertical quads from
    ``y=h`` down to ``y=-large`` (we use ``y=0`` instead — just a polyline).
    """
    h = np.asarray(heights, dtype=np.float32).reshape(-1)
    if h.size < 2:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    sx, sy = float(scale[0]), float(scale[1])
    n = h.size
    xs = (np.arange(n, dtype=np.float32) / (n - 1) - 0.5) * sx
    ys = h * sy
    # Render as a thin filled strip from y=ys down to y=0.
    top = np.stack([xs, ys, np.zeros_like(xs)], axis=-1)
    bot = np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], axis=-1)
    pts = np.concatenate([top, bot], axis=0).astype(np.float32)
    quad_i = np.arange(n - 1, dtype=np.uint32)
    v_top = quad_i
    v_top_next = quad_i + 1
    v_bot = quad_i + n
    v_bot_next = quad_i + n + 1
    idx = np.stack([
        np.stack([v_top, v_bot, v_bot_next], axis=-1),
        np.stack([v_top, v_bot_next, v_top_next], axis=-1),
    ], axis=1).reshape(-1, 3).astype(np.uint32)
    return _trimesh_mesh(pts, idx)


def _convex_polyhedron_mesh(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convex polyhedron via a tiny QuickHull (NumPy-only).

    Returns a per-face flat-shaded mesh. Falls back to the AABB if the
    hull build fails (≤3 unique points).
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = np.unique(pts, axis=0)
    if pts.shape[0] < 4:
        # Degenerate: emit a tiny box around the centroid so something
        # at least shows up in the viewport.
        if pts.shape[0] == 0:
            return _cuboid_mesh(0.01, 0.01, 0.01)
        c = pts.mean(axis=0).astype(np.float32)
        pos, nrm, idx = _cuboid_mesh(0.05, 0.05, 0.05)
        return pos + c, nrm, idx
    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(pts)
        verts = hull.points.astype(np.float32)
        faces = hull.simplices.astype(np.uint32)
        return _trimesh_mesh(verts, faces)
    except Exception:
        # No scipy — fall back to AABB of the points.
        mn = pts.min(axis=0)
        mx = pts.max(axis=0)
        hx, hy, hz = (mx - mn) * 0.5
        center = (mn + mx) * 0.5
        pos, nrm, idx = _cuboid_mesh(float(hx), float(hy), float(hz))
        return pos + center.astype(np.float32), nrm, idx


def _disk_mesh_2d(radius: float, segments: int = 36) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2D ball → filled disk in the z=0 plane, normal +Z."""
    n = max(6, int(segments))
    r = float(radius)
    phi = np.arange(n, dtype=np.float32) * (2 * math.pi / n)
    cx = (np.cos(phi) * r).astype(np.float32)
    cy = (np.sin(phi) * r).astype(np.float32)
    center = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    ring = np.stack([cx, cy, np.zeros_like(cx)], axis=-1)
    pos = np.concatenate([center, ring], axis=0).astype(np.float32)
    nrm = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=np.float32), (pos.shape[0], 1))
    i_arr = np.arange(n, dtype=np.uint32)
    idx = np.stack([
        np.zeros(n, dtype=np.uint32),
        1 + i_arr,
        1 + (i_arr + 1) % n,
    ], axis=-1).astype(np.uint32)
    return pos, nrm, idx


def _convex_polygon_mesh_2d(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2D convex polygon → fan triangulation in the z=0 plane.

    Normals point along +Z (out of the screen) so the shape lights up
    correctly under the default front-facing directional light.
    """
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    n = pts.shape[0]
    if n < 3:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    pos3 = np.stack([pts[:, 0], pts[:, 1], np.zeros(n, dtype=np.float32)], axis=-1)
    nrm = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=np.float32), (n, 1))
    i = np.arange(1, n - 1, dtype=np.uint32)
    idx = np.stack([np.zeros_like(i), i, i + 1], axis=-1)
    return pos3, nrm, idx


def _polyline_mesh_2d(vertices: np.ndarray, indices: np.ndarray | None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2D polyline → thin extruded strip in the z=0 plane.

    We render the polyline as a flat strip 0.02 units wide (in world
    units) so it's at least visible. For something fancier (per-segment
    normals etc.) we'd need offset polygons; this is the simplest
    approach.
    """
    v = np.asarray(vertices, dtype=np.float32).reshape(-1, 2)
    if v.shape[0] < 2:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    if indices is None:
        idx = np.stack([np.arange(v.shape[0] - 1, dtype=np.uint32),
                        np.arange(1, v.shape[0], dtype=np.uint32)], axis=-1)
    else:
        idx = np.asarray(indices, dtype=np.uint32).reshape(-1, 2)

    # Render each segment as a thin quad in the z=0 plane.
    half_w = 0.02
    verts_out: List[np.ndarray] = []
    tris_out: List[np.ndarray] = []
    cur_base = 0
    for seg_a, seg_b in idx:
        a = v[seg_a]
        b = v[seg_b]
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-9:
            continue
        d /= L
        perp = np.array([-d[1], d[0]], dtype=np.float32) * half_w
        p0 = np.array([a[0] - perp[0], a[1] - perp[1], 0.0], dtype=np.float32)
        p1 = np.array([a[0] + perp[0], a[1] + perp[1], 0.0], dtype=np.float32)
        p2 = np.array([b[0] + perp[0], b[1] + perp[1], 0.0], dtype=np.float32)
        p3 = np.array([b[0] - perp[0], b[1] - perp[1], 0.0], dtype=np.float32)
        verts_out.append(np.stack([p0, p1, p2, p3], axis=0))
        tris_out.append(np.array([[cur_base, cur_base + 1, cur_base + 2],
                                  [cur_base, cur_base + 2, cur_base + 3]], dtype=np.uint32))
        cur_base += 4
    if not verts_out:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.uint32))
    positions = np.concatenate(verts_out, axis=0).astype(np.float32, copy=False)
    indices_out = np.concatenate(tris_out, axis=0).astype(np.uint32, copy=False)
    normals = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=np.float32), (positions.shape[0], 1))
    return positions, normals, indices_out


# ---------------------------------------------------------------------------
# Dispatch + Geom assembly
# ---------------------------------------------------------------------------


def _identity_iso(dim: int) -> Any:
    """Return a fresh identity isometry for the requested dimension."""
    if dim == 2:
        import rapier2d as rp2
        return rp2.Isometry2.identity()
    import rapier3d as rp3
    return rp3.Isometry3.identity()


def _shape_to_arrays(shape: Any, dim: int) -> List[Tuple[Any, np.ndarray, np.ndarray, np.ndarray]]:
    """Lower a SharedShape into a flat list of ``(local_pose, pos, nrm, idx)``.

    Compound shapes recurse and prepend their sub-shape poses; everything
    else returns a single entry whose pose is identity.
    """
    out: List[Tuple[Any, np.ndarray, np.ndarray, np.ndarray]] = []
    # Use the integer code: the 2D and 3D ShapeType enums are different
    # Python objects but share the same numeric values.
    stype = int(shape.shape_type)
    iso_identity = _identity_iso(dim)

    if stype == _ST_BALL:
        ball = shape.as_ball()
        if dim == 2:
            # 2D ball = filled disk in the (x, 0, y) plane.
            p, n, i = _disk_mesh_2d(ball.radius)
        else:
            p, n, i = _sphere_mesh(ball.radius)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CUBOID:
        cub = shape.as_cuboid()
        he = cub.half_extents
        if dim == 2:
            # 2D world lives in the z=0 plane; cuboid is a thin slab.
            p, n, i = _cuboid_mesh(he.x, he.y, 0.01)
        else:
            p, n, i = _cuboid_mesh(he.x, he.y, he.z)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CAPSULE:
        cap = shape.as_capsule()
        if dim == 2:
            # 2D world (x, y) → 3D (x, y, 0).
            ax = (float(cap.a.x), float(cap.a.y), 0.0)
            bx = (float(cap.b.x), float(cap.b.y), 0.0)
        else:
            ax = (float(cap.a.x), float(cap.a.y), float(cap.a.z))
            bx = (float(cap.b.x), float(cap.b.y), float(cap.b.z))
        p, n, i = _capsule_mesh(ax, bx, cap.radius)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CYLINDER:
        cy = shape.as_cylinder()
        p, n, i = _cylinder_mesh(cy.half_height, cy.radius)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CONE:
        co = shape.as_cone()
        p, n, i = _cone_mesh(co.half_height, co.radius)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_TRIANGLE:
        tri = shape.as_triangle()
        if dim == 2:
            # 2D world (x, y) → 3D (x, y, 0).
            a = (float(tri.a.x), float(tri.a.y), 0.0)
            b = (float(tri.b.x), float(tri.b.y), 0.0)
            c = (float(tri.c.x), float(tri.c.y), 0.0)
        else:
            a = (float(tri.a.x), float(tri.a.y), float(tri.a.z))
            b = (float(tri.b.x), float(tri.b.y), float(tri.b.z))
            c = (float(tri.c.x), float(tri.c.y), float(tri.c.z))
        p, n, i = _triangle_mesh(a, b, c)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_SEGMENT:
        # 2D-only; render as a tiny polyline strip.
        if dim == 2:
            import rapier2d as rp2
            shape_seg = shape.as_segment()
            v = np.array([[shape_seg.a.x, shape_seg.a.y], [shape_seg.b.x, shape_seg.b.y]], dtype=np.float32)
            p, n, i = _polyline_mesh_2d(v, None)
            out.append((iso_identity, p, n, i))
            return out
        # 3D segment isn't exposed via SharedShape constructors; fall through to AABB.

    if stype == _ST_TRIMESH:
        tm = shape.as_trimesh()
        v = np.asarray(tm.vertices, dtype=np.float32)
        if v.ndim == 2 and v.shape[1] == 2:
            # 2D trimesh: lift into z=0 plane (x, y, 0).
            v3 = np.empty((v.shape[0], 3), dtype=np.float32)
            v3[:, 0] = v[:, 0]
            v3[:, 1] = v[:, 1]
            v3[:, 2] = 0.0
            v = v3
        p, n, i = _trimesh_mesh(v, tm.indices)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_HEIGHTFIELD:
        hf = shape.as_heightfield()
        sc = hf.scale
        if dim == 2:
            heights = np.asarray(hf.heights, dtype=np.float32)
            p, n, i = _heightfield_mesh_2d(heights, (float(sc.x), float(sc.y)))
        else:
            heights = np.asarray(hf.heights, dtype=np.float32)
            p, n, i = _heightfield_mesh_3d(
                heights, (float(sc.x), float(sc.y), float(sc.z))
            )
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CONVEX_POLYHEDRON:
        ch = shape.as_convex_polyhedron()
        p, n, i = _convex_polyhedron_mesh(ch.points)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_CONVEX_POLYGON:
        # 2D convex polygon.
        cp = shape.as_convex_polygon()
        p, n, i = _convex_polygon_mesh_2d(cp.points)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_POLYLINE:
        po = shape.as_polyline()
        p, n, i = _polyline_mesh_2d(po.vertices, po.indices)
        out.append((iso_identity, p, n, i))
        return out

    if stype == _ST_COMPOUND:
        comp = shape.as_compound()
        for sub_iso, sub_shape in comp.shapes():
            for inner_iso, p, n, i in _shape_to_arrays(sub_shape, dim):
                # Compose sub_iso ∘ inner_iso.
                if dim == 2:
                    pose = _compose_iso_2d(sub_iso, inner_iso)
                else:
                    pose = _compose_iso_3d(sub_iso, inner_iso)
                out.append((pose, p, n, i))
        return out

    # Round shapes: render the underlying shape, ignoring the border radius.
    if stype == _ST_ROUND_CUBOID:
        cub = shape.as_round_cuboid()
        he = cub.half_extents
        if dim == 2:
            p, n, i = _cuboid_mesh(he.x, he.y, 0.01)
        else:
            p, n, i = _cuboid_mesh(he.x, he.y, he.z)
        out.append((iso_identity, p, n, i))
        return out
    if stype == _ST_ROUND_TRIANGLE:
        tri = shape.as_round_triangle()
        if dim == 2:
            a = (float(tri.a.x), float(tri.a.y), 0.0)
            b = (float(tri.b.x), float(tri.b.y), 0.0)
            c = (float(tri.c.x), float(tri.c.y), 0.0)
        else:
            a = (float(tri.a.x), float(tri.a.y), float(tri.a.z))
            b = (float(tri.b.x), float(tri.b.y), float(tri.b.z))
            c = (float(tri.c.x), float(tri.c.y), float(tri.c.z))
        p, n, i = _triangle_mesh(a, b, c)
        out.append((iso_identity, p, n, i))
        return out
    if stype == _ST_ROUND_CYLINDER:
        cy = shape.as_round_cylinder()
        p, n, i = _cylinder_mesh(cy.half_height, cy.radius)
        out.append((iso_identity, p, n, i))
        return out
    if stype == _ST_ROUND_CONE:
        co = shape.as_round_cone()
        p, n, i = _cone_mesh(co.half_height, co.radius)
        out.append((iso_identity, p, n, i))
        return out
    if stype == _ST_ROUND_CONVEX_POLYHEDRON:
        ch = shape.as_round_convex_polyhedron()
        p, n, i = _convex_polyhedron_mesh(ch.points)
        out.append((iso_identity, p, n, i))
        return out
    if stype == _ST_ROUND_CONVEX_POLYGON:
        cp = shape.as_round_convex_polygon()
        p, n, i = _convex_polygon_mesh_2d(cp.points)
        out.append((iso_identity, p, n, i))
        return out

    # HALFSPACE, VOXELS, CUSTOM — no triangulation available, draw the AABB.
    return _aabb_fallback(shape, dim)


def _aabb_fallback(shape: Any, dim: int) -> List[Tuple[Any, np.ndarray, np.ndarray, np.ndarray]]:
    """Approximate a shape with its AABB, baking offset into the geometry."""
    iso = _identity_iso(dim)
    a = shape.compute_aabb(iso)
    if dim == 2:
        # 2D AABB has 2-component points; render as a flat slab in the z=0
        # plane (thin Z extent).
        hx = float(a.half_extents.x)
        hy = float(a.half_extents.y)
        cx = float(a.center.x)
        cy = float(a.center.y)
        p, n, i = _cuboid_mesh(hx, hy, 0.01)
        p = p + np.array([cx, cy, 0.0], dtype=np.float32)
    else:
        hx = float(a.half_extents.x)
        hy = float(a.half_extents.y)
        hz = float(a.half_extents.z)
        cx = float(a.center.x)
        cy = float(a.center.y)
        cz = float(a.center.z)
        p, n, i = _cuboid_mesh(hx, hy, hz)
        p = p + np.array([cx, cy, cz], dtype=np.float32)
    return [(iso, p, n, i)]


def _compose_iso_3d(outer: Any, inner: Any) -> Any:
    """Return ``outer * inner`` as a fresh Isometry3 (right-multiply)."""
    import rapier3d as rp3

    # Compose using to_matrix → 4x4 multiply → reconstruct. The bindings
    # don't expose `*` on Isometry directly, but the matrix path is fast
    # enough and runs once per (collider, shape) lifetime.
    # to_matrix() returns a flat-16 row-major array.
    om = np.asarray(outer.to_matrix(), dtype=np.float64).reshape(4, 4)
    im = np.asarray(inner.to_matrix(), dtype=np.float64).reshape(4, 4)
    m = om @ im
    rot_m = m[:3, :3]
    trans = m[:3, 3]
    # Extract quaternion from rotation matrix.
    qw, qx, qy, qz = _mat3_to_quat(rot_m)
    rot = rp3.Rotation3.from_quaternion(qw, qx, qy, qz)
    return rp3.Isometry3((float(trans[0]), float(trans[1]), float(trans[2])), rot)


def _compose_iso_2d(outer: Any, inner: Any) -> Any:
    """Return ``outer * inner`` as a fresh Isometry2."""
    import rapier2d as rp2

    a_o = float(outer.rotation.angle)
    a_i = float(inner.rotation.angle)
    t_i_x = float(inner.translation.x)
    t_i_y = float(inner.translation.y)
    # Outer rotation applied to inner translation, then plus outer translation.
    c, s = math.cos(a_o), math.sin(a_o)
    tx = float(outer.translation.x) + c * t_i_x - s * t_i_y
    ty = float(outer.translation.y) + s * t_i_x + c * t_i_y
    rot = rp2.Rotation2.from_angle(a_o + a_i)
    return rp2.Isometry2((tx, ty), rot)


def _mat3_to_quat(m: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3×3 rotation matrix to (w, x, y, z)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S
    return float(qw), float(qx), float(qy), float(qz)


# ---------------------------------------------------------------------------
# Flat shading
# ---------------------------------------------------------------------------


def _flatten_shading(
    pos: np.ndarray, _nrm: np.ndarray, idx: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Re-emit the mesh with one flat normal per triangle.

    The input ``_nrm`` is ignored; we recompute per-face normals from
    the triangle winding (``(v1-v0) × (v2-v0)``, normalized). Each
    triangle's three vertices are duplicated so per-face normals don't
    get interpolated — giving sharp faceted shading on every shape.

    Computing the normal from the actual winding also means lighting
    can never disagree with the visible side of the face.
    """
    nt = int(idx.shape[0])
    if nt == 0:
        return pos, _nrm, idx

    v0 = pos[idx[:, 0]]
    v1 = pos[idx[:, 1]]
    v2 = pos[idx[:, 2]]
    face_n = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(face_n, axis=1, keepdims=True)
    face_n = face_n / np.maximum(norms, 1e-12)

    new_pos = np.empty((3 * nt, 3), dtype=np.float32)
    new_pos[0::3] = v0
    new_pos[1::3] = v1
    new_pos[2::3] = v2

    new_nrm = np.empty((3 * nt, 3), dtype=np.float32)
    new_nrm[0::3] = face_n
    new_nrm[1::3] = face_n
    new_nrm[2::3] = face_n

    new_idx = np.arange(3 * nt, dtype=np.uint32).reshape(nt, 3)
    return new_pos.astype(np.float32, copy=False), new_nrm.astype(np.float32, copy=False), new_idx


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def shape_to_meshes(
    shape: Any, *, dim: int
) -> List[Tuple[Any, np.ndarray, np.ndarray, np.ndarray]]:
    """Decompose a :class:`SharedShape` into raw flat-shaded NumPy mesh data.

    Returns a list of ``(local_pose, positions, normals, indices)`` —
    one entry per primitive (compound shapes contribute multiple).
    ``positions`` is ``(V, 3) float32``, ``normals`` is ``(V, 3)
    float32``, ``indices`` is ``(T, 3) uint32``.

    All shapes are emitted with **flat shading**: each triangle has a
    single face normal computed from its winding, with vertices
    duplicated so the normal isn't interpolated across the face.
    """
    raw = _shape_to_arrays(shape, dim)
    return [(pose, *_flatten_shading(p, n, i)) for pose, p, n, i in raw]


def shape_to_geoms(
    shape: Any, *, dim: int, color_rgba: Tuple[float, float, float, float]
) -> List[Tuple[Any, Any]]:
    """Convert a Rapier ``SharedShape`` into a list of ``(local_pose, Geom)`` pairs.

    Imports Panda3D lazily so the calling test/CI doesn't need a working
    GL stack when it only exercises the NumPy path.
    """
    from panda3d.core import (
        Geom,
        GeomNode,  # noqa: F401 — kept for downstream consumers if any
        GeomTriangles,
        GeomVertexData,
        GeomVertexFormat,
    )

    raw_smooth = _shape_to_arrays(shape, dim)
    # Convert every shape to flat shading: compute per-face normals from
    # the actual triangle winding and explode vertices so the normal
    # isn't interpolated. This also fixes any winding/stored-normal
    # disagreement that would otherwise show as "inverted" lighting.
    raw = [(pose, *_flatten_shading(p, n, i)) for pose, p, n, i in raw_smooth]
    result: List[Tuple[Any, Any]] = []
    cr, cg, cb, ca = (float(v) for v in color_rgba)
    color_u8 = np.array([
        max(0, min(255, int(cr * 255.0))),
        max(0, min(255, int(cg * 255.0))),
        max(0, min(255, int(cb * 255.0))),
        max(0, min(255, int(ca * 255.0))),
    ], dtype=np.uint8)

    fmt = GeomVertexFormat.getV3n3c4()
    interleaved_dtype = np.dtype([
        ("pos", np.float32, 3),
        ("nrm", np.float32, 3),
        ("color", np.uint8, 4),
    ])

    for local_pose, pos, nrm, idx in raw:
        nv = int(pos.shape[0])
        nt = int(idx.shape[0])
        vdata = GeomVertexData("rapier-mesh", fmt, Geom.UHStatic)
        vdata.unclean_set_num_rows(nv)
        if nv > 0:
            buf = np.empty(nv, dtype=interleaved_dtype)
            buf["pos"] = pos
            buf["nrm"] = nrm
            buf["color"] = color_u8
            arr_handle = vdata.modifyArrayHandle(0)
            arr_handle.copyDataFrom(buf.tobytes())

        prim = GeomTriangles(Geom.UHStatic)
        if nt > 0:
            # Force the index buffer to uint32 (Panda picks uint16 by
            # default for ≤65k verts; this normalizes the layout).
            prim.setIndexType(Geom.NTUint32)
            prim.reserveNumVertices(int(nt * 3))
            # Per-triangle addVertices runs once per (collider, shape)
            # lifetime, not per frame — fast enough at <2k triangles
            # each.
            for t in idx:
                prim.addVertices(int(t[0]), int(t[1]), int(t[2]))
            prim.closePrimitive()
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        result.append((local_pose, geom))
    return result


# ---------------------------------------------------------------------------
# Deterministic per-body coloring
# ---------------------------------------------------------------------------


def _hsl_to_rgb(h: float, s: float, l: float) -> Tuple[float, float, float]:
    """HSL → RGB. ``h`` in [0,1), ``s`` and ``l`` in [0,1]."""
    if s == 0:
        return l, l, l

    def _hue_to_rgb(p: float, q: float, t: float) -> float:
        if t < 0.0:
            t += 1.0
        if t > 1.0:
            t -= 1.0
        if t < 1.0 / 6.0:
            return p + (q - p) * 6.0 * t
        if t < 0.5:
            return q
        if t < 2.0 / 3.0:
            return p + (q - p) * (2.0 / 3.0 - t) * 6.0
        return p

    q = l * (1.0 + s) if l < 0.5 else (l + s - l * s)
    p = 2.0 * l - q
    r = _hue_to_rgb(p, q, h + 1.0 / 3.0)
    g = _hue_to_rgb(p, q, h)
    b = _hue_to_rgb(p, q, h - 1.0 / 3.0)
    return r, g, b


def color_for_body(body_handle_index: int, body_type: Any) -> Tuple[float, float, float, float]:
    """Deterministic RGBA for a dynamic body's debug-render color.

    Fixed bodies get a neutral gray; kinematic bodies get a cool blue;
    dynamic bodies hash their handle index into a hue in the warm pastel
    palette.
    """
    # ``RigidBodyType`` is an ``IntEnum``; comparing as integer keeps
    # this dim-agnostic (2D and 3D enums are distinct objects).
    bt = int(body_type) if body_type is not None else -1
    # See ``rapier::dynamics::RigidBodyType``:
    # 0 = Dynamic, 1 = Fixed, 2 = KinematicPositionBased, 3 = KinematicVelocityBased.
    if bt == 1:
        return (0.60, 0.60, 0.60, 1.0)
    if bt == 2 or bt == 3:
        return (0.35, 0.55, 0.85, 1.0)
    # Dynamic (or unknown): golden-ratio hash → hue.
    h = ((int(body_handle_index) * 2654435769) % (1 << 32)) / float(1 << 32)
    r, g, b = _hsl_to_rgb(h, 0.55, 0.58)
    return (r, g, b, 1.0)


def color_for_free_collider(collider_handle_index: int) -> Tuple[float, float, float, float]:
    """Color for a collider with no parent body (anchored to world)."""
    return (0.55, 0.55, 0.55, 1.0)


__all__ = [
    "shape_to_geoms",
    "shape_to_meshes",
    "color_for_body",
    "color_for_free_collider",
]
