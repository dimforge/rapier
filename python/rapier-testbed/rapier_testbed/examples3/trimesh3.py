"""Port of examples3d/trimesh3.rs.

Reuses the heightfield-derived trimesh from
``stress_tests/trimesh3.rs`` style — a wavy ground built from a HeightField
converted to a trimesh — and rains a grid of mixed primitives on top. We
build the trimesh ourselves with numpy.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "TriMesh"


def _heightfield_to_trimesh(heights, scale):
    """Convert a (nrows+1, ncols+1) height grid into (vertices, indices).

    Mirrors ``HeightField::to_trimesh()``. Each cell is split into two
    triangles. ``scale`` is the heightfield's bounding extent ``(sx, sy, sz)``
    (note: ``sy`` is multiplied by the height values directly).
    """
    nrows = heights.shape[0] - 1
    ncols = heights.shape[1] - 1
    sx, sy, sz = scale
    # Match HeightField axis layout (XZ plane, heights along Y).
    verts = np.zeros((heights.shape[0] * heights.shape[1], 3), dtype=np.float32)
    for i in range(heights.shape[0]):
        for j in range(heights.shape[1]):
            x = (i / float(nrows) - 0.5) * sx
            z = (j / float(ncols) - 0.5) * sz
            verts[i * heights.shape[1] + j] = (x, heights[i, j] * sy, z)
    idx = []
    cols = heights.shape[1]
    for i in range(nrows):
        for j in range(ncols):
            a = i * cols + j
            b = a + 1
            c = (i + 1) * cols + j
            d = c + 1
            idx.append([a, b, d])
            idx.append([a, d, c])
    return verts, np.array(idx, dtype=np.uint32)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = (100.0, 1.0, 100.0)
    nsubdivs = 20
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            if i == 0 or i == nsubdivs or j == 0 or j == nsubdivs:
                heights[i, j] = 10.0
            else:
                x = i * ground_size[0] / nsubdivs
                z = j * ground_size[2] / nsubdivs
                heights[i, j] = math.sin(x) + math.cos(z)

    verts, idx = _heightfield_to_trimesh(heights, ground_size)
    ground = rp.RigidBody.fixed()
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.trimesh(verts, idx, rp.TriMeshFlags.MERGE_DUPLICATE_VERTICES),
        ground_h,
        bodies,
    )

    num = 8
    rad = 1.0
    shift = rad * 2.0 + rad
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)
    for j in range(20):
        for i in range(num):
            for k in range(num):
                x = i * shift - centerx
                y = j * shift + centery + 3.0
                z = k * shift - centerz
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                kind = j % 6
                if kind == 0:
                    col = rp.Collider.cuboid(rad, rad, rad)
                elif kind == 1:
                    col = rp.Collider.ball(rad)
                elif kind == 2:
                    col = rp.Collider.round_cylinder(rad, rad, rad / 10.0)
                elif kind == 3:
                    col = rp.Collider.cone(rad, rad)
                elif kind == 4:
                    col = rp.Collider.capsule_y(rad, rad)
                else:
                    shapes = [
                        (
                            rp.Isometry3.identity(),
                            rp.SharedShape.cuboid(rad, rad / 2.0, rad / 2.0),
                        ),
                        (
                            rp.Isometry3.from_translation(rad, 0.0, 0.0),
                            rp.SharedShape.cuboid(rad / 2.0, rad, rad / 2.0),
                        ),
                        (
                            rp.Isometry3.from_translation(-rad, 0.0, 0.0),
                            rp.SharedShape.cuboid(rad / 2.0, rad, rad / 2.0),
                        ),
                    ]
                    col = rp.Collider.compound(shapes)
                colliders.insert_with_parent(col, h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
