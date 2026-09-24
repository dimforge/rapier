"""Port of examples3d/voxels3.rs.

A voxelized wavy floor (built with ``Collider.voxels_from_points``) with a
grid of dynamic primitives — balls, cuboids, cylinders, cones, capsules —
raining onto it. Each filled voxel renders as its own cube.

The Rust example also supports interactive voxel editing with the mouse
(add/remove voxels under the cursor); that relies on testbed ray-picking
hooks the Python testbed doesn't expose, so it's omitted here. Counts are
scaled down from the Rust demo to keep the Python viewer responsive.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Voxels"

# Grid resolution of the floor (Rust uses 200; scaled down for the viewer).
_N = 64
_VOXEL_SIZE_Y = 1.0
# How many voxels thick the floor is. The Rust demo samples only the surface;
# at 200×200 the per-cell slope is gentle enough that the single-voxel shell is
# watertight. At our lower resolution the surface steps by up to ~2 voxels
# between neighbors, which would leave gaps a falling object slips through, so
# we fill a few voxels deep to keep the floor solid.
_THICKNESS = 3


def _floor_samples():
    """Point cloud sampling a wavy surface, with raised walls at the edges."""
    pts = []
    n = _N
    for i in range(n):
        for j in range(n):
            y = (
                np.clip(math.sin(i / n * 10.0), -0.8, 0.8)
                * np.clip(math.cos(j / n * 10.0), -0.8, 0.8)
                * 16.0
            )
            # Fill downward from the surface so the floor is a solid slab.
            for d in range(_THICKNESS):
                pts.append((float(i), (y - d) * _VOXEL_SIZE_Y, float(j)))
            # Walls along the border so edge objects don't fall into the void.
            if i == 0 or i == n - 1 or j == 0 or j == n - 1:
                for k in range(1, 5):
                    pts.append((float(i), (y + k) * _VOXEL_SIZE_Y, float(j)))
    return np.asarray(pts, dtype=np.float32)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Voxelized wavy floor (one static voxel-grid collider).
    samples = _floor_samples()
    voxel_size = (1.0, _VOXEL_SIZE_Y, 1.0)
    colliders.insert(rp.Collider.voxels_from_points(voxel_size, samples))

    # Floor bounds, to scatter the falling objects over the surface.
    mins = samples.min(axis=0)
    maxs = samples.max(axis=0)
    extents = (maxs - mins) * 0.75
    margin = (maxs - mins - extents) / 2.0

    # Dynamic primitives raining down (kind cycles per layer, like "Mixed").
    nik = 12
    nlayers = 5
    ball_radius = 0.5
    for i in range(nik):
        for j in range(nlayers):
            for k in range(nik):
                x = mins[0] + margin[0] + i * extents[0] / nik
                y = maxs[1] + j * 2.0
                z = mins[2] + margin[2] + k * extents[2] / nik
                body = rp.RigidBody.dynamic().translation(
                    (float(x), float(y), float(z))
                )
                h = bodies.insert(body)
                kind = j % 5
                if kind == 0:
                    co = rp.Collider.ball(ball_radius)
                elif kind == 1:
                    co = rp.Collider.cuboid(ball_radius, ball_radius, ball_radius)
                elif kind == 2:
                    co = rp.Collider.cylinder(ball_radius, ball_radius)
                elif kind == 3:
                    co = rp.Collider.cone(ball_radius, ball_radius)
                else:
                    co = rp.Collider.capsule_y(ball_radius, ball_radius)
                colliders.insert_with_parent(co, h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    # Look at the middle of the floor from a corner.
    center = (maxs + mins) * 0.5
    testbed.look_at(
        (float(maxs[0]) + 30.0, float(maxs[1]) + 40.0, float(maxs[2]) + 30.0),
        (float(center[0]), float(center[1]), float(center[2])),
    )


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
