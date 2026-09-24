"""Port of examples3d/dynamic_trimesh3.rs.

A wavy heightfield-derived trimesh ground, with a grid of OBJ meshes (loaded
from ``assets/3d/``) dropped on top as dynamic trimesh colliders — each model
centered and scaled to a uniform size, exactly like the Rust example.

The meshes are read with the testbed's small built-in OBJ loader
(:mod:`rapier_testbed._obj`); if the ``assets/3d`` directory can't be located
the example raises :class:`NotImplementedError` (set ``RAPIER_REPO_ROOT`` to
the repository root).
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp

from .._obj import find_asset, load_obj, normalize_to_unit
from .._registry import register
from .trimesh3 import _heightfield_to_trimesh

CATEGORY = "Collisions"
NAME = "Dynamic trimeshes"

# Same model list as examples3d/dynamic_trimesh3.rs (the two commented-out
# hand meshes there are likewise omitted).
_MODELS = [
    "camel_decimated.obj",
    "chair.obj",
    "cup_decimated.obj",
    "dilo_decimated.obj",
    "tstTorusModel2.obj",
    "feline_decimated.obj",
    "genus3_decimated.obj",
    "hornbug.obj",
    "tstTorusModel.obj",
    "octopus_decimated.obj",
    "rabbit_decimated.obj",
    "rust_logo_simplified.obj",
    "screwdriver_decimated.obj",
    "table.obj",
    "tstTorusModel3.obj",
]


def init_world(testbed) -> None:
    do_init_world(testbed, use_convex_decomposition=False)


def do_init_world(testbed, use_convex_decomposition: bool) -> None:
    """Shared scene for the trimesh / convex-decomposition examples.

    Mirrors ``dynamic_trimesh3::do_init_world`` in the Rust testbed: the same
    OBJ models are dropped on a wavy trimesh ground either as raw trimeshes
    or as convex decompositions (``use_convex_decomposition``).
    """
    # Locate the asset directory up-front so a missing checkout fails clearly.
    first = find_asset(f"assets/3d/{_MODELS[0]}")
    if first is None:
        raise NotImplementedError(
            "binding gap: assets/3d/*.obj not located. Set RAPIER_REPO_ROOT "
            "to the repository root to load the trimesh models."
        )
    asset_dir = first.parent

    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Wavy ground built from a heightfield, then converted to a TriMesh — the
    # Rust example uses `TriMesh::from(heightfield)`. This matters for physics:
    # the dynamic objects are trimeshes, and trimesh-vs-heightfield generates
    # no contacts, so a heightfield ground would let them fall through. A
    # trimesh ground (with FIX_INTERNAL_EDGES pseudo-normals) collides.
    nsubdivs = 100
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            heights[i, j] = (
                -math.cos(i * 40.0 / nsubdivs / 2.0)
                - math.cos(j * 40.0 / nsubdivs / 2.0)
            )
    ground_verts, ground_idx = _heightfield_to_trimesh(heights, (100.0, 2.0, 100.0))
    colliders.insert(
        rp.Collider.trimesh(
            ground_verts, ground_idx, rp.TriMeshFlags.FIX_INTERNAL_EDGES
        )
    )

    # Load every model, centered and scaled to a uniform size, and turn it
    # into a reusable trimesh shape.
    shapes = []
    for name in _MODELS:
        path = asset_dir / name
        if not path.is_file():
            continue
        verts, idx = load_obj(path)
        if verts.shape[0] == 0 or idx.shape[0] == 0:
            continue
        verts = normalize_to_unit(verts, target_diag=10.0)
        try:
            if use_convex_decomposition:
                shape = rp.SharedShape.convex_decomposition(verts, idx)
            else:
                shape = rp.SharedShape.trimesh(
                    verts, idx, rp.TriMeshFlags.FIX_INTERNAL_EDGES
                )
        except Exception:
            # Degenerate mesh after dedup/topology fixes — skip it.
            continue
        shapes.append(shape)

    if not shapes:
        raise NotImplementedError(
            "binding gap: no OBJ models could be loaded from "
            f"{asset_dir}."
        )

    # Scatter duplicated copies above the ground, matching the Rust layout.
    width = int(math.sqrt(len(shapes)))
    width = max(width, 1)
    num_duplications = 4
    shift_y = 8.0
    shift_xz = 9.0
    for igeom, shape in enumerate(shapes):
        for k in range(1, num_duplications + 1):
            x = (igeom % width) * shift_xz - num_duplications * shift_xz / 2.0
            y = (igeom // width) * shift_y + 7.0
            z = k * shift_xz - num_duplications * shift_xz / 2.0
            body = rp.RigidBody.dynamic().translation((x, y, z))
            h = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.new(shape).contact_skin(0.1), h, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
