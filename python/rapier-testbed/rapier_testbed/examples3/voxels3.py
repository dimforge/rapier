"""Port of examples3d/voxels3.rs.

Skipped: the Rust example builds a ``voxels_from_points`` collider plus
runtime voxel editing via the mouse. Neither
``ColliderBuilder::voxels_from_points`` nor voxel mutation is exposed to
Python yet, so we raise :class:`NotImplementedError` from
``init_world``.
"""
from __future__ import annotations

import rapier3d as rp  # noqa: F401  - keep parity with sibling modules
from .._registry import register

CATEGORY = "Collisions"
NAME = "Voxels"


def init_world(testbed) -> None:
    raise NotImplementedError(
        "binding gap: voxel colliders "
        "(Collider.voxels_from_points / set_voxel) are not exposed to Python."
    )


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
