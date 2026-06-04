"""Port of ``examples3d/convex_decomposition3.rs``.

The Rust example reuses ``dynamic_trimesh3::do_init_world`` with convex
decomposition enabled: the same OBJ models are dropped on a wavy trimesh
ground, but each is converted to a ``SharedShape::convex_decomposition``
(a compound of convex hulls) instead of a raw trimesh.
"""
from __future__ import annotations

from .._registry import register
from .dynamic_trimesh3 import do_init_world

CATEGORY = "Collisions"
NAME = "Convex decomposition"


def init_world(testbed) -> None:
    do_init_world(testbed, use_convex_decomposition=True)


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
