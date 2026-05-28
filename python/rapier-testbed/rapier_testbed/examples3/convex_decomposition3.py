"""Port of ``examples3d/convex_decomposition3.rs``.

The Rust example loads OBJ models from disk, converts each to a
``SharedShape::convex_decomposition`` and scatters copies on a wavy
heightfield. Loading OBJ files is not part of the bundled Rapier Python
bindings (no ``obj`` parser is shipped) so this port raises
``NotImplementedError``. Users that bring their own loader can call
``rapier.SharedShape.convex_decomposition(vertices, indices)`` directly.
"""
from __future__ import annotations

import rapier3d as rp  # noqa: F401  -- imported for symmetry with siblings

from .._registry import register

CATEGORY = "Collisions"
NAME = "Convex decomposition"


def init_world(testbed) -> None:
    raise NotImplementedError(
        "binding gap: examples3d/convex_decomposition3.rs depends on "
        "loading OBJ assets; the Python testbed has no OBJ parser plumbed."
    )


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
