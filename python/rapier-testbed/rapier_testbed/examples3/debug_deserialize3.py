"""Port of ``examples3d/debug_deserialize3.rs``.

The Rust example loads a bincode-serialized world snapshot from a
developer-specific path on disk. The Python bindings don't expose
``PhysicsState`` deserialization at this granularity (snapshots are
opt-in via ``pickle`` on individual sets), so this port is a no-op that
documents the gap.
"""
from __future__ import annotations

import rapier3d as rp  # noqa: F401  -- imported for symmetry with siblings

from .._registry import register

CATEGORY = "Debug"
NAME = "Deserialize"


def init_world(testbed) -> None:
    raise NotImplementedError(
        "binding gap: examples3d/debug_deserialize3.rs deserializes a "
        "full PhysicsState from a hard-coded local path; the Python bindings "
        "don't expose that combined deserialization yet."
    )


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
