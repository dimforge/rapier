"""Port of examples3d/stress_tests/ray_cast3.rs.

Skipped: the Rust example dynamically picks another stress-test scene and
adds 10000 raycasts per frame using the ``QueryPipeline`` exposed by
``BroadPhaseBvh::as_query_pipeline``. The Python bindings don't yet
expose that bridge (or the per-frame ``settings`` UI), so we degrade
gracefully to a small static scene instead of raising.
"""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register
from .balls3 import init_world as _balls_init_world

CATEGORY = "Stress Tests"
NAME = "Ray cast"


def init_world(testbed) -> None:
    # Reuse the balls3 stress-test scene as a non-trivial collider set;
    # the actual per-frame raycasting from the Rust example is omitted
    # because ``BroadPhaseBvh::as_query_pipeline`` isn't exposed yet.
    _balls_init_world(testbed)


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
