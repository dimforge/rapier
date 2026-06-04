"""2D debug-render tests (abbreviated).

Parametrized across the f32 (`rapier.dim2`) and f64 (`rapier.dim2.f64`)
flavors.
"""

from __future__ import annotations

import numpy as np
import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def _ball_on_ground_world(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1).build())
    bh = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(0, 5)).build()
    )
    w.colliders.insert_with_parent(
        ns.Collider.ball(0.5), bh, w.rigid_bodies
    )
    for _ in range(20):
        w.step()
    return w


def test_render_to_arrays_2d(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline()
    lines, colors, objects = pipe.render_to_arrays(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
    )
    assert isinstance(lines, np.ndarray)
    assert lines.ndim == 3
    assert lines.shape[1] == 2
    assert lines.shape[2] == 2  # 2D
    assert lines.shape[0] > 0
    assert colors.shape == (lines.shape[0], 4)
    assert objects.shape == (lines.shape[0],)


def test_collector_iter_2d(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline()
    coll = ns.DebugLineCollector()
    pipe.render(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
        coll,
    )
    items = list(coll)
    assert len(items) > 0
    obj, a, b, color = items[0]
    assert isinstance(obj, ns.DebugRenderObject)
    assert isinstance(a, ns.Vec2)
    assert isinstance(b, ns.Vec2)
    assert isinstance(color, ns.DebugColor)


def test_aabb_mode_2d(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline(mode=ns.DebugRenderMode.COLLIDER_AABBS)
    lines, _, objects = pipe.render_to_arrays(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
    )
    # Two colliders × 4 AABB edges = at least 4 lines.
    assert lines.shape[0] >= 4
    assert (objects == ns.DebugRenderObject.COLLIDER_AABB.kind).all()


def test_python_backend_2d(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline()

    class Recorder:
        def __init__(self):
            self.n = 0

        def draw_line(self, obj, a, b, color):
            self.n += 1

    r = Recorder()
    pipe.render(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
        r,
    )
    assert r.n > 0
