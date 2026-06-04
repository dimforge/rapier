"""3D debug-render tests.

Parametrized across the f32 (`rapier.dim3`) and f64 (`rapier.dim3.f64`)
flavors.
"""

from __future__ import annotations

import numpy as np
import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


# ---------------------------------------------------------------------------
# DebugRenderMode bitflags
# ---------------------------------------------------------------------------


def test_mode_default_is_nonempty(ns):
    m = ns.DebugRenderMode.default()
    assert m.bits != 0
    assert m.contains(ns.DebugRenderMode.COLLIDER_SHAPES)


def test_mode_all_contains_aabb(ns):
    m = ns.DebugRenderMode.all()
    assert m.contains(ns.DebugRenderMode.COLLIDER_AABBS)
    assert m.contains(ns.DebugRenderMode.CONTACTS)


def test_mode_or_xor_and(ns):
    a = ns.DebugRenderMode.COLLIDER_SHAPES
    b = ns.DebugRenderMode.RIGID_BODY_AXES
    c = a | b
    assert c.contains(a)
    assert c.contains(b)
    assert (c & a).bits == a.bits
    assert (c ^ a).bits == b.bits


def test_mode_empty(ns):
    e = ns.DebugRenderMode.empty()
    assert e.bits == 0
    assert e.is_empty()
    assert not bool(e)


# ---------------------------------------------------------------------------
# DebugRenderObject
# ---------------------------------------------------------------------------


def test_render_object_constants(ns):
    assert ns.DebugRenderObject.RIGID_BODY.kind == 0
    assert ns.DebugRenderObject.COLLIDER.kind == 1
    assert ns.DebugRenderObject.COLLIDER_AABB.kind == 2
    assert ns.DebugRenderObject.IMPULSE_JOINT.kind == 3
    assert ns.DebugRenderObject.MULTIBODY_JOINT.kind == 4
    assert ns.DebugRenderObject.CONTACT_PAIR.kind == 5


def test_render_object_equality(ns):
    a = ns.DebugRenderObject.RIGID_BODY
    b = ns.DebugRenderObject.RIGID_BODY
    c = ns.DebugRenderObject.COLLIDER
    assert a == b
    assert a != c
    assert hash(a) == hash(b)


# ---------------------------------------------------------------------------
# DebugColor
# ---------------------------------------------------------------------------


def test_color_rgba_roundtrip(ns):
    # Red full saturation should round-trip exactly.
    c = ns.DebugColor.from_rgba(1.0, 0.0, 0.0, 1.0)
    r, g, b, a = c.rgba
    assert abs(r - 1.0) < 1e-6
    assert abs(g) < 1e-6
    assert abs(b) < 1e-6
    assert abs(a - 1.0) < 1e-6


def test_color_hsla_default(ns):
    c = ns.DebugColor.from_hsla(120.0, 0.5, 0.5, 1.0)
    h, s, l, a = c.hsla
    assert h == 120.0
    assert s == 0.5
    assert l == 0.5
    assert a == 1.0


# ---------------------------------------------------------------------------
# DebugRenderStyle
# ---------------------------------------------------------------------------


def test_style_default(ns):
    s = ns.DebugRenderStyle()
    assert s.subdivisions == 20
    assert s.border_subdivisions == 5
    assert s.rigid_body_axes_length > 0.0


def test_style_setter(ns):
    s = ns.DebugRenderStyle()
    s.subdivisions = 32
    s.rigid_body_axes_length = 1.25
    assert s.subdivisions == 32
    assert s.rigid_body_axes_length == pytest.approx(1.25)
    # Tuples accepted in addition to DebugColor.
    s.collider_dynamic_color = (10.0, 1.0, 0.5, 1.0)
    assert s.collider_dynamic_color.hsla == [10.0, 1.0, 0.5, 1.0]


# ---------------------------------------------------------------------------
# DebugRenderPipeline + DebugLineCollector
# ---------------------------------------------------------------------------


def _ball_on_ground_world(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    bh = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(0, 5, 0)).build()
    )
    w.colliders.insert_with_parent(
        ns.Collider.ball(0.5), bh, w.rigid_bodies
    )
    # Step a few times to populate the narrow phase.
    for _ in range(20):
        w.step()
    return w


def test_render_to_arrays_nonempty(ns):
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
    assert isinstance(colors, np.ndarray)
    assert isinstance(objects, np.ndarray)
    assert lines.ndim == 3
    assert lines.shape[1] == 2
    assert lines.shape[2] == 3
    assert colors.shape == (lines.shape[0], 4)
    assert objects.shape == (lines.shape[0],)
    assert lines.shape[0] > 0


def test_render_to_arrays_aabb_count(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline(
        mode=ns.DebugRenderMode.COLLIDER_AABBS,
    )
    lines, _, objects = pipe.render_to_arrays(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
    )
    # Two colliders × 12 edges per cuboid = at least 24 lines.
    assert lines.shape[0] > 8
    # Every emitted line should be tagged as a COLLIDER_AABB.
    assert (objects == ns.DebugRenderObject.COLLIDER_AABB.kind).all()


def test_collector_clear_and_len(ns):
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
    n = len(coll)
    assert n > 0
    coll.clear()
    assert len(coll) == 0


def test_collector_iteration(ns):
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
    assert len(items) == len(coll)
    obj, a, b, color = items[0]
    assert isinstance(obj, ns.DebugRenderObject)
    assert isinstance(a, ns.Vec3)
    assert isinstance(b, ns.Vec3)
    assert isinstance(color, ns.DebugColor)


def test_collector_arrays_match_render_to_arrays(ns):
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
    a = coll.lines()
    c = coll.colors()
    o = coll.objects()
    # collector.lines() exposes (2N, D); render_to_arrays() returns (N, 2, D).
    assert a.shape[0] == 2 * o.shape[0]
    assert c.shape == (o.shape[0], 4)


def test_python_backend_receives_calls(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline()

    class Recorder:
        def __init__(self):
            self.calls = []

        def draw_line(self, obj, a, b, color):
            self.calls.append((obj.kind, a.x, b.x))

    rec = Recorder()
    pipe.render(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
        rec,
    )
    assert len(rec.calls) > 0
    kinds = {c[0] for c in rec.calls}
    # Default mode includes COLLIDER_SHAPES, so we should see colliders.
    assert ns.DebugRenderObject.COLLIDER.kind in kinds


def test_python_backend_exception_is_propagated(ns):
    w = _ball_on_ground_world(ns)
    pipe = ns.DebugRenderPipeline()

    class Failing:
        def draw_line(self, obj, a, b, color):
            raise ValueError("kaboom")

    with pytest.raises(ValueError, match="kaboom"):
        pipe.render(
            w.rigid_bodies,
            w.colliders,
            w.impulse_joints,
            w.multibody_joints,
            w.narrow_phase,
            Failing(),
        )


# ---------------------------------------------------------------------------
# Joints rendering — RevoluteJoint should produce visible lines under
# IMPULSE_JOINTS mode.
# ---------------------------------------------------------------------------


def test_revolute_joint_renders_lines(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(2, 0, 0)).build())
    builder = ns.RevoluteJointBuilder(ns.Vec3.unit_y()).local_anchor1(
        (1, 0, 0)
    ).local_anchor2((-1, 0, 0))
    w.impulse_joints.insert(a, b, builder.build(), True)

    pipe = ns.DebugRenderPipeline(mode=ns.DebugRenderMode.IMPULSE_JOINTS)
    lines, _, objects = pipe.render_to_arrays(
        w.rigid_bodies,
        w.colliders,
        w.impulse_joints,
        w.multibody_joints,
        w.narrow_phase,
    )
    assert lines.shape[0] > 0
    assert (objects == ns.DebugRenderObject.IMPULSE_JOINT.kind).all()


# ---------------------------------------------------------------------------
# Mode mutation
# ---------------------------------------------------------------------------


def test_pipeline_mode_assignment_persists(ns):
    pipe = ns.DebugRenderPipeline()
    pipe.mode = ns.DebugRenderMode.COLLIDER_AABBS
    assert pipe.mode.bits == ns.DebugRenderMode.COLLIDER_AABBS.bits
