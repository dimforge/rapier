"""3D serialization, snapshots, pickle.

Parametrized across the f32 (`rapier.dim3`) and f64 (`rapier.dim3.f64`)
flavors.
"""

from __future__ import annotations

import pickle

import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


# ---- Helpers --------------------------------------------------------------


def _build_small_world(ns):
    """Builds a deterministic 3-body, 5-collider, 2-joint sandbox."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    # ground (collider only, no body)
    w.colliders.insert(ns.Collider.cuboid(50.0, 0.1, 50.0).build())
    # two dynamic bodies with one collider each
    h1 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(0, 5, 0)).build())
    w.colliders.insert_with_parent(ns.Collider.ball(0.5), h1, w.rigid_bodies)
    h2 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1.5, 5, 0)).build())
    w.colliders.insert_with_parent(ns.Collider.cuboid(0.3, 0.3, 0.3), h2, w.rigid_bodies)
    # third dynamic body for joint variety
    h3 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(3.0, 5, 0)).build())
    w.colliders.insert_with_parent(ns.Collider.ball(0.4), h3, w.rigid_bodies)
    # two joints: a fixed between h1,h2 and a revolute between h2,h3
    w.impulse_joints.insert(h1, h2, ns.FixedJointBuilder().build())
    w.impulse_joints.insert(h2, h3, ns.RevoluteJointBuilder(axis=(0, 1, 0)).build())
    return w, [h1, h2, h3]


def _body_positions(w, handles):
    return [tuple(w.rigid_bodies[h].translation) for h in handles]


# ---- Magic header / version checks ---------------------------------------


def test_snapshot_has_magic_header(ns):
    w, _ = _build_small_world(ns)
    blob = w.snapshot()
    assert isinstance(blob, (bytes, bytearray))
    assert blob[:4] == b"RPYS"
    # version is u32 LE = 1
    assert blob[4:8] == b"\x01\x00\x00\x00"


def test_bad_magic_raises(ns):
    bad = b"BADX" + b"\x01\x00\x00\x00" + b"deadbeef"
    with pytest.raises(ns.SerializationError):
        ns.PhysicsWorld.restore(bad)


def test_bad_version_raises(ns):
    # well-formed magic + version 99
    bad = b"RPYS" + (99).to_bytes(4, "little") + b"deadbeef"
    with pytest.raises(ns.SerializationError):
        ns.PhysicsWorld.restore(bad)


def test_truncated_blob_raises(ns):
    with pytest.raises(ns.SerializationError):
        ns.PhysicsWorld.restore(b"RPY")  # too short


# ---- snapshot / restore round-trip ---------------------------------------


def test_world_snapshot_round_trip(ns):
    w, handles = _build_small_world(ns)
    before = _body_positions(w, handles)
    blob = w.snapshot()
    w2 = ns.PhysicsWorld.restore(blob)
    after = _body_positions(w2, handles)
    # Same positions immediately after restore.
    for b, a in zip(before, after):
        for bi, ai in zip(b, a):
            assert abs(bi - ai) <= 1e-5
    # Same counts.
    assert len(w2.rigid_bodies) == len(w.rigid_bodies)
    assert len(w2.colliders) == len(w.colliders)
    assert len(w2.impulse_joints) == len(w.impulse_joints)


def test_step_after_restore_matches(ns):
    w, handles = _build_small_world(ns)
    blob = w.snapshot()
    w2 = ns.PhysicsWorld.restore(blob)

    for _ in range(100):
        w.step()
        w2.step()

    after_a = _body_positions(w, handles)
    after_b = _body_positions(w2, handles)
    for a, b in zip(after_a, after_b):
        for ai, bi in zip(a, b):
            # Determinism: same input → same output. Default builds use
            # platform-native math (no `enhanced-determinism`), so allow a
            # tiny tolerance; with the deterministic build this would be 0.
            assert abs(ai - bi) <= 1e-5, f"divergence at {(ai, bi)}"


# ---- pickle round-trip ----------------------------------------------------


def test_pickle_world_round_trip(ns):
    w, handles = _build_small_world(ns)
    blob = pickle.dumps(w)
    w2 = pickle.loads(blob)
    assert len(w2.rigid_bodies) == len(w.rigid_bodies)
    # Step both N times, compare.
    for _ in range(50):
        w.step()
        w2.step()
    after_a = _body_positions(w, handles)
    after_b = _body_positions(w2, handles)
    for a, b in zip(after_a, after_b):
        for ai, bi in zip(a, b):
            assert abs(ai - bi) <= 1e-5


def test_pickle_rigid_body(ns):
    rb = ns.RigidBody.dynamic(translation=(1, 2, 3), linvel=(0.5, 0, 0)).build()
    rb2 = pickle.loads(pickle.dumps(rb))
    assert tuple(rb2.translation) == pytest.approx((1.0, 2.0, 3.0))
    assert tuple(rb2.linvel) == pytest.approx((0.5, 0.0, 0.0))


def test_pickle_collider(ns):
    c = ns.Collider.cuboid(1, 2, 3).build()
    c2 = pickle.loads(pickle.dumps(c))
    aabb_a = c.compute_aabb()
    aabb_b = c2.compute_aabb()
    assert tuple(aabb_a.mins.to_tuple()) == tuple(aabb_b.mins.to_tuple())
    assert tuple(aabb_a.maxs.to_tuple()) == tuple(aabb_b.maxs.to_tuple())


def test_pickle_mass_properties(ns):
    mp = ns.MassProperties.from_ball(1.0, 1.0)
    mp2 = pickle.loads(pickle.dumps(mp))
    assert mp2.mass == pytest.approx(mp.mass)
    assert tuple(mp2.local_com.to_tuple()) == tuple(mp.local_com.to_tuple())


def test_pickle_shared_shape(ns):
    s = ns.SharedShape.cuboid(1, 1, 1)
    s2 = pickle.loads(pickle.dumps(s))
    iso = ns.Isometry3.identity()
    a, b = s.compute_aabb(iso), s2.compute_aabb(iso)
    assert tuple(a.mins.to_tuple()) == tuple(b.mins.to_tuple())
    assert tuple(a.maxs.to_tuple()) == tuple(b.maxs.to_tuple())


def test_pickle_handle(ns):
    h = ns.RigidBodyHandle.from_raw_parts(7, 42)
    h2 = pickle.loads(pickle.dumps(h))
    assert h2.index == 7
    assert h2.generation == 42
    assert h2 == h


def test_pickle_integration_parameters(ns):
    p = ns.IntegrationParameters()
    p.dt = 0.0123
    p.num_solver_iterations = 8
    p2 = pickle.loads(pickle.dumps(p))
    assert p2.dt == pytest.approx(0.0123)
    assert p2.num_solver_iterations == 8


def test_pickle_interaction_groups(ns):
    g = ns.InteractionGroups(ns.Group(0xFF), ns.Group(0x0F))
    g2 = pickle.loads(pickle.dumps(g))
    assert g2 == g


def test_pickle_joints(ns):
    builders = [
        ns.FixedJointBuilder(),
        ns.RevoluteJointBuilder(axis=(0, 1, 0)),
        ns.PrismaticJointBuilder(axis=(1, 0, 0)),
        ns.SphericalJointBuilder(),
        ns.RopeJointBuilder(),
        ns.SpringJointBuilder(rest_length=1.0, stiffness=10.0, damping=0.5),
        ns.GenericJointBuilder(ns.JointAxesMask.empty()),
    ]
    for b in builders:
        joint = b.build()
        joint2 = pickle.loads(pickle.dumps(joint))
        # Smoke: rebuild succeeds and produces a working GenericJoint.
        gen_a = joint.data if hasattr(joint, "data") else None
        gen_b = joint2.data if hasattr(joint2, "data") else None
        assert (gen_a is None) == (gen_b is None)


# ---- event_handler / physics_hooks are dropped on snapshot ---------------


def test_snapshot_drops_event_handler(ns):
    w, _ = _build_small_world(ns)
    coll = ns.ChannelEventCollector()
    w.event_handler = coll
    w.event_error_policy = "strict"
    blob = w.snapshot()
    w2 = ns.PhysicsWorld.restore(blob)
    assert w2.event_handler is None
    assert w2.physics_hooks is None
    assert w2.event_error_policy == "defer"


def test_channel_event_collector_refuses_pickle(ns):
    coll = ns.ChannelEventCollector()
    with pytest.raises(ns.SerializationError):
        pickle.dumps(coll)


def test_physics_pipeline_refuses_pickle(ns):
    pp = ns.PhysicsPipeline()
    with pytest.raises(ns.SerializationError):
        pickle.dumps(pp)


def test_collision_pipeline_refuses_pickle(ns):
    cp = ns.CollisionPipeline()
    with pytest.raises(ns.SerializationError):
        pickle.dumps(cp)


# ---- JSON debug mode ------------------------------------------------------


def test_snapshot_json_round_trip(ns):
    w, handles = _build_small_world(ns)
    s = w.snapshot_json()
    assert isinstance(s, str)
    # Self-describing envelope is present.
    import json
    env = json.loads(s)
    assert env["_magic"] == "RPYS"
    assert env["_version"] == 1
    w2 = ns.PhysicsWorld.restore_json(s)
    for a, b in zip(_body_positions(w, handles), _body_positions(w2, handles)):
        for ai, bi in zip(a, b):
            assert abs(ai - bi) <= 1e-5


def test_snapshot_json_bad_magic_raises(ns):
    import json
    bad = json.dumps({"_magic": "BADX", "_version": 1, "payload": {}})
    with pytest.raises(ns.SerializationError):
        ns.PhysicsWorld.restore_json(bad)


def test_snapshot_json_bad_version_raises(ns):
    import json
    bad = json.dumps({"_magic": "RPYS", "_version": 999, "payload": {}})
    with pytest.raises(ns.SerializationError):
        ns.PhysicsWorld.restore_json(bad)
