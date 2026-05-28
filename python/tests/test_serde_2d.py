"""2D serialization, snapshots, pickle.

Smaller analogue of `test_serde.py`. Parametrized across `dim2` (f32) and
`dim2.f64`.
"""

from __future__ import annotations

import pickle

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def _build_small_world(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    w.colliders.insert(ns.Collider.cuboid(50.0, 0.1).build())
    h1 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(0, 5)).build())
    w.colliders.insert_with_parent(ns.Collider.ball(0.5), h1, w.rigid_bodies)
    h2 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1.5, 5)).build())
    w.colliders.insert_with_parent(ns.Collider.cuboid(0.3, 0.3), h2, w.rigid_bodies)
    h3 = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(3.0, 5)).build())
    w.colliders.insert_with_parent(ns.Collider.ball(0.4), h3, w.rigid_bodies)
    w.impulse_joints.insert(h1, h2, ns.FixedJointBuilder().build())
    w.impulse_joints.insert(h2, h3, ns.RevoluteJointBuilder().build())
    return w, [h1, h2, h3]


def _body_positions(w, handles):
    return [tuple(w.rigid_bodies[h].translation) for h in handles]


def test_world_snapshot_round_trip(ns):
    w, handles = _build_small_world(ns)
    before = _body_positions(w, handles)
    blob = w.snapshot()
    assert blob[:4] == b"RPYS"
    w2 = ns.PhysicsWorld.restore(blob)
    after = _body_positions(w2, handles)
    for a, b in zip(before, after):
        for ai, bi in zip(a, b):
            assert abs(ai - bi) <= 1e-5


def test_pickle_world(ns):
    w, handles = _build_small_world(ns)
    w2 = pickle.loads(pickle.dumps(w))
    for _ in range(50):
        w.step()
        w2.step()
    for a, b in zip(_body_positions(w, handles), _body_positions(w2, handles)):
        for ai, bi in zip(a, b):
            assert abs(ai - bi) <= 1e-5


def test_pickle_handles(ns):
    h = ns.RigidBodyHandle.from_raw_parts(11, 22)
    assert pickle.loads(pickle.dumps(h)) == h


def test_pickle_shapes(ns):
    s = ns.SharedShape.cuboid(1, 2)
    s2 = pickle.loads(pickle.dumps(s))
    iso = ns.Isometry2.identity()
    assert tuple(s.compute_aabb(iso).mins.to_tuple()) == tuple(
        s2.compute_aabb(iso).mins.to_tuple()
    )


def test_pickle_mass_properties(ns):
    mp = ns.MassProperties.from_ball(1.0, 1.0)
    mp2 = pickle.loads(pickle.dumps(mp))
    assert mp2.mass == pytest.approx(mp.mass)


def test_snapshot_json_round_trip(ns):
    w, _ = _build_small_world(ns)
    s = w.snapshot_json()
    w2 = ns.PhysicsWorld.restore_json(s)
    assert len(w2.rigid_bodies) == len(w.rigid_bodies)


def test_channel_event_collector_refuses_pickle(ns):
    coll = ns.ChannelEventCollector()
    with pytest.raises(ns.SerializationError):
        pickle.dumps(coll)
