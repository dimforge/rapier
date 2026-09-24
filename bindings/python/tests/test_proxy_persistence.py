"""Regression tests for handle-backed proxies.

Objects fetched from a set (``world.rigid_bodies[h]``, ``world.colliders[h]``,
``world.impulse_joints[h]``, ``world.multibody_joints.multibody(h)``) are live
**views**: mutating them persists in place, with no copy and no write-back
call. This is the behavior that the old clone-on-get + ``replace()`` design got
wrong (mutations were silently dropped), so these guard against regressing it.
"""

from __future__ import annotations

import pytest

import rapier3d


@pytest.fixture(params=[rapier3d], ids=["3d-f32"])
def ns3(request):
    return request.param


def test_rigid_body_mutation_persists(ns3):
    w = ns3.PhysicsWorld(gravity=(0, -9.81, 0))
    h = w.add_body(ns3.RigidBody.dynamic(translation=(0, 5, 0)))
    # Mutate via the fetched view; no replace().
    w.rigid_bodies[h].linvel = ns3.Vec3(5.0, 0.0, -2.0)
    assert abs(w.rigid_bodies[h].linvel.x - 5.0) < 1e-5
    w.step()
    assert abs(w.rigid_bodies[h].linvel.x - 5.0) < 1e-3  # persisted through a step


def test_collider_mutation_persists(ns3):
    w = ns3.PhysicsWorld(gravity=(0, -9.81, 0))
    h = w.add_body(ns3.RigidBody.dynamic())
    ch = w.add_collider(ns3.Collider.ball(0.5), parent=h)
    assert w.colliders[ch].is_sensor is False
    w.colliders[ch].is_sensor = True  # mutate the view (no replace)
    assert w.colliders[ch].is_sensor is True


def test_impulse_joint_data_persists(ns3):
    w = ns3.PhysicsWorld(gravity=(0, -9.81, 0))
    a = w.add_body(ns3.RigidBody.fixed())
    b = w.add_body(ns3.RigidBody.dynamic(translation=(1, 0, 0)))
    jh = w.impulse_joints.insert(a, b, ns3.RevoluteJointBuilder((0, 0, 1)).build(), True)
    # `.data` is a live view: mutating it in place persists (no assign-back).
    w.impulse_joints[jh].data.set_limits(ns3.JointAxis.ANG_X, -0.5, 0.5)
    lim = w.impulse_joints[jh].data.limits(ns3.JointAxis.ANG_X)
    assert lim is not None and abs(lim.min + 0.5) < 1e-6 and abs(lim.max - 0.5) < 1e-6


def test_multibody_mutation_persists(ns3):
    w = ns3.PhysicsWorld(gravity=(0, -9.81, 0))
    root = w.add_body(ns3.RigidBody.fixed())
    link = w.add_body(ns3.RigidBody.dynamic(translation=(1, 0, 0)))
    jh = w.multibody_joints.insert(root, link, ns3.RevoluteJointBuilder((0, 0, 1)).build(), True)
    mb = w.multibody_joints.multibody(jh)
    n = mb.ndofs
    mb.set_damping([0.7] * n)  # mutate the view
    again = w.multibody_joints.multibody(jh)
    assert all(abs(d - 0.7) < 1e-5 for d in again.damping())
