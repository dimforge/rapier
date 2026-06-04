"""3D pipeline tests.

Parametrized across the f32 (`rapier.dim3`) and f64 (`rapier.dim3.f64`)
flavors.
"""

from __future__ import annotations

import threading

import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


# ---- PhysicsWorld smoke ---------------------------------------------------


def test_world_default_constructor(ns):
    w = ns.PhysicsWorld()
    assert len(w.rigid_bodies) == 0
    assert len(w.colliders) == 0
    assert w.gravity == ns.Vec3(0, 0, 0)


def test_world_subsets_are_shared_python_objects(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    # Critical invariant: property reads return the same Python object,
    # not a fresh wrapper.
    assert w.rigid_bodies is w.rigid_bodies
    assert w.colliders is w.colliders
    assert w.broad_phase is w.broad_phase
    assert w.narrow_phase is w.narrow_phase
    assert w.islands is w.islands
    assert w.ccd_solver is w.ccd_solver
    assert w.impulse_joints is w.impulse_joints
    assert w.multibody_joints is w.multibody_joints
    assert w.query_pipeline is w.query_pipeline
    assert w.integration_parameters is w.integration_parameters

    # Mutating via the property persists on the world.
    h = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 2, 3)).build())
    assert h in w.rigid_bodies
    assert len(w.rigid_bodies) == 1


def test_world_gravity_setter(ns):
    w = ns.PhysicsWorld()
    w.gravity = (0, -9.81, 0)
    assert w.gravity == ns.Vec3(0, -9.81, 0)


def test_world_step_drops_ball_onto_ground(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    # Wide ground cuboid at y=0
    w.colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    # Dynamic ball at y=5
    ball_h = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(0, 5, 0)).build()
    )
    w.colliders.insert_with_parent(
        ns.Collider.ball(0.5), ball_h, w.rigid_bodies
    )
    for _ in range(120):
        w.step()
    ball = w.rigid_bodies[ball_h]
    # Ball comes to rest with center near y = 0.1 (ground top) + 0.5 (radius).
    assert ball.translation.y < 5.0  # has fallen
    assert ball.translation.y > -1.0  # didn't tunnel
    # In settled state should be roughly 0.6 (ground top + ball radius).
    assert abs(ball.translation.y - 0.6) < 0.2


def test_world_step_releases_gil(ns):
    """Stepping must release the GIL so other Python threads can run."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    ball_h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[ns.Collider.ball(0.5)],
    )
    counter = [0]

    def bump():
        for _ in range(50):
            counter[0] += 1

    t = threading.Thread(target=bump)
    t.start()
    for _ in range(60):
        w.step()
    t.join()
    assert counter[0] == 50
    assert ball_h in w.rigid_bodies


def test_world_clear(ns):
    w = ns.PhysicsWorld()
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 1, 0)),
        colliders=[ns.Collider.ball(0.5)],
    )
    assert len(w.rigid_bodies) == 1
    assert len(w.colliders) == 1
    w.clear()
    assert len(w.rigid_bodies) == 0
    assert len(w.colliders) == 0
    # The Py<RigidBodySet> handle stayed the same, but it's empty now.
    assert h not in w.rigid_bodies


def test_add_body_attaches_colliders(ns):
    w = ns.PhysicsWorld()
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 1, 0)),
        colliders=[ns.Collider.ball(0.5), ns.Collider.cuboid(0.5, 0.5, 0.5)],
    )
    body = w.rigid_bodies[h]
    assert len(body.colliders) == 2
    for ch in body.colliders:
        assert ch in w.colliders


def test_add_collider_with_parent(ns):
    w = ns.PhysicsWorld()
    h = w.add_body(ns.RigidBody.dynamic(translation=(0, 1, 0)))
    ch = w.add_collider(ns.Collider.ball(0.5), parent=h)
    assert ch in w.colliders
    assert w.colliders[ch].parent == h


def test_remove_body(ns):
    w = ns.PhysicsWorld()
    h = w.add_body(ns.RigidBody.dynamic())
    removed = w.remove_body(h)
    assert removed is not None
    assert h not in w.rigid_bodies


def test_remove_collider(ns):
    w = ns.PhysicsWorld()
    ch = w.add_collider(ns.Collider.cuboid(1, 1, 1))
    removed = w.remove_collider(ch)
    assert removed is not None
    assert ch not in w.colliders


def test_world_wake_up_and_active_bodies(ns):
    """`active_bodies` tracks awake bodies; `wake_up` revives a sleeper."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    ball_h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 0.6, 0)),
        colliders=[ns.Collider.ball(0.5)],
    )
    # Let the ball settle and fall asleep.
    for _ in range(400):
        w.step()
    assert ball_h not in w.active_bodies()
    # Waking it puts it back among the active bodies.
    w.wake_up(ball_h, True)
    assert ball_h in w.active_bodies()


def test_world_wake_up_all(ns):
    """`wake_up_all` revives every sleeping body."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    handles = [
        w.add_body(
            ns.RigidBody.dynamic(translation=(float(i), 0.6, 0)),
            colliders=[ns.Collider.ball(0.5)],
        )
        for i in range(3)
    ]
    for _ in range(400):
        w.step()
    assert w.active_bodies() == []
    w.wake_up_all(True)
    active = w.active_bodies()
    for h in handles:
        assert h in active


# ---- PhysicsPipeline standalone -------------------------------------------


def test_physics_pipeline_step(ns):
    pp = ns.PhysicsPipeline()
    ip = ns.IntegrationParameters()
    bodies = ns.RigidBodySet()
    colliders = ns.ColliderSet()
    ij = ns.ImpulseJointSet()
    mj = ns.MultibodyJointSet()
    bp = ns.BroadPhaseBvh()
    np_ = ns.NarrowPhase()
    islands = ns.IslandManager()
    ccd = ns.CCDSolver()

    colliders.insert(ns.Collider.cuboid(50, 0.1, 50).build())
    h = bodies.insert(ns.RigidBody.dynamic(translation=(0, 5, 0)).build())
    colliders.insert_with_parent(ns.Collider.ball(0.5), h, bodies)

    for _ in range(60):
        pp.step(
            (0, -9.81, 0),
            ip, islands, bp, np_, bodies, colliders, ij, mj, ccd,
        )
    assert bodies[h].translation.y < 5.0


def test_physics_pipeline_step_accepts_event_collector(ns):
    """Hooks/events are now plumbed through. Passing a
    `ChannelEventCollector` should be accepted (no `NotImplementedError`)."""
    pp = ns.PhysicsPipeline()
    ip = ns.IntegrationParameters()
    bodies = ns.RigidBodySet()
    colliders = ns.ColliderSet()
    ij = ns.ImpulseJointSet()
    mj = ns.MultibodyJointSet()
    bp = ns.BroadPhaseBvh()
    np_ = ns.NarrowPhase()
    islands = ns.IslandManager()
    ccd = ns.CCDSolver()
    collector = ns.ChannelEventCollector()
    # Should not raise — an empty world stepped with a real collector is fine.
    pp.step(
        (0, -9.81, 0),
        ip, islands, bp, np_, bodies, colliders, ij, mj, ccd,
        events=collector,
    )


# ---- Counters -------------------------------------------------------------


def test_counters_basic(ns):
    c = ns.Counters()
    assert not c.enabled
    c.enable()
    assert c.enabled
    c.disable()
    assert not c.enabled
    # nested views
    assert hasattr(c.stages, "solver_time_ms")
    assert hasattr(c.cd, "ncontact_pairs")
    assert hasattr(c.solver, "ncontacts")
    assert hasattr(c.ccd, "num_substeps")


def test_pipeline_counters_property(ns):
    pp = ns.PhysicsPipeline()
    c = pp.counters
    assert isinstance(c, ns.Counters)
