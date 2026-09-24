"""World-level features: quarantine, build features, deterministic math, counters, collision
detection, clearing, stale views, and cross-thread use."""

from __future__ import annotations

import math
import threading

import pytest

import rapier3d as rp
import rapier3d.math as rpm


def _ball_world():
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(10.0, 0.1, 10.0))
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 0.55, 0.0)),
        colliders=[rp.Collider.ball(0.5)],
    )
    return world, ball


# ---- defaults -------------------------------------------------------------


def test_world_gravity_defaults_to_zero():
    assert rp.PhysicsWorld().gravity == rp.Vec3(0.0, 0.0, 0.0)


# ---- quarantine -----------------------------------------------------------


def test_quarantine_is_empty_for_a_finite_simulation():
    world, _ = _ball_world()
    world.step()
    quarantine = world.quarantine
    assert isinstance(quarantine, rp.Quarantine)
    assert quarantine.is_empty()
    assert quarantine.bodies == []
    assert quarantine.colliders == []
    assert quarantine.soft_bodies == []


def test_quarantine_reports_and_disables_non_finite_bodies():
    world, ball = _ball_world()
    world.step()
    world.rigid_bodies[ball].linvel = (math.nan, 0.0, 0.0)
    world.step()
    quarantine = world.quarantine
    assert not quarantine.is_empty()
    assert quarantine.bodies == [ball]
    assert world.physics_pipeline.quarantine.bodies == [ball]
    body = world.rigid_bodies[ball]
    assert not body.is_enabled
    assert all(math.isfinite(x) for x in body.translation)
    # Re-enabled, the body simulates again; the next step clears the reports.
    body.is_enabled = True
    world.step()
    assert world.quarantine.is_empty()
    assert world.rigid_bodies[ball].is_enabled


# ---- integration parameters -----------------------------------------------


def test_integration_parameters_solver_flags():
    params = rp.IntegrationParameters()
    assert params.warmstart_joints is False
    assert params.friction_in_bias_pass is False
    params.warmstart_joints = True
    params.friction_in_bias_pass = True
    assert params.warmstart_joints and params.friction_in_bias_pass
    world, _ = _ball_world()
    world.integration_parameters = params
    assert world.integration_parameters.warmstart_joints
    world.step()


def test_integration_parameters_equality():
    a = rp.IntegrationParameters()
    b = rp.IntegrationParameters()
    assert a == b
    assert not (a != b)
    b.warmstart_joints = True
    assert a != b
    assert a != "not parameters"
    with pytest.raises(TypeError):
        _ = a < b
    world = rp.PhysicsWorld()
    assert world.integration_parameters == a


def test_friction_model_names():
    assert rp.FrictionModel.COEFFICIENT == rp.FrictionModel.SIMPLIFIED
    assert rp.IntegrationParameters().friction_model == rp.FrictionModel.SIMPLIFIED
    params = rp.IntegrationParameters()
    params.friction_model = rp.FrictionModel.COULOMB
    assert params.friction_model == rp.FrictionModel.COULOMB
    params.friction_model = rp.FrictionModel.COEFFICIENT
    assert params.friction_model == rp.FrictionModel.SIMPLIFIED


# ---- build features -------------------------------------------------------


def test_build_features():
    features = rp.build_features()
    assert isinstance(features, rp.BuildFeatures)
    assert features.profile in ("debug", "release")
    assert isinstance(features.enhanced_determinism, bool)
    assert features.parallel is True
    assert features.profiler is True
    assert "BuildFeatures(" in repr(features)


# ---- deterministic math ---------------------------------------------------


@pytest.mark.parametrize(
    "name, args, expected",
    [
        ("sin", (2.0,), math.sin(2.0)),
        ("cos", (3.0,), math.cos(3.0)),
        ("tan", (0.5,), math.tan(0.5)),
        ("asin", (0.5,), math.asin(0.5)),
        ("acos", (0.5,), math.acos(0.5)),
        ("atan", (2.0,), math.atan(2.0)),
        ("atan2", (1.0, -1.0), math.atan2(1.0, -1.0)),
        ("exp", (1.0,), math.exp(1.0)),
        ("log", (10.0,), math.log(10.0)),
        ("pow", (2.0, 0.5), math.pow(2.0, 0.5)),
        ("sqrt", (2.0,), math.sqrt(2.0)),
    ],
)
def test_math_functions(name, args, expected):
    value = getattr(rpm, name)(*args)
    assert isinstance(value, float)
    # Computed in f32.
    assert value == pytest.approx(expected, rel=1.0e-6)


def test_math_functions_are_exported():
    for name in ("sin", "cos", "tan", "asin", "acos", "atan", "atan2", "exp", "log", "pow", "sqrt"):
        assert name in rpm.__all__


# ---- counters -------------------------------------------------------------


def test_pipeline_counters_are_a_live_view():
    world, _ = _ball_world()
    counters = world.physics_pipeline.counters
    # Enabled by default, like in Rust.
    assert counters.enabled
    counters.disable()
    # The pipeline itself is modified, not a copy.
    assert not world.physics_pipeline.counters.enabled
    counters.enable()
    for _ in range(5):
        world.step()
    assert counters.step_time_ms > 0.0
    assert counters.stages.solver_time_ms >= 0.0
    assert counters.cd.ncontact_pairs == 1
    assert "enabled=True" in repr(counters)
    counters.disable()
    assert not world.physics_pipeline.counters.enabled


def test_owned_counters_are_independent():
    counters = rp.Counters()
    counters.enable()
    assert counters.enabled
    assert rp.Counters().enabled is False


# ---- collision detection --------------------------------------------------


def test_detect_collisions_updates_contacts_and_queries():
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(10.0, 0.1, 10.0))
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 5.0, 0.0)),
        colliders=[rp.Collider.ball(0.5)],
    )
    assert isinstance(world.collision_pipeline, rp.CollisionPipeline)
    assert world.collision_pipeline is world.collision_pipeline
    world.detect_collisions()
    assert len(world.narrow_phase.contact_pairs()) == 0
    # Teleported onto the ground, the contact appears without stepping.
    world.rigid_bodies[ball].translation = (0.0, 0.55, 0.0)
    world.detect_collisions()
    assert world.rigid_bodies[ball].translation.y == pytest.approx(0.55)
    assert len(world.narrow_phase.contact_pairs()) == 1
    hit = world.query_pipeline.cast_ray(rp.Ray((0.0, 10.0, 0.0), (0.0, -1.0, 0.0)), 100.0, True)
    assert hit is not None and hit[1] == pytest.approx(8.95, abs=1.0e-3)


def test_detect_collisions_calls_the_event_handler():
    world = rp.PhysicsWorld()
    world.add_collider(rp.Collider.cuboid(10.0, 0.1, 10.0))
    world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 0.55, 0.0)),
        colliders=[rp.Collider.ball(0.5).active_events(rp.ActiveEvents.COLLISION_EVENTS)],
    )
    events = rp.ChannelEventCollector()
    world.event_handler = events
    world.detect_collisions()
    assert len(events.drain_collision_events()) == 1


# ---- clear ----------------------------------------------------------------


def test_clear_resets_soft_bodies_and_pipelines():
    world, _ = _ball_world()
    world.add_soft_body(rp.SoftBody.rope((0.0, 2.0, 0.0), (1.0, 2.0, 0.0), 5))
    world.physics_pipeline.counters.disable()
    world.set_num_threads(2)
    world.step()
    world.rigid_bodies[world.add_body(rp.RigidBody.dynamic())].linvel = (math.inf, 0.0, 0.0)
    world.step()
    assert not world.quarantine.is_empty()
    world.clear()
    assert len(world.rigid_bodies) == 0
    assert len(world.colliders) == 0
    assert len(world.soft_bodies) == 0
    assert world.quarantine.is_empty()
    # The configuration of the pipeline survives.
    assert not world.physics_pipeline.counters.enabled
    assert world.num_threads == 2
    world, _ = _ball_world()
    world.step()


# ---- stale views ----------------------------------------------------------


def test_removed_rigid_body_and_collider_views_raise_invalid_handle():
    world, ball = _ball_world()
    body = world.rigid_bodies[ball]
    collider = world.colliders[body.colliders[0]]
    world.remove_body(ball)
    with pytest.raises(rp.InvalidHandle):
        _ = body.translation
    with pytest.raises(rp.InvalidHandle):
        body.linvel = (1.0, 0.0, 0.0)
    with pytest.raises(rp.InvalidHandle):
        body.apply_impulse((0.0, 1.0, 0.0))
    with pytest.raises(rp.InvalidHandle):
        _ = collider.shape
    with pytest.raises(rp.InvalidHandle):
        collider.friction = 0.3


def test_removed_joint_views_raise_invalid_handle():
    world = rp.PhysicsWorld()
    a = world.add_body(rp.RigidBody.fixed())
    b = world.add_body(rp.RigidBody.dynamic(translation=(1.0, 0.0, 0.0)))
    c = world.add_body(rp.RigidBody.dynamic(translation=(2.0, 0.0, 0.0)))
    handle = world.impulse_joints.insert(a, b, rp.FixedJoint())
    joint = world.impulse_joints[handle]
    data = joint.data
    world.impulse_joints.remove(handle)
    with pytest.raises(rp.InvalidHandle):
        _ = joint.body1
    with pytest.raises(rp.InvalidHandle):
        _ = data.local_anchor1
    mb_joint = world.multibody_joints.insert(
        b, c, rp.RevoluteJoint.builder(axis=(0.0, 1.0, 0.0)).build()
    )
    multibody = world.multibody_joints.multibody(mb_joint)
    world.remove_body(c)
    world.remove_body(b)
    with pytest.raises(rp.InvalidHandle):
        _ = multibody.num_links


# ---- soft body enabled flag -----------------------------------------------


def test_soft_body_is_enabled_is_writable():
    world = rp.PhysicsWorld()
    handle = world.add_soft_body(rp.SoftBody.rope((0.0, 2.0, 0.0), (1.0, 2.0, 0.0), 5))
    body = world.soft_bodies[handle]
    assert body.is_enabled
    body.is_enabled = False
    assert not world.soft_bodies[handle].is_enabled
    body.set_enabled(True)
    assert world.soft_bodies[handle].is_enabled


# ---- threads --------------------------------------------------------------


def test_world_can_be_used_from_another_thread():
    world, ball = _ball_world()
    results = []

    def run():
        for _ in range(10):
            world.step()
        results.append(world.rigid_bodies[ball].translation.y)

    thread = threading.Thread(target=run)
    thread.start()
    thread.join()
    assert len(results) == 1 and math.isfinite(results[0])
    # And back on the thread that created it.
    world.step()


def test_world_modified_while_another_thread_steps_it_raises_runtime_error():
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(10.0, 0.1, 10.0))
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 0.55, 0.0)),
        colliders=[rp.Collider.ball(0.5).active_events(rp.ActiveEvents.COLLISION_EVENTS)],
    )
    view = world.rigid_bodies[ball]
    stepping = threading.Event()
    resume = threading.Event()

    class BlockingHandler:
        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            # Waiting releases the GIL while the world is being stepped.
            stepping.set()
            resume.wait(10.0)

        def handle_contact_force_event(self, *args):
            pass

    world.event_handler = BlockingHandler()
    thread = threading.Thread(target=world.step)
    thread.start()
    try:
        assert stepping.wait(10.0)
        with pytest.raises(RuntimeError, match="in use"):
            world.step()
        with pytest.raises(RuntimeError, match="in use"):
            world.detect_collisions()
        with pytest.raises(RuntimeError, match="in use"):
            _ = world.quarantine
        with pytest.raises(RuntimeError, match="in use"):
            view.linvel = (1.0, 0.0, 0.0)
        # The sets lent to the running callback can still be read, from any thread.
        assert view.translation.y == pytest.approx(0.55, abs=0.1)
        assert world.rigid_bodies[ball].is_enabled
    finally:
        resume.set()
        thread.join()
    world.event_handler = None
    world.step()
    assert world.rigid_bodies[ball].is_enabled


# ---- testbed --------------------------------------------------------------


def test_testbed_steps_a_physics_world():
    testbed_module = pytest.importorskip("rapier_testbed")
    world, ball = _ball_world()
    world.rigid_bodies[ball].translation = (0.0, 5.0, 0.0)
    testbed = testbed_module.Testbed(headless=True)
    testbed.set_world(world)
    assert testbed.bodies is world.rigid_bodies
    testbed.set_gravity((0.0, -20.0, 0.0))
    assert world.gravity == rp.Vec3(0.0, -20.0, 0.0)
    for _ in range(10):
        testbed.step_once()
    assert world.rigid_bodies[ball].translation.y < 5.0
    with pytest.raises(TypeError):
        testbed.set_world(world, world.colliders)
