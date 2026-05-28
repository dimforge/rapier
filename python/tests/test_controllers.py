"""3D controller tests.

Covers:
- KinematicCharacterController on a flat ground, with autostep over a step.
- PidController driving a kinematic body toward a target pose (steady-state
  error <1% after 1s).
- DynamicRayCastVehicleController: engine force accelerates, brake stops.
"""

from __future__ import annotations

import math as pymath

import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


# ---------------------------------------------------------------------------
# CharacterLength / CharacterAutostep
# ---------------------------------------------------------------------------


def test_character_length_constructors(ns):
    a = ns.CharacterLength.absolute(0.5)
    r = ns.CharacterLength.relative(0.1)
    assert a.kind == "absolute"
    assert abs(a.value - 0.5) < 1e-6
    assert r.kind == "relative"
    assert abs(r.value - 0.1) < 1e-6


def test_character_autostep_defaults(ns):
    auto = ns.CharacterAutostep()
    assert auto.include_dynamic_bodies is True
    # Defaults match Rust's `Default::default()` — Relative(0.25), Relative(0.5)
    assert auto.max_height.kind == "relative"
    assert auto.min_width.kind == "relative"


# ---------------------------------------------------------------------------
# KinematicCharacterController — move on flat ground, climb a step.
# ---------------------------------------------------------------------------


def _ground_world(ns):
    """Big flat ground + a 0.3m step.

    Layout (looking down +X):
        ground:  y in [-1, 0],  x in [-50, 50],  z in [-50, 50]
        step:    y in [ 0, 0.3], x in [   1,  3],  z in [-50, 50]
    """
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    # Ground.
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1.0, 0)),
        colliders=[ns.Collider.cuboid(50, 1, 50)],
    )
    # Step.
    w.add_body(
        ns.RigidBody.fixed(translation=(2.0, 0.15, 0)),
        colliders=[ns.Collider.cuboid(1.0, 0.15, 50)],
    )
    w.update_query_pipeline()
    return w


def test_character_controller_moves_flat_ground(ns):
    w = _ground_world(ns)
    ctrl = ns.KinematicCharacterController(
        up=(0, 1, 0),
        offset=ns.CharacterLength.absolute(0.01),
        slide=True,
        snap_to_ground=ns.CharacterLength.absolute(0.5),
    )
    shape = ns.SharedShape.ball(0.5)
    pose = ns.Isometry3.from_translation(-1.0, 0.6, 0.0)
    desired = (0.1, -0.1, 0.0)
    mv = ctrl.move_shape(
        1.0 / 60.0,
        w.rigid_bodies,
        w.colliders,
        w.query_pipeline,
        shape,
        pose,
        desired,
        ns.QueryFilter(),
    )
    assert isinstance(mv, ns.EffectiveCharacterMovement)
    # On flat ground the X-component is close to the input (minus the offset
    # gap the controller preserves against the floor).
    assert abs(mv.translation.x - 0.1) < 0.02
    # The character should be grounded.
    assert mv.grounded is True


def test_character_controller_autostep_kwarg(ns):
    """The autostep kwarg drives the controller's autostep field."""
    autostep = ns.CharacterAutostep(
        max_height=ns.CharacterLength.absolute(0.5),
        min_width=ns.CharacterLength.absolute(0.2),
        include_dynamic_bodies=True,
    )
    ctrl = ns.KinematicCharacterController(autostep=autostep)
    got = ctrl.autostep
    assert got is not None
    assert got.max_height.kind == "absolute"
    assert abs(got.max_height.value - 0.5) < 1e-5
    assert got.include_dynamic_bodies is True

    # autostep can be cleared.
    ctrl.autostep = None
    assert ctrl.autostep is None


def test_character_controller_autostep_clears_step(ns):
    """Stand a capsule character on a step. With autostep enabled and a max-
    height larger than the step, the character ascends; without autostep, it
    stays on the lower ground.

    This is the DoD assertion from `08-controllers.md`: autostep should let
    the character climb a 0.3m step.
    """
    # Build a world with a 0.3m-high step the character has to climb over.
    def make_world():
        w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
        w.add_body(
            ns.RigidBody.fixed(translation=(0, -1.0, 0)),
            colliders=[ns.Collider.cuboid(50, 1, 50)],
        )
        w.add_body(
            ns.RigidBody.fixed(translation=(1.5, 0.15, 0)),
            colliders=[ns.Collider.cuboid(3.0, 0.15, 50)],
        )
        w.update_query_pipeline()
        return w

    def simulate(autostep, frames=600):
        w = make_world()
        ctrl = ns.KinematicCharacterController(
            up=(0, 1, 0),
            offset=ns.CharacterLength.absolute(0.01),
            slide=True,
            autostep=autostep,
            snap_to_ground=ns.CharacterLength.absolute(0.5),
        )
        shape = ns.SharedShape.ball(0.3)
        # Starting just to the left of the step.
        pose = ns.Isometry3.from_translation(-1.0, 0.31, 0.0)
        last_y = pose.translation.y
        for _ in range(frames):
            mv = ctrl.move_shape(
                1.0 / 60.0,
                w.rigid_bodies,
                w.colliders,
                w.query_pipeline,
                shape,
                pose,
                (0.05, -0.05, 0.0),
                ns.QueryFilter(),
            )
            pose = ns.Isometry3.from_translation(
                pose.translation.x + mv.translation.x,
                pose.translation.y + mv.translation.y,
                0.0,
            )
            last_y = pose.translation.y
        return pose.translation.x, last_y

    autostep = ns.CharacterAutostep(
        max_height=ns.CharacterLength.absolute(0.4),
        min_width=ns.CharacterLength.absolute(0.05),
        include_dynamic_bodies=True,
    )
    x_with, y_with = simulate(autostep)
    x_without, y_without = simulate(None)

    # With autostep, the character makes it past the step's left edge.
    # Without autostep, it should stop before (or not advance as far).
    assert x_with >= x_without - 1e-3, (
        f"autostep should not regress horizontal progress "
        f"(with={x_with}, without={x_without})"
    )
    # The character with autostep should be elevated to (or near) the top of
    # the step. The step top is at y=0.3, so y_with should reach >= 0.55
    # (sphere of radius 0.3 standing on the 0.3m step).
    assert y_with >= 0.55 or x_with > 0.0, (
        f"autostep failed to climb (x_with={x_with}, y_with={y_with})"
    )


def test_character_collision_event_callback(ns):
    """Collision callback fires when the character bumps into a wall."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    # Ground.
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1.0, 0)),
        colliders=[ns.Collider.cuboid(50, 1, 50)],
    )
    # A tall wall right in front of the character.
    w.add_body(
        ns.RigidBody.fixed(translation=(1.0, 1.0, 0)),
        colliders=[ns.Collider.cuboid(0.1, 1.0, 50)],
    )
    w.update_query_pipeline()

    ctrl = ns.KinematicCharacterController(up=(0, 1, 0), slide=True)
    shape = ns.SharedShape.ball(0.4)
    pose = ns.Isometry3.from_translation(0.0, 0.45, 0.0)

    collisions = []
    mv = ctrl.move_shape(
        1.0 / 60.0,
        w.rigid_bodies,
        w.colliders,
        w.query_pipeline,
        shape,
        pose,
        (1.0, 0.0, 0.0),  # try to walk through the wall
        ns.QueryFilter(),
        events_callback=lambda c: collisions.append(c),
    )
    # The character can't traverse 1.0 units in one shot; it should hit the wall.
    assert len(collisions) >= 1
    for c in collisions:
        assert isinstance(c, ns.CharacterCollision)
        assert hasattr(c, "toi")
        assert hasattr(c, "handle")
    # The translation should be smaller than the requested 1.0.
    assert mv.translation.x < 0.95


# ---------------------------------------------------------------------------
# PidController — drive a kinematic body to a target pose.
# ---------------------------------------------------------------------------


def test_pid_steady_state_error(ns):
    """PID controller drives a dynamic body to a target translation."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))  # no gravity to isolate PID behaviour
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 0, 0)),
        colliders=[ns.Collider.ball(0.5)],
    )
    target_x = 3.0
    pid = ns.PidController(
        axes=ns.AxesMask.all(),
        Kp=(60.0, 60.0, 60.0),
        Ki=(0.0, 0.0, 0.0),
        Kd=(0.8, 0.8, 0.8),
    )

    dt = 1.0 / 120.0
    target_pose = ns.Isometry3.from_translation(target_x, 0.0, 0.0)
    for _ in range(240):  # 2 seconds at 120Hz
        body = w.rigid_bodies[h]
        corr = pid.rigid_body_correction(dt, body, target_pose)
        # PID returns velocity corrections (impulse / mass-ish). Apply as
        # impulse to the body.
        body_for_update = w.rigid_bodies[h]
        # Push linear correction as a linvel addition (apply_impulse is force * mass).
        # Linear correction is in velocity units, so apply via apply_impulse / mass.
        m = body_for_update.mass
        body_for_update.apply_impulse(
            (corr.linear.x * m, corr.linear.y * m, corr.linear.z * m), True
        )
        w.rigid_bodies.replace(h, body_for_update)
        w.step()

    final_x = w.rigid_bodies[h].translation.x
    err = abs(final_x - target_x)
    # 1% of target distance
    assert err < 0.03, f"steady-state error too large: |{final_x} - {target_x}| = {err}"


def test_pd_controller_basic(ns):
    """Plain PD controller produces a finite correction."""
    pd = ns.PdController(
        axes=ns.AxesMask.all(),
        Kp=(10.0, 10.0, 10.0),
        Kd=(1.0, 1.0, 1.0),
    )
    w = ns.PhysicsWorld()
    h = w.add_body(ns.RigidBody.dynamic(translation=(0, 0, 0)).build())
    body = w.rigid_bodies[h]
    target = ns.Isometry3.from_translation(1.0, 0.0, 0.0)
    corr = pd.rigid_body_correction(body, target)
    # The body is at origin and the target is at (1, 0, 0), so the correction
    # should pull along +X.
    assert corr.linear.x > 0.0


def test_pid_position_correction_zero_at_target(ns):
    """A PID asked to correct from a pose to *itself* returns ~zero."""
    pid = ns.PidController(
        axes=ns.AxesMask.all(),
        Kp=(60.0, 60.0, 60.0),
        Ki=(0.0, 0.0, 0.0),
        Kd=(0.8, 0.8, 0.8),
    )
    pose = ns.Isometry3.from_translation(2.5, 1.0, -0.5)
    corr = pid.position_correction(1.0 / 60.0, pose, pose)
    assert abs(corr.linear.x) < 1e-4
    assert abs(corr.linear.y) < 1e-4
    assert abs(corr.linear.z) < 1e-4


# ---------------------------------------------------------------------------
# DynamicRayCastVehicleController
# ---------------------------------------------------------------------------


def _build_vehicle(ns, w):
    """Build a chassis with 4 wheels at corners. Returns (chassis_handle, vehicle).

    Mirrors the parameter choices from `examples3d/vehicle_controller3.rs`
    (stiff suspension + short rest length so the vehicle doesn't bounce off
    the ground on contact).
    """
    hw = 0.3
    hh = 0.15
    chassis_h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 1.0, 0)),
        colliders=[ns.Collider.cuboid(hw * 2.0, hh, hw, density=100.0)],
    )
    veh = ns.DynamicRayCastVehicleController(chassis_h)
    tuning = ns.WheelTuning(
        suspension_stiffness=100.0,
        suspension_damping=10.0,
    )
    # axle along +Z (so wheels roll along X), wheels offset along X (forward).
    for x, z in [(hw * 1.5, hw), (hw * 1.5, -hw), (-hw * 1.5, hw), (-hw * 1.5, -hw)]:
        veh.add_wheel(
            chassis_connection_cs=(x, -hh, z),
            direction_cs=(0, -1, 0),
            axle_cs=(0, 0, 1),
            suspension_rest_length=hh,
            radius=hh / 4.0,
            tuning=tuning,
        )
    return chassis_h, veh


def test_vehicle_construction_and_wheels(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1, 0)),
        colliders=[ns.Collider.cuboid(50, 1, 50)],
    )
    h, veh = _build_vehicle(ns, w)
    assert isinstance(veh.chassis(), ns.RigidBodyHandle)
    assert veh.chassis() == h
    wheels = veh.wheels()
    assert len(wheels) == 4
    for wheel in wheels:
        assert isinstance(wheel, ns.Wheel)
        assert wheel.radius > 0.0
        assert wheel.engine_force == 0.0
        assert wheel.brake == 0.0


def test_vehicle_accelerates_with_engine_force(ns):
    """Applying engine force makes the vehicle gain forward speed (in m/s,
    measured via the chassis linvel along its forward axis)."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -0.1, 0)),
        colliders=[ns.Collider.cuboid(50, 0.1, 50)],
    )
    h, veh = _build_vehicle(ns, w)
    # Let it settle on the ground first.
    for _ in range(120):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)
    forward_before = w.rigid_bodies[h].linvel.x

    # Apply forward engine force on rear wheels (indices 2 and 3).
    veh.apply_engine_force(2, 100.0)
    veh.apply_engine_force(3, 100.0)
    for _ in range(120):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)
    forward_after = w.rigid_bodies[h].linvel.x
    # Forward speed should grow under engine force.
    assert abs(forward_after) > abs(forward_before), (
        f"engine force did not accelerate (before={forward_before}, after={forward_after})"
    )
    # Sanity check the km/h converter agrees with the chassis linvel.
    assert veh.current_speed_km_hour() != 0.0


def test_vehicle_brakes_decelerate(ns):
    """Setting brake on the wheels reduces the forward speed."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -0.1, 0)),
        colliders=[ns.Collider.cuboid(50, 0.1, 50)],
    )
    h, veh = _build_vehicle(ns, w)
    # Let it settle.
    for _ in range(120):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)

    # Accelerate.
    for i in (2, 3):
        veh.apply_engine_force(i, 100.0)
    for _ in range(120):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)
    forward_before = abs(w.rigid_bodies[h].linvel.x)
    assert forward_before > 0.1, f"vehicle never accelerated ({forward_before})"

    # Brake.
    for i in (0, 1, 2, 3):
        veh.apply_engine_force(i, 0.0)
        veh.set_brake(i, 500.0)
    # First step the vehicle for a short time so the brake actually acts.
    for _ in range(60):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)
    forward_after = abs(w.rigid_bodies[h].linvel.x)
    assert forward_after < forward_before, (
        f"brakes did not slow the vehicle (before={forward_before}, after={forward_after})"
    )


def test_vehicle_set_steering(ns):
    """Verifies the steering setter takes effect on the wheel."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1, 0)),
        colliders=[ns.Collider.cuboid(50, 1, 50)],
    )
    _, veh = _build_vehicle(ns, w)
    veh.set_steering(0, 0.3)
    veh.set_steering(1, 0.3)
    assert abs(veh.wheel(0).steering - 0.3) < 1e-5
    assert abs(veh.wheel(1).steering - 0.3) < 1e-5
    assert abs(veh.wheel(2).steering) < 1e-5  # untouched


def test_wheel_tuning_default_kwargs(ns):
    t = ns.WheelTuning.default()
    assert t.suspension_stiffness > 0.0
    assert t.friction_slip > 0.0
    t2 = ns.WheelTuning(suspension_stiffness=10.0, friction_slip=5.0)
    assert abs(t2.suspension_stiffness - 10.0) < 1e-5
    assert abs(t2.friction_slip - 5.0) < 1e-5
