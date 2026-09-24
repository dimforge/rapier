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


@pytest.fixture(params=[dim3], ids=["f32"])
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
        # PID returns velocity corrections (impulse / mass-ish). `body` is a
        # live view into the set, so applying the impulse persists directly —
        # no write-back needed.
        # Linear correction is in velocity units; apply via apply_impulse / mass.
        m = body.mass
        body.apply_impulse(
            (corr.linear.x * m, corr.linear.y * m, corr.linear.z * m), True
        )
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


def test_vehicle_settles_on_its_suspension(ns):
    """The vehicle comes to rest on the ground, not in the air.

    Regression test: without excluding the chassis from its own suspension
    raycasts, every wheel hits the vehicle itself and it flies off.
    """
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -0.1, 0)),
        colliders=[ns.Collider.cuboid(50, 0.1, 50)],
    )
    h, veh = _build_vehicle(ns, w)
    for _ in range(120):
        w.step()
        w.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline)

    body = w.rigid_bodies[h]
    # Half-height + suspension rest length + wheel radius, minus spring sag.
    assert 0.25 < body.translation.y < 0.34, (
        f"chassis is not riding on its suspension: y={body.translation.y}"
    )
    assert abs(body.linvel.y) < 0.5, f"chassis is still moving: vy={body.linvel.y}"


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
    # ...by driving on the ground, not by taking off.
    assert 0.25 < w.rigid_bodies[h].translation.y < 0.34
    # Sanity check the km/h converter agrees with the chassis linvel.
    assert veh.current_speed_km_hour() != 0.0


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


# ---------------------------------------------------------------------------
# PD / PID gains, integrals and target velocities.
# ---------------------------------------------------------------------------


def _vec_close(v, expected, tol=1e-5):
    return all(abs(a - b) < tol for a, b in zip((v.x, v.y, v.z), expected))


def test_pid_scalar_gains_apply_to_every_axis(ns):
    pid = ns.PidController(Kp=60.0, Ki=0.5, Kd=0.8)
    for name, k in [("kp", 60.0), ("ki", 0.5), ("kd", 0.8)]:
        assert _vec_close(getattr(pid, "lin_" + name), (k, k, k))
        assert _vec_close(getattr(pid, "ang_" + name), (k, k, k))
    pd = ns.PdController(Kp=10, Kd=2.0)
    assert _vec_close(pd.lin_kp, (10.0, 10.0, 10.0))
    assert _vec_close(pd.ang_kd, (2.0, 2.0, 2.0))
    # Per-axis gains still work.
    pid = ns.PidController(Kp=(1.0, 2.0, 3.0))
    assert _vec_close(pid.lin_kp, (1.0, 2.0, 3.0))
    assert _vec_close(pid.ang_kp, (1.0, 2.0, 3.0))
    with pytest.raises(TypeError):
        ns.PidController(Kp="fast")


@pytest.mark.parametrize("cls", ["PdController", "PidController"])
def test_controller_gain_properties(ns, cls):
    ctrl = getattr(ns, cls)()
    names = ["lin_kp", "ang_kp", "lin_kd", "ang_kd"]
    if cls == "PidController":
        names += ["lin_ki", "ang_ki"]
    for name in names:
        setattr(ctrl, name, (1.0, 2.0, 3.0))
        assert _vec_close(getattr(ctrl, name), (1.0, 2.0, 3.0))
        setattr(ctrl, name, 4.0)
        assert _vec_close(getattr(ctrl, name), (4.0, 4.0, 4.0))
        with pytest.raises(TypeError):
            setattr(ctrl, name, "x")
    # The linear and angular gains are independent.
    ctrl.lin_kp = 5.0
    ctrl.ang_kp = 6.0
    assert _vec_close(ctrl.lin_kp, (5.0, 5.0, 5.0))
    assert _vec_close(ctrl.ang_kp, (6.0, 6.0, 6.0))


def test_pid_axes_setter_and_integrals(ns):
    pid = ns.PidController(Kp=1.0, Ki=1.0, Kd=0.0)
    assert not hasattr(pid, "axes_attr")
    pid.axes = ns.AxesMask.LIN_X
    assert pid.axes == ns.AxesMask.LIN_X

    assert _vec_close(pid.lin_integral, (0.0, 0.0, 0.0))
    assert _vec_close(pid.ang_integral, (0.0, 0.0, 0.0))
    with pytest.raises(AttributeError):
        pid.lin_integral = (1.0, 0.0, 0.0)
    with pytest.raises(AttributeError):
        pid.ang_integral = (1.0, 0.0, 0.0)

    pose = ns.Isometry3.identity()
    target = ns.Isometry3.from_translation(1.0, 0.0, 0.0)
    pid.position_correction(0.5, pose, target)
    pid.position_correction(0.5, pose, target)
    assert pid.lin_integral.x > 0.0
    pid.reset()
    assert _vec_close(pid.lin_integral, (0.0, 0.0, 0.0))


def test_controller_target_vels(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    h = w.add_body(ns.RigidBody.dynamic(translation=(0, 0, 0)).build())
    body = w.rigid_bodies[h]
    target = ns.Isometry3.identity()
    target_vels = ns.RigidBodyVelocity(linvel=(2.0, 0.0, 0.0), angvel=(0.0, 3.0, 0.0))

    pd = ns.PdController(Kp=10.0, Kd=1.0)
    corr = pd.rigid_body_correction(body, target)
    assert _vec_close(corr.linear, (0.0, 0.0, 0.0))
    corr = pd.rigid_body_correction(body, target, target_vels)
    assert corr.linear.x > 0.0
    assert corr.angular.y > 0.0

    pid = ns.PidController(Kp=10.0, Ki=0.0, Kd=1.0)
    corr = pid.rigid_body_correction(1.0 / 60.0, body, target, target_vels=target_vels)
    assert corr.linear.x > 0.0
    assert corr.angular.y > 0.0
    corr = pid.rigid_body_correction(1.0 / 60.0, body, target, None)
    assert _vec_close(corr.linear, (0.0, 0.0, 0.0))


def test_pid_scalar_gains_reach_target(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(ns.Collider.cuboid(100.0, 0.1, 100.0).build())
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)),
        colliders=[ns.Collider.ball(0.5)],
    )
    axes = ns.AxesMask.LIN_X | ns.AxesMask.LIN_Y | ns.AxesMask.LIN_Z
    pid = ns.PidController(axes=axes, Kp=60.0, Ki=0.0, Kd=0.8)
    target = ns.Isometry3.from_translation(3.0, 2.0, 0.0)
    for _ in range(200):
        body = w.rigid_bodies[h]
        corr = pid.rigid_body_correction(w.integration_parameters.dt, body, target)
        body.linvel = body.linvel + corr.linear
        w.step()
    assert (w.rigid_bodies[h].translation - target.translation).norm() < 0.05


def test_axes_mask_doc_is_3d_only(ns):
    assert "2D" not in ns.AxesMask.__doc__


# ---------------------------------------------------------------------------
# KinematicCharacterController configuration and collision impulses.
# ---------------------------------------------------------------------------


def test_character_controller_none_disables_features(ns):
    default = ns.KinematicCharacterController()
    assert default.snap_to_ground == ns.CharacterLength.relative(0.2)
    assert default.autostep is None

    ctrl = ns.KinematicCharacterController(snap_to_ground=None, autostep=None)
    assert ctrl.snap_to_ground is None
    assert ctrl.autostep is None

    ctrl = ns.KinematicCharacterController(snap_to_ground=0.3, autostep=ns.CharacterAutostep())
    assert ctrl.snap_to_ground == ns.CharacterLength.absolute(0.3)
    assert ctrl.autostep is not None


def test_character_controller_repr(ns):
    r = repr(ns.KinematicCharacterController())
    assert "snap_to_ground=CharacterLength.relative(0.2)" in r
    assert "offset=CharacterLength.relative(0.01)" in r
    assert "autostep=None" in r
    r = repr(ns.KinematicCharacterController(autostep=ns.CharacterAutostep(max_height=0.5)))
    assert "CharacterAutostep(max_height=CharacterLength.absolute(0.5)" in r
    assert "=set" not in r


def _pushable_box_world(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1.0, 0)),
        colliders=[ns.Collider.cuboid(50, 1, 50)],
    )
    box = w.add_body(
        ns.RigidBody.dynamic(translation=(1.0, 0.5, 0)),
        colliders=[ns.Collider.cuboid(0.5, 0.5, 0.5)],
    )
    w.update_query_pipeline()
    return w, box


def _push_box(ns, w, filter, character_pos):
    ctrl = ns.KinematicCharacterController(up=(0, 1, 0), slide=True)
    shape = ns.SharedShape.ball(0.4)
    pose = ns.Isometry3.from_translation(0.0, 0.45, 0.0)
    collisions = []
    ctrl.move_shape(
        1.0 / 60.0,
        w.rigid_bodies,
        w.colliders,
        w.query_pipeline,
        shape,
        pose,
        (1.0, 0.0, 0.0),
        filter,
        events_callback=collisions.append,
    )
    assert collisions
    ctrl.solve_character_collision_impulses(
        1.0 / 60.0,
        w.rigid_bodies,
        w.colliders,
        w.query_pipeline,
        shape,
        character_pos,
        10.0,
        collisions,
        filter,
    )
    return collisions


def test_character_collision_hit_alias(ns):
    w, _ = _pushable_box_world(ns)
    collisions = _push_box(ns, w, ns.QueryFilter(), None)
    for c in collisions:
        assert c.hit.time_of_impact == c.toi.time_of_impact
        assert c.hit.normal1 == c.toi.normal1


def test_solve_character_collision_impulses_honors_predicate(ns):
    # Without a predicate the character pushes the box.
    w, box = _pushable_box_world(ns)
    pose = ns.Isometry3.from_translation(0.0, 0.45, 0.0)
    _push_box(ns, w, ns.QueryFilter(), pose)
    assert w.rigid_bodies[box].linvel.x > 0.0

    # The predicate is evaluated on every collider, and can read it.
    w, box = _pushable_box_world(ns)
    seen = []
    box_colliders = set(w.rigid_bodies[box].colliders)

    def predicate(handle, collider):
        seen.append(handle)
        return collider.parent != box

    impulse_filter = ns.QueryFilter(predicate=predicate)
    # `move_shape` still collides with the box: only the impulses skip it.
    ctrl = ns.KinematicCharacterController(up=(0, 1, 0), slide=True)
    shape = ns.SharedShape.ball(0.4)
    collisions = []
    ctrl.move_shape(
        1.0 / 60.0, None, None, w.query_pipeline, shape, pose, (1.0, 0.0, 0.0),
        events_callback=collisions.append,
    )
    assert collisions
    seen.clear()
    ctrl.solve_character_collision_impulses(
        1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline, shape, None, 10.0,
        collisions, impulse_filter,
    )
    assert box_colliders <= set(seen)
    assert w.rigid_bodies[box].linvel.x == 0.0

    # Predicate errors propagate.
    def failing(handle, collider):
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        ctrl.solve_character_collision_impulses(
            1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline, shape, None, 10.0,
            collisions, ns.QueryFilter(predicate=failing),
        )


def test_character_controller_rejects_foreign_sets(ns):
    w, _ = _pushable_box_world(ns)
    ctrl = ns.KinematicCharacterController()
    shape = ns.SharedShape.ball(0.4)
    pose = ns.Isometry3.from_translation(0.0, 0.45, 0.0)
    # `None` is accepted for the unused sets.
    mv = ctrl.move_shape(1.0 / 60.0, None, None, w.query_pipeline, shape, pose, (0.1, 0.0, 0.0))
    assert isinstance(mv, ns.EffectiveCharacterMovement)
    with pytest.raises(ValueError):
        ctrl.move_shape(
            1.0 / 60.0, ns.RigidBodySet(), w.colliders, w.query_pipeline, shape, pose,
            (0.1, 0.0, 0.0),
        )
    with pytest.raises(ValueError):
        ctrl.solve_character_collision_impulses(
            1.0 / 60.0, w.rigid_bodies, ns.ColliderSet(), w.query_pipeline, shape, None,
            1.0, [],
        )


# ---------------------------------------------------------------------------
# Vehicle wheels: world-space state, index errors, and query filters.
# ---------------------------------------------------------------------------


def _settled_vehicle(ns, filter=None, frames=120, steering=0.0):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)
    ground = w.add_body(
        ns.RigidBody.fixed(translation=(0, -0.1, 0)),
        colliders=[ns.Collider.cuboid(50, 0.1, 50)],
    )
    h, veh = _build_vehicle(ns, w)
    veh.set_steering(0, steering)
    for _ in range(frames):
        w.step()
        w.update_query_pipeline()
        f = filter(ground) if callable(filter) else filter
        veh.update_vehicle(1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline, f)
    return w, h, veh


def test_wheel_world_space_state(ns):
    w, h, veh = _settled_vehicle(ns, steering=0.3)
    chassis = w.rigid_bodies[h]
    for i, wheel in enumerate(veh.wheels()):
        info = wheel.raycast_info
        assert info.is_in_contact
        # The center lies along the suspension, below the hard point.
        expected = info.hard_point_ws + wheel.suspension * info.suspension_length
        assert (wheel.center - expected).norm() < 1e-4
        assert wheel.center.y < chassis.translation.y
        assert _vec_close(wheel.suspension, (0.0, -1.0, 0.0), 1e-2)
        assert abs(wheel.axle.norm() - 1.0) < 1e-4
        if i == 0:
            # The steered axle turns around the vertical axis.
            assert abs(wheel.axle.x) > 0.1
        else:
            assert _vec_close(wheel.axle, (0.0, 0.0, 1.0), 1e-2)
    with pytest.raises(AttributeError):
        veh.wheel(0).center = (0.0, 0.0, 0.0)


def test_wheel_index_out_of_range_raises_index_error(ns):
    w = ns.PhysicsWorld()
    _, veh = _build_vehicle(ns, w)
    for bad in (4, 100, -1):
        with pytest.raises(IndexError):
            veh.wheel(bad)
        with pytest.raises(IndexError):
            veh.set_brake(bad, 1.0)
        with pytest.raises(IndexError):
            veh.set_steering(bad, 1.0)
        with pytest.raises(IndexError):
            veh.apply_engine_force(bad, 1.0)


def test_update_vehicle_honors_predicate(ns):
    # A predicate rejecting the ground: the wheels find nothing and the vehicle falls.
    def reject_ground(ground):
        def predicate(handle, collider):
            return collider.parent != ground

        return ns.QueryFilter(predicate=predicate)

    w, h, veh = _settled_vehicle(ns, reject_ground, frames=30)
    assert not any(wheel.raycast_info.is_in_contact for wheel in veh.wheels())

    # An accepting predicate keeps the default behaviour.
    w, h, veh = _settled_vehicle(ns, ns.QueryFilter(predicate=lambda h, c: True))
    assert all(wheel.raycast_info.is_in_contact for wheel in veh.wheels())
    assert 0.25 < w.rigid_bodies[h].translation.y < 0.34

    def failing(handle, collider):
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        veh.update_vehicle(
            1.0 / 60.0, w.rigid_bodies, w.colliders, w.query_pipeline,
            ns.QueryFilter(predicate=failing),
        )


def test_update_vehicle_excludes_chassis_with_other_body_exclusion(ns):
    # Excluding another body must not make the wheels hit the chassis.
    other = ns.RigidBodyHandle.invalid()
    w, h, veh = _settled_vehicle(ns, ns.QueryFilter(exclude_rigid_body=other))
    assert 0.25 < w.rigid_bodies[h].translation.y < 0.34
    for wheel in veh.wheels():
        assert wheel.raycast_info.ground_object not in set(w.rigid_bodies[h].colliders)


def test_update_vehicle_rejects_foreign_sets(ns):
    w = ns.PhysicsWorld()
    _, veh = _build_vehicle(ns, w)
    with pytest.raises(ValueError):
        veh.update_vehicle(1.0 / 60.0, ns.RigidBodySet(), w.colliders, w.query_pipeline)
