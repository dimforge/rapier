"""2D controller tests.

Only the character controller and the PID/PD family are bound in 2D (vehicle
is gated to 3D upstream — see `src/control/mod.rs`).
"""

from __future__ import annotations

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_character_length_2d(ns):
    a = ns.CharacterLength.absolute(0.5)
    r = ns.CharacterLength.relative(0.1)
    assert a.kind == "absolute"
    assert r.kind == "relative"


def test_axes_mask_2d_constants(ns):
    am = ns.AxesMask.LIN_X | ns.AxesMask.LIN_Y | ns.AxesMask.ANG_Z
    assert ns.AxesMask.LIN_X in am
    assert ns.AxesMask.LIN_Y in am
    assert ns.AxesMask.ANG_Z in am
    # In 2D there's no LIN_Z / ANG_X / ANG_Y.
    assert not hasattr(ns.AxesMask, "LIN_Z")
    assert not hasattr(ns.AxesMask, "ANG_X")
    assert not hasattr(ns.AxesMask, "ANG_Y")


def test_character_controller_2d_moves(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81), auto_update_query=True)
    # Flat ground.
    w.add_body(
        ns.RigidBody.fixed(translation=(0, -1.0)),
        colliders=[ns.Collider.cuboid(50, 1)],
    )
    w.update_query_pipeline()

    ctrl = ns.KinematicCharacterController(
        up=(0, 1),
        offset=ns.CharacterLength.absolute(0.01),
        slide=True,
        snap_to_ground=ns.CharacterLength.absolute(0.5),
    )
    shape = ns.SharedShape.ball(0.5)
    pose = ns.Isometry2.from_translation(0.0, 0.55)
    mv = ctrl.move_shape(
        1.0 / 60.0,
        w.rigid_bodies,
        w.colliders,
        w.query_pipeline,
        shape,
        pose,
        (0.1, -0.05),
        ns.QueryFilter(),
    )
    assert isinstance(mv, ns.EffectiveCharacterMovement)
    # Movement along +X should make it through.
    assert abs(mv.translation.x - 0.1) < 1e-3
    assert mv.grounded is True


def test_pid_2d_construction(ns):
    pid = ns.PidController(
        axes=ns.AxesMask.all(),
        Kp=(60.0, 60.0),
        Ki=(0.0, 0.0),
        Kd=(0.8, 0.8),
    )
    pose = ns.Isometry2.from_translation(2.0, 0.0)
    target = ns.Isometry2.from_translation(2.0, 0.0)
    corr = pid.position_correction(1.0 / 60.0, pose, target)
    # No error → no correction.
    assert abs(corr.linear.x) < 1e-4
    assert abs(corr.linear.y) < 1e-4


def test_pd_2d_correction(ns):
    pd = ns.PdController(
        axes=ns.AxesMask.all(),
        Kp=(10.0, 10.0),
        Kd=(1.0, 1.0),
    )
    w = ns.PhysicsWorld()
    h = w.add_body(ns.RigidBody.dynamic(translation=(0, 0)).build())
    body = w.rigid_bodies[h]
    target = ns.Isometry2.from_translation(1.0, 0.0)
    corr = pd.rigid_body_correction(body, target)
    assert corr.linear.x > 0.0


def test_no_vehicle_controller_in_2d(ns):
    """The 3D-only vehicle controller is intentionally absent in 2D."""
    assert not hasattr(ns, "DynamicRayCastVehicleController")
    assert not hasattr(ns, "Wheel")
    assert not hasattr(ns, "WheelTuning")
    assert not hasattr(ns, "RayCastInfo")
