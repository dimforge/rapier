//! Regression test for https://github.com/dimforge/rapier/issues/499
//!
//! Angular limit rows compare half-angle sines, which only rank angles correctly over a half
//! turn. Measured from the joint's rest frame that capped limits at +-pi and folded anything
//! past it back: `[0, 270deg]` stopped the joint at 90deg, `[-200deg, 200deg]` at 160deg, and a
//! joint pushed beyond pi escaped through the back of its own limit. The rows now measure the
//! angle from the middle of the allowed range, so any range up to a full turn works wherever
//! it sits on the circle.

use rapier3d::prelude::*;
use std::f32::consts::{PI, TAU};

/// How the joint is pushed against its limit — which also picks the constraint path the limit
/// row is built by.
#[derive(Copy, Clone, PartialEq, Debug)]
enum Drive {
    /// A velocity motor. In 3D a motorized joint has no wide row formulation, so this goes
    /// through the scalar impulse-joint path.
    Motor,
    /// A constant external torque, so the joint's rows are limit rows only: the SIMD path.
    Torque,
    /// Like `Torque`, but the joint is a multibody joint (the reduced-coordinates path).
    MultibodyTorque,
}

/// Pushes a revolute joint (about +Z) against its `[min, max]` limit and returns the angle, in
/// degrees, it settles at (unwrapped: it keeps counting past +-180).
fn settled_angle_with(drive: Drive, limits_deg: [f32; 2], dir: f32) -> f32 {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = 1.0 / 60.0;

    let body1 = world.bodies.insert(RigidBodyBuilder::fixed());
    let body2 = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0, 0.0))
            // Caps the speed the joint arrives at its limit with, so the (soft) limit row
            // settles it right at the limit instead of a few degrees past it.
            .angular_damping(3.0)
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.1, 0.1),
        body2,
        &mut world.bodies,
    );

    let mut joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
        .limits([limits_deg[0].to_radians(), limits_deg[1].to_radians()]);
    if drive == Drive::Motor {
        joint = joint.motor_velocity(dir * 5.0, 20.0);
    }
    if drive == Drive::MultibodyTorque {
        world
            .multibody_joints
            .insert(body1, body2, joint, true)
            .unwrap();
    } else {
        world.impulse_joints.insert(body1, body2, joint, true);
    }

    let mut unwrapped = 0.0;
    let mut prev = 0.0;
    for _ in 0..600 {
        if drive != Drive::Motor {
            // Enough torque to push against the limit, little enough not to blow through it.
            world.bodies[body2].add_torque(Vector::Z * dir * 0.1, true);
        }
        world.step();
        let rot = *world.bodies[body2].rotation();
        // Signed angle about +Z, in (-pi, pi].
        let ang = 2.0 * rot.z.atan2(rot.w);
        let mut delta = ang - prev;
        if delta > PI {
            delta -= TAU;
        } else if delta < -PI {
            delta += TAU;
        }
        unwrapped += delta;
        prev = ang;
    }

    unwrapped.to_degrees()
}

fn settled_angle(limits_deg: [f32; 2], dir: f32) -> f32 {
    settled_angle_with(Drive::Motor, limits_deg, dir)
}

/// The joint, pushed either way, settles at `limits[1]` (resp. `limits[0]`) — on every
/// constraint path.
fn assert_limits_reached(limits_deg: [f32; 2]) {
    for drive in [Drive::Motor, Drive::Torque, Drive::MultibodyTorque] {
        let max = settled_angle_with(drive, limits_deg, 1.0);
        let min = settled_angle_with(drive, limits_deg, -1.0);
        assert!(
            (max - limits_deg[1]).abs() < 2.0,
            "limits {limits_deg:?} ({drive:?}): driving + settled at {max} deg instead of {} deg",
            limits_deg[1]
        );
        assert!(
            (min - limits_deg[0]).abs() < 2.0,
            "limits {limits_deg:?} ({drive:?}): driving - settled at {min} deg instead of {} deg",
            limits_deg[0]
        );
    }
}

#[test]
fn angular_limits_within_half_a_turn_are_reached() {
    // Ranges that already worked before the fix — they must keep working.
    assert_limits_reached([-45.0, 45.0]);
    assert_limits_reached([-135.0, 135.0]);
    assert_limits_reached([0.0, 90.0]);
    assert_limits_reached([-170.0, -10.0]);
}

#[test]
fn angular_limits_past_half_a_turn_are_reached() {
    // Issue #499: these used to fold back (`[0, 270]` stopped at 90 deg, `[-200, 200]` at 160).
    assert_limits_reached([0.0, 270.0]);
    assert_limits_reached([-270.0, 0.0]);
    assert_limits_reached([-90.0, 200.0]);
    assert_limits_reached([-350.0, 0.0]);
}

#[test]
fn angular_limits_straddling_half_a_turn_are_reached() {
    // Ranges going through the +-pi seam: representable at all only because the row measures
    // the angle from the middle of the range (here 180 deg).
    assert_limits_reached([45.0, 315.0]);
    assert_limits_reached([-315.0, -45.0]);
    assert_limits_reached([135.0, 225.0]);
}

#[test]
fn angular_limits_wider_than_a_turn_leave_the_joint_free() {
    // A wrapped angle can't tell a more-than-full-turn range from no limit at all, so the row
    // is disabled instead of clamping at some arbitrary folded-back angle (it used to stop a
    // `[-200, 200]` joint at 160 deg, and a `[-350, 350]` one at 10 deg).
    for limits in [[-180.0, 180.0], [-200.0, 200.0], [-350.0, 350.0]] {
        let angle = settled_angle(limits, 1.0);
        assert!(
            angle > 360.0,
            "limits {limits:?} span more than a turn but stopped the joint at {angle} deg"
        );
    }
}

#[test]
fn a_joint_shoved_past_its_limit_comes_back() {
    // A joint knocked beyond the +-pi seam used to keep going the wrong way round: the row
    // read its angle as a large NEGATIVE one, so it pushed it further forward instead of back
    // to the limit — the "bodies bug out at the limit" symptom reported on the issue.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = 1.0 / 60.0;

    let body1 = world.bodies.insert(RigidBodyBuilder::fixed());
    // Starts at 185 deg: 15 deg past the joint's 170 deg limit, and past the seam.
    let body2 = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-0.996, -0.087, 0.0))
            .rotation(Vector::new(0.0, 0.0, 185f32.to_radians()))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.1, 0.1),
        body2,
        &mut world.bodies,
    );
    world.impulse_joints.insert(
        body1,
        body2,
        RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(Vector::ZERO)
            .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
            .limits([0.0, 170f32.to_radians()]),
        true,
    );

    for _ in 0..300 {
        world.step();
    }

    let rot = *world.bodies[body2].rotation();
    let angle = (2.0 * rot.z.atan2(rot.w)).to_degrees();
    // Back inside `[0, 170]` (it keeps whatever momentum the push-back left it with, so it
    // may coast anywhere inside the range) instead of having run off past 180 deg.
    assert!(
        (-2.0..172.0).contains(&angle),
        "the joint should have been pushed back into its [0, 170] deg range, but ended at {angle} deg"
    );
}
