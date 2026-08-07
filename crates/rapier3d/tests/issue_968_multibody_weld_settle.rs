//! Regression test for <https://github.com/dimforge/rapier/issues/968>.
//!
//! A ball welded to a collider-less dynamic body via a multibody `FixedJoint` must settle on
//! the ground like the ball alone; the multibody contact path used to pop it ~0.15 upward
//! forever instead.

use rapier3d::prelude::*;

fn run(with_joint: bool) -> (Real, Real) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vec3::new(0.0, -9.81, 0.0);

    // Ground: top surface at y = -2.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vec3::new(0.0, -3.0, 0.0)),
        ColliderBuilder::cuboid(10.0, 1.0, 10.0),
    );

    // The ball (radius 1), resting position: y = -1.
    let (ball, _) = world.insert(RigidBodyBuilder::dynamic(), ColliderBuilder::ball(1.0));

    // A collider-less dynamic body welded 5 units above the ball.
    let extra_rb = world.insert_body(RigidBodyBuilder::dynamic().translation(Vec3::Y * 5.0));

    if with_joint {
        let joint = FixedJointBuilder::new()
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(Vec3::new(0.0, -5.0, 0.0));
        world.insert_multibody_joint(ball, extra_rb, joint);
    }

    let mut min_y = Real::MAX;
    let mut max_y = Real::MIN;
    for i in 0..1000 {
        world.wake_up_all(true);
        world.step();
        let y = world.bodies[ball].translation().y;
        assert!(y.is_finite(), "ball y became non-finite at step {i}");
        if i >= 500 {
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
    }
    (min_y, max_y)
}

#[test]
fn multibody_welded_ball_settles_on_ground() {
    let (baseline_min, baseline_max) = run(false);
    let (min_y, max_y) = run(true);

    // Sanity: the joint-less baseline settles at rest on the ground.
    assert!(
        (baseline_max - baseline_min) < 0.005 && baseline_min > -1.05,
        "baseline should settle: y in [{baseline_min}, {baseline_max}]"
    );

    // Regression: the welded pair must settle too. With the bug the ball kept
    // popping up, oscillating with amplitude ~0.12 in [-1.003, -0.883].
    assert!(
        max_y - min_y < 0.005,
        "welded ball never settled: y amplitude {} over steps 500..1000 (y in [{min_y}, {max_y}])",
        max_y - min_y
    );
    assert!(
        min_y > -1.05,
        "welded ball sank below resting height: min y = {min_y}"
    );
}
