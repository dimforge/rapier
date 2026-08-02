//! Isolate: does CCD catch a fast body fired at an ORIENTED polyline wall,
//! the same way it does against a cuboid wall?

use rapier2d::prelude::*;

fn fires_through(wall: ColliderBuilder, speed: f32) -> f32 {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(wall, ground, &mut bodies);

    // Ball fired at +X from x=-3 toward the wall at x=0.
    let ball = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0))
            .linvel(Vector::new(speed, 0.0)),
    );
    colliders.insert_with_parent(ColliderBuilder::ball(0.2), ball, &mut bodies);

    for _ in 0..120 {
        pipeline.step(
            Vector::ZERO,
            &params,
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
    }
    bodies[ball].translation().x
}

/// A vertical wall as an oriented polyline (solid on the -X side, since the
/// segment is wound so its outward normal faces -X). A ball fired at +X into it
/// must not pass through.
#[test]
fn ccd_catches_fast_ball_oriented_polyline() {
    // Segment from (0, +5) down to (0, -5): outward normal faces +X (right-hand
    // rule on the downward edge), so the solid side is +X and a ball coming from
    // -X hits the front face.
    let pts = vec![Vector::new(0.0, 5.0), Vector::new(0.0, -5.0)];
    let wall = ColliderBuilder::oriented_polyline(pts, Some(vec![[0, 1]])).friction(0.1);
    let x = fires_through(wall, 400.0);
    println!("oriented polyline: final x = {x}");
    assert!(
        x < 0.2,
        "ball tunneled through the oriented polyline wall (x = {x})"
    );
}

/// Same, but a cuboid wall (known-good CCD target) as a control.
#[test]
fn ccd_catches_fast_ball_cuboid_control() {
    let wall = ColliderBuilder::cuboid(0.05, 5.0).friction(0.1);
    let x = fires_through(wall, 400.0);
    println!("cuboid: final x = {x}");
    assert!(x < 0.2, "ball tunneled through the cuboid wall (x = {x})");
}

/// Same, but a non-oriented (two-sided) polyline.
#[test]
fn ccd_catches_fast_ball_plain_polyline() {
    let pts = vec![Vector::new(0.0, 5.0), Vector::new(0.0, -5.0)];
    let wall = ColliderBuilder::polyline(pts, Some(vec![[0, 1]])).friction(0.1);
    let x = fires_through(wall, 400.0);
    println!("plain polyline: final x = {x}");
    assert!(
        x < 0.2,
        "ball tunneled through the plain polyline wall (x = {x})"
    );
}
