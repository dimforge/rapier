//! A ball rolling across the cells of a `Voxels` collider produces one contact manifold
//! per touched cell. The incremental solver-contact graph must stay consistent while the
//! set of touched cells changes (a debug assertion checks this in debug builds).

use rapier3d::prelude::*;

#[test]
fn ball_rolling_on_voxels_floor_stays_on_the_floor() {
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut soft_bodies = SoftBodySet::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();

    // A 16x1x16 floor of 1 m cells whose top face is at y = 0.
    let mut cells = Vec::new();
    for x in 0..16 {
        for z in 0..16 {
            cells.push(IVector::new(x - 8, -1, z - 8));
        }
    }
    colliders.insert(ColliderBuilder::voxels(Vector::new(1.0, 1.0, 1.0), &cells));

    let radius = 0.4;
    let ball = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-6.0, radius, 0.0))
            .can_sleep(false),
    );
    colliders.insert_with_parent(ColliderBuilder::ball(radius), ball, &mut bodies);

    // Let the ball settle, then push it horizontally.
    let mut step = |bodies: &mut RigidBodySet, colliders: &mut ColliderSet| {
        pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            bodies,
            colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut soft_bodies,
            &mut ccd,
            &(),
            &(),
        );
    };
    for _ in 0..30 {
        step(&mut bodies, &mut colliders);
    }
    bodies[ball].set_linvel(Vector::new(4.0, 0.0, 0.0), true);

    for _ in 0..150 {
        step(&mut bodies, &mut colliders);
        let vy = bodies[ball].linvel().y.abs();
        assert!(vy < 0.05, "ball bounced on the voxels floor: vy = {vy}");
    }
}
