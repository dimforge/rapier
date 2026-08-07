//! Regression test for #862: `solve_character_collision_impulses` must apply the impulse at
//! the contact point transformed by the COLLIDER's world pose, not its parent body's.
//!
//! With an offset collider the two poses differ, and the impulse used to land at the wrong
//! point, yielding a wrong torque.

use rapier3d::control::KinematicCharacterController;
use rapier3d::prelude::*;

/// Pushes a kinematic character against a tall dynamic box whose world geometry is identical
/// across runs: `body_translation` (the body's pose) and `collider_offset` (the collider's
/// pose w.r.t. its body) always add up to (0, 3, 0).
///
/// Any pair of runs therefore simulates the same physical object and must react identically.
fn run_push(body_translation: Vector, collider_offset: Vector) -> (Vector, Vector) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let params = IntegrationParameters::default();

    let box_body = bodies.insert(RigidBodyBuilder::dynamic().translation(body_translation));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 3.5, 0.5).translation(collider_offset),
        box_body,
        &mut bodies,
    );

    // One step (zero gravity) so the broad-phase knows about the collider.
    pipeline.step(
        Vector::ZERO,
        &params,
        &mut islands,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut CCDSolver::new(),
        &(),
        &(),
    );

    // The character is a free-standing ball shape pushing toward the box's -x face,
    // at the same height as the contact point (y = 0), well below the box's center
    // of mass (y = 3): the impulse must produce a torque.
    let controller = KinematicCharacterController::default();
    let character_shape = ColliderBuilder::ball(0.5).build();
    let character_pos = Pose::from_translation(Vector::new(-1.2, 0.0, 0.0));

    let query_pipeline = broad_phase.as_query_pipeline(
        narrow_phase.query_dispatcher(),
        &bodies,
        &colliders,
        QueryFilter::default(),
    );

    let mut collisions = vec![];
    let _ = controller.move_shape(
        params.dt,
        &query_pipeline,
        character_shape.shape(),
        &character_pos,
        Vector::new(0.5, 0.0, 0.0),
        |c| collisions.push(c),
    );
    assert!(
        !collisions.is_empty(),
        "the character never hit the dynamic box"
    );

    let mut query_pipeline_mut = broad_phase.as_query_pipeline_mut(
        narrow_phase.query_dispatcher(),
        &mut bodies,
        &mut colliders,
        QueryFilter::default(),
    );
    controller.solve_character_collision_impulses(
        params.dt,
        &mut query_pipeline_mut,
        character_shape.shape(),
        1.0,
        &collisions,
    );

    let body = &bodies[box_body];
    (body.linvel(), body.angvel())
}

#[test]
fn impulse_applied_at_collider_world_point() {
    // Same physical scene, two representations:
    // - baseline: the collider's offset is baked into the body position;
    // - offset: the body sits at the origin and the collider carries the offset.
    let (baseline_linvel, baseline_angvel) = run_push(Vector::new(0.0, 3.0, 0.0), Vector::ZERO);
    let (offset_linvel, offset_angvel) = run_push(Vector::ZERO, Vector::new(0.0, 3.0, 0.0));

    // The push happens 3 units below the center of mass, so it must spin the box.
    assert!(
        baseline_angvel.length() > 1.0e-4,
        "expected a torque from the off-center push, got angvel {baseline_angvel:?}"
    );

    assert!(
        (baseline_linvel - offset_linvel).length() <= 1.0e-5 * baseline_linvel.length().max(1.0),
        "linvel mismatch: baseline = {baseline_linvel:?}, offset = {offset_linvel:?}"
    );
    assert!(
        (baseline_angvel - offset_angvel).length() <= 1.0e-5 * baseline_angvel.length(),
        "angvel mismatch: baseline = {baseline_angvel:?}, offset = {offset_angvel:?}"
    );
}
