//! Regression test for #772: a 2D joint allowing 1 relative translation + free
//! relative rotation (Godot's "groove joint"). This is `PinSlotJoint`.

use rapier2d::prelude::*;

#[test]
fn pin_slot_joint_allows_one_translation_and_free_rotation() {
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
    let gravity = Vector::new(0.0, -9.81);

    let anchor = bodies.insert(RigidBodyBuilder::fixed());
    let slider = bodies.insert(
        RigidBodyBuilder::dynamic()
            .linvel(Vector::new(1.0, 0.0))
            .angvel(2.0)
            .can_sleep(false),
    );
    colliders.insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1), slider, &mut bodies);

    // Slot along the X axis: X translation free, Y translation locked, rotation free.
    let joint = PinSlotJointBuilder::new(Vector::new(1.0, 0.0));
    impulse_joints.insert(anchor, slider, joint, true);

    for _ in 0..200 {
        pipeline.step(
            gravity,
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

    let slider = &bodies[slider];
    // The constrained perpendicular translation must hold despite gravity.
    assert!(
        slider.translation().y.abs() < 1.0e-3,
        "perpendicular translation drifted: {}",
        slider.translation().y
    );
    // The free axis must have translated.
    assert!(
        slider.translation().x > 0.5,
        "no translation along the slot axis: {}",
        slider.translation().x
    );
    // The rotation must be free: with no torque applied, the initial angular
    // velocity is preserved instead of being cancelled by the joint.
    assert!(
        (slider.angvel() - 2.0).abs() < 0.1,
        "angular velocity was affected by the joint: {}",
        slider.angvel()
    );
    // The body did rotate away from its initial orientation.
    assert!(slider.rotation().angle().abs() > 1.0e-2);
}
