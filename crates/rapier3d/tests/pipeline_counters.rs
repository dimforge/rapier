//! `PhysicsPipeline::counters`: the contact pair, contact and constraint counts are filled by
//! the step, and the CCD substep count is zero unless the CCD actually ran.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

#[test]
fn contact_and_constraint_counters() {
    let mut world = world_with_ground();
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    let (cube, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(5.0, 0.5, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    // A joint between the two resting bodies.
    world.insert_impulse_joint(
        ball,
        cube,
        SphericalJointBuilder::new()
            .local_anchor1(Vector::new(2.5, 0.0, 0.0))
            .local_anchor2(Vector::new(-2.5, 0.0, 0.0)),
    );

    for _ in 0..5 {
        world.step();
        let counters = &world.physics_pipeline.counters;
        // The ball-ground and cube-ground pairs.
        assert_eq!(counters.cd.ncontact_pairs, 2);
        // One manifold per pair plus the joint.
        assert_eq!(counters.solver.nconstraints, 3);
        // One contact point for the ball, four for the cube's face.
        assert_eq!(counters.solver.ncontacts, 5);
        // No CCD-enabled body.
        assert_eq!(counters.ccd.num_substeps, 0);
    }

    // Once the scene sleeps, nothing is solved anymore (the pairs are still tracked).
    for _ in 0..300 {
        world.step();
    }
    assert!(world.bodies[ball].is_sleeping());
    let counters = &world.physics_pipeline.counters;
    assert_eq!(counters.cd.ncontact_pairs, 2);
    assert_eq!(counters.solver.nconstraints, 0);
    assert_eq!(counters.solver.ncontacts, 0);
}

#[test]
fn ccd_substeps_counter() {
    let mut world = world_with_ground();
    world.integration_parameters.max_ccd_substeps = 4;
    world.gravity = Vector::ZERO;

    // A slow CCD-enabled body: the CCD never needs to act.
    let (slow, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0, 0.0))
            .ccd_enabled(true),
        ColliderBuilder::ball(0.5),
    );
    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 0);

    // A fast one heading to the ground: the CCD splits the step.
    world.bodies[slow].set_linvel(Vector::new(0.0, -500.0, 0.0), true);
    world.step();
    assert!(world.physics_pipeline.counters.ccd.num_substeps >= 1);
    assert!(world.bodies[slow].translation().y > -0.5);
}
