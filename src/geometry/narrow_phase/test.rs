//! Narrow-phase regression tests (collider re-parenting interactions).

#[allow(unused_imports)]
use crate::alloc_prelude::*;
use crate::math::Vector;
use crate::prelude::{
    CCDSolver, ColliderBuilder, DefaultBroadPhase, IntegrationParameters, PhysicsPipeline,
    RigidBodyBuilder,
};
use std::println;

use super::*;

use crate::dynamics::{ImpulseJointSet, MultibodyJointSet};

/// Test for https://github.com/dimforge/rapier/issues/734.
#[test]
pub fn collider_set_parent_depenetration() {
    // This tests the scenario:
    // 1. Body A has two colliders attached (and overlapping), Body B has none.
    // 2. One of the colliders from Body A gets re-parented to Body B.
    //    -> Collision is properly detected between the colliders of A and B.
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::ball(0.5);

    /* Create body 1, which will contain both colliders at first. */
    let rigid_body_1 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_1_handle = rigid_body_set.insert(rigid_body_1);

    /* Create collider 1. Parent it to rigid body 1. */
    let collider_1_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create collider 2. Parent it to rigid body 1. */
    let collider_2_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create body 2. No attached colliders yet. */
    let rigid_body_2 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_2_handle = rigid_body_set.insert(rigid_body_2);

    /* Create other structures necessary for the simulation. */
    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );
    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    assert!((collider_1_position.translation - collider_2_position.translation).length() < 0.5f32);

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(contact_pair.manifolds.len(), 0);
    assert!(
        narrow_phase
            .intersection_pair(collider_1_handle, collider_2_handle)
            .is_none(),
        "Interaction pair is for sensors"
    );
    /* Parent collider 2 to body 2. */
    collider_set.set_parent(collider_2_handle, Some(body_2_handle), &mut rigid_body_set);

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(contact_pair.manifolds.len(), 1);
    assert!(
        narrow_phase
            .intersection_pair(collider_1_handle, collider_2_handle)
            .is_none(),
        "Interaction pair is for sensors"
    );

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        physics_pipeline.step(
            gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
        let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
        println!("collider 1 position: {}", collider_1_position.translation);
        println!("collider 2 position: {}", collider_2_position.translation);
    }

    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    println!("collider 2 position: {}", collider_2_position.translation);
    assert!(
        (collider_1_position.translation - collider_2_position.translation).length() >= 0.5f32,
        "colliders should no longer be penetrating."
    );
}

/// Test for https://github.com/dimforge/rapier/issues/734.
#[test]
pub fn collider_set_parent_no_self_intersection() {
    // This tests the scenario:
    // 1. Body A and Body B each have one collider attached.
    //    -> There should be a collision detected between A and B.
    // 2. The collider from Body B gets attached to Body A.
    //    -> There should no longer be any collision between A and B.
    // 3. Re-parent one of the collider from Body A to Body B again.
    //    -> There should a collision again.
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::ball(0.5);

    /* Create body 1, which will contain collider 1. */
    let rigid_body_1 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_1_handle = rigid_body_set.insert(rigid_body_1);

    /* Create collider 1. Parent it to rigid body 1. */
    let collider_1_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create body 2, which will contain collider 2 at first. */
    let rigid_body_2 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_2_handle = rigid_body_set.insert(rigid_body_2);

    /* Create collider 2. Parent it to rigid body 2. */
    let collider_2_handle =
        collider_set.insert_with_parent(collider.build(), body_2_handle, &mut rigid_body_set);

    /* Create other structures necessary for the simulation. */
    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds.len(),
        1,
        "There should be a contact manifold."
    );

    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    assert!((collider_1_position.translation - collider_2_position.translation).length() < 0.5f32);

    /* Parent collider 2 to body 1. */
    collider_set.set_parent(collider_2_handle, Some(body_1_handle), &mut rigid_body_set);
    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should no longer exist.");
    assert_eq!(
        contact_pair.manifolds.len(),
        0,
        "Colliders with same parent should not be in contact together."
    );

    /* Parent collider 2 back to body 1. */
    collider_set.set_parent(collider_2_handle, Some(body_2_handle), &mut rigid_body_set);
    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds.len(),
        1,
        "There should be a contact manifold."
    );
}
