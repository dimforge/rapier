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

use crate::dynamics::{ImpulseJointSet, MultibodyJointSet, SoftBodySet};

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
    let mut soft_body_set = SoftBodySet::new();
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
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );
    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    assert!((collider_1_position.translation - collider_2_position.translation).length() < 0.5f32);

    // Same-parent pairs are filtered out by the broad phase (issue #970), so no (empty)
    // contact pair is registered while both colliders share their parent. If one of them
    // is re-parented, the broad phase must re-generate the pair (asserted below).
    assert!(
        narrow_phase
            .contact_pair(collider_1_handle, collider_2_handle)
            .is_none_or(|pair| pair.manifolds().is_empty()),
        "No contact should be simulated between same-parent colliders."
    );
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
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(contact_pair.manifolds().len(), 1);
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
            &mut soft_body_set,
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
    let mut soft_body_set = SoftBodySet::new();
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
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
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
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should no longer exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
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
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
        1,
        "There should be a contact manifold."
    );
}

/// Removing a collider releases the solver colours of its contact pairs: every body's colour
/// mask stays the union of the colours of its live pairs.
#[test]
pub fn collider_removal_releases_solver_colors() {
    use crate::geometry::contact_pair::SOLVER_COLOR_OVERFLOW;
    use crate::pipeline::PhysicsWorld;

    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
    );
    let mut colliders = Vec::new();
    for k in 0..4 {
        let (_, co) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(
                0.0,
                0.5 + k as crate::math::Real,
                0.0,
            )),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        colliders.push(co);
    }
    for _ in 0..30 {
        world.step();
    }

    world.remove_collider(colliders[2]);
    world.step();

    let np = &world.narrow_phase;
    let mut expected = std::vec![0u128; np.body_solver_color_masks.len()];
    for pair in np.contact_graph.interactions() {
        let Some(rigid) = pair.rigid() else { continue };
        if rigid.solver_color < SOLVER_COLOR_OVERFLOW {
            for id in rigid.solver_color_bodies {
                if id != u32::MAX {
                    expected[id as usize] |= 1u128 << rigid.solver_color;
                }
            }
        }
    }
    assert_eq!(np.body_solver_color_masks, expected);
}

/// The collision pipeline never reconciles contact-force events, so the per-step force-event
/// flags of user-modified colliders must not accumulate.
#[test]
pub fn collision_pipeline_does_not_accumulate_force_event_flags() {
    use crate::dynamics::{IslandManager, RigidBodySet};
    use crate::geometry::ColliderSet;
    use crate::pipeline::CollisionPipeline;

    let mut pipeline = CollisionPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    colliders.insert(ColliderBuilder::cuboid(1.0, 1.0, 1.0));
    let body = bodies.insert(RigidBodyBuilder::dynamic());
    let moving = colliders.insert_with_parent(ColliderBuilder::ball(0.5), body, &mut bodies);

    for i in 0..100 {
        colliders[moving].set_translation(Vector::new(
            0.0,
            0.5 + i as crate::math::Real * 1.0e-3,
            0.0,
        ));
        pipeline.step(
            0.0,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &(),
            &(),
        );
    }
    assert!(narrow_phase.force_event_flagged.len() <= 1);
}

/// The recycle state's relative pose is not serialized but recomputed from the stored collider
/// poses: it must come back bit for bit.
#[cfg(feature = "serde-serialize")]
#[test]
fn restored_recycle_states_keep_their_relative_pose() {
    use crate::math::Real;
    let mut world = crate::pipeline::PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(5.0, 0.5, 5.0),
    );
    for i in 0..6 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(
                    i as Real * 0.7 - 2.0,
                    0.6 + i as Real * 1.1,
                    0.1,
                ))
                .rotation(Vector::new(0.1 * i as Real, 0.2, 0.3)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
    }
    for _ in 0..40 {
        world.step();
    }
    let bytes = bincode::serialize(&world.narrow_phase).unwrap();
    let restored: NarrowPhase = bincode::deserialize(&bytes).unwrap();
    let bits = |pose: &crate::math::Pose| {
        let (t, r) = (pose.translation, pose.rotation);
        #[cfg(feature = "dim3")]
        let r = [r.x, r.y, r.z, r.w];
        #[cfg(feature = "dim2")]
        let r = [r.re, r.im];
        (t.to_array().map(Real::to_bits), r.map(Real::to_bits))
    };
    let mut checked = 0;
    for (a, b) in world
        .narrow_phase
        .contact_graph
        .graph
        .edges
        .iter()
        .zip(restored.contact_graph.graph.edges.iter())
    {
        let states = (a.weight.rigid(), b.weight.rigid());
        if let (Some(a), Some(b)) = states {
            if let (Some(a), Some(b)) = (&a.recycle_state, &b.recycle_state) {
                assert_eq!(bits(&a.pos12), bits(&b.pos12));
                checked += 1;
            }
        }
    }
    assert!(checked > 0);
}

/// While a pair is clustered, its plain manifolds keep the warm-start tangent a declustering
/// reads back, with or without the `serde-serialize` feature.
#[test]
fn clustered_plain_manifolds_keep_their_warmstart_tangent() {
    use crate::geometry::contact_pair::{replay_tangent_basis, replay_tangent_world};
    let mut world = crate::pipeline::PhysicsWorld::new();
    // A ground of many triangles under one box: several coplanar manifolds, clustered.
    let (n, s) = (8u32, 1.0);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            vertices.push(Vector::new(i as f32 * s - 4.0, 0.0, j as f32 * s - 4.0));
        }
    }
    for i in 0..n {
        for j in 0..n {
            let a = i * (n + 1) + j;
            indices.push([a, a + 1, a + n + 1]);
            indices.push([a + 1, a + n + 2, a + n + 1]);
        }
    }
    world.insert_collider(ColliderBuilder::trimesh(vertices, indices).unwrap(), None);
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.3, 0.5, 0.2))
            .can_sleep(false),
        ColliderBuilder::cuboid(1.5, 0.5, 1.5),
    );
    // Unclustered first, so the plain manifolds hold the solver's warm-start data, then
    // clustered while the box tilts (the stored tangents no longer match the normals).
    world.integration_parameters.contact_clustering = false;
    for _ in 0..30 {
        world.step();
    }
    world.integration_parameters.contact_clustering = true;
    // Full updates every step, so clustering applies right away.
    world.integration_parameters.contact_recycling = false;
    world.bodies[body].set_angvel(Vector::new(0.05, 0.0, 0.03), true);
    for _ in 0..5 {
        world.step();
    }
    let mut checked = 0;
    for pair in world.narrow_phase.contact_pairs() {
        let Some(rigid) = pair.rigid() else { continue };
        if rigid.solver_clusters.is_empty() {
            continue;
        }
        for manifold in &rigid.manifolds {
            let (t1, t2) = replay_tangent_basis(&manifold.data.normal);
            for pt in &manifold.points {
                assert_eq!(pt.data.warmstart_tangent_world, replay_tangent_world(&t1, &t2, &pt.data));
                checked += 1;
            }
        }
    }
    assert!(checked > 0, "no clustered pair in the scene");
}
