//! Regression tests for stale `ContactRef`s left in the persistent solver
//! contact graph (both found through crashing demos):
//!
//! 1. `remove_collider` swap-removes contact-graph nodes/edges and defers the
//!    solver-graph repair to a full rebuild — but `remove_pair` (broad-phase
//!    separations handled later in the same step) used to run its incremental
//!    surgical cleanup unconditionally, dereferencing the now-garbage
//!    `graph_pos` back-references (fountain3 crash: index OOB in
//!    `remove_and_fixup`).
//!
//! 2. The contact-update filter early-outs (`pair.clear()` on same-parent,
//!    joint/group filters, soft-CCD swept-AABB miss) destroy a pair's
//!    manifolds — and with them the `graph_pos` back-refs — without pulling
//!    the pair's entries out of the graph (debug_thin_cube_on_mesh3 crash:
//!    "stale ContactRef manifold ordinal" at solve time).
//!
//! With debug assertions on (tests always are), the narrow phase's shadow
//! validator asserts graph == from-scratch selection every step, so these
//! scenarios fail fast when either hole reopens.
#![cfg(feature = "dim3")]

use rapier3d::prelude::*;

/// Fountain-style body churn: spawn a body every step, remove the outermost
/// ones once over the cap. Exercises collider removal (node swap-removes) and
/// broad-phase pair removal in the same step, plus sleep/wake transitions.
#[test]
fn body_churn_keeps_solver_graph_exact() {
    const MAX_BODIES: usize = 120;

    let mut world = PhysicsWorld::new();

    let rad = 0.5;
    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -2.1, 0.0));
    let collider = ColliderBuilder::cuboid(40.0, 2.1, 40.0);
    world.insert(rigid_body, collider);

    for step_id in 1..1500usize {
        world.step();

        let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0));
        let handle = world.bodies.insert(rigid_body);
        let collider = match step_id % 3 {
            0 => ColliderBuilder::round_cylinder(rad, rad, rad / 10.0),
            1 => ColliderBuilder::cone(rad, rad),
            _ => ColliderBuilder::cuboid(rad, rad, rad),
        };
        world
            .colliders
            .insert_with_parent(collider, handle, &mut world.bodies);

        if world.bodies.len() > MAX_BODIES {
            let mut to_remove: Vec<(RigidBodyHandle, Vector)> = world
                .bodies
                .iter()
                .filter(|e| e.1.is_dynamic())
                .map(|e| (e.0, e.1.translation()))
                .collect();
            to_remove.sort_by(|a, b| {
                (a.1.x.abs() + a.1.z.abs())
                    .partial_cmp(&(b.1.x.abs() + b.1.z.abs()))
                    .unwrap()
                    .reverse()
            });

            let num_to_remove = to_remove.len().saturating_sub(MAX_BODIES);
            for (handle, _) in &to_remove[..num_to_remove] {
                world.bodies.remove(
                    *handle,
                    &mut world.islands,
                    &mut world.colliders,
                    &mut world.impulse_joints,
                    &mut world.multibody_joints,
                    true,
                );
            }
        }
    }
}

/// Thin cuboid slammed into a heightfield with soft-CCD: on the bounce, the
/// swept-AABB early-out clears the (composite, solver-active) pair's
/// manifolds. Exercises the filter-clear paths of the contact update.
#[test]
fn soft_ccd_filter_clear_keeps_solver_graph_exact() {
    let mut world = PhysicsWorld::new();

    let heights = Array2::repeat(2, 2, 0.0);
    let collider = ColliderBuilder::heightfield_with_flags(
        heights,
        Vector::new(50.0, 1.0, 50.0),
        HeightFieldFlags::FIX_INTERNAL_EDGES,
    );
    world.insert_collider(collider, None);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 5.0, 0.0))
        .rotation(Vector::new(0.5, 0.0, 0.5))
        .linvel(Vector::new(0.0, -100.0, 0.0))
        .soft_ccd_prediction(10.0);
    let collider = ColliderBuilder::cuboid(5.0, 0.015, 5.0);
    let (handle, _) = world.insert(rigid_body, collider);

    for _ in 0..2000 {
        world.step();
    }

    // The cube must have come to rest on the heightfield, not tunneled through.
    assert!(world.bodies[handle].translation().y > -0.5);
}
