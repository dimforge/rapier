//! Regression test for the 2D heightfield stress demo segfault.
//!
//! Heightfield/composite pairs rebuild their manifold list in BVH-traversal
//! order every full update, so a manifold's ordinal is unstable and dropped
//! subshapes lose their stored `graph_pos`. The persistent solver contact
//! graph must not keep stale `ContactRef`s pointing at those ordinals — a
//! stale one used to index out of the shrunken manifold list at solve time and
//! segfault (only debug builds caught it, via the shadow validator). Stepping
//! a busy heightfield scene exercises the reorder/drop churn.
#![cfg(feature = "dim2")]

use rapier2d::prelude::*;

#[test]
fn heightfield_stress_solver_graph_consistency() {
    let mut world = PhysicsWorld::new();

    let ground_size = Vec2::new(50.0, 1.0);
    let nsubdivs = 2000;

    let heights = (0..nsubdivs + 1)
        .map(|i| {
            if i == 0 || i == nsubdivs {
                80.0
            } else {
                (i as f32 * ground_size.x / (nsubdivs as f32)).cos() * 2.0
            }
        })
        .collect();

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::heightfield(heights, ground_size);
    let handle = world.bodies.insert(rigid_body);
    world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);

    let num = 26;
    let rad = 0.5;
    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 3.0;
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vec2::new(x, y));
            let handle = world.bodies.insert(rigid_body);
            if j % 2 == 0 {
                world.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(rad, rad),
                    handle,
                    &mut world.bodies,
                );
            } else {
                world.colliders.insert_with_parent(
                    ColliderBuilder::ball(rad),
                    handle,
                    &mut world.bodies,
                );
            }
        }
    }

    for _ in 0..300 {
        world.step();
    }
}
