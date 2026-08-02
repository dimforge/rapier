//! Regression test for the 3D heightfield stress scene (composite pairs +
//! contact clustering) — the 3D analogue of the 2D heightfield segfault. See
//! the 2D test for the failure mechanism; the clustered solver-manifold list
//! is the extra wrinkle exercised here.
#![cfg(feature = "dim3")]

use rapier3d::prelude::*;

#[test]
fn heightfield_stress_solver_graph_consistency_3d() {
    let mut world = PhysicsWorld::new();

    let ground_size = Vec3::new(50.0, 1.0, 50.0);
    let nsubdivs = 50;

    let heights = Array2::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        if i == 0 || i == nsubdivs || j == 0 || j == nsubdivs {
            8.0
        } else {
            let x = i as f32 * ground_size.x / (nsubdivs as f32);
            let z = j as f32 * ground_size.z / (nsubdivs as f32);
            (x.cos() + z.sin()) * 1.5
        }
    });

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::heightfield(heights, ground_size);
    let handle = world.bodies.insert(rigid_body);
    world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);

    let num = 8;
    let rad = 0.5;
    let shift = rad * 2.5;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    for i in 0..num {
        for j in 0usize..num * 4 {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz;
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vec3::new(x, y, z));
                let handle = world.bodies.insert(rigid_body);
                let collider = if (i + j + k) % 2 == 0 {
                    ColliderBuilder::cuboid(rad, rad, rad)
                } else {
                    ColliderBuilder::ball(rad)
                };
                world
                    .colliders
                    .insert_with_parent(collider, handle, &mut world.bodies);
            }
        }
    }

    for _ in 0..200 {
        world.step();
    }
}
