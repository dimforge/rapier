//! Stress test: a rain of soft ropes (structural and bending edges, colliding through their
//! particles) poured over a few rods into a bin, alternating directions layer by layer so they
//! weave into a heap.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin, with rods across it.
     */
    let half = 6.0;
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half + 0.5, 0.5, half + 0.5),
    );
    // Half-height of the walls: enough for the settled pile, not for the drop column.
    let wall_height = 3.0;
    for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let (hx, hz) = if dx != 0.0 {
            (0.25, half + 0.5)
        } else {
            (half + 0.5, 0.25)
        };
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(
                dx * (half + 0.25),
                wall_height,
                dz * (half + 0.25),
            )),
            ColliderBuilder::cuboid(hx, wall_height, hz),
        );
    }
    for k in 0..4 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, 4.0, -4.5 + k as Real * 3.0)),
            ColliderBuilder::capsule_x(5.0, 0.12),
        );
    }

    /*
     * Layers of ropes, along X on the even layers and along Z on the odd ones.
     */
    let length = 3.5;
    for layer in 0..10 {
        for i in 0..6 {
            for j in 0..3 {
                let across = -5.0 + i as Real * 2.0 + (layer % 2) as Real * 0.5;
                let along = -5.7 + j as Real * 3.8;
                let y = 8.0 + layer as Real * 1.5;
                let (start, end) = if layer % 2 == 0 {
                    (
                        Vector::new(along, y, across),
                        Vector::new(along + length, y + 0.3, across + 0.4),
                    )
                } else {
                    (
                        Vector::new(across, y, along),
                        Vector::new(across + 0.4, y + 0.3, along + length),
                    )
                };
                let rope = SoftBodyBuilder::rope(start, end, 40)
                    .softness(SpringCoefficients::new(30.0, 1.0))
                    .particle_mass(0.03)
                    .surface_collider(ColliderBuilder::ball(0.08).friction(0.6));
                world.insert_soft_body(rope);
            }
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    // Translucent bin, to see the pile inside.
    let bin: Vec<_> = world
        .bodies
        .iter()
        .filter(|(_, body)| body.is_fixed())
        .map(|(handle, _)| handle)
        .collect();
    for handle in bin {
        viewer.set_body_color(handle, [0.6, 0.7, 1.0, 0.3].into(), false);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(16.0, 14.0, 16.0), Vec3::new(0.0, 4.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
