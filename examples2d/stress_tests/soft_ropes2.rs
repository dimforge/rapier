//! Stress test: a rain of soft ropes (structural and bending edges, colliding as deformable
//! polylines) poured over a row of pins into a bin, where they tangle into a heap.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin, with pins across it.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(10.0, 0.5),
    );
    for x in [-10.0, 10.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 5.0)),
            ColliderBuilder::cuboid(0.5, 10.0),
        );
    }
    for i in 0..5 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-6.0 + i as Real * 3.0, 5.0)),
            ColliderBuilder::capsule_x(0.6, 0.1),
        );
    }

    /*
     * Layers of ropes, slightly tilted so they slide off each other.
     */
    for layer in 0..80 {
        for i in 0..4 {
            let x0 = -8.6 + i as Real * 4.4 + (layer % 2) as Real * 0.6;
            let y = 8.0 + layer as Real * 1.2;
            let rope =
                SoftBodyBuilder::rope(Vector::new(x0, y), Vector::new(x0 + 4.0, y + 0.4), 40)
                    .particle_mass(0.03)
                    .surface_collider(ColliderBuilder::ball(0.4).friction(0.6));
            world.insert_soft_body(rope);
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 8.0), 25.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
