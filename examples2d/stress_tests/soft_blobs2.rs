//! Stress test: more than a thousand pressurized blobs poured into a tall container, each with
//! self-contacts so a hard squeeze cannot push its loop through itself, and a long floppy strip
//! dropped onto the pile. Soft-vs-soft contacts through deformable polylines, under load.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Container.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(20.0, 0.5),
    );
    for x in [-20.0, 20.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 30.0)),
            ColliderBuilder::cuboid(0.5, 30.0),
        );
    }

    /*
     * Blobs.
     */
    for j in 0..60 {
        for i in 0..19 {
            let radius = 0.45 + 0.1 * ((i + j) % 3) as Real;
            let x = -19.0 + i as Real * 2.0 + (j % 2) as Real * 0.5;
            let y = 3.0 + j as Real * 2.0;
            let blob = SoftBodyBuilder::disk(Vector::new(x, y), radius, 20)
                .softness(SpringCoefficients::new(60.0, 3.0))
                .volume_factor(1.05)
                .self_contacts(true)
                .particle_mass(0.05);
            world.insert_soft_body(blob);
        }
    }

    /*
     * A long floppy strip with self-contacts, dropped on the pile.
     */
    let strip = SoftBodyBuilder::grid(Vector::new(0.0, 16.0), Vector::new(3.0, 0.15), 40, 3)
        .softness(SpringCoefficients::new(120.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.1);
    world.insert_soft_body(strip);

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 6.0), 30.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
