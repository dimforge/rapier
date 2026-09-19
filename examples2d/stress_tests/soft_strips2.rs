//! Stress test: long floppy elastic strips with self-contacts dropped through a lattice of pegs
//! into a bin, where they fold onto themselves and onto each other.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin, with a lattice of pegs.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(10.0, 0.5),
    );
    for x in [-10.0, 10.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 30.0)),
            ColliderBuilder::cuboid(0.5, 30.0),
        );
    }
    for row in 0..4 {
        for i in 0..6 {
            let x = -7.5 + i as Real * 3.0 + (row % 2) as Real * 1.5;
            world.insert(
                RigidBodyBuilder::fixed().translation(Vector::new(x, 4.0 + row as Real * 3.0)),
                ColliderBuilder::ball(0.25),
            );
        }
    }

    /*
     * Layers of strips.
     */
    for layer in 0..30 {
        for i in 0..5 {
            let x = -7.8 + i as Real * 3.7 + (layer % 2) as Real * 0.7;
            let y = 17.0 + layer as Real * 1.5;
            let strip = SoftBodyBuilder::grid(Vector::new(x, y), Vector::new(1.6, 0.1), 33, 2)
                .cell_model(SoftBodyCellModel::Corotational)
                .material(SoftBodyMaterial {
                    young_modulus: 2.0e4,
                    poisson_ratio: 0.4,
                    elastic_damping_ratio: 0.5,
                    ..Default::default()
                })
                .self_contacts(true)
                .particle_mass(0.05)
                .particle_radius(0.06)
                .surface_collider(ColliderBuilder::ball(0.06).friction(0.6));
            world.insert_soft_body(strip);
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
