//! Stress test: one large elastic slab (a corotational grid of a few thousand particles) laid
//! over a bed of rigid balls and crates, showered with hundreds more. One big body's worth of cells, plus
//! many rigid-vs-soft contacts on a single deformable surface.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and side walls.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(14.0, 0.5),
    );
    for x in [-14.0, 14.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 30.0)),
            ColliderBuilder::cuboid(0.5, 30.0),
        );
    }

    /*
     * The bed of rigid objects the slab lands on.
     */
    for i in 0..12 {
        let x = -11.0 + i as Real * 2.0;
        let body = RigidBodyBuilder::dynamic().translation(Vector::new(x, 0.5));
        if i % 2 == 0 {
            world.insert(body, ColliderBuilder::ball(0.5));
        } else {
            world.insert(body, ColliderBuilder::cuboid(0.5, 0.5));
        }
    }

    /*
     * The slab.
     */
    let slab = SoftBodyBuilder::grid(Vector::new(0.0, 3.0), Vector::new(12.0, 1.5), 161, 21)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 4.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.2)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.7));
    world.insert_soft_body(slab);

    /*
     * The shower: alternating balls and boxes.
     */
    for j in 0..40 {
        for i in 0..24 {
            let x = -11.5 + i as Real + (j % 2) as Real * 0.5;
            let y = 7.5 + j as Real * 1.2;
            let body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
            if (i + j) % 2 == 0 {
                world.insert(body, ColliderBuilder::ball(0.3));
            } else {
                world.insert(body, ColliderBuilder::cuboid(0.3, 0.3));
            }
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
