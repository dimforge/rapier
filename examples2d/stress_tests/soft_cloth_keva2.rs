//! Stress test: a big rectangle of cloth (a long thin elastic strip with self-contacts) dropped
//! flat onto a tapered Keva-style tower of planks, which it drapes over and topples.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(40.0, 0.5),
    );

    /*
     * The tower: pairs of layers, standing planks bridged by flat ones, one span narrower
     * per pair.
     */
    let (thickness, length) = (0.1, 1.0);
    let spans = 20usize;
    let pair_height = 2.0 * (length + thickness);
    let mut y = 0.0;
    for pair in 0..spans - 2 {
        let n = spans - pair;
        let x0 = -(n as Real) * length;
        for k in 0..=n {
            world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x0 + k as Real * 2.0 * length, y + length)),
                ColliderBuilder::cuboid(thickness, length),
            );
        }
        for k in 0..n {
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    x0 + (k as Real * 2.0 + 1.0) * length,
                    y + 2.0 * length + thickness,
                )),
                ColliderBuilder::cuboid(length, thickness),
            );
        }
        y += pair_height;
    }

    /*
     * The cloth, centered on the tower.
     */
    let cloth = SoftBodyBuilder::grid(Vector::new(0.0, y + 6.0), Vector::new(24.0, 0.1), 241, 2)
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
    world.insert_soft_body(cloth);

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 20.0), 12.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
