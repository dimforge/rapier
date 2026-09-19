//! Stress test: hundreds of elastic jelly squares (corotational and Neo-Hookean cells, assorted
//! stiffnesses) poured into a bin, colliding through their deformable boundaries. Mostly
//! soft-vs-soft contacts against elastic cells under the weight of the pile.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(16.0, 0.5),
    );
    for x in [-16.0, 16.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 30.0)),
            ColliderBuilder::cuboid(0.5, 30.0),
        );
    }

    /*
     * Layers of jelly squares: every fifth stiffness step, every third one Neo-Hookean.
     */
    let mut k = 0usize;
    for layer in 0..40 {
        for i in 0..12 {
            let x = -14.0 + i as Real * 2.5 + (layer % 2) as Real * 0.8;
            let y = 2.0 + layer as Real * 2.4;
            let young = 2.0e3 * (1.0 + (k % 5) as Real * 3.0);
            let model = if k.is_multiple_of(3) {
                SoftBodyCellModel::NeoHookean
            } else {
                SoftBodyCellModel::Corotational
            };
            let square = SoftBodyBuilder::grid(Vector::new(x, y), Vector::splat(0.6), 5, 5)
                .cell_model(model)
                .material(SoftBodyMaterial {
                    young_modulus: young,
                    poisson_ratio: 0.4,
                    elastic_damping_ratio: 0.5,
                    ..Default::default()
                })
                .particle_mass(0.1)
                .particle_radius(0.08)
                .surface_collider(ColliderBuilder::ball(0.08).friction(0.7));
            world.insert_soft_body(square);
            k += 1;
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
