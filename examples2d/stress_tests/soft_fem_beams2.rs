//! Stress test for the FEM soft-body solver: a grid of fifty stiff cantilevers, each solved by
//! [`SoftBodySolver::Fem`] (one factorization per step and per body) and loaded by a heavy box
//! dropped on its free end.

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
     * The cantilevers, bolted at their left end, stiffer on the upper rows.
     */
    let (length, thickness) = (4.0, 0.4);
    for row in 0..5 {
        for col in 0..10 {
            let x0 = -30.0 + col as Real * 6.0;
            let y = 3.0 + row as Real * 5.0;
            let beam = SoftBodyBuilder::grid(
                Vector::new(x0 + length * 0.5, y),
                Vector::new(length * 0.5, thickness * 0.5),
                33,
                5,
            )
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e6 * (1.0 + row as Real),
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .mass(8.0)
            .solver(SoftBodySolver::Fem);
            let pinned: Vec<u32> = beam
                .particle_positions()
                .iter()
                .enumerate()
                .filter(|(_, p)| p.x < x0 + 1.0e-4)
                .map(|(i, _)| i as u32)
                .collect();
            world.insert_soft_body(beam.pinned_particles(pinned));

            /*
             * The load, dropped on the free end.
             */
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(x0 + length - 0.5, y + 2.0)),
                ColliderBuilder::cuboid(0.4, 0.4).density(20.0),
            );
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 12.0), 15.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
