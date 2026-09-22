//! Stress test for the FEM soft-body solver: a grid of sixty-four stiff cantilevers, each solved
//! by [`SoftBodySolver::Fem`] (one factorization per step and per body) and loaded by a heavy
//! box dropped on its free end.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.1, 30.0),
    );

    /*
     * The cantilevers, bolted at their left end, stiffer on the farther rows.
     */
    let (length, thickness) = (3.0, 0.3);
    for row in 0..8 {
        for col in 0..8 {
            let x0 = -20.0 + col as Real * 5.0;
            let z = -17.5 + row as Real * 5.0;
            let beam = SoftBodyBuilder::cuboid(
                Vector::new(x0 + length * 0.5, 2.5, z),
                Vector::new(length * 0.5, thickness * 0.5, thickness * 0.5),
                13,
                3,
                3,
            )
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e6 * (1.0 + row as Real),
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .mass(12.0)
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
                RigidBodyBuilder::dynamic().translation(Vector::new(x0 + length - 0.4, 4.0, z)),
                ColliderBuilder::cuboid(0.3, 0.3, 0.3).density(30.0),
            );
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 28.0, 34.0), Vec3::new(0.0, 1.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
