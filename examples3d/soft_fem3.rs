//! The FEM soft-body solver, side by side with the constraint solver: the same bodies twice,
//! solved by [`SoftBodySolver::Fem`] on the left (blue) and by the constraint solver on the right.
//! At four substeps the FEM cantilevers and planks hold their deflection; the constraint ones sag.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// Distance between the FEM column (at `-Z`) and the constraint column (at `+Z`).
const COLUMN: Real = 3.5;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.1, 20.0),
    );

    let solvers = [SoftBodySolver::Fem, SoftBodySolver::Constraints];

    for (column, solver) in solvers.iter().enumerate() {
        let z = if column == 0 { -COLUMN } else { COLUMN };

        /*
         * A stiff cantilever bolted to a wall: the deflection under its own weight.
         */
        let (length, thickness) = (3.0, 0.3);
        let beam = SoftBodyBuilder::cuboid(
            Vector::new(length * 0.5 - 5.0, 3.0, z),
            Vector::new(length * 0.5, thickness * 0.5, thickness * 0.5),
            13,
            3,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e6,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .mass(12.0);
        let pinned: Vec<u32> = beam
            .particle_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.x < -5.0 + 1.0e-4)
            .map(|(i, _)| i as u32)
            .collect();
        world.insert_soft_body(beam.pinned_particles(pinned).solver(*solver));

        /*
         * A plank pinned at both ends, loaded by a heavy box dropped in its middle.
         */
        let plank = SoftBodyBuilder::cuboid(
            Vector::new(1.0, 2.0, z),
            Vector::new(2.0, 0.15, 0.6),
            17,
            3,
            5,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 4.0e6,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .mass(20.0);
        let pinned: Vec<u32> = plank
            .particle_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| (p.x - 1.0).abs() > 1.9)
            .map(|(i, _)| i as u32)
            .collect();
        world.insert_soft_body(plank.pinned_particles(pinned).solver(*solver));
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 4.5, z)),
            ColliderBuilder::cuboid(0.35, 0.35, 0.35).density(30.0),
        );

        /*
         * A Neo-Hookean jelly cube dropped on the ground: the two solvers agree here.
         */
        let jelly = SoftBodyBuilder::cuboid(Vector::new(6.0, 2.0, z), Vector::splat(0.7), 5, 5, 5)
            .cell_model(SoftBodyCellModel::NeoHookean)
            .material(SoftBodyMaterial {
                young_modulus: 2.0e4,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.1);
        world.insert_soft_body(jelly.solver(*solver));
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(2.0, 8.0, 16.0), Vec3::new(0.5, 2.0, 0.0));
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
