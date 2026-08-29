//! The FEM soft-body solver (2D) next to the constraint solver: the same bodies twice, solved by
//! [`SoftBodySolver::Fem`] on the lower row and by the constraint solver on the upper one.
//! At four substeps the FEM cantilever and plank hold their static deflection; the others sag.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// Vertical distance between the FEM row and the constraint-solver row.
const ROW: Real = 6.0;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );

    let solvers = [SoftBodySolver::Fem, SoftBodySolver::Constraints];

    for (row, solver) in solvers.iter().enumerate() {
        let y = 2.0 + row as Real * ROW;

        /*
         * A stiff cantilever bolted to a wall.
         */
        let (length, thickness) = (4.0, 0.4);
        let beam = SoftBodyBuilder::grid(
            Vector::new(length * 0.5 - 8.0, y),
            Vector::new(length * 0.5, thickness * 0.5),
            17,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e6,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .mass(8.0);
        let pinned: Vec<u32> = beam
            .particle_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.x < -8.0 + 1.0e-4)
            .map(|(i, _)| i as u32)
            .collect();
        world.insert_soft_body(beam.pinned_particles(pinned).solver(*solver));

        /*
         * A plank pinned at both ends, loaded by a heavy box dropped in its middle.
         */
        let plank = SoftBodyBuilder::grid(Vector::new(2.0, y), Vector::new(3.0, 0.25), 21, 3)
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
            .filter(|(_, p)| (p.x - 2.0).abs() > 2.9)
            .map(|(i, _)| i as u32)
            .collect();
        world.insert_soft_body(plank.pinned_particles(pinned).solver(*solver));
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(2.0, y + 2.5)),
            ColliderBuilder::cuboid(0.5, 0.5).density(20.0),
        );

        /*
         * A Neo-Hookean jelly square dropped on the plank.
         */
        let jelly = SoftBodyBuilder::grid(Vector::new(9.0, y + 1.0), Vector::splat(0.8), 5, 5)
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
    viewer.look_at(Vec2::new(1.0, 5.0), 25.0);
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
