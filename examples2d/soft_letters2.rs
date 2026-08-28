//! The "rapier" letters of the trimesh demo as soft bodies: each glyph outline is filled with
//! cells by `SoftBodyBuilder::volumetric` and dropped into a bin, one row per stiffness, from
//! jelly to nearly rigid. Every letter collides through its deformable polyline surface.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Bin (same as the trimesh demo).
     */
    let ground_size = 25.0;
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(ground_size, 1.2),
    );
    for x in [-ground_size, ground_size] {
        world.insert(
            RigidBodyBuilder::fixed()
                .rotation(std::f32::consts::FRAC_PI_2)
                .translation(Vector::new(x, ground_size)),
            ColliderBuilder::cuboid(ground_size, 1.2),
        );
    }

    /*
     * Soft letters: one row per Young's modulus.
     */
    let cell_size = 2.4;
    let letters: Vec<_> = crate::utils::svg::rapier_logo()
        .iter()
        .filter_map(|(vtx, idx)| {
            let outline = crate::utils::svg::outline(idx);
            let letter = SoftBodyBuilder::volumetric(vtx, &outline, cell_size)?;
            Some((letter.positions, letter.cells))
        })
        .collect();
    let stiffnesses = [1.0e3, 5.0e3, 1.0e4, 5.0e4, 1.0e5, 5.0e5, 1.0e6, 5.0e6];
    for (row, young) in stiffnesses.into_iter().rev().enumerate() {
        for (ith, (positions, cells)) in letters.iter().enumerate() {
            let offset = Vector::new(ith as Real * 8.0 - 22.0, 12.0 + row as Real * 11.0);
            let letter = SoftBodyBuilder::new(positions.iter().map(|p| *p + offset).collect())
                .cells(cells.clone())
                .cell_model(SoftBodyCellModel::Corotational)
                .material(SoftBodyMaterial {
                    young_modulus: young,
                    poisson_ratio: 0.4,
                    elastic_damping_ratio: 0.5,
                    // The stiff letters sway on their feet (solver compliance, undamped by
                    // the material constraints): damp their deformation, the more the stiffer.
                    deformation_damping: (young / 4.0e4).min(50.0),
                    ..Default::default()
                })
                .particle_mass(0.1)
                .particle_radius(0.15)
                .self_contacts(true);
            world.insert_soft_body(letter);
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 20.0), 17.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
