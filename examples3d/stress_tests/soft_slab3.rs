//! Stress test: one large elastic slab (a corotational tetrahedral block of a few thousand
//! particles) laid over a bed of rigid balls and crates in a bin, showered with hundreds more. One big body's
//! worth of cells, plus many rigid-vs-soft contacts on a single deformable surface.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin.
     */
    let half = 9.0;
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half + 0.5, 0.5, half + 0.5),
    );
    // Half-height of the walls: enough for the settled pile, not for the drop column.
    let wall_height = 3.5;
    for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let (hx, hz) = if dx != 0.0 {
            (0.25, half + 0.5)
        } else {
            (half + 0.5, 0.25)
        };
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(
                dx * (half + 0.25),
                wall_height,
                dz * (half + 0.25),
            )),
            ColliderBuilder::cuboid(hx, wall_height, hz),
        );
    }

    /*
     * The bed of rigid objects the slab lands on.
     */
    for i in 0..5 {
        for j in 0..5 {
            let x = -6.0 + i as Real * 3.0;
            let z = -6.0 + j as Real * 3.0;
            let body = RigidBodyBuilder::dynamic().translation(Vector::new(x, 0.5, z));
            if (i + j) % 2 == 0 {
                world.insert(body, ColliderBuilder::ball(0.5));
            } else {
                world.insert(body, ColliderBuilder::cuboid(0.5, 0.5, 0.5));
            }
        }
    }

    /*
     * The slab.
     */
    let slab = SoftBodyBuilder::cuboid(
        Vector::new(0.0, 2.2, 0.0),
        Vector::new(8.0, 1.0, 8.0),
        25,
        4,
        25,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(SoftBodyMaterial {
        young_modulus: 4.0e4,
        poisson_ratio: 0.4,
        elastic_damping_ratio: 0.5,
        ..Default::default()
    })
    .particle_mass(0.2)
    .surface_collider(ColliderBuilder::ball(0.3).friction(0.7));
    world.insert_soft_body(slab);

    /*
     * The shower: three waves of alternating balls and boxes.
     */
    for wave in 0..3 {
        for i in 0..16 {
            for j in 0..16 {
                let x = -7.5 + i as Real + (wave % 2) as Real * 0.5;
                let z = -7.5 + j as Real + (wave % 3) as Real * 0.3;
                let y = 5.5 + wave as Real * 3.0 + ((i + j) % 3) as Real * 0.7;
                let body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
                if (i + j + wave) % 2 == 0 {
                    world.insert(body, ColliderBuilder::ball(0.35));
                } else {
                    world.insert(body, ColliderBuilder::cuboid(0.35, 0.35, 0.35));
                }
            }
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    // Translucent bin, to see the pile inside.
    let bin: Vec<_> = world
        .bodies
        .iter()
        .filter(|(_, body)| body.is_fixed())
        .map(|(handle, _)| handle)
        .collect();
    for handle in bin {
        viewer.set_body_color(handle, [0.6, 0.7, 1.0, 0.3].into(), false);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(20.0, 14.0, 20.0), Vec3::new(0.0, 2.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
