//! Stress test: hundreds of elastic jelly cubes (corotational and Neo-Hookean tetrahedra,
//! assorted stiffnesses) poured into a bin, colliding through their deformable surfaces. Mostly
//! soft-vs-soft contacts against elastic cells under the weight of the pile.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin.
     */
    let half = 6.0;
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
     * Layers of jelly cubes: every fifth stiffness step, every third one Neo-Hookean.
     */
    let mut k = 0usize;
    for layer in 0..8 {
        for i in 0..5 {
            for j in 0..5 {
                let x = -4.8 + i as Real * 2.4 + (layer % 2) as Real * 0.6;
                let z = -4.8 + j as Real * 2.4 + (layer % 3) as Real * 0.4;
                let y = 2.0 + layer as Real * 2.2;
                let young = 2.0e3 * (1.0 + (k % 5) as Real * 3.0);
                let model = if k.is_multiple_of(3) {
                    SoftBodyCellModel::NeoHookean
                } else {
                    SoftBodyCellModel::Corotational
                };
                let cube =
                    SoftBodyBuilder::cuboid(Vector::new(x, y, z), Vector::splat(0.55), 4, 4, 4)
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
                world.insert_soft_body(cube);
                k += 1;
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
    viewer.look_at(Vec3::new(18.0, 16.0, 18.0), Vec3::new(0.0, 5.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
