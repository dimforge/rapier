//! Stress test: a big pile of deformable objects (jelly squares of various stiffnesses,
//! pressurized blobs, floppy strips, ropes) poured into a bin with a few rigid boxes and thin
//! pins in the way. Everything collides through deformable polylines.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Bin, with thin pins across it.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(9.0, 0.5),
    );
    for x in [-9.0, 9.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 12.0)),
            ColliderBuilder::cuboid(0.5, 12.0),
        );
    }
    for i in 0..5 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-6.0 + i as Real * 3.0, 6.0)),
            ColliderBuilder::capsule_x(0.6, 0.1),
        );
    }

    /*
     * Layers of soft objects.
     */
    let mut k = 0usize;
    for layer in 0..40 {
        for i in 0..6 {
            let x = -7.0 + i as Real * 2.8 + (layer % 2) as Real * 1.0;
            let y = 9.0 + layer as Real * 2.6;
            match k % 5 {
                0 | 3 => {
                    let young = 3.0e3 * (1.0 + (k % 7) as Real * 4.0);
                    let square = SoftBodyBuilder::grid(Vector::new(x, y), Vector::splat(0.6), 4, 4)
                        .cell_model(SoftBodyCellModel::Corotational)
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
                }
                1 => {
                    let blob = SoftBodyBuilder::disk(Vector::new(x, y), 0.6, 20)
                        .softness(SpringCoefficients::new(20.0, 1.0))
                        .volume_factor(1.1)
                        .self_contacts(true)
                        .particle_mass(0.05)
                        .particle_radius(0.06)
                        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6));
                    world.insert_soft_body(blob);
                }
                2 => {
                    let strip =
                        SoftBodyBuilder::grid(Vector::new(x, y), Vector::new(1.2, 0.12), 13, 2)
                            .cell_model(SoftBodyCellModel::Corotational)
                            .material(SoftBodyMaterial {
                                young_modulus: 2.0e4,
                                poisson_ratio: 0.4,
                                elastic_damping_ratio: 0.5,
                                ..Default::default()
                            })
                            .particle_mass(0.1)
                            .particle_radius(0.06)
                            .surface_collider(ColliderBuilder::ball(0.06).friction(0.6));
                    world.insert_soft_body(strip);
                }
                _ => {
                    world.insert(
                        RigidBodyBuilder::dynamic().translation(Vector::new(x, y)),
                        ColliderBuilder::cuboid(0.4, 0.4).density(0.5),
                    );
                }
            }
            k += 1;
        }
    }

    /*
     * Ropes thrown on top.
     */
    for i in 0..3 {
        let rope = SoftBodyBuilder::rope(
            Vector::new(-6.0 + i as Real, 36.0 + i as Real),
            Vector::new(4.0 + i as Real, 37.0 + i as Real),
            30,
        )
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.03)
        .surface_collider(ColliderBuilder::ball(0.08).friction(0.6));
        world.insert_soft_body(rope);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 10.0), 20.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
