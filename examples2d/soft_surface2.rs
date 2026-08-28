//! Deformable polyline collisions: soft bodies colliding through their boundary polyline (a
//! hammock catching small balls, a pinned bridge, stacked jelly squares, a self-colliding strip).

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and walls.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(22.0, 0.5),
    );
    for x in [-22.0, 22.0] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 2.0)),
            ColliderBuilder::cuboid(0.5, 3.0),
        );
    }

    /*
     * A hammock with a coarse particle spacing (0.3) under a rain of balls smaller than that
     * spacing: they rest on the segments instead of falling between the particles.
     */
    {
        let x0 = -12.0;
        let hammock =
            SoftBodyBuilder::grid(Vector::new(x0 + 3.0, 5.0), Vector::new(3.0, 0.15), 21, 2)
                .cell_model(SoftBodyCellModel::Volume)
                .pinned_particles([0, 1, 40, 41])
                .softness(SpringCoefficients::new(30.0, 1.0))
                .particle_mass(0.05)
                .particle_radius(0.06)
                .surface_collider(ColliderBuilder::ball(0.06).friction(0.6));
        world.insert_soft_body(hammock);

        for i in 0..12 {
            for h in 0..4 {
                let pos = Vector::new(
                    x0 + 0.8 + i as Real * 0.4 + (h % 2) as Real * 0.15,
                    7.0 + h as Real * 0.5,
                );
                world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::ball(0.08).density(2.0),
                );
            }
        }
    }

    /*
     * A soft bridge loaded with thin pins standing on their tips: with surface collisions the
     * pins push on the segments between the particles.
     */
    let bridge = SoftBodyBuilder::grid(Vector::new(2.0, 3.0), Vector::new(3.0, 0.2), 31, 3)
        .cell_model(SoftBodyCellModel::Volume)
        .pinned_particles([0, 1, 2, 90, 91, 92])
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.05)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
    world.insert_soft_body(bridge);
    for i in 0..8 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(-0.4 + i as Real * 0.65, 4.5)),
            ColliderBuilder::cuboid(0.03, 0.4).density(3.0),
        );
    }

    /*
     * A stack of jelly squares topped by a pressurized blob: soft-vs-soft contacts go through
     * the surfaces.
     */
    for i in 0..3 {
        let square = SoftBodyBuilder::grid(
            Vector::new(8.0, 0.75 + i as Real * 1.6),
            Vector::splat(0.7),
            5,
            5,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
        world.insert_soft_body(square);
    }
    let blob = SoftBodyBuilder::disk(Vector::new(8.0, 5.0), 0.8, 24)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .volume_factor(1.1)
        .particle_mass(0.05)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    world.insert_soft_body(blob);

    /*
     * A long strip with self-contacts dropped on a peg: it folds and piles up on itself.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(14.0, 1.0)),
        ColliderBuilder::cuboid(0.15, 1.0),
    );
    let strip = SoftBodyBuilder::grid(Vector::new(14.0, 9.0), Vector::new(4.0, 0.1), 60, 2)
        .cell_model(SoftBodyCellModel::Volume)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.02)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    world.insert_soft_body(strip);

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 4.0), 18.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
