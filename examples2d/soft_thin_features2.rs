//! Stress test (2D): deformable bodies against thin rigid features and vice versa (beds of
//! nails, thin pins, raining needles, thin plates, a stretched strip under a thin rod).

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(40.0, 0.5),
    );

    let jelly = |center: Vector, half: Real, n: usize, young: Real| {
        SoftBodyBuilder::grid(center, Vector::splat(half), n, n)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06).friction(0.7))
    };
    let strip = |center: Vector, half: Vector, nx: usize| {
        SoftBodyBuilder::grid(center, half, nx, 2)
            .cell_model(SoftBodyCellModel::Volume)
            .softness(SpringCoefficients::new(30.0, 1.0))
            .particle_mass(0.03)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05).friction(0.6))
    };
    let blob = |center: Vector, radius: Real| {
        SoftBodyBuilder::disk(center, radius, 24)
            .softness(SpringCoefficients::new(20.0, 1.0))
            .volume_factor(1.2)
            .particle_mass(0.05)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
    };

    /*
     * Bed of nails: jelly squares of two stiffnesses and a blob land on it, a strip drapes over.
     */
    for i in 0..24 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-16.0 + i as Real * 0.5, 0.6)),
            ColliderBuilder::capsule_y(0.6, 0.03),
        );
    }
    world.insert_soft_body(jelly(Vector::new(-14.5, 3.5), 0.75, 5, 3.0e3));
    world.insert_soft_body(jelly(Vector::new(-11.5, 3.5), 0.75, 5, 5.0e4));
    world.insert_soft_body(blob(Vector::new(-8.5, 3.5), 0.8));
    world.insert_soft_body(strip(Vector::new(-6.0, 6.0), Vector::new(2.5, 0.1), 31));

    /*
     * Needle rain on a hammock (strip pinned at both ends) and on a jelly block.
     */
    let hammock =
        strip(Vector::new(0.0, 4.0), Vector::new(3.0, 0.1), 31).pinned_particles([0, 1, 60, 61]);
    world.insert_soft_body(hammock);
    world.insert_soft_body(jelly(Vector::new(6.5, 0.9), 0.9, 6, 2.0e4));
    for i in 0..10 {
        for j in 0..3 {
            for cx in [0.0, 6.5] {
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            cx - 2.0 + i as Real * 0.45,
                            8.0 + j as Real * 1.2,
                        ))
                        .rotation(0.4 * (i + j) as Real),
                    ColliderBuilder::capsule_y(0.5, 0.02).density(3.0),
                );
            }
        }
    }

    /*
     * Thin plates falling on a blob.
     */
    world.insert_soft_body(blob(Vector::new(11.0, 0.9), 0.9));
    for i in 0..4 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(11.0, 3.5 + i as Real * 0.5))
                .rotation(0.15 * i as Real),
            ColliderBuilder::cuboid(0.9, 0.015).density(1.0),
        );
    }

    /*
     * A clamped strip stretched to twice its length and released, with a rod resting on it.
     */
    let (sx, sy) = (2.5, 0.1);
    let stretched =
        strip(Vector::new(16.0, 3.0), Vector::new(sx, sy), 31).pinned_particles([0, 1, 60, 61]);
    let stretched = world.insert_soft_body(stretched);
    let right_rest: Vec<Vector> = [60usize, 61]
        .iter()
        .map(|&i| world.soft_bodies[stretched].particle_position(i))
        .collect();
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(16.0, 4.0)),
        ColliderBuilder::capsule_x(0.8, 0.03).density(2.0),
    );
    let animate = move |world: &mut PhysicsWorld, t: Real| {
        let stretch = 2.5 * (1.0 - (0.5 * t).cos());
        let sb = &mut world.soft_bodies[stretched];
        for (&i, rest) in [60usize, 61].iter().zip(right_rest.iter()) {
            sb.set_particle_kinematic_target(i, *rest + Vector::new(stretch, 0.0));
        }
    };

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 4.0), 22.0);

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            animate(&mut world, t);
            world.step();
        }
    }
    Ok(())
}
