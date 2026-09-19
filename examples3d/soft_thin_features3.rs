//! Stress test: deformable bodies against thin rigid features (nails, knife edges, a wire grid)
//! and thin rigid bodies (needles, plates, a rod) falling on soft bodies.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.5, 30.0),
    );

    let jelly = |center: Vector, half: Real, n: usize, young: Real| {
        SoftBodyBuilder::cuboid(center, Vector::splat(half), n, n, n)
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
    let cloth = |origin: Vector, nu: usize, nv: usize| {
        SoftBodyBuilder::cloth(
            origin,
            Vector::new(0.12, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.12),
            nu,
            nv,
        )
        .softness(SpringCoefficients::new(30.0, 1.0))
        .material(SoftBodyMaterial {
            bend_softness: SpringCoefficients::new(3.0, 1.0),
            ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
        })
        .particle_mass(0.02)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6))
    };

    /*
     * Bed of nails: a jelly block, a balloon and a cloth land on it.
     */
    for i in 0..10 {
        for j in 0..10 {
            world.insert(
                RigidBodyBuilder::fixed().translation(Vector::new(
                    -8.0 + i as Real * 0.45,
                    0.6,
                    -2.0 + j as Real * 0.45,
                )),
                ColliderBuilder::capsule_y(0.6, 0.03),
            );
        }
    }
    world.insert_soft_body(jelly(Vector::new(-7.0, 3.5, -1.0), 0.75, 5, 2.0e4));
    world.insert_soft_body(
        SoftBodyBuilder::sphere(Vector::new(-4.8, 3.5, 1.2), 0.7, 2)
            .softness(SpringCoefficients::new(15.0, 1.0))
            .volume_factor(1.2)
            .particle_mass(0.03)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06).friction(0.6)),
    );
    world.insert_soft_body(cloth(Vector::new(-8.5, 5.5, -2.5), 30, 30));

    /*
     * Knife edges: thin vertical plates; a soft cube dropped across them.
     */
    for i in 0..4 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-1.5, 0.75, -3.0 + i as Real * 0.8)),
            ColliderBuilder::cuboid(1.2, 0.75, 0.015),
        );
    }
    world.insert_soft_body(jelly(Vector::new(-1.5, 3.0, -1.8), 0.9, 5, 5.0e3));

    /*
     * Wire grid: a cloth sags through it.
     */
    for i in 0..7 {
        let t = -1.8 + i as Real * 0.6;
        world.insert(
            RigidBodyBuilder::fixed()
                .translation(Vector::new(t, 2.5, 5.0))
                .rotation(Vector::new(core::f32::consts::FRAC_PI_2, 0.0, 0.0)),
            ColliderBuilder::capsule_y(1.8, 0.02),
        );
        world.insert(
            RigidBodyBuilder::fixed()
                .translation(Vector::new(0.0, 2.5, 5.0 + t))
                .rotation(Vector::new(0.0, 0.0, core::f32::consts::FRAC_PI_2)),
            ColliderBuilder::capsule_y(1.8, 0.02),
        );
    }
    world.insert_soft_body(cloth(Vector::new(-1.5, 4.5, 3.5), 26, 26).self_contacts(true));

    /*
     * Needle rain on a trampoline (cloth pinned along its edges) and on a jelly block.
     */
    let n = 26;
    let edge = |i: usize, j: usize| i == 0 || j == 0 || i == n - 1 || j == n - 1;
    let pinned = (0..n * n)
        .filter(|&id| edge(id / n, id % n))
        .map(|id| id as u32);
    world.insert_soft_body(
        cloth(Vector::new(3.0, 2.5, -3.5), n, n)
            .pinned_particles(pinned)
            .softness(SpringCoefficients::new(40.0, 1.0)),
    );
    world.insert_soft_body(jelly(Vector::new(4.5, 0.9, 3.0), 0.9, 5, 3.0e4));
    for i in 0..6 {
        for j in 0..6 {
            for (cx, cz, h) in [(4.5, -2.0, 6.0), (4.5, 3.0, 5.0)] {
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            cx - 1.0 + i as Real * 0.4,
                            h + (i + j) as Real * 0.3,
                            cz - 1.0 + j as Real * 0.4,
                        ))
                        .rotation(Vector::new(0.3 * i as Real, 0.0, 0.2 * j as Real)),
                    ColliderBuilder::capsule_y(0.5, 0.02).density(3.0),
                );
            }
        }
    }

    /*
     * Thin plates falling flat on a jelly cube, and a long thin rod across a balloon.
     */
    world.insert_soft_body(jelly(Vector::new(9.0, 0.9, -2.0), 0.9, 5, 1.0e4));
    for i in 0..4 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(9.0, 3.5 + i as Real * 0.5, -2.0))
                .rotation(Vector::new(0.1 * i as Real, 0.5 * i as Real, 0.05)),
            ColliderBuilder::cuboid(0.8, 0.015, 0.8).density(1.0),
        );
    }
    world.insert_soft_body(
        SoftBodyBuilder::sphere(Vector::new(9.0, 1.0, 3.0), 0.9, 2)
            .softness(SpringCoefficients::new(15.0, 1.0))
            .volume_factor(1.2)
            .particle_mass(0.03)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06).friction(0.6)),
    );
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(9.0, 3.5, 3.0))
            .rotation(Vector::new(0.0, 0.0, core::f32::consts::FRAC_PI_2)),
        ColliderBuilder::capsule_y(2.5, 0.03).density(2.0),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(2.0, 10.0, 18.0), Vec3::new(1.0, 1.5, 1.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
