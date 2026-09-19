//! Deformable trimesh collisions: soft bodies colliding through their surface mesh. A coarse
//! trampoline catches a rain of balls smaller than its particle spacing, a tarp drapes over thin
//! poles, soft cubes stack on each other, and a self-colliding ribbon piles up on a pedestal.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(40.0, 1.0, 40.0),
    );

    /*
     * A trampoline with a coarse particle spacing (0.2) under a rain of balls smaller than
     * that spacing: they rest on the triangles instead of falling between the particles.
     */
    let n = 20;
    {
        let x0 = -2.0;
        let edge = |i: usize, j: usize| i == 0 || j == 0 || i == n - 1 || j == n - 1;
        let pinned = (0..n * n)
            .filter(|&id| edge(id / n, id % n))
            .map(|id| id as u32);
        let trampoline = SoftBodyBuilder::cloth(
            Vector::new(x0, 3.0, -2.0),
            Vector::new(0.2, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.2),
            n,
            n,
        )
        .pinned_particles(pinned)
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.05)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
        world.insert_soft_body(trampoline);

        // A rain of balls smaller than the particle spacing.
        for i in 0..6 {
            for j in 0..6 {
                for h in 0..3 {
                    let pos = Vector::new(
                        x0 + 0.8 + i as Real * 0.5 + (h % 2) as Real * 0.2,
                        5.0 + h as Real * 0.6,
                        -1.2 + j as Real * 0.5 + (h % 2) as Real * 0.2,
                    );
                    world.insert(
                        RigidBodyBuilder::dynamic().translation(pos),
                        ColliderBuilder::ball(0.08).density(2.0),
                    );
                }
            }
        }
    }

    /*
     * A tarp draped over a row of thin poles and a sharp-edged bar: with surface collisions the
     * poles push on the triangles, not only on the particles.
     */
    for i in 0..5 {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-3.0 + i as Real * 1.5, 0.5, 6.0)),
            ColliderBuilder::capsule_y(0.5, 0.08),
        );
    }
    world.insert(
        RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, 1.0, 8.0))
            .rotation(Vector::new(core::f32::consts::FRAC_PI_4, 0.0, 0.0)),
        ColliderBuilder::cuboid(3.5, 0.4, 0.4),
    );
    let tarp = SoftBodyBuilder::cloth(
        Vector::new(-4.0, 3.0, 5.0),
        Vector::new(0.2, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.2),
        40,
        25,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .particle_mass(0.02)
    .particle_radius(0.04)
    .surface_collider(ColliderBuilder::ball(0.04).friction(0.8));
    world.insert_soft_body(tarp);

    /*
     * A stack of soft cubes: soft-vs-soft contacts go through the surfaces.
     */
    for i in 0..3 {
        let cube = SoftBodyBuilder::cuboid(
            Vector::new(9.0, 0.8 + i as Real * 1.7, 6.0),
            Vector::splat(0.75),
            4,
            4,
            4,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 6.0e3,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
        world.insert_soft_body(cube);
    }
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(9.0, 6.5, 6.0)),
        ColliderBuilder::cuboid(0.4, 0.4, 0.4).density(2.0),
    );

    /*
     * A long ribbon with self-contacts dropped on a small pedestal: it folds and piles up on
     * itself instead of passing through its own layers.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-9.0, 0.75, 6.0)),
        ColliderBuilder::cuboid(0.4, 0.75, 0.4),
    );
    let ribbon = SoftBodyBuilder::cloth(
        Vector::new(-13.0, 4.0, 5.6),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        80,
        8,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(2.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .self_contacts(true)
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    world.insert_soft_body(ribbon);

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 12.0, 24.0), Vec3::new(0.0, 1.5, 3.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
