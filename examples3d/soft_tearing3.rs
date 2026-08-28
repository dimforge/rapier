//! Tearing (`SoftBodyMaterial::tear_strain`): a pinned sheet ripped by a dropped ball, a curtain
//! shot through by a ball, and a jelly bar pulled apart by its kinematic ends. Cloth springs are
//! stiff (100 Hz) so only impacts pass the 0.4 tear strain; the panel sets `min_piece`.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.1, 30.0),
    );

    /*
     * A sheet pinned along its border: the heavy ball dropped on it rips through.
     */
    let n = 40usize;
    let border = (0..n * n).filter_map(|k| {
        let (i, j) = (k / n, k % n);
        (i == 0 || j == 0 || i == n - 1 || j == n - 1).then_some(k as u32)
    });
    let sheet = SoftBodyBuilder::cloth(
        Vector::new(-6.0, 3.0, -2.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles(border)
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        tear_strain: Some(0.4),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(100.0, 1.0))
    })
    .particle_mass(0.02)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
    world.insert_soft_body(sheet);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-4.0, 6.0, 0.0)),
        ColliderBuilder::ball(0.6).density(30.0),
    );

    /*
     * A curtain pinned along its top edge, with a heavy ball shot through it.
     */
    let curtain = SoftBodyBuilder::cloth(
        Vector::new(0.0, 4.5, 4.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, -0.1, 0.0),
        50,
        40,
    )
    .pinned_particles((0..50).map(|i| (i * 40) as u32))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        tear_strain: Some(0.2),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(100.0, 1.0))
    })
    .particle_mass(0.02);
    world.insert_soft_body(curtain);
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.5, 2.5, 12.0))
            .linvel(Vector::new(0.0, 0.0, -25.0)),
        ColliderBuilder::cuboid(0.3, 0.3, 0.3)
            .rotation(Vector::new(0.5, 0.5, 0.5))
            .density(20.0),
    );

    /*
     * A jelly bar whose two ends are pinned: the right end is pulled away until the bar snaps.
     */
    let bar = SoftBodyBuilder::cuboid(
        Vector::new(3.0, 1.0, -4.0),
        Vector::new(2.0, 0.4, 0.4),
        21,
        5,
        5,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(SoftBodyMaterial {
        // Stiff enough not to tear from sagging between the clamps (E = 5e3 did).
        young_modulus: 5.0e4,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        tear_strain: Some(0.4),
        ..Default::default()
    })
    .particle_mass(0.05)
    .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let bar = world.insert_soft_body(bar);
    let sb = &world.soft_bodies[bar];
    let ends: Vec<(usize, Vector, bool)> = (0..sb.num_particles())
        .filter_map(|i| {
            let p = sb.particle_position(i);
            if p.x < 1.01 {
                Some((i, p, false))
            } else if p.x > 4.99 {
                Some((i, p, true))
            } else {
                None
            }
        })
        .collect();
    for &(i, _, _) in &ends {
        world.soft_bodies[bar].set_particle_pinned(i, true);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(9.0, 8.0, 16.0), Vec3::new(0.0, 1.5, 1.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // The right end of the bar starts moving after a second, at half a meter per
            // second, and stops once the bar has doubled its length.
            let shift = ((t - 1.0).max(0.0) * 0.5).min(4.0);
            let sb = &mut world.soft_bodies[bar];
            for &(i, rest, right) in &ends {
                if right {
                    sb.set_particle_kinematic_target(i, rest + Vector::new(shift, 0.0, 0.0));
                }
            }
            world.step();
        }
    }
    Ok(())
}
