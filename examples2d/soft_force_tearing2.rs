//! Force-based tearing (`SoftBodyMaterial::tear_force`), interior strength and tear smoothing:
//! bars hanging a crate (the strain criterion never fires, the force one tears), a notched slab
//! pulled apart with a tougher interior, and crate bars struck by a disk, one of them smoothed.

use rapier2d::pipeline::DebugRenderMode;
use rapier2d::prelude::*;
use rapier_testbed2d::TestbedViewer;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let settings = viewer.example_settings_mut();
    let tear_force = settings.get_or_set_f32("Tear force (crate bar)", 8.0, 2.0..=30.0);
    let interior_strength = settings.get_or_set_f32("Interior strength (slab)", 4.0, 1.0..=8.0);
    let smoothing = settings.get_or_set_f32("Tear smoothing (right bar, s)", 1.0, 0.0..=3.0);

    let mut world = PhysicsWorld::new();

    /*
     * Ground and the beam everything hangs from.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 9.3)),
        ColliderBuilder::cuboid(30.0, 0.3),
    );

    // A stiff bar: its edges stretch by a few percent under the loads below.
    let stiff = |material: SoftBodyMaterial| SoftBodyMaterial {
        edge_softness: SpringCoefficients::new(150.0, 1.0),
        volume_softness: SpringCoefficients::new(150.0, 1.0),
        ..material
    };
    let (nx, ny) = (3usize, 13usize);
    let idx = |i: usize, j: usize| (i * ny + j) as u32;
    let hanging_bar = |x: Real, material: SoftBodyMaterial| {
        SoftBodyBuilder::grid(Vector::new(x, 7.5), Vector::new(0.3, 1.5), nx, ny)
            .material(material)
            .pinned_particles((0..nx).map(|i| idx(i, ny - 1)))
            .particle_mass(0.05)
            .particle_radius(0.1)
            .surface_collider(ColliderBuilder::ball(0.1).friction(0.8))
    };

    /*
     * Strain vs force: the same crate hung from a strain-tearing bar and a force-tearing bar;
     * it loads the top edges with about 11 N, which stretch by 4%.
     */
    for (x, material) in [
        (
            -13.0,
            stiff(SoftBodyMaterial {
                tear_strain: Some(0.5),
                ..Default::default()
            }),
        ),
        (
            -10.0,
            stiff(SoftBodyMaterial {
                tear_force: Some(tear_force),
                tear_smoothing: 0.5,
                ..Default::default()
            }),
        ),
    ] {
        let bar = world.insert_soft_body(hanging_bar(x, material));
        let (crate_body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(x, 5.4)),
            ColliderBuilder::cuboid(0.6, 0.5).density(1.2),
        );
        for i in 0..nx {
            world.soft_bodies[bar].attach_particle(idx(i, 0) as usize, crate_body, &world.bodies);
        }
    }

    /*
     * Interior strength: a slab clamped over two columns at each end and pulled apart; its
     * weight loads the clamps with 20 N at most, the pull reaches 250 N.
     */
    let (sx, sy) = (25usize, 7usize);
    let sidx = |i: usize, j: usize| (i * sy + j) as u32;
    let slab = SoftBodyBuilder::grid(Vector::new(-1.0, 2.0), Vector::new(3.0, 0.75), sx, sy)
        .material(SoftBodyMaterial {
            edge_softness: SpringCoefficients::new(60.0, 1.0),
            volume_softness: SpringCoefficients::new(60.0, 1.0),
            tear_force: Some(60.0),
            tear_smoothing: 0.05,
            interior_strength,
            ..Default::default()
        })
        .pinned_particles(
            (0..sy).flat_map(|j| [sidx(0, j), sidx(1, j), sidx(sx - 2, j), sidx(sx - 1, j)]),
        )
        .particle_mass(0.05)
        .particle_radius(0.1)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let slab = world.insert_soft_body(slab);
    // The notch: the top-skin edges of three middle columns give at 40% of the threshold.
    {
        let sb = &mut world.soft_bodies[slab];
        let top = |p: u32| p % sy as u32 == sy as u32 - 1;
        let middle = |p: u32| (11..=13).contains(&(p / sy as u32));
        let notched: Vec<usize> = sb
            .edges()
            .iter()
            .enumerate()
            .filter(|(_, e)| e.vertices.iter().all(|&p| top(p) && middle(p)))
            .map(|(i, _)| i)
            .collect();
        for i in notched {
            sb.set_edge_tear_resistance(i, 0.4);
        }
    }
    let mut right_end: Vec<((SoftBodyHandle, u32), Vector)> = (0..sy)
        .flat_map(|j| [sidx(sx - 2, j), sidx(sx - 1, j)])
        .map(|i| ((slab, i), world.soft_bodies[slab].particle_position(i as usize)))
        .collect();

    /*
     * Smoothing: two identical bars hanging a wide crate (15 N steady load on their top edges,
     * threshold 25 N), each struck by a light disk falling on the crate; the jerk loads the
     * edges with over 60 N for a few steps.
     */
    let shot_bar = |smoothing: Real| {
        stiff(SoftBodyMaterial {
            tear_force: Some(25.0),
            tear_smoothing: smoothing,
            ..Default::default()
        })
    };
    let mut disks = Vec::new();
    for (x, smoothing) in [(8.0, 0.0), (13.0, smoothing)] {
        let bar = world.insert_soft_body(hanging_bar(x, shot_bar(smoothing)));
        let (crate_body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(x, 5.4)),
            ColliderBuilder::cuboid(1.2, 0.4).density(0.75),
        );
        for i in 0..nx {
            world.soft_bodies[bar].attach_particle(idx(i, 0) as usize, crate_body, &world.bodies);
        }
        // Held under the beam until the bars hang still.
        let (disk, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x + 0.8, 8.6))
                .enabled(false),
            ColliderBuilder::ball(0.2).density(3.0),
        );
        disks.push(disk);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 4.5), 40.0);
    viewer.set_debug_render(
        true,
        DebugRenderMode::SOFT_BODIES | DebugRenderMode::SOFT_BODY_STRESS,
    );

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            let dt = world.integration_parameters.dt;
            // The disks are shot down at the crates once the bars hang still.
            if t < 1.0 && t + dt >= 1.0 {
                for &disk in &disks {
                    let disk = &mut world.bodies[disk];
                    disk.set_enabled(true);
                    disk.set_linvel(Vector::new(0.0, -15.0), true);
                }
            }
            t += dt;
            // The slab's right clamp starts moving after two seconds, at a quarter meter per
            // second, and stops once the slab has stretched by half.
            let shift = ((t - 2.0).max(0.0) * 0.25).min(3.0);
            for &((body, i), rest) in &right_end {
                world.soft_bodies[body]
                    .set_particle_kinematic_target(i as usize, rest + Vector::new(shift, 0.0));
            }
            world.step_with_events(&(), &events);
            let tears: Vec<SoftBodyTearEvent> = tear_recv.try_iter().collect();
            for (particle, _) in &mut right_end {
                follow_tears(&tears, particle);
            }
        }
    }
    Ok(())
}

