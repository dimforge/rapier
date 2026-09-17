//! Tearing (`SoftBodyMaterial::tear_strain`): a pinned sheet ripped by a dropped ball, a curtain
//! shot through by a ball, and a jelly bar pulled apart by its kinematic ends. Cloth springs are
//! stiff (100 Hz) so only impacts pass the 0.4 tear strain; the panel sets `min_piece`.

use rapier3d::prelude::*;
use rapier_testbed3d::{
    TestbedViewer,
    egui::{Align2, Slider, Window},
};

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
    let mut ends: Vec<((SoftBodyHandle, u32), Vector, bool)> = (0..sb.num_particles())
        .filter_map(|i| {
            let p = sb.particle_position(i);
            if p.x < 1.01 {
                Some(((bar, i as u32), p, false))
            } else if p.x > 4.99 {
                Some(((bar, i as u32), p, true))
            } else {
                None
            }
        })
        .collect();
    for &((_, i), _, _) in &ends {
        world.soft_bodies[bar].set_particle_pinned(i as usize, true);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(9.0, 8.0, 16.0), Vec3::new(0.0, 1.5, 1.0));

    // The tears of every step, to follow the driven particles through them.
    let (collision_send, _collision_recv) = std::sync::mpsc::channel();
    let (force_send, _force_recv) = std::sync::mpsc::channel();
    let (tear_send, tear_recv) = std::sync::mpsc::channel();
    let events = ChannelEventCollector::new(collision_send, force_send, tear_send);
    // Demo UI: the smallest piece a tear may split off, applied to every soft body (the pieces
    // a tear splits off inherit their origin's material).
    let mut min_piece_default = true;
    let mut min_piece: u32 = 6;
    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        let mut changed = false;
        Window::new("Tearing")
            .anchor(Align2::RIGHT_TOP, [-15.0, 15.0])
            .show(viewer.egui_context(), |ui| {
                changed |= ui
                    .checkbox(
                        &mut min_piece_default,
                        "Default minimum piece (10 triangles or 6 cells)",
                    )
                    .changed();
                changed |= ui
                    .add_enabled(
                        !min_piece_default,
                        Slider::new(&mut min_piece, 1..=40).text("minimum piece (elements)"),
                    )
                    .changed();
            });
        if changed {
            let value = (!min_piece_default).then_some(min_piece);
            for (_, sb) in world.soft_bodies.iter_mut() {
                sb.material_mut().min_piece = value;
            }
        }
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // The right end of the bar starts moving after a second, at half a meter per
            // second, and stops once the bar has doubled its length.
            let shift = ((t - 1.0).max(0.0) * 0.5).min(4.0);
            for &((body, i), rest, right) in &ends {
                if right {
                    let target = rest + Vector::new(shift, 0.0, 0.0);
                    world.soft_bodies[body].set_particle_kinematic_target(i as usize, target);
                }
            }
            world.step_with_events(&(), &events);
            let tears: Vec<SoftBodyTearEvent> = tear_recv.try_iter().collect();
            for (particle, _, _) in &mut ends {
                follow_tears(&tears, particle);
            }
        }
    }
    Ok(())
}

/// Where a particle driven by index is after the tears of a step: a tear compacts the torn
/// body's particles and splits the disconnected pieces off as soft bodies of their own, so a
/// particle's body and index follow the events.
fn follow_tears(events: &[SoftBodyTearEvent], particle: &mut (SoftBodyHandle, u32)) {
    for event in events {
        if event.soft_body == particle.0 {
            if let Some(destination) = event.particle_destination(particle.1) {
                *particle = destination;
            }
        }
    }
}

