//! Tearing (2D, `SoftBodyMaterial::tear_strain`): a pinned jelly bridge broken by a heavy disk, a
//! hanging strip shot through by a fast disk, and a jelly bar pulled apart by its kinematic ends.
//! Cracks split particles and shed pieces as new soft bodies; the panel sets `min_piece`.

use rapier_testbed2d::{
    TestbedViewer,
    egui::{Align2, Slider, Window},
};
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );
    // A wall catching the disk shot through the strip.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(16.0, 3.0)),
        ColliderBuilder::cuboid(0.3, 3.0),
    );

    let jelly = |young: Real, tear: Real| SoftBodyMaterial {
        young_modulus: young,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        tear_strain: Some(tear),
        ..Default::default()
    };

    /*
     * A bridge pinned at both ends: the heavy disk dropped on its middle breaks it.
     */
    let (nx, ny) = (31usize, 6usize);
    let idx = |i: usize, j: usize| (i * ny + j) as u32;
    // Stiff and thick enough to support its own weight with strains under 0.15 (a slender bridge
    // of soft jelly sagged past the tear strain by itself).
    let bridge = SoftBodyBuilder::grid(Vector::new(-8.0, 4.0), Vector::new(3.0, 0.5), nx, ny)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(1.0e6, 0.35))
        .pinned_particles((0..ny).flat_map(|j| [idx(0, j), idx(nx - 1, j)]))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    world.insert_soft_body(bridge);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-8.0, 9.0)),
        ColliderBuilder::ball(0.6).density(20.0),
    );

    /*
     * A strip hanging from its top edge, with a fast disk shot through it.
     */
    let (nx, ny) = (7usize, 41usize);
    let idx = |i: usize, j: usize| (i * ny + j) as u32;
    let curtain = SoftBodyBuilder::grid(Vector::new(2.0, 5.0), Vector::new(0.45, 3.0), nx, ny)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(3.0e4, 0.35))
        .pinned_particles((0..nx).map(|i| idx(i, ny - 1)))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.08).friction(0.8));
    world.insert_soft_body(curtain);
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-4.0, 4.5))
            .linvel(Vector::new(20.0, 0.0)),
        ColliderBuilder::ball(0.4).density(10.0),
    );

    /*
     * A jelly bar whose two ends are pinned: the right end is pulled away until the bar snaps.
     */
    let bar = SoftBodyBuilder::grid(Vector::new(-3.0, 1.0), Vector::new(2.0, 0.4), 21, 5)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(5.0e4, 0.4))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let bar = world.insert_soft_body(bar);
    let sb = &world.soft_bodies[bar];
    let mut ends: Vec<((SoftBodyHandle, u32), Vector, bool)> = (0..sb.num_particles())
        .filter_map(|i| {
            let p = sb.particle_position(i);
            if p.x < -4.99 {
                Some(((bar, i as u32), p, false))
            } else if p.x > -1.01 {
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
    viewer.look_at(Vec2::new(0.0, 4.0), 40.0);

    // The tears of every step, to follow the driven particles through them.
    let (collision_send, _collision_recv) = std::sync::mpsc::channel();
    let (force_send, _force_recv) = std::sync::mpsc::channel();
    let (tear_send, tear_recv) = std::sync::mpsc::channel();
    let events = ChannelEventCollector::new(collision_send, force_send, tear_send);
    // Demo UI: the smallest piece a tear may split off, applied to every soft body (the pieces
    // a tear splits off inherit their origin's material).
    let mut min_piece_default = true;
    let mut min_piece: u32 = 3;
    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        let mut changed = false;
        Window::new("Tearing")
            .anchor(Align2::RIGHT_TOP, [-15.0, 15.0])
            .show(viewer.egui_context(), |ui| {
                changed |= ui
                    .checkbox(&mut min_piece_default, "Default minimum piece (3 cells)")
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
                    world.soft_bodies[body]
                        .set_particle_kinematic_target(i as usize, rest + Vector::new(shift, 0.0));
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
        if event.soft_body == particle.0
            && let Some(destination) = event.particle_destination(particle.1)
        {
            *particle = destination;
        }
    }
}
