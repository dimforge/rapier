//! Cutting soft bodies (`PhysicsWorld::cut_soft_body`): hold `C`, move the mouse, release `C` to
//! cut every soft body along the line from key-down to key-up; a saw blade also rises through the
//! suspended slab. Cuts remove no material and separated pieces become soft bodies.

use kiss3d::color::Color;
use rapier_testbed2d::{Key, TestbedViewer};
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );

    let jelly = |young: Real| SoftBodyMaterial {
        young_modulus: young,
        poisson_ratio: 0.35,
        elastic_damping_ratio: 1.0,
        ..Default::default()
    };

    /*
     * A jelly block to slice by hand.
     */
    let block = SoftBodyBuilder::grid(Vector::new(-8.0, 2.0), Vector::new(2.0, 2.0), 13, 13)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(2.0e4))
        .particle_mass(0.05)
        .particle_radius(0.15)
        .surface_collider(ColliderBuilder::ball(0.15).friction(0.8));
    world.insert_soft_body(block);

    /*
     * A blob: a ring of particles holding its area. Cut open, it falls limp.
     */
    let blob = SoftBodyBuilder::disk(Vector::new(-2.0, 2.0), 1.6, 40)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    world.insert_soft_body(blob);

    /*
     * A curtain pinned along its top edge.
     */
    let (cx, cy) = (9usize, 31usize);
    let cidx = |i: usize, j: usize| (i * cy + j) as u32;
    let curtain = SoftBodyBuilder::grid(Vector::new(3.0, 4.5), Vector::new(0.6, 3.0), cx, cy)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(3.0e4))
        .pinned_particles((0..cx).map(|i| cidx(i, cy - 1)))
        .particle_mass(0.05)
        .particle_radius(0.1)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    world.insert_soft_body(curtain);

    /*
     * A slab suspended between two posts, with a saw rising through it.
     */
    let (sx, sy) = (25usize, 5usize);
    let sidx = |i: usize, j: usize| (i * sy + j) as u32;
    let slab = SoftBodyBuilder::grid(Vector::new(10.0, 4.0), Vector::new(3.0, 0.5), sx, sy)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(jelly(5.0e4))
        .pinned_particles((0..sy).flat_map(|j| [sidx(0, j), sidx(sx - 1, j)]))
        .particle_mass(0.05)
        .particle_radius(0.1)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    world.insert_soft_body(slab);
    // The saw: a kinematic blade drawn as a thin plate, a sensor so it pushes nothing (the
    // cut does the work).
    let saw_start = Vector::new(10.0, 1.5);
    let saw =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(saw_start));
    world.insert_collider(ColliderBuilder::cuboid(0.05, 1.0).sensor(true), Some(saw));

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(1.0, 3.5), 40.0);

    let handles: Vec<SoftBodyHandle> = world.soft_bodies().map(|(h, _)| h).collect();
    let mut blade_start: Option<Vector> = None;
    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        // The hand blade: from where `C` went down to the cursor.
        let cursor = viewer.mouse().point.map(|p| Vector::new(p.x, p.y));
        if viewer.keys().pressed(Key::C) {
            if blade_start.is_none() {
                blade_start = cursor;
            }
            if let (Some(a), Some(b)) = (blade_start, cursor) {
                viewer.window_mut().draw_line_2d(
                    Vec2::new(a.x, a.y),
                    Vec2::new(b.x, b.y),
                    Color::new(1.0, 0.35, 0.25, 1.0),
                    3.0,
                );
            }
        } else if let (Some(a), Some(b)) = (blade_start.take(), cursor) {
            for &handle in &handles {
                world.cut_soft_body(handle, &[a, b]);
            }
        }

        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // The saw rises at a fifth of a meter per second, from a second in.
            let rise = ((t - 1.0).max(0.0) * 0.2).min(4.5);
            let position = saw_start + Vector::new(0.0, rise);
            world.bodies[saw].set_next_kinematic_translation(position);
            let edge = [position - Vector::Y, position + Vector::Y];
            for &handle in &handles {
                world.cut_soft_body(handle, &edge);
            }
            world.step();
        }
    }
    Ok(())
}
