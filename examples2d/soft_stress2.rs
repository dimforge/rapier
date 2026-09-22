//! Stress coloring (`DebugRenderMode::SOFT_BODY_STRESS`): elements are colored by their smoothed
//! `SoftBodyEdge::stress`, blue when slack, red at the tear threshold; nothing tears. Scenes: a
//! bridge, a bar and a block, each under a crate, and a blob colored by its stretch instead.

use rapier_testbed2d::TestbedViewer;
use rapier2d::pipeline::DebugRenderMode;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and the beam the bar hangs from.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(8.0, 9.3)),
        ColliderBuilder::cuboid(3.0, 0.3),
    );

    let stiff = |tear_force: Real| SoftBodyMaterial {
        edge_softness: SpringCoefficients::new(100.0, 1.0),
        volume_softness: SpringCoefficients::new(100.0, 1.0),
        tear_force: Some(tear_force),
        // Averaged over half a second, so the crates landing do not tear anything.
        tear_smoothing: 0.5,
        ..Default::default()
    };

    /*
     * The bridge and its crate.
     */
    let (bx, by) = (31usize, 5usize);
    let bidx = |i: usize, j: usize| (i * by + j) as u32;
    let bridge = SoftBodyBuilder::grid(Vector::new(-7.0, 3.0), Vector::new(4.5, 0.6), bx, by)
        .material(stiff(65.0))
        .pinned_particles((0..by).flat_map(|j| [bidx(0, j), bidx(bx - 1, j)]))
        .particle_mass(0.05)
        .particle_radius(0.15)
        .surface_collider(ColliderBuilder::ball(0.15).friction(0.8));
    world.insert_soft_body(bridge);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-7.0, 4.4)),
        ColliderBuilder::cuboid(0.6, 0.6).density(1.5),
    );

    /*
     * The hanging bar and its crate.
     */
    let (hx, hy) = (3usize, 17usize);
    let hidx = |i: usize, j: usize| (i * hy + j) as u32;
    let bar = SoftBodyBuilder::grid(Vector::new(8.0, 6.8), Vector::new(0.3, 2.2), hx, hy)
        .material(stiff(16.0))
        .pinned_particles((0..hx).map(|i| hidx(i, hy - 1)))
        .particle_mass(0.05)
        .particle_radius(0.1)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let bar = world.insert_soft_body(bar);
    let (crate_body, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(8.0, 4.0)),
        ColliderBuilder::cuboid(0.6, 0.5).density(1.0),
    );
    for i in 0..hx {
        world.soft_bodies[bar].attach_particle(hidx(i, 0) as usize, crate_body, &world.bodies);
    }

    /*
     * The block under a crate.
     */
    let block = SoftBodyBuilder::grid(Vector::new(2.0, 1.2), Vector::new(1.2, 1.2), 9, 9)
        .material(stiff(10.0))
        .particle_mass(0.05)
        .particle_radius(0.15)
        .surface_collider(ColliderBuilder::ball(0.15).friction(0.8));
    world.insert_soft_body(block);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(2.0, 3.2)),
        ColliderBuilder::cuboid(0.8, 0.6).density(2.0),
    );

    /*
     * A blob without tear threshold: colored by its stretch.
     */
    let blob = SoftBodyBuilder::disk(Vector::new(13.0, 1.6), 1.5, 36)
        .softness(SpringCoefficients::new(8.0, 1.0))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    world.insert_soft_body(blob);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(13.0, 4.0)),
        ColliderBuilder::cuboid(0.6, 0.4).density(1.0),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(2.0, 4.0), 40.0);
    viewer.set_debug_render(
        true,
        DebugRenderMode::SOFT_BODIES | DebugRenderMode::SOFT_BODY_STRESS,
    );

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
