//! Soft-body showcase (2D): pressurized blobs, a rope holding a weight, a jelly square and a
//! hanging chain of soft polygons.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and walls.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(15.0, 0.5),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-15.0, 5.0)),
        ColliderBuilder::cuboid(0.5, 5.0),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(15.0, 5.0)),
        ColliderBuilder::cuboid(0.5, 5.0),
    );

    /*
     * Pressurized blobs of various sizes.
     */
    for i in 0..5 {
        let radius = 0.6 + 0.15 * i as Real;
        let blob = SoftBodyBuilder::disk(
            Vector::new(-10.0 + i as Real * 2.5, 2.0 + i as Real),
            radius,
            24,
        )
        .softness(SpringCoefficients::new(20.0, 1.0))
        .volume_factor(1.1)
        .self_contacts(true)
        .particle_mass(0.05);
        world.insert_soft_body(blob);
    }

    /*
     * Jelly squares (corotational and Neo-Hookean elastic cells) and one with per-cell area
     * constraints.
     */
    let jelly = SoftBodyBuilder::grid(Vector::new(2.0, 1.2), Vector::splat(1.0), 6, 6)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 3.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.2);
    world.insert_soft_body(jelly);
    let jelly = SoftBodyBuilder::grid(Vector::new(8.0, 1.2), Vector::splat(1.0), 6, 6)
        .cell_model(SoftBodyCellModel::NeoHookean)
        .material(SoftBodyMaterial {
            young_modulus: 3.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.2);
    world.insert_soft_body(jelly);
    let jelly = SoftBodyBuilder::grid(Vector::new(5.0, 1.2), Vector::splat(1.0), 6, 6)
        .cell_model(SoftBodyCellModel::Volume)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .particle_mass(0.2);
    world.insert_soft_body(jelly);

    /*
     * A rope hanging from a fixed anchor, holding a rigid box.
     */
    let rope = SoftBodyBuilder::rope(Vector::new(8.0, 9.0), Vector::new(12.0, 9.0), 25)
        .pinned_particles([0])
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.05);
    let rope_handle = world.insert_soft_body(rope);
    let last_pos = world.soft_bodies[rope_handle].particle_position(24);
    let (weight, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(last_pos + Vector::new(0.0, -0.4)),
        ColliderBuilder::cuboid(0.3, 0.3).density(2.0),
    );
    world.soft_bodies[rope_handle].attach_particle(24, weight, &world.bodies);

    /*
     * A stack of rigid boxes for the blobs to knock down.
     */
    for i in 0..6 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(-4.0, 0.3 + 0.6 * i as Real)),
            ColliderBuilder::cuboid(0.3, 0.3),
        );
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 4.0), 30.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
