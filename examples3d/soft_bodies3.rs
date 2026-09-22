//! Soft-body showcase: a pinned cloth catching a box, a pressurized balloon, jelly cubes
//! (corotational, Neo-Hookean) and a volume-preserving one, and a rope holding a rigid weight.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(12.0, 0.1, 12.0),
    );

    /*
     * A cloth pinned by its four corners, with a box dropped on it.
     */
    let n = 24;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-3.5, 2.5, -1.2),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles([0, (n - 1) as u32, (n * (n - 1)) as u32, (n * n - 1) as u32])
    .softness(SpringCoefficients::new(30.0, 1.0))
    .particle_mass(0.05);
    world.insert_soft_body(cloth);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-2.35, 4.0, 0.0)),
        ColliderBuilder::cuboid(0.3, 0.3, 0.3).density(0.5),
    );

    /*
     * A balloon: hollow sphere with pressure (global volume preservation).
     */
    let balloon = SoftBodyBuilder::sphere(Vector::new(0.5, 3.0, 0.0), 0.8, 2)
        .softness(SpringCoefficients::new(15.0, 1.0))
        .volume_factor(1.2)
        .particle_mass(0.05);
    world.insert_soft_body(balloon);

    /*
     * Jelly cubes: one corotational, one Neo-Hookean, one with per-cell volume constraints.
     */
    let jelly = SoftBodyBuilder::cuboid(Vector::new(3.0, 1.0, 1.5), Vector::splat(0.6), 5, 5, 5)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.2);
    world.insert_soft_body(jelly);
    let jelly = SoftBodyBuilder::cuboid(Vector::new(3.0, 1.0, 4.5), Vector::splat(0.6), 5, 5, 5)
        .cell_model(SoftBodyCellModel::NeoHookean)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.2);
    world.insert_soft_body(jelly);
    let jelly = SoftBodyBuilder::cuboid(Vector::new(3.0, 1.0, -1.5), Vector::splat(0.6), 5, 5, 5)
        .cell_model(SoftBodyCellModel::Volume)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .particle_mass(0.2);
    world.insert_soft_body(jelly);

    /*
     * A rope hanging from a fixed anchor, holding a rigid ball attached by a joint.
     */
    let rope = SoftBodyBuilder::rope(Vector::new(-0.5, 5.0, 3.0), Vector::new(2.5, 5.0, 3.0), 30)
        .pinned_particles([0])
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.05);
    let rope_handle = world.insert_soft_body(rope);
    let last_pos = world.soft_bodies[rope_handle].particle_position(29);
    let (weight, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(last_pos + Vector::new(0.0, -0.3, 0.0)),
        ColliderBuilder::ball(0.25).density(2.0),
    );
    world.soft_bodies[rope_handle].attach_particle(29, weight, &world.bodies);

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(9.0, 6.0, 12.0), Vec3::new(0.0, 1.5, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
