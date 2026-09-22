//! Cloth draped over rigid obstacles, and a hanging curtain pushed by a rolling ball.

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
     * A sheet dropped over a rigid ball and a rigid box.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-1.0, 1.0, 0.0)),
        ColliderBuilder::ball(1.0),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(1.5, 0.6, 0.0)),
        ColliderBuilder::cuboid(0.6, 0.6, 0.6),
    );
    let n = 40;
    let sheet = SoftBodyBuilder::cloth(
        Vector::new(-3.0, 3.0, -2.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n + 20,
        n,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .particle_mass(0.02)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
    world.insert_soft_body(sheet);

    /*
     * A curtain pinned along its top edge, with a heavy ball rolling into it.
     */
    let curtain = SoftBodyBuilder::cloth(
        Vector::new(-2.0, 4.0, 5.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, -0.1, 0.0),
        40,
        30,
    )
    .pinned_particles((0..40).map(|i| (i * 30) as u32))
    .softness(SpringCoefficients::new(30.0, 1.0))
    .particle_mass(0.02);
    world.insert_soft_body(curtain);
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 9.0))
            .linvel(Vector::new(0.0, 0.0, -6.0)),
        ColliderBuilder::ball(0.5).density(3.0),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(8.0, 6.0, 14.0), Vec3::new(0.0, 1.5, 2.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
