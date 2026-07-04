use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 100.1;
    let ground_height = 2.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 4.0, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

    let rad = 1.0;
    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 7.0 * rad, 0.0))
        .can_sleep(false);
    let collider = ColliderBuilder::ball(rad);
    let (_handle, _) = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 2.0 * rad, 0.0))
        .can_sleep(false);
    let collider = ColliderBuilder::ball(rad);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    viewer.look_at(Vec3::new(100.0, -10.0, 100.0), Vec3::ZERO);
    viewer.set_world(&mut world);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
