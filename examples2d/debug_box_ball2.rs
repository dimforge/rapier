use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let rad = 1.0;
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, -rad))
        .rotation(std::f32::consts::PI / 4.0);
    let collider = ColliderBuilder::cuboid(rad, rad);
    let _ = world.insert(rigid_body, collider);

    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 3.0 * rad))
        .can_sleep(false);
    let collider = ColliderBuilder::ball(rad);
    let _ = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::ZERO, 50.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
