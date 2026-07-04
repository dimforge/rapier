use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    // Build many balls, all spawned at the same point.
    let rad = 0.5;

    for _ in 0..100 {
        let rigid_body = RigidBodyBuilder::dynamic();
        let collider = ColliderBuilder::cuboid(rad, rad);
        let _ = world.insert(rigid_body, collider);
    }

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
