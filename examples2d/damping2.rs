use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    /*
     * Create the balls
     */
    let num = 10;
    let rad = 0.2;

    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * std::f32::consts::PI * 2.0).sin_cos();

        // Build the rigid body.
        let rb = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y))
            .linvel(Vector::new(x * 10.0, y * 10.0))
            .angvel(100.0)
            .linear_damping((i + 1) as f32 * subdiv * 10.0)
            .angular_damping((num - i) as f32 * subdiv * 10.0);
        // Build the collider.
        let co = ColliderBuilder::cuboid(rad, rad);
        let _ = world.insert(rb, co);
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(3.0, 2.0), 50.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
