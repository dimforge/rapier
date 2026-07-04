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
    let ground_size = 20.;
    let ground_height = 1.0;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height).restitution(1.0);
    let _ = world.insert(rigid_body, collider);

    let num = 10;
    let rad = 0.5;

    for j in 0..2 {
        for i in 0..=num {
            let x = (i as f32) - num as f32 / 2.0;
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x * 2.0, 10.0 * (j as f32 + 1.0)));
            let collider = ColliderBuilder::ball(rad).restitution((i as f32) / (num as f32));
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 1.0), 25.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
