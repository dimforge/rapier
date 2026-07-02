use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4);
    let (ground_handle, mut ground_collider_handle) = world.insert(rigid_body, collider);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.2, 0.0));
    let collider = ColliderBuilder::ball(ball_rad).density(100.0);
    let (_ball_handle, _) = world.insert(rb, collider);

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();

            // Remove then re-add the ground collider.
            let removed_collider_handle = ground_collider_handle;
            let coll = world
                .colliders
                .remove(
                    removed_collider_handle,
                    &mut world.islands,
                    &mut world.bodies,
                    true,
                )
                .unwrap();
            ground_collider_handle =
                world
                    .colliders
                    .insert_with_parent(coll, ground_handle, &mut world.bodies);
        }
    }
    Ok(())
}
