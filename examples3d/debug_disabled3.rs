use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 10.1;
    let ground_height = 2.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * Platform that will be enabled/disabled.
     */
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0));
    let collider = ColliderBuilder::cuboid(5.0, 1.0, 5.0);
    let (_handle, handle_to_disable) = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(30.0, 4.0, 30.0), Vec3::new(0.0, 1.0, 0.0));

    let mut step_id = 0usize;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            step_id += 1;

            if step_id.is_multiple_of(250) {
                let co = &mut world.colliders[handle_to_disable];
                let enabled = co.is_enabled();
                co.set_enabled(!enabled);
                println!("Platform is now enabled: {}", co.is_enabled());
            }

            if step_id.is_multiple_of(25) {
                let rigid_body =
                    RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 20.0, 0.0));
                let handle = world.bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                world
                    .colliders
                    .insert_with_parent(collider, handle, &mut world.bodies);

                viewer.add_body(handle, &world);
            }
        }
    }
    Ok(())
}
