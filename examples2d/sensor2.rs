use kiss3d::color::Color;
use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    let (ground_handle, _) = world.insert(rigid_body, collider);

    /*
     * Create some boxes.
     */
    let num = 10;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;

    for i in 0usize..num {
        let x = i as f32 * shift - centerx;
        let y = 3.0;

        // Build the rigid body.
        let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
        let collider = ColliderBuilder::cuboid(rad, rad);
        let (handle, _) = world.insert(rigid_body, collider);

        viewer.set_initial_body_color(handle, Color::new(0.5, 0.5, 1.0, 1.0));
    }

    /*
     * Create a cube that will have a ball-shaped sensor attached.
     */

    // Rigid body so that the sensor can move.
    let sensor = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0));

    // Solid cube attached to the sensor which
    // other colliders can touch.
    let collider = ColliderBuilder::cuboid(rad, rad);
    let (sensor_handle, _) = world.insert(sensor, collider);

    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderBuilder::ball(rad * 5.0)
        .density(0.0)
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    world.insert_collider(sensor_collider, Some(sensor_handle));

    viewer.set_initial_body_color(sensor_handle, Color::new(0.5, 1.0, 1.0, 1.0));

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    let (collision_send, collision_recv) = std::sync::mpsc::channel();
    let (contact_force_send, _contact_force_recv) = std::sync::mpsc::channel();
    let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);
    viewer.look_at(Vec2::new(0.0, 1.0), 100.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step_with_events(&(), &event_handler);

            // Callback that handles proximities.
            while let Ok(prox) = collision_recv.try_recv() {
                let color = if prox.started() {
                    Color::new(1.0, 1.0, 0.0, 1.0)
                } else {
                    Color::new(0.5, 0.5, 1.0, 1.0)
                };

                let parent_handle1 = world.colliders[prox.collider1()].parent().unwrap();
                let parent_handle2 = world.colliders[prox.collider2()].parent().unwrap();

                if parent_handle1 != ground_handle && parent_handle1 != sensor_handle {
                    viewer.set_body_color(parent_handle1, color, false);
                }
                if parent_handle2 != ground_handle && parent_handle2 != sensor_handle {
                    viewer.set_body_color(parent_handle2, color, false);
                }
            }
        }
    }
    Ok(())
}
