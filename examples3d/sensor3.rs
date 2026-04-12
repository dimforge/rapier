use kiss3d::color::Color;
use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 10.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (ground_handle, _) = world.insert(rigid_body, collider);

    /*
     * Create some boxes.
     */
    let num = 10;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centerz = shift * num as f32 / 2.0;

    for i in 0usize..num {
        for k in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = 3.0;
            let z = k as f32 * shift - centerz;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            let (handle, _) = world.insert(rigid_body, collider);

            testbed.set_initial_body_color(handle, Color::new(0.5, 0.5, 1.0, 1.0));
        }
    }

    /*
     * Create a cube that will have a ball-shaped sensor attached.
     */

    // Rigid body so that the sensor can move.
    let sensor = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0));

    // Solid cube attached to the sensor which
    // other colliders can touch.
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    let (sensor_handle, _) = world.insert(sensor, collider);

    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderBuilder::ball(rad * 5.0)
        .density(0.0)
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    world.insert_collider_with_parent(sensor_collider, sensor_handle);

    testbed.set_initial_body_color(sensor_handle, Color::new(0.5, 1.0, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, events, _| {
        while let Ok(prox) = events.collision_events.try_recv() {
            let color = if prox.started() {
                Color::new(1.0, 1.0, 0.0, 1.0)
            } else {
                Color::new(0.5, 0.5, 1.0, 1.0)
            };

            let parent_handle1 = physics.colliders[prox.collider1()].parent().unwrap();
            let parent_handle2 = physics.colliders[prox.collider2()].parent().unwrap();

            if let Some(graphics) = &mut graphics {
                if parent_handle1 != ground_handle && parent_handle1 != sensor_handle {
                    graphics.set_body_color(parent_handle1, color, false);
                }
                if parent_handle2 != ground_handle && parent_handle2 != sensor_handle {
                    graphics.set_body_color(parent_handle2, color, false);
                }
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(6.0, 4.0, 6.0), Vec3::new(0.0, 1.0, 0.0));
}
