use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground.
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

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
        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        testbed.set_initial_body_color(handle, [0.5, 0.5, 1.0]);
    }

    /*
     * Create a cube that will have a ball-shaped sensor attached.
     */

    // Rigid body so that the sensor can move.
    let sensor = RigidBodyBuilder::dynamic().translation(vector![0.0, 10.0]);
    let sensor_handle = bodies.insert(sensor);

    // Solid cube attached to the sensor which
    // other colliders can touch.
    let collider = ColliderBuilder::cuboid(rad, rad);
    colliders.insert_with_parent(collider, sensor_handle, &mut bodies);

    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderBuilder::ball(rad * 5.0)
        .density(0.0)
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    colliders.insert_with_parent(sensor_collider, sensor_handle, &mut bodies);

    testbed.set_initial_body_color(sensor_handle, [0.5, 1.0, 1.0]);

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, events, _| {
        while let Ok(prox) = events.collision_events.try_recv() {
            let color = if prox.started() {
                [1.0, 1.0, 0.0]
            } else {
                [0.5, 0.5, 1.0]
            };

            let parent_handle1 = physics.colliders[prox.collider1()].parent().unwrap();
            let parent_handle2 = physics.colliders[prox.collider2()].parent().unwrap();

            if let Some(graphics) = &mut graphics {
                if parent_handle1 != ground_handle && parent_handle1 != sensor_handle {
                    graphics.set_body_color(parent_handle1, color);
                }
                if parent_handle2 != ground_handle && parent_handle2 != sensor_handle {
                    graphics.set_body_color(parent_handle2, color);
                }
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 1.0], 100.0);
}
