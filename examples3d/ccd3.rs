use kiss3d::color::Color;
use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn create_wall(
    testbed: &mut Testbed,
    world: &mut PhysicsWorld,
    offset: Vector,
    stack_height: usize,
    half_extents: Vector,
) {
    let shift = half_extents * 2.0;
    let mut k = 0;
    for i in 0usize..stack_height {
        for j in i..stack_height {
            let fj = j as f32;
            let fi = i as f32;
            let x = offset.x;
            let y = fi * shift.y + offset.y;
            let z = (fi * shift.z / 2.0) + (fj - fi) * shift.z + offset.z
                - stack_height as f32 * half_extents.z;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
            let collider = ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z);
            let (handle, _) = world.insert(rigid_body, collider);
            k += 1;
            if k % 2 == 0 {
                testbed
                    .set_initial_body_color(handle, Color::new(1., 131. / 255., 244.0 / 255., 1.0));
            } else {
                testbed
                    .set_initial_body_color(handle, Color::new(131. / 255., 1., 244.0 / 255., 1.0));
            }
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (ground_handle, _) = world.insert(rigid_body, collider);

    /*
     * Create the pyramids.
     */
    let num_z = 8;
    let num_x = 5;
    let shift_y = ground_height + 0.5;
    let shift_z = (num_z as f32 + 2.0) * 2.0;

    for i in 0..num_x {
        let x = i as f32 * 6.0;
        create_wall(
            testbed,
            &mut world,
            Vector::new(x, shift_y, 0.0),
            num_z,
            Vector::new(0.5, 0.5, 1.0),
        );

        create_wall(
            testbed,
            &mut world,
            Vector::new(x, shift_y, shift_z),
            num_z,
            Vector::new(0.5, 0.5, 1.0),
        );
    }

    /*
     * Create two very fast rigid-bodies.
     * The first one has CCD enabled and a sensor collider attached to it.
     * The second one has CCD enabled and a collider attached to it.
     */
    let collider = ColliderBuilder::ball(1.0)
        .density(10.0)
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    let rigid_body = RigidBodyBuilder::dynamic()
        .linvel(Vector::new(1000.0, 0.0, 0.0))
        .translation(Vector::new(-20.0, shift_y + 2.0, 0.0))
        .ccd_enabled(true);
    let (sensor_handle, _) = world.insert(rigid_body, collider);

    // Second rigid-body with CCD enabled.
    let collider = ColliderBuilder::ball(1.0).density(10.0);
    let rigid_body = RigidBodyBuilder::dynamic()
        .linvel(Vector::new(1000.0, 0.0, 0.0))
        .translation(Vector::new(-20.0, shift_y + 2.0, shift_z))
        .ccd_enabled(true);
    let (handle, _) = world.insert(rigid_body, collider.clone());
    testbed.set_initial_body_color(handle, Color::new(0.2, 0.2, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, events, _| {
        while let Ok(prox) = events.collision_events.try_recv() {
            let color = if prox.started() {
                Color::new(1.0, 1.0, 0.0, 1.0)
            } else {
                Color::new(0.5, 0.5, 1.0, 1.0)
            };

            let parent_handle1 = physics
                .colliders
                .get(prox.collider1())
                .unwrap()
                .parent()
                .unwrap();
            let parent_handle2 = physics
                .colliders
                .get(prox.collider2())
                .unwrap()
                .parent()
                .unwrap();

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
    testbed.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);
}
