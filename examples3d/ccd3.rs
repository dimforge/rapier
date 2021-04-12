use na::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

fn create_wall(
    testbed: &mut Testbed,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector3<f32>,
    stack_height: usize,
    half_extents: Vector3<f32>,
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
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
            let handle = bodies.insert(rigid_body);
            let collider =
                ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z).build();
            colliders.insert(collider, handle, bodies);
            k += 1;
            if k % 2 == 0 {
                testbed.set_body_color(handle, Point3::new(255. / 255., 131. / 255., 244.0 / 255.));
            } else {
                testbed.set_body_color(handle, Point3::new(131. / 255., 255. / 255., 244.0 / 255.));
            }
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, ground_handle, &mut bodies);

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
            &mut bodies,
            &mut colliders,
            Vector3::new(x, shift_y, 0.0),
            num_z,
            Vector3::new(0.5, 0.5, 1.0),
        );

        create_wall(
            testbed,
            &mut bodies,
            &mut colliders,
            Vector3::new(x, shift_y, shift_z),
            num_z,
            Vector3::new(0.5, 0.5, 1.0),
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
        .build();
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .linvel(1000.0, 0.0, 0.0)
        .translation(-20.0, shift_y + 2.0, 0.0)
        .ccd_enabled(true)
        .build();
    let sensor_handle = bodies.insert(rigid_body);
    colliders.insert(collider, sensor_handle, &mut bodies);

    // Second rigid-body with CCD enabled.
    let collider = ColliderBuilder::ball(1.0).density(10.0).build();
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .linvel(1000.0, 0.0, 0.0)
        .translation(-20.0, shift_y + 2.0, shift_z)
        .ccd_enabled(true)
        .build();
    let handle = bodies.insert(rigid_body);
    colliders.insert(collider.clone(), handle, &mut bodies);
    testbed.set_body_color(handle, Point3::new(0.2, 0.2, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |_, mut graphics, physics, events, _| {
        while let Ok(prox) = events.intersection_events.try_recv() {
            let color = if prox.intersecting {
                Point3::new(1.0, 1.0, 0.0)
            } else {
                Point3::new(0.5, 0.5, 1.0)
            };

            let parent_handle1 = physics.colliders.get(prox.collider1).unwrap().parent();
            let parent_handle2 = physics.colliders.get(prox.collider2).unwrap().parent();

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
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(100.0, 100.0, 100.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
