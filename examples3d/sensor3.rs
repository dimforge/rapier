use na::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground.
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, ground_handle, &mut bodies);

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
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
            colliders.insert(collider, handle, &mut bodies);

            testbed.set_body_color(handle, Point3::new(0.5, 0.5, 1.0));
        }
    }

    /*
     * Create a cube that will have a ball-shaped sensor attached.
     */

    // Rigid body so that the sensor can move.
    let sensor = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 5.0, 0.0)
        .build();
    let sensor_handle = bodies.insert(sensor);

    // Solid cube attached to the sensor which
    // other colliders can touch.
    let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
    colliders.insert(collider, sensor_handle, &mut bodies);

    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderBuilder::ball(rad * 5.0).sensor(true).build();
    colliders.insert(sensor_collider, sensor_handle, &mut bodies);

    testbed.set_body_color(sensor_handle, Point3::new(0.5, 1.0, 1.0));

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
    testbed.look_at(Point3::new(-6.0, 4.0, -6.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Sensor", init_world)]);
    testbed.run()
}
