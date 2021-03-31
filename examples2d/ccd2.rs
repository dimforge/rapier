use na::{Isometry2, Point2, Point3};
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet, SharedShape};
use rapier_testbed2d::Testbed;

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
    let ground_size = 25.0;
    let ground_thickness = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().ccd_enabled(true).build();
    let ground_handle = bodies.insert(rigid_body);

    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness).build();
    colliders.insert(collider, ground_handle, &mut bodies);

    let collider = ColliderBuilder::cuboid(ground_thickness, ground_size)
        .translation(-3.0, 0.0)
        .build();
    colliders.insert(collider, ground_handle, &mut bodies);

    let collider = ColliderBuilder::cuboid(ground_thickness, ground_size)
        .translation(6.0, 0.0)
        .build();
    colliders.insert(collider, ground_handle, &mut bodies);

    let collider = ColliderBuilder::cuboid(ground_thickness, ground_size)
        .translation(2.5, 0.0)
        .sensor(true)
        .build();
    let sensor_handle = colliders.insert(collider, ground_handle, &mut bodies);

    /*
     * Create the shapes
     */
    let radx = 0.4;
    let rady = 0.05;

    let delta1 = Isometry2::translation(0.0, radx - rady);
    let delta2 = Isometry2::translation(-radx + rady, 0.0);
    let delta3 = Isometry2::translation(radx - rady, 0.0);

    let mut compound_parts = Vec::new();
    let vertical = SharedShape::cuboid(rady, radx);
    let horizontal = SharedShape::cuboid(radx, rady);
    compound_parts.push((delta1, horizontal));
    compound_parts.push((delta2, vertical.clone()));
    compound_parts.push((delta3, vertical));

    let compound_shape = SharedShape::compound(compound_parts);

    let num = 6;
    let shift = (radx + 0.01) * 2.0;
    let centerx = shift * num as f32 / 2.0 - 0.5;
    let centery = shift / 2.0 + 4.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .translation(x, y)
                .linvel(100.0, -10.0)
                .ccd_enabled(true)
                .build();
            let handle = bodies.insert(rigid_body);

            // for part in &compound_parts {
            //     let collider = ColliderBuilder::new(part.1.clone())
            //         .position_wrt_parent(part.0)
            //         .build();
            //     colliders.insert(collider, handle, &mut bodies);
            // }

            let collider = ColliderBuilder::new(compound_shape.clone()).build();
            // let collider = ColliderBuilder::cuboid(radx, rady).build();
            colliders.insert(collider, handle, &mut bodies);
        }
    }

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
                if parent_handle1 != ground_handle && prox.collider1 != sensor_handle {
                    graphics.set_body_color(parent_handle1, color);
                }
                if parent_handle2 != ground_handle && prox.collider2 != sensor_handle {
                    graphics.set_body_color(parent_handle2, color);
                }
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 2.5), 20.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Balls", init_world)]);
    testbed.run()
}
