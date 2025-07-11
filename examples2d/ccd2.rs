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
     * Ground
     */
    let ground_size = 25.0;
    let ground_thickness = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().ccd_enabled(true);
    let ground_handle = bodies.insert(rigid_body);

    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    let collider =
        ColliderBuilder::cuboid(ground_thickness, ground_size).translation(vector![-3.0, 0.0]);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    let collider =
        ColliderBuilder::cuboid(ground_thickness, ground_size).translation(vector![6.0, 0.0]);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    let collider = ColliderBuilder::cuboid(ground_thickness, ground_size)
        .translation(vector![2.5, 0.0])
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    let sensor_handle = colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the shapes
     */
    let radx = 0.4;
    let rady = 0.05;

    let delta1 = Isometry::translation(0.0, radx - rady);
    let delta2 = Isometry::translation(-radx + rady, 0.0);
    let delta3 = Isometry::translation(radx - rady, 0.0);

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
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![x, y])
                .linvel(vector![100.0, -10.0])
                .ccd_enabled(true);
            let handle = bodies.insert(rigid_body);

            // for part in &compound_parts {
            //     let collider = ColliderBuilder::new(part.1.clone())
            //         .position_wrt_parent(part.0)
            //         ;
            //     colliders.insert_with_parent(collider, handle, &mut bodies);
            // }

            let collider = ColliderBuilder::new(compound_shape.clone());
            // let collider = ColliderBuilder::cuboid(radx, rady);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, events, _| {
        while let Ok(prox) = events.collision_events.try_recv() {
            let color = if prox.started() {
                [1.0, 1.0, 0.0]
            } else {
                [0.5, 0.5, 1.0]
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
                if parent_handle1 != ground_handle && prox.collider1() != sensor_handle {
                    graphics.set_body_color(parent_handle1, color);
                }

                if parent_handle2 != ground_handle && prox.collider2() != sensor_handle {
                    graphics.set_body_color(parent_handle2, color);
                }
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
