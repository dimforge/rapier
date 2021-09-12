use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let rad = 0.5;

    let positions = [vector![5.0, -1.0], vector![-5.0, -1.0]];

    let platform_handles = std::array::IntoIter::new(positions)
        .map(|pos| {
            let rigid_body = RigidBodyBuilder::new_kinematic_position_based()
                .translation(pos)
                // .ccd_enabled(true)
                .build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad * 10.0, rad)
                .active_events(ActiveEvents::CONTACT_EVENTS)
                .build();
            let coll_handle = colliders.insert_with_parent(collider, handle, &mut bodies);
            (pos, handle, coll_handle)
        })
        .collect::<Vec<_>>();

    let mut rotation = 0.0;

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, events, state| {
        let rot = state.time * -1.0;
        let pos = state.time.sin() * 5.0;
        for (p, i, _) in platform_handles.iter() {
            let mut c = physics.bodies.get_mut(*i).unwrap();
            c.set_next_kinematic_rotation(rot);
            //c.set_next_kinematic_translation(vector![pos, 0.0]);
        }

        if state.timestep_id % 10 == 0 {
            let x = rand::random::<f32>() * 10.0 - 5.0;
            let y = rand::random::<f32>() * 10.0 + 10.0;
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .translation(vector![x, y])
                // .ccd_enabled(true)
                .build();
            let handle = physics.bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad).build();
            physics
                .colliders
                .insert_with_parent(collider, handle, &mut physics.bodies);

            if let Some(graphics) = &mut graphics {
                graphics.add_body(handle, &physics.bodies, &physics.colliders);
            }
        }

        // let to_remove: Vec<_> = physics
        //     .bodies
        //     .iter()
        //     .filter(|(_, b)| b.position().translation.vector.y < -10.0)
        //     .map(|e| e.0)
        //     .collect();
        // for handle in to_remove {
        //     physics.bodies.remove(
        //         handle,
        //         &mut physics.islands,
        //         &mut physics.colliders,
        //         &mut physics.joints,
        //     );
        //
        //     if let Some(graphics) = &mut graphics {
        //         graphics.remove_body(handle);
        //     }
        // }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![0.0, 0.0], 20.0);
}
