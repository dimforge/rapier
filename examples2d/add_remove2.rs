use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let bodies = RigidBodySet::new();
    let colliders = ColliderSet::new();
    let joints = JointSet::new();
    let rad = 0.5;

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |graphics, physics, _, _| {
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![0.0, 10.0])
            .build();
        let handle = physics.bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad).build();
        physics
            .colliders
            .insert_with_parent(collider, handle, &mut physics.bodies);

        graphics.add_body(handle, &physics.bodies, &physics.colliders);

        let to_remove: Vec<_> = physics
            .bodies
            .iter()
            .filter(|(_, b)| b.position().translation.vector.y < -10.0)
            .map(|e| e.0)
            .collect();
        for handle in to_remove {
            physics.bodies.remove(
                handle,
                &mut physics.islands,
                &mut physics.colliders,
                &mut physics.joints,
            );

            graphics.remove_body(handle);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![0.0, 0.0], 20.0);
}
