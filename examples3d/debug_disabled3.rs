use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 10.1;
    let ground_height = 2.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Platform that will be enabled/disabled.
     */
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0));
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(5.0, 1.0, 5.0);
    let handle_to_disable = colliders.insert_with_parent(collider, handle, &mut bodies);

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut graphics, physics, _, run_state| {
        if run_state.timestep_id % 250 == 0 {
            let co = &mut physics.colliders[handle_to_disable];
            let enabled = co.is_enabled();
            co.set_enabled(!enabled);
            println!("Platform is now enabled: {}", co.is_enabled());
        }

        if run_state.timestep_id % 25 == 0 {
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 20.0, 0.0));
            let handle = physics.bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            physics
                .colliders
                .insert_with_parent(collider, handle, &mut physics.bodies);

            if let Some(graphics) = &mut graphics {
                graphics.add_body(handle, &physics.bodies, &physics.colliders);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(30.0, 4.0, 30.0), Vec3::new(0.0, 1.0, 0.0));
}
