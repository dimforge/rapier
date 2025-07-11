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
    let ground_size = 10.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift / 2.0 + 3.04;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    /*
     * Setup a position-based kinematic rigid body.
     */
    let platform_body =
        RigidBodyBuilder::kinematic_velocity_based().translation(vector![-10.0 * rad, 1.5 + 0.8]);
    let velocity_based_platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
    colliders.insert_with_parent(collider, velocity_based_platform_handle, &mut bodies);

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_position_based()
        .translation(vector![-10.0 * rad, 2.0 + 1.5 + 0.8]);
    let position_based_platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
    colliders.insert_with_parent(collider, position_based_platform_handle, &mut bodies);

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, physics, _, run_state| {
        let velocity = vector![run_state.time.sin() * 5.0, (run_state.time * 5.0).sin()];

        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(velocity_based_platform_handle) {
            platform.set_linvel(velocity, true);
        }

        // Update the position-based kinematic body by setting its next position.
        if let Some(platform) = physics.bodies.get_mut(position_based_platform_handle) {
            let mut next_tra = *platform.translation();
            next_tra += velocity * physics.integration_parameters.dt;
            platform.set_next_kinematic_translation(next_tra);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 1.0], 40.0);
}
