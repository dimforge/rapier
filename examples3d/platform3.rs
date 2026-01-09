use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

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

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centerz = shift * num as f32 / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let centery = if j >= num / 2 { 5.0 } else { 3.0 };
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body =
        RigidBodyBuilder::kinematic_velocity_based().translation(Vector::new(0.0, 1.5 + 0.8, 0.0));
    let velocity_based_platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad, rad * 10.0);
    colliders.insert_with_parent(collider, velocity_based_platform_handle, &mut bodies);

    /*
     * Setup a position-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_position_based().translation(Vector::new(
        0.0,
        3.0 + 1.5 + 0.8,
        0.0,
    ));
    let position_based_platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad, rad * 10.0);
    colliders.insert_with_parent(collider, position_based_platform_handle, &mut bodies);

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, physics, _, run_state| {
        let velocity = Vector::new(
            0.0,
            (run_state.time * 2.0).cos(),
            run_state.time.sin() * 2.0,
        );

        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(velocity_based_platform_handle) {
            platform.set_linvel(velocity, true);
            platform.set_angvel(Vector::new(0.0, 1.0, 0.0), true);
        }

        // Update the position-based kinematic body by setting its next position.
        if let Some(platform) = physics.bodies.get_mut(position_based_platform_handle) {
            let next_tra = platform.translation() + (-velocity * physics.integration_parameters.dt);
            let next_rot = platform.rotation();
            let delta_rot = Rotation::from_rotation_y(-0.5 * physics.integration_parameters.dt);
            let next_rot = delta_rot * next_rot;
            platform.set_next_kinematic_translation(next_tra);
            platform.set_next_kinematic_rotation(next_rot);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(10.0, 5.0, 10.0), Vec3::ZERO);
}
