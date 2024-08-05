use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let scale = 1.0; // Set to a larger value to check if it still works with larger units.
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body =
        RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0] * scale);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(
        ground_size * scale,
        ground_height * scale,
        ground_size * scale,
    );
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    let rigid_body =
        RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, -ground_size] * scale); //.rotation(vector![-0.1, 0.0, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(
        ground_size * scale,
        ground_size * scale,
        ground_height * scale,
    );
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Character we will control manually.
     */
    let rigid_body =
        RigidBodyBuilder::kinematic_position_based().translation(vector![-3.0, 5.0, 0.0] * scale);
    let character_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::capsule_y(0.3 * scale, 0.15 * scale); // 0.15, 0.3, 0.15);
    colliders.insert_with_parent(collider, character_handle, &mut bodies);
    testbed.set_initial_body_color(character_handle, [0.8, 0.1, 0.1]);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = rad;

    for j in 0usize..4 {
        for k in 0usize..4 {
            for i in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift + centerx;

                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z] * scale);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad * scale, rad * scale, rad * scale);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Create some stairs.
     */
    let stair_width = 1.0;
    let stair_height = 0.1;
    for i in 0..10 {
        let x = i as f32 * stair_width / 2.0;
        let y = i as f32 * stair_height * 1.5 + 3.0;

        let collider = ColliderBuilder::cuboid(
            stair_width / 2.0 * scale,
            stair_height / 2.0 * scale,
            stair_width * scale,
        )
        .translation(vector![x * scale, y * scale, 0.0]);
        colliders.insert(collider);
    }

    /*
     * Create a slope we can climb.
     */
    let slope_angle = 0.2;
    let slope_size = 2.0;
    let collider = ColliderBuilder::cuboid(
        slope_size * scale,
        ground_height * scale,
        slope_size * scale,
    )
    .translation(vector![ground_size + slope_size, -ground_height + 0.4, 0.0] * scale)
    .rotation(Vector::z() * slope_angle);
    colliders.insert(collider);

    /*
     * Create a slope we can’t climb.
     */
    let impossible_slope_angle = 0.9;
    let impossible_slope_size = 2.0;
    let collider = ColliderBuilder::cuboid(
        slope_size * scale,
        ground_height * scale,
        ground_size * scale,
    )
    .translation(
        vector![
            ground_size + slope_size * 2.0 + impossible_slope_size - 0.9,
            -ground_height + 2.3,
            0.0
        ] * scale,
    )
    .rotation(Vector::z() * impossible_slope_angle);
    colliders.insert(collider);

    /*
     * Create a moving platform.
     */
    let body =
        RigidBodyBuilder::kinematic_velocity_based().translation(vector![-8.0, 1.5, 0.0] * scale);
    // .rotation(-0.3);
    let platform_handle = bodies.insert(body);
    let collider = ColliderBuilder::cuboid(2.0 * scale, ground_height * scale, 2.0 * scale);
    colliders.insert_with_parent(collider, platform_handle, &mut bodies);

    /*
     * More complex ground.
     */
    let ground_size = Vector::new(10.0, 1.0, 10.0);
    let nsubdivs = 20;

    let heights = DMatrix::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        (i as f32 * ground_size.x / (nsubdivs as f32) / 2.0).cos()
            + (j as f32 * ground_size.z / (nsubdivs as f32) / 2.0).cos()
    });

    let collider = ColliderBuilder::heightfield(heights, ground_size * scale)
        .translation(vector![-8.0, 5.0, 0.0] * scale);
    colliders.insert(collider);

    /*
     * A tilting dynamic body with a limited joint.
     */
    let ground = RigidBodyBuilder::fixed().translation(vector![0.0, 5.0, 0.0] * scale);
    let ground_handle = bodies.insert(ground);
    let body = RigidBodyBuilder::dynamic().translation(vector![0.0, 5.0, 0.0] * scale);
    let handle = bodies.insert(body);
    let collider = ColliderBuilder::cuboid(1.0 * scale, 0.1 * scale, 2.0 * scale);
    colliders.insert_with_parent(collider, handle, &mut bodies);
    let joint = RevoluteJointBuilder::new(Vector::z_axis()).limits([-0.3, 0.3]);
    impulse_joints.insert(ground_handle, handle, joint, true);

    /*
     * Setup a callback to move the platform.
     */
    testbed.add_callback(move |_, physics, _, run_state| {
        let linvel = vector![
            (run_state.time * 2.0).sin() * 2.0,
            (run_state.time * 5.0).sin() * 1.5,
            0.0
        ] * scale;
        // let angvel = run_state.time.sin() * 0.5;

        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(platform_handle) {
            platform.set_linvel(linvel, true);
            // NOTE: interaction with rotating platforms isn’t handled very well yet.
            // platform.set_angvel(angvel, true);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.set_character_body(character_handle);
    testbed.look_at(point!(10.0, 10.0, 10.0), Point::origin());
}
