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

    let num_levels = 4;
    let stick_len = 2.0;
    let stick_rad = 0.2;

    /*
     * Floor.
     */
    let floor_body =
        RigidBodyBuilder::fixed().translation(vector![0.0, -stick_len - stick_rad, 0.0]);
    let floor_handle = bodies.insert(floor_body);
    let floor_cube = ColliderBuilder::cuboid(stick_len, stick_len, stick_len);
    colliders.insert_with_parent(floor_cube, floor_handle, &mut bodies);

    /*
     * Create a stack of capsule with a very heavy cube on top.
     */
    for i in 0..num_levels {
        let fi = i as f32;

        let body = RigidBodyBuilder::dynamic().translation(vector![
            0.0,
            fi * stick_rad * 4.0,
            -(stick_len / 2.0 - stick_rad)
        ]);
        let handle = bodies.insert(body);
        let capsule = ColliderBuilder::cuboid(stick_len / 2.0, stick_rad, stick_rad);
        colliders.insert_with_parent(capsule, handle, &mut bodies);

        let body = RigidBodyBuilder::dynamic().translation(vector![
            0.0,
            fi * stick_rad * 4.0,
            (stick_len / 2.0 - stick_rad)
        ]);
        let handle = bodies.insert(body);
        let capsule = ColliderBuilder::cuboid(stick_len / 2.0, stick_rad, stick_rad);
        colliders.insert_with_parent(capsule, handle, &mut bodies);

        let body = RigidBodyBuilder::dynamic().translation(vector![
            -(stick_len / 2.0 - stick_rad),
            (fi + 0.5) * stick_rad * 4.0,
            0.0
        ]);
        let handle = bodies.insert(body);
        let capsule = ColliderBuilder::cuboid(stick_rad, stick_rad, stick_len / 2.0);
        colliders.insert_with_parent(capsule, handle, &mut bodies);

        let body = RigidBodyBuilder::dynamic().translation(vector![
            (stick_len / 2.0 - stick_rad),
            (fi + 0.5) * stick_rad * 4.0,
            0.0
        ]);
        let handle = bodies.insert(body);
        let capsule = ColliderBuilder::cuboid(stick_rad, stick_rad, stick_len / 2.0);
        colliders.insert_with_parent(capsule, handle, &mut bodies);
    }

    /*
     * Big cube on top.
     */
    let cube_len = stick_len * 2.0;
    let floor_body = RigidBodyBuilder::dynamic()
        .translation(vector![
            0.0,
            cube_len / 2.0 + (num_levels as f32 - 0.25) * stick_rad * 4.0,
            0.0
        ])
        .additional_solver_iterations(36);
    let floor_handle = bodies.insert(floor_body);
    let floor_cube = ColliderBuilder::cuboid(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);
    colliders.insert_with_parent(floor_cube, floor_handle, &mut bodies);

    let small_mass =
        MassProperties::from_cuboid(1.0, vector![stick_rad, stick_rad, stick_len / 2.0]).mass();
    let big_mass =
        MassProperties::from_cuboid(1.0, vector![cube_len / 2.0, cube_len / 2.0, cube_len / 2.0])
            .mass();
    println!(
        "debug_cube_high_mass_ratio3: small stick mass: {small_mass}, big cube mass: {big_mass}, mass_ratio: {}",
        big_mass / small_mass
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
