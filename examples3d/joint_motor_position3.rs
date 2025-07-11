use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Fixed ground to attach one end of the joints.
     */
    let rigid_body = RigidBodyBuilder::fixed();
    let ground_handle = bodies.insert(rigid_body);
    let mut target_angles = vec![];

    /*
     * A rectangle on a motor with target position.
     */
    for num in 0..9 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x_pos, 2.0, 0.0])
            .can_sleep(false);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(0.1, 0.5, 0.1);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        let target_angle = -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vector::z_axis())
            .local_anchor1(point![x_pos, 1.5, 0.0])
            .local_anchor2(point![0.0, -0.5, 0.0])
            .motor_position(target_angle, 1000.0, 150.0);
        impulse_joints.insert(ground_handle, handle, joint, true);
        target_angles.push(target_angle);
    }

    /*
     * A rectangle on a motor with limits.
     */
    for num in 0..8 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x_pos, 4.5, 0.0])
            .rotation(vector![0.0, 0.0, std::f32::consts::PI])
            .can_sleep(false);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(0.1, 0.5, 0.1);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        let max_angle_limit = -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vector::z_axis())
            .local_anchor1(point![x_pos, 5.0, 0.0])
            .local_anchor2(point![0.0, -0.5, 0.0])
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits([-std::f32::consts::PI, max_angle_limit]);
        impulse_joints.insert(ground_handle, handle, joint, true);
        target_angles.push(max_angle_limit);
    }

    testbed.add_callback(move |_, physics, _, state| {
        for ((_, joint), target) in physics.impulse_joints.iter().zip(target_angles.iter()) {
            let rb1 = &physics.bodies[joint.body1];
            let rb2 = &physics.bodies[joint.body2];
            let revolute = joint.data.as_revolute().unwrap();
            println!(
                "[Step {}] rev angle: {} (target = {})",
                state.timestep_id,
                revolute.angle(rb1.rotation(), rb2.rotation()),
                target
            );
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        vector![0.0, 0.0, 0.0],
        (),
    );
    testbed.look_at(point![15.0, 5.0, 42.0], point![13.0, 1.0, 1.0]);
}
