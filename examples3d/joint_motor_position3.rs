use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Fixed ground to attach one end of the joints.
     */
    let rigid_body = RigidBodyBuilder::fixed();
    let ground_handle = world.insert_body(rigid_body);
    let mut target_angles = vec![];

    /*
     * A rectangle on a motor with target position.
     */
    for num in 0..9 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x_pos, 2.0, 0.0))
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(0.1, 0.5, 0.1);
        let (handle, _) = world.insert(rigid_body, collider);

        let target_angle = -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(Vector::new(x_pos, 1.5, 0.0))
            .local_anchor2(Vector::new(0.0, -0.5, 0.0))
            .motor_position(target_angle, 1000.0, 150.0);
        world.insert_impulse_joint(ground_handle, handle, joint);
        target_angles.push(target_angle);
    }

    /*
     * A rectangle on a motor with limits.
     */
    for num in 0..8 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x_pos, 4.5, 0.0))
            .rotation(Vector::new(0.0, 0.0, std::f32::consts::PI))
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(0.1, 0.5, 0.1);
        let (handle, _) = world.insert(rigid_body, collider);

        let max_angle_limit = -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32;
        let joint = RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(Vector::new(x_pos, 5.0, 0.0))
            .local_anchor2(Vector::new(0.0, -0.5, 0.0))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits([-std::f32::consts::PI, max_angle_limit]);
        world.insert_impulse_joint(ground_handle, handle, joint);
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
    world.gravity = Vector::new(0.0, 0.0, 0.0);
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(15.0, 5.0, 42.0), Vec3::new(13.0, 1.0, 1.0));
}
