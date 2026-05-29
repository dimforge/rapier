use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

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

    /*
     * A rectangle on a motor with target position.
     */
    for num in 0..9 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x_pos, 2.0))
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(0.1, 0.5);
        let (handle, _) = world.insert(rigid_body, collider);

        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(x_pos, 1.5))
            .local_anchor2(Vector::new(0.0, -0.5))
            .motor_position(
                -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32,
                1000.0,
                150.0,
            );
        world.insert_impulse_joint(ground_handle, handle, joint);
    }

    /*
     * A rectangle on a motor with limits.
     */
    for num in 0..8 {
        let x_pos = -6.0 + 1.5 * num as f32;
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x_pos, 4.5))
            .rotation(std::f32::consts::PI)
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(0.1, 0.5);
        let (handle, _) = world.insert(rigid_body, collider);

        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(x_pos, 5.0))
            .local_anchor2(Vector::new(0.0, -0.5))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits([
                -std::f32::consts::PI,
                -std::f32::consts::PI + std::f32::consts::PI / 4.0 * num as f32,
            ]);
        world.insert_impulse_joint(ground_handle, handle, joint);
    }

    /*
     * Set up the testbed.
     */
    world.gravity = Vector::new(0.0, 0.0);
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 40.0);
}
