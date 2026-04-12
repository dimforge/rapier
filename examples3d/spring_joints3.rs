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

    /*
     * Spring joints with a variety of spring parameters.
     * The middle one has uses critical damping.
     */
    let num = 30;
    let radius = 0.5;
    let mass = Ball::new(radius).mass_properties(1.0).mass();
    let stiffness = 1.0e3;
    let critical_damping = 2.0 * (stiffness * mass).sqrt();
    for i in 0..=num {
        let x_pos = -6.0 + 1.5 * i as f32;
        let ball_pos = Vector::new(x_pos, 4.5, 0.0);
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(ball_pos)
            .can_sleep(false);
        let collider = ColliderBuilder::ball(radius);
        let (handle, _) = world.insert(rigid_body, collider);

        let damping_ratio = i as f32 / (num as f32 / 2.0);
        let damping = damping_ratio * critical_damping;
        let joint = SpringJointBuilder::new(0.0, stiffness, damping)
            .local_anchor1(ball_pos - Vector::Y * 3.0);
        world.insert_impulse_joint(ground_handle, handle, joint);

        // Box that will fall on to of the springed balls, makes the simulation funnier to watch.
        let rigid_body = RigidBodyBuilder::dynamic().translation(ball_pos + Vector::Y * 5.0);
        let collider = ColliderBuilder::cuboid(radius, radius, radius).density(100.0);
        world.insert(rigid_body, collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(15.0, 5.0, 42.0), Vec3::new(13.0, 1.0, 1.0));
}
