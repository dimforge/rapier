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
        let ball_pos = point![x_pos, 4.5, 0.0];
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(ball_pos.coords)
            .can_sleep(false);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::ball(radius);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        let damping_ratio = i as f32 / (num as f32 / 2.0);
        let damping = damping_ratio * critical_damping;
        let joint = SpringJointBuilder::new(0.0, stiffness, damping)
            .local_anchor1(ball_pos - Vector::y() * 3.0);
        impulse_joints.insert(ground_handle, handle, joint, true);

        // Box that will fall on to of the springed balls, makes the simulation funier to watch.
        let rigid_body =
            RigidBodyBuilder::dynamic().translation(ball_pos.coords + Vector::y() * 5.0);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(radius, radius, radius).density(100.0);
        colliders.insert_with_parent(collider, handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        vector![0.0, -9.81, 0.0],
        (),
    );
    testbed.look_at(point![15.0, 5.0, 42.0], point![13.0, 1.0, 1.0]);
}
