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
     * Create the cubes
     */
    let num = 10;
    let rad = 0.2;

    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * std::f32::consts::PI * 2.0).sin_cos();

        // Build the rigid body.
        let rb = RigidBodyBuilder::dynamic()
            .translation(vector![x, y, 0.0])
            .linvel(vector![x * 10.0, y * 10.0, 0.0])
            .angvel(Vector::z() * 100.0)
            .linear_damping((i + 1) as f32 * subdiv * 10.0)
            .angular_damping((num - i) as f32 * subdiv * 10.0);
        let rb_handle = bodies.insert(rb);

        // Build the collider.
        let co = ColliderBuilder::cuboid(rad, rad, rad);
        colliders.insert_with_parent(co, rb_handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        Vector::zeros(),
        (),
    );
    testbed.look_at(point![2.0, 2.5, 20.0], point![2.0, 2.5, 0.0]);
}
