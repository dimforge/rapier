use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    // Halfspace ground.
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -1.0, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::halfspace(Vector::y_axis());
    colliders.insert_with_parent(collider, handle, &mut bodies);

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.01, 10.0, 0.01])
        .rotation(vector![1.0, 0.0, 1.0])
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![24.0, 24.0, 24.0], Point::origin());
}