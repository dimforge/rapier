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
     * Ground
     */
    let ground_size = 100.1;
    let ground_height = 2.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, 4.0, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rad = 1.0;
    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 7.0 * rad, 0.0])
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 2.0 * rad, 0.0])
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.look_at(point![100.0, -10.0, 100.0], Point::origin());
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
}
