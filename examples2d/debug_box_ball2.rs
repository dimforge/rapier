use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

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
    let rad = 1.0;
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -rad])
        .rotation(std::f32::consts::PI / 4.0);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(rad, rad);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 3.0 * rad])
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 50.0);
}
