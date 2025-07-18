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

    // Build many balls, all spawned at the same point.
    let rad = 0.5;

    for _ in 0..100 {
        let rigid_body = RigidBodyBuilder::dynamic();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad);
        colliders.insert_with_parent(collider, handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 50.0);
}
