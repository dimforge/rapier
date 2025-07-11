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
    let ground_size = 10.0;
    let ground_height = 10.0;

    for _ in 0..1 {
        let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        colliders.insert_with_parent(collider, handle, &mut bodies);
    }

    // Build the dynamic box rigid body.
    for _ in 0..1 {
        let rigid_body = RigidBodyBuilder::dynamic()
            // .translation(vector![0.0, 0.1, 0.0])
            // .rotation(vector![0.8, 0.2, 0.1])
            .can_sleep(false);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
        colliders.insert_with_parent(collider.clone(), handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
