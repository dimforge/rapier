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
    let ground_size = 100.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).friction(1.5);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    // Build a dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 1.1, 0.0])
        .rotation(Vector::y() * 0.3);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(2.0, 1.0, 3.0).friction(1.5);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = &mut bodies[handle];
    let force = rigid_body.position() * Vector::z() * 50.0;
    rigid_body.set_linvel(force, true);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
