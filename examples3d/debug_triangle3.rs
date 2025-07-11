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

    // Triangle ground.
    let vtx = [
        point![-10.0, 0.0, -10.0],
        point![10.0, 0.0, -10.0],
        point![0.0, 0.0, 10.0],
    ];

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, 0.0, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::triangle(vtx[0], vtx[1], vtx[2]);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![1.1, 0.01, 0.0])
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(20.0, 0.1, 1.0);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
