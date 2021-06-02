use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    // Triangle ground.
    let vtx = [
        point![-10.0, 0.0, -10.0],
        point![10.0, 0.0, -10.0],
        point![0.0, 0.0, 10.0],
    ];

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(vector![0.0, 0.0, 0.0])
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::triangle(vtx[0], vtx[1], vtx[2]).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(vector![1.1, 0.01, 0.0])
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(20.0, 0.1, 1.0).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
