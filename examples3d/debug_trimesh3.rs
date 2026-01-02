use kiss3d::color::LIGHT_GRAY;
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
    let width = 0.5;
    let vtx = vec![
        Vector::new(-width, 0.0, -width),
        Vector::new(width, 0.0, -width),
        Vector::new(width, 0.0, width),
        Vector::new(-width, 0.0, width),
        Vector::new(-width, -width, -width),
        Vector::new(width, -width, -width),
        Vector::new(width, -width, width),
        Vector::new(-width, -width, width),
    ];
    let idx = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 4, 7],
        [0, 7, 3],
        [1, 6, 5],
        [1, 2, 6],
        [3, 7, 2],
        [2, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
    ];

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 35.0, 0.0))
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(1.0, 2.0, 1.0);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0, 0.0));
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::trimesh(vtx, idx).expect("Could not create trimesh collider.");
    colliders.insert_with_parent(collider, handle, &mut bodies);
    testbed.set_initial_body_color(handle, LIGHT_GRAY);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
