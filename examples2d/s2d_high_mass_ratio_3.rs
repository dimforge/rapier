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

    let extent = 1.0;
    let friction = 0.6;

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -2.0]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(40.0, 2.0).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    let rigid_body = RigidBodyBuilder::dynamic().translation(vector![-9.0 * extent, 0.5 * extent]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(0.5 * extent, 0.5 * extent).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::dynamic().translation(vector![9.0 * extent, 0.5 * extent]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(0.5 * extent, 0.5 * extent).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::dynamic().translation(vector![0.0, (10.0 + 16.0) * extent]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(10.0 * extent, 10.0 * extent).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
