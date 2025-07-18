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
    let ground_size = 10.0;
    let ground_thickness = 1.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 6;
    let mut rad = 0.5;
    let mut y = rad;

    for _ in 0usize..num {
        // Build the rigid body.
        let rigid_body =
            RigidBodyBuilder::dynamic().translation(vector![0.0, y + ground_thickness]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad);
        colliders.insert_with_parent(collider, handle, &mut bodies);
        y += rad + rad * 2.0;
        rad *= 2.0;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
