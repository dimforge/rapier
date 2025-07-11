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

    let num = 80;
    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 1.0;
    let ground_thickness = 1.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness).friction(0.3);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */

    for i in 0..num {
        let y = i as f32 * rad * 2.0 + ground_thickness + rad;

        // Build the rigid body.
        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![0.0, y]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad).friction(0.3);
        colliders.insert_with_parent(collider, handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    // testbed.harness_mut().physics.gravity.y = -981.0;
    testbed.look_at(point![0.0, 2.5], 5.0);
}
