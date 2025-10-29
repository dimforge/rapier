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

    let origin = vector![100_000.0, -80_000.0];
    let friction = 0.6;

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -1.0] + origin);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(100.0, 1.0).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    let base_count = 10;

    let h = 0.5;
    let shift = 1.25 * h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift + 0.5;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j as f32 - i as f32) * shift
                - h * base_count as f32;
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y] + origin);
            let ground_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(h, h).friction(friction);
            colliders.insert_with_parent(collider, ground_handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5] + origin, 20.0);
}
