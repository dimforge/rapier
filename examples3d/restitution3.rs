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
    let ground_size = 20.;
    let ground_height = 1.0;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 2.0).restitution(1.0);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let num = 10;
    let rad = 0.5;

    for j in 0..2 {
        for i in 0..=num {
            let x = (i as f32) - num as f32 / 2.0;
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![
                x * 2.0,
                10.0 * (j as f32 + 1.0),
                0.0
            ]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad).restitution((i as f32) / (num as f32));
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 3.0, 30.0], point![0.0, 3.0, 0.0]);
}
