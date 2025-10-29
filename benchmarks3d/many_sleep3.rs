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
     * Create the balls
     */
    let num = 50;
    let rad = 1.0;

    let shift = rad * 2.0 + 1.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let status = if j == 0 {
                    RigidBodyType::Fixed
                } else {
                    RigidBodyType::Dynamic
                };
                let density = 0.477;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new(status)
                    .translation(vector![x, y, z])
                    .sleeping(true); // j < num - 1);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::ball(rad).density(density);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
