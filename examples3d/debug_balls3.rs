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
    let num_j = 10;
    let num_ik = 10;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num_ik as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num_ik as f32) / 2.0;

    for i in 0..num_ik {
        for j in 0usize..num_j {
            for k in 0..num_ik {
                let x = i as f32 * shift - centerx;
                let z = k as f32 * shift - centerz;

                let status = if j == 0 || i == 0 || k == 0 || i == num_ik - 1 || k == num_ik - 1 {
                    RigidBodyType::Fixed
                } else {
                    RigidBodyType::Dynamic
                };

                let y = if status.is_fixed() {
                    j as f32 * shift + centery
                } else {
                    j as f32 * shift * 2.0 + centery
                };

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new(status)
                    .translation(vector![x, y, z])
                    .can_sleep(false);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::ball(rad).friction(0.0);
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
