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
    let num = 30;
    let rad = 1.0;

    let shift = rad * 6.0 + 1.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift - centery;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let velocity = Vector::new(
                    rand::random::<f32>() - 0.5,
                    rand::random::<f32>() - 0.5,
                    rand::random::<f32>() - 0.5,
                ) * 30.0;
                let rigid_body = RigidBodyBuilder::new(RigidBodyType::KinematicVelocityBased)
                    .translation(vector![x, y, z])
                    .linvel(velocity);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::ball(rad);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    testbed.add_callback(move |_, physics, _, _| {
        for (_, rb) in physics.bodies.iter_mut() {
            let mut linvel = *rb.linvel();

            for dim in 0..3 {
                if (linvel[dim] > 0.0 && rb.translation()[dim] > (shift * num as f32) / 2.0)
                    || (linvel[dim] < 0.0 && rb.translation()[dim] < -(shift * num as f32) / 2.0)
                {
                    linvel[dim] = -linvel[dim];
                }
            }

            rb.set_linvel(linvel, false);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
