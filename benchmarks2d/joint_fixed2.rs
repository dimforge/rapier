use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create the balls
     */
    // Build the rigid body.
    let rad = 0.4;
    let num = 30; // Num vertical nodes.
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for xx in 0..4 {
        let x = xx as f32 * shift * (num as f32 + 2.0);

        for yy in 0..4 {
            let y = yy as f32 * shift * (num as f32 + 4.0);

            for k in 0..num {
                for i in 0..num {
                    let fk = k as f32;
                    let fi = i as f32;

                    let status = if k == 0 {
                        RigidBodyType::Fixed
                    } else {
                        RigidBodyType::Dynamic
                    };

                    let rigid_body = RigidBodyBuilder::new(status)
                        .translation(vector![x + fk * shift, y - fi * shift]);
                    let child_handle = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::ball(rad);
                    colliders.insert_with_parent(collider, child_handle, &mut bodies);

                    // Vertical joint.
                    if i > 0 {
                        let parent_handle = *body_handles.last().unwrap();
                        let joint = FixedJointBuilder::new()
                            .local_frame2(Isometry::translation(0.0, shift));
                        impulse_joints.insert(parent_handle, child_handle, joint, true);
                    }

                    // Horizontal joint.
                    if k > 0 {
                        let parent_index = body_handles.len() - num;
                        let parent_handle = body_handles[parent_index];
                        let joint = FixedJointBuilder::new()
                            .local_frame2(Isometry::translation(-shift, 0.0));
                        impulse_joints.insert(parent_handle, child_handle, joint, true);
                    }

                    body_handles.push(child_handle);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![50.0, 50.0], 5.0);
}
