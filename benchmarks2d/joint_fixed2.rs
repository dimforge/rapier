use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

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
                        RigidBodyType::Static
                    } else {
                        RigidBodyType::Dynamic
                    };

                    let rigid_body = RigidBodyBuilder::new(status)
                        .translation(vector![x + fk * shift, y - fi * shift])
                        .build();
                    let child_handle = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::ball(rad).build();
                    colliders.insert_with_parent(collider, child_handle, &mut bodies);

                    // Vertical joint.
                    if i > 0 {
                        let parent_handle = *body_handles.last().unwrap();
                        let joint = FixedJoint::new(
                            Isometry::identity(),
                            Isometry::translation(0.0, shift),
                        );
                        joints.insert(parent_handle, child_handle, joint);
                    }

                    // Horizontal joint.
                    if k > 0 {
                        let parent_index = body_handles.len() - num;
                        let parent_handle = body_handles[parent_index];
                        let joint = FixedJoint::new(
                            Isometry::identity(),
                            Isometry::translation(-shift, 0.0),
                        );
                        joints.insert(parent_handle, child_handle, joint);
                    }

                    body_handles.push(child_handle);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![50.0, 50.0], 5.0);
}
