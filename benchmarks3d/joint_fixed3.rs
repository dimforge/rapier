use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    let rad = 0.4;
    let num = 5;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for m in 0..10 {
        let z = m as f32 * shift * (num as f32 + 2.0);

        for l in 0..10 {
            let y = l as f32 * shift * 3.0;

            for j in 0..5 {
                let x = j as f32 * shift * (num as f32) * 2.0;

                for k in 0..num {
                    for i in 0..num {
                        let fk = k as f32;
                        let fi = i as f32;

                        // NOTE: the num - 2 test is to avoid two consecutive
                        // fixed bodies. Because physx will crash if we add
                        // a joint between these.

                        let status = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                            RigidBodyType::Static
                        } else {
                            RigidBodyType::Dynamic
                        };

                        let rigid_body = RigidBodyBuilder::new(status)
                            .translation(vector![x + fk * shift, y, z + fi * shift])
                            .build();
                        let child_handle = bodies.insert(rigid_body);
                        let collider = ColliderBuilder::ball(rad).build();
                        colliders.insert_with_parent(collider, child_handle, &mut bodies);

                        // Vertical joint.
                        if i > 0 {
                            let parent_handle = *body_handles.last().unwrap();
                            let joint = FixedJoint::new(
                                Isometry::identity(),
                                Isometry::translation(0.0, 0.0, -shift),
                            );
                            joints.insert(parent_handle, child_handle, joint);
                        }

                        // Horizontal joint.
                        if k > 0 {
                            let parent_index = body_handles.len() - num;
                            let parent_handle = body_handles[parent_index];
                            let joint = FixedJoint::new(
                                Isometry::identity(),
                                Isometry::translation(-shift, 0.0, 0.0),
                            );
                            joints.insert(parent_handle, child_handle, joint);
                        }

                        body_handles.push(child_handle);
                    }
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![-38.0, 14.0, 108.0], point![46.0, 12.0, 23.0]);
}
