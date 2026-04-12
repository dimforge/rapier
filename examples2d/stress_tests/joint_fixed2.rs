use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

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
                        .translation(Vec2::new(x + fk * shift, y - fi * shift));
                    let collider = ColliderBuilder::ball(rad);
                    let (child_handle, _) = world.insert(rigid_body, collider);

                    // Vertical joint.
                    if i > 0 {
                        let parent_handle = *body_handles.last().unwrap();
                        let joint =
                            FixedJointBuilder::new().local_frame2(Pose2::translation(0.0, shift));
                        world.insert_impulse_joint(parent_handle, child_handle, joint);
                    }

                    // Horizontal joint.
                    if k > 0 {
                        let parent_index = body_handles.len() - num;
                        let parent_handle = body_handles[parent_index];
                        let joint =
                            FixedJointBuilder::new().local_frame2(Pose2::translation(-shift, 0.0));
                        world.insert_impulse_joint(parent_handle, child_handle, joint);
                    }

                    body_handles.push(child_handle);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(50.0, 50.0), 5.0);
}
