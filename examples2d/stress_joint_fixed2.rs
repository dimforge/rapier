use na::{Isometry2, Point2};
use rapier2d::dynamics::{BodyStatus, FixedJoint, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
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
                        BodyStatus::Static
                    } else {
                        BodyStatus::Dynamic
                    };

                    let rigid_body = RigidBodyBuilder::new(status)
                        .translation(x + fk * shift, y - fi * shift)
                        .build();
                    let child_handle = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::ball(rad).density(1.0).build();
                    colliders.insert(collider, child_handle, &mut bodies);

                    // Vertical joint.
                    if i > 0 {
                        let parent_handle = *body_handles.last().unwrap();
                        let joint = FixedJoint::new(
                            Isometry2::identity(),
                            Isometry2::translation(0.0, shift),
                        );
                        joints.insert(&mut bodies, parent_handle, child_handle, joint);
                    }

                    // Horizontal joint.
                    if k > 0 {
                        let parent_index = body_handles.len() - num;
                        let parent_handle = body_handles[parent_index];
                        let joint = FixedJoint::new(
                            Isometry2::identity(),
                            Isometry2::translation(-shift, 0.0),
                        );
                        joints.insert(&mut bodies, parent_handle, child_handle, joint);
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
    testbed.look_at(Point2::new(50.0, 50.0), 5.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
