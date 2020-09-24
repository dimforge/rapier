use na::Point2;
use rapier2d::dynamics::{BallJoint, BodyStatus, JointSet, RigidBodyBuilder, RigidBodySet};
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
    let numi = 100; // Num vertical nodes.
    let numk = 100; // Num horizontal nodes.
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let status = if k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0 {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status)
                .translation(fk * shift, -fi * shift)
                .build();
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::new_ball(rad).density(1.0).build();
            colliders.insert(&mut bodies, collider, child_handle);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = BallJoint::new(Point2::origin(), Point2::new(0.0, shift));
                joints.insert(&mut bodies, joint, parent_handle, child_handle);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - numi;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point2::origin(), Point2::new(-shift, 0.0));
                joints.insert(&mut bodies, joint, parent_handle, child_handle);
            }

            body_handles.push(child_handle);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(numk as f32 * rad, numi as f32 * -rad), 5.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
