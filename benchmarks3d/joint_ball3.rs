use na::Point3;
use rapier3d::dynamics::{BallJoint, BodyStatus, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    let rad = 0.4;
    let num = 100;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status)
                .translation(fk * shift, 0.0, fi * shift)
                .build();
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad).build();
            colliders.insert(collider, child_handle, &mut bodies);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift));
                joints.insert(&mut bodies, parent_handle, child_handle, joint);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
                joints.insert(&mut bodies, parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(
        Point3::new(-110.0, -46.0, 170.0),
        Point3::new(54.0, -38.0, 29.0),
    );
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
