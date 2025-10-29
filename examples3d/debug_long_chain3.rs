use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let use_articulations = false;

    /*
     * Create the long chain.
     */
    let num = 100;
    let rad = 0.2;
    let shift = rad * 2.2;

    let mut body_handles = Vec::new();

    for i in 0..num {
        let fi = i as f32;

        let status = if i == 0 {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        };

        let rigid_body = RigidBodyBuilder::new(status).translation(vector![0.0, 0.0, fi * shift]);
        let child_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::ball(rad);
        colliders.insert_with_parent(collider, child_handle, &mut bodies);

        // Vertical joint.
        if i > 0 {
            let parent_handle = *body_handles.last().unwrap();
            let joint = if i == 1 {
                SphericalJointBuilder::new().local_anchor2(point![0.0, 0.0, -shift])
            } else {
                SphericalJointBuilder::new()
                    .local_anchor1(point![0.0, 0.0, shift / 2.0])
                    .local_anchor2(point![0.0, 0.0, -shift / 2.0])
            };

            if use_articulations {
                multibody_joints.insert(parent_handle, child_handle, joint, true);
            } else {
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }
        }

        body_handles.push(child_handle);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
