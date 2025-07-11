use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn create_ball_articulations(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    num: usize,
) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 {
                // && (k % 4 == 0 || k == num - 1) {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(vector![
                fk * shift,
                0.0,
                fi * shift * 2.0
            ]);
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::capsule_z(rad * 1.25, rad);
            colliders.insert_with_parent(collider, child_handle, bodies);

            // Vertical multibody_joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint =
                    SphericalJointBuilder::new().local_anchor2(point![0.0, 0.0, -shift * 2.0]);
                multibody_joints.insert(parent_handle, child_handle, joint, true);
            }

            // Horizontal multibody_joint.
            if k > 0 && i > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = SphericalJointBuilder::new().local_anchor2(point![-shift, 0.0, 0.0]);
                // let joint =
                //     PrismaticJoint::new(Vector::y_axis()).local_anchor2(point![-shift, 0.0, 0.0]);
                // let joint = FixedJoint::new().local_anchor2(point![-shift, 0.0, 0.0]);
                // let joint =
                //     RevoluteJoint::new(Vector::x_axis()).local_anchor2(point![-shift, 0.0, 0.0]);
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }

            body_handles.push(child_handle);
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let collider = ColliderBuilder::cuboid(30.0, 0.01, 30.0)
        .translation(vector![0.0, -3.02, 0.0])
        .rotation(vector![0.1, 0.0, 0.1]);
    colliders.insert(collider);

    let rigid_body = RigidBodyBuilder::dynamic();
    let collider = ColliderBuilder::cuboid(30.0, 0.01, 30.0)
        .translation(vector![0.0, -3.0, 0.0])
        .rotation(vector![0.1, 0.0, 0.1]);
    let handle = bodies.insert(rigid_body);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    create_ball_articulations(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        15,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![15.0, 5.0, 42.0], point![13.0, 1.0, 1.0]);
}
