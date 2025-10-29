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
     * Ground
     */
    let ground = bodies.insert(RigidBodyBuilder::fixed());

    /*
     * Create the bridge.
     */
    let count = 40;
    let hx = 0.5;
    let density = 20.0;
    let friction = 0.6;
    let capsule = ColliderBuilder::capsule_x(hx, 0.125)
        .friction(friction)
        .density(density);

    let mut prev = ground;
    for i in 0..count {
        let rigid_body = RigidBodyBuilder::dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation(vector![(1.0 + 2.0 * i as f32) * hx, count as f32 * hx]);
        let handle = bodies.insert(rigid_body);
        colliders.insert_with_parent(capsule.clone(), handle, &mut bodies);

        let pivot = point![(2.0 * i as f32) * hx, count as f32 * hx];
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(bodies[prev].position().inverse_transform_point(&pivot))
            .local_anchor2(bodies[handle].position().inverse_transform_point(&pivot))
            .contacts_enabled(false);
        impulse_joints.insert(prev, handle, joint, true);
        prev = handle;
    }

    let radius = 8.0;
    let rigid_body = RigidBodyBuilder::dynamic()
        .linear_damping(0.1)
        .angular_damping(0.1)
        .translation(vector![
            (1.0 + 2.0 * count as f32) * hx + radius - hx,
            count as f32 * hx
        ]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(radius)
        .friction(friction)
        .density(density);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let pivot = point![(2.0 * count as f32) * hx, count as f32 * hx];
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(bodies[prev].position().inverse_transform_point(&pivot))
        .local_anchor2(bodies[handle].position().inverse_transform_point(&pivot))
        .contacts_enabled(false);
    impulse_joints.insert(prev, handle, joint, true);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
