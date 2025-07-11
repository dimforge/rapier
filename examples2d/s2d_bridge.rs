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
    let density = 20.0;
    let x_base = -80.0;
    let count = 160;
    let mut prev = ground;

    for i in 0..count {
        let rigid_body = RigidBodyBuilder::dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation(vector![x_base + 0.5 + 1.0 * i as f32, 20.0]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(0.5, 0.125).density(density);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        let pivot = point![x_base + 1.0 * i as f32, 20.0];
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(bodies[prev].position().inverse_transform_point(&pivot))
            .local_anchor2(bodies[handle].position().inverse_transform_point(&pivot))
            .contacts_enabled(false);
        impulse_joints.insert(prev, handle, joint, true);
        prev = handle;
    }

    let pivot = point![x_base + 1.0 * count as f32, 20.0];
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(bodies[prev].position().inverse_transform_point(&pivot))
        .local_anchor2(bodies[ground].position().inverse_transform_point(&pivot))
        .contacts_enabled(false);
    impulse_joints.insert(prev, ground, joint, true);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
