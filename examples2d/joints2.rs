use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Enable/disable softness.
     */
    let settings = testbed.example_settings_mut();
    let variable_softness = settings.get_or_set_bool("Variable softness", false);

    /*
     * Create the balls
     */
    // Build the rigid body.
    // NOTE: a smaller radius (e.g. 0.1) breaks Box2D so
    // in order to be able to compare rapier with Box2D,
    // we set it to 0.4.
    let rad = 0.4;
    let numi = 10; // Num vertical nodes.
    let numk = 10; // Num horizontal nodes.
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && k == 0 {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body =
                RigidBodyBuilder::new(status).translation(Vector::new(fk * shift, -fi * shift));
            let collider = ColliderBuilder::ball(rad);
            let (child_handle, _) = world.insert(rigid_body, collider);

            let softness = if variable_softness {
                // If variable softness is enabled, joints closer to the fixed body are softer.
                SpringCoefficients {
                    natural_frequency: 5.0 * (i.max(k) + 1) as f32,
                    damping_ratio: 0.1 * (i.max(k) + 1) as f32,
                }
            } else {
                SpringCoefficients::joint_defaults()
            };

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = RevoluteJointBuilder::new()
                    .local_anchor2(Vector::new(0.0, shift))
                    .softness(softness);
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - numi;
                let parent_handle = body_handles[parent_index];
                let joint = RevoluteJointBuilder::new()
                    .local_anchor2(Vector::new(-shift, 0.0))
                    .softness(softness);
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(numk as f32 * rad, numi as f32 * -rad), 20.0);
}
