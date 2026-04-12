use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Create the joint grid.
     */
    let rad = 0.4;
    let numi = 100;
    let numk = 100;
    let shift = 1.0;
    let mut index = 0;
    let mut handles = vec![RigidBodyHandle::invalid(); numi * numk];

    for k in 0..numk {
        for i in 0..numi {
            let body_type = if k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0 {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(body_type)
                .translation(Vector::new(k as f32 * shift, -(i as f32) * shift));
            let collider = ColliderBuilder::ball(rad);
            let (handle, _) = world.insert(rigid_body, collider);

            if i > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, -0.5 * shift))
                    .local_anchor2(Vector::new(0.0, 0.5 * shift))
                    .contacts_enabled(false);
                world.insert_impulse_joint(handles[index - 1], handle, joint);
            }

            if k > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(0.5 * shift, 0.0))
                    .local_anchor2(Vector::new(-0.5 * shift, 0.0))
                    .contacts_enabled(false);
                world.insert_impulse_joint(handles[index - numi], handle, joint);
            }

            handles[index] = handle;
            index += 1;
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
