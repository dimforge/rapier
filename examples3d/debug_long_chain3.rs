use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
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

        let rigid_body =
            RigidBodyBuilder::new(status).translation(Vector::new(0.0, 0.0, fi * shift));
        let collider = ColliderBuilder::ball(rad);
        let (child_handle, _) = world.insert(rigid_body, collider);

        // Vertical joint.
        if i > 0 {
            let parent_handle = *body_handles.last().unwrap();
            let joint = if i == 1 {
                SphericalJointBuilder::new().local_anchor2(Vector::new(0.0, 0.0, -shift))
            } else {
                SphericalJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, 0.0, shift / 2.0))
                    .local_anchor2(Vector::new(0.0, 0.0, -shift / 2.0))
            };

            if use_articulations {
                world.insert_multibody_joint(parent_handle, child_handle, joint);
            } else {
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }
        }

        body_handles.push(child_handle);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
