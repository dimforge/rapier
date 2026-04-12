use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    let use_articulations = false;

    /*
     * Create a chain with a very heavy ball at the end.
     */
    let num = 17;
    let rad = 0.2;

    let mut body_handles = Vec::new();

    for i in 0..num {
        let fi = i as f32;

        let status = if i == 0 {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        };

        let ball_rad = if i == num - 1 { rad * 10.0 } else { rad };
        let shift1 = rad * 1.1;
        let shift2 = ball_rad + rad * 0.1;
        let z = if i == 0 {
            0.0
        } else {
            (fi - 1.0) * 2.0 * shift1 + shift1 + shift2
        };

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(Vector::new(0.0, 0.0, z))
            .additional_solver_iterations(16);
        let collider = ColliderBuilder::ball(ball_rad);
        let (child_handle, _) = world.insert(rigid_body, collider);

        // Vertical joint.
        if i > 0 {
            let parent_handle = *body_handles.last().unwrap();
            let joint = if i == 1 {
                SphericalJointBuilder::new().local_anchor2(Vector::new(0.0, 0.0, -shift1 * 2.0))
            } else {
                SphericalJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, 0.0, shift1))
                    .local_anchor2(Vector::new(0.0, 0.0, -shift2))
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
