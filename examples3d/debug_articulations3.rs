use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn create_ball_articulations(world: &mut PhysicsWorld, num: usize) {
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

            let rigid_body = RigidBodyBuilder::new(status).translation(Vector::new(
                fk * shift,
                0.0,
                fi * shift * 2.0,
            ));
            let collider = ColliderBuilder::capsule_z(rad * 1.25, rad);
            let (child_handle, _) = world.insert(rigid_body, collider);

            // Vertical multibody_joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint =
                    SphericalJointBuilder::new().local_anchor2(Vector::new(0.0, 0.0, -shift * 2.0));
                world.insert_multibody_joint(parent_handle, child_handle, joint);
            }

            // Horizontal multibody_joint.
            if k > 0 && i > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint =
                    SphericalJointBuilder::new().local_anchor2(Vector::new(-shift, 0.0, 0.0));
                // let joint =
                //     PrismaticJoint::new(Vector::Y).local_anchor2(Vector::new(-shift, 0.0, 0.0));
                // let joint = FixedJoint::new().local_anchor2(Vector::new(-shift, 0.0, 0.0));
                // let joint =
                //     RevoluteJoint::new(Vector::X).local_anchor2(Vector::new(-shift, 0.0, 0.0));
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let collider = ColliderBuilder::cuboid(30.0, 0.01, 30.0)
        .translation(Vector::new(0.0, -3.02, 0.0))
        .rotation(Vector::new(0.1, 0.0, 0.1));
    world.insert_collider(collider);

    let rigid_body = RigidBodyBuilder::dynamic();
    let collider = ColliderBuilder::cuboid(30.0, 0.01, 30.0)
        .translation(Vector::new(0.0, -3.0, 0.0))
        .rotation(Vector::new(0.1, 0.0, 0.1));
    let (_handle, _) = world.insert(rigid_body, collider);

    create_ball_articulations(&mut world, 15);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(15.0, 5.0, 42.0), Vec3::new(13.0, 1.0, 1.0));
}
