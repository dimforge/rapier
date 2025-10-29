use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let extent = 1.0;
    let friction = 0.5;

    /*
     * Ground
     */
    let ground_width = 66.0 * extent;

    let rigid_body = RigidBodyBuilder::fixed();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::segment(
        point![-0.5 * 2.0 * ground_width, 0.0],
        point![0.5 * 2.0 * ground_width, 0.0],
    )
    .friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */

    for j in 0..3 {
        let mut count = 10;
        let offset = -20.0 * extent + 2.0 * (count as f32 + 1.0) * extent * j as f32;
        let mut y = extent;

        while count > 0 {
            for i in 0..count {
                let coeff = i as f32 - 0.5 * count as f32;
                let yy = if count == 1 { y + 2.0 } else { y };
                let position = vector![2.0 * coeff * extent + offset, yy];
                let rigid_body = RigidBodyBuilder::dynamic().translation(position);
                let parent = bodies.insert(rigid_body);

                let collider = ColliderBuilder::cuboid(extent, extent)
                    .density(if count == 1 {
                        (j as f32 + 1.0) * 100.0
                    } else {
                        1.0
                    })
                    .friction(friction);
                colliders.insert_with_parent(collider, parent, &mut bodies);
            }

            count -= 1;
            y += 2.0 * extent;
        }
    }
    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
