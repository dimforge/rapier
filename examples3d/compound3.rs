use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 8;
    let numy = 15;
    let rad = 0.2;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..numy {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift * 5.0 - centerx + offset;
                let y = j as f32 * (shift * 5.0) + centery + 3.0;
                let z = k as f32 * shift * 2.0 - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(rigid_body);

                // First option: attach several colliders to a single rigid-body.
                if j < numy / 2 {
                    let collider1 = ColliderBuilder::cuboid(rad * 10.0, rad, rad);
                    let collider2 = ColliderBuilder::cuboid(rad, rad * 10.0, rad)
                        .translation(vector![rad * 10.0, rad * 10.0, 0.0]);
                    let collider3 = ColliderBuilder::cuboid(rad, rad * 10.0, rad)
                        .translation(vector![-rad * 10.0, rad * 10.0, 0.0]);
                    colliders.insert_with_parent(collider1, handle, &mut bodies);
                    colliders.insert_with_parent(collider2, handle, &mut bodies);
                    colliders.insert_with_parent(collider3, handle, &mut bodies);
                } else {
                    // Second option: create a compound shape and attach it to a single collider.
                    let shapes = vec![
                        (
                            Isometry::identity(),
                            SharedShape::cuboid(rad * 10.0, rad, rad),
                        ),
                        (
                            Isometry::translation(rad * 10.0, rad * 10.0, 0.0),
                            SharedShape::cuboid(rad, rad * 10.0, rad),
                        ),
                        (
                            Isometry::translation(-rad * 10.0, rad * 10.0, 0.0),
                            SharedShape::cuboid(rad, rad * 10.0, rad),
                        ),
                    ];

                    let collider = ColliderBuilder::compound(shapes);
                    colliders.insert_with_parent(collider, handle, &mut bodies);
                }
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
