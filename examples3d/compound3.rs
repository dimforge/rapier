use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

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
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));

                // First option: attach several colliders to a single rigid-body.
                if j < numy / 2 {
                    let collider1 = ColliderBuilder::cuboid(rad * 10.0, rad, rad);
                    let collider2 = ColliderBuilder::cuboid(rad, rad * 10.0, rad)
                        .translation(Vector::new(rad * 10.0, rad * 10.0, 0.0));
                    let collider3 = ColliderBuilder::cuboid(rad, rad * 10.0, rad)
                        .translation(Vector::new(-rad * 10.0, rad * 10.0, 0.0));
                    let (handle, _) = world.insert(rigid_body, collider1);
                    world.insert_collider_with_parent(collider2, handle);
                    world.insert_collider_with_parent(collider3, handle);
                } else {
                    // Second option: create a compound shape and attach it to a single collider.
                    let shapes = vec![
                        (Pose::IDENTITY, SharedShape::cuboid(rad * 10.0, rad, rad)),
                        (
                            Pose::from_translation(Vector::new(rad * 10.0, rad * 10.0, 0.0)),
                            SharedShape::cuboid(rad, rad * 10.0, rad),
                        ),
                        (
                            Pose::from_translation(Vector::new(-rad * 10.0, rad * 10.0, 0.0)),
                            SharedShape::cuboid(rad, rad * 10.0, rad),
                        ),
                    ];

                    let collider = ColliderBuilder::compound(shapes);
                    let (_handle, _) = world.insert(rigid_body, collider);
                }
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);
}
