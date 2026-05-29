use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let extent = 1.0;
    let friction = 0.5;

    /*
     * Ground
     */
    let ground_width = 66.0 * extent;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::segment(
        Vector::new(-0.5 * 2.0 * ground_width, 0.0),
        Vector::new(0.5 * 2.0 * ground_width, 0.0),
    )
    .friction(friction);
    let _ = world.insert(rigid_body, collider);

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
                let position = Vector::new(2.0 * coeff * extent + offset, yy);
                let rigid_body = RigidBodyBuilder::dynamic().translation(position);
                let collider = ColliderBuilder::cuboid(extent, extent)
                    .density(if count == 1 {
                        (j as f32 + 1.0) * 100.0
                    } else {
                        1.0
                    })
                    .friction(friction);
                let _ = world.insert(rigid_body, collider);
            }

            count -= 1;
            y += 2.0 * extent;
        }
    }
    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
