use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let origin = Vector::new(100_000.0, -80_000.0);
    let friction = 0.6;

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0) + origin);
    let collider = ColliderBuilder::cuboid(100.0, 1.0).friction(friction);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let base_count = 10;

    let h = 0.5;
    let shift = 1.25 * h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift + 0.5;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j as f32 - i as f32) * shift
                - h * base_count as f32;
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y) + origin);
            let collider = ColliderBuilder::cuboid(h, h).friction(friction);
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(origin.x + 0.0, origin.y + 2.5), 20.0);
}
