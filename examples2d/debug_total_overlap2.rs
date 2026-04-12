use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    // Build many balls, all spawned at the same point.
    let rad = 0.5;

    for _ in 0..100 {
        let rigid_body = RigidBodyBuilder::dynamic();
        let collider = ColliderBuilder::cuboid(rad, rad);
        let _ = world.insert(rigid_body, collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 50.0);
}
