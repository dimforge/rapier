use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let rad = 1.0;
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, -rad))
        .rotation(std::f32::consts::PI / 4.0);
    let collider = ColliderBuilder::cuboid(rad, rad);
    let _ = world.insert(rigid_body, collider);

    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 3.0 * rad))
        .can_sleep(false);
    let collider = ColliderBuilder::ball(rad);
    let _ = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 50.0);
}
