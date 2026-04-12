use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

// This shows a bug when a cylinder is in contact with a very large
// but very thin cuboid. In this case the EPA returns an incorrect
// contact normal, resulting in the cylinder falling through the floor.
pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    let _ = world.insert(rigid_body, collider);

    /*
     * A rectangle that only rotate.
     */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 3.0))
        .lock_translations();
    let collider = ColliderBuilder::cuboid(2.0, 0.6);
    let _ = world.insert(rigid_body, collider);

    /*
     * A tilted capsule that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 5.0))
        .rotation(1.0)
        .lock_rotations();
    let collider = ColliderBuilder::capsule_y(0.6, 0.4);
    let _ = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 40.0);
}
