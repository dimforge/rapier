use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

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

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 3.0, 0.0))
        .lock_translations()
        .enabled_rotations(true, false, false);
    let collider = ColliderBuilder::cuboid(0.2, 0.6, 2.0);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * A tilted capsule that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 5.0, 0.0))
        .rotation(Vector::X * 1.0)
        .lock_rotations();
    let collider = ColliderBuilder::capsule_y(0.6, 0.4);
    let (handle, _) = world.insert(rigid_body, collider);
    let collider = ColliderBuilder::capsule_x(0.6, 0.4);
    world.insert_collider_with_parent(collider, handle);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 3.0, 0.0), Vec3::new(0.0, 3.0, 0.0));
}
