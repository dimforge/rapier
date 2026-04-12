use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let extent = 1.0;
    let friction = 0.6;

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
    let rigid_body =
        RigidBodyBuilder::dynamic().translation(Vector::new(-9.0 * extent, 0.5 * extent));
    let collider = ColliderBuilder::cuboid(0.5 * extent, 0.5 * extent).friction(friction);
    let _ = world.insert(rigid_body, collider);

    let rigid_body =
        RigidBodyBuilder::dynamic().translation(Vector::new(9.0 * extent, 0.5 * extent));
    let collider = ColliderBuilder::cuboid(0.5 * extent, 0.5 * extent).friction(friction);
    let _ = world.insert(rigid_body, collider);

    let rigid_body =
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, (10.0 + 16.0) * extent));
    let collider = ColliderBuilder::cuboid(10.0 * extent, 10.0 * extent).friction(friction);
    let _ = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
