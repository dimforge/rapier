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
    let ground_size = 100.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).friction(1.5);
    let (_handle, _) = world.insert(rigid_body, collider);

    // Build a dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 1.1, 0.0))
        .rotation(Vector::Y * 0.3);
    let collider = ColliderBuilder::cuboid(2.0, 1.0, 3.0).friction(1.5);
    let (handle, _) = world.insert(rigid_body, collider);

    let rigid_body = &mut world.bodies[handle];
    let force = rigid_body.rotation() * Vector::Z * 50.0;
    rigid_body.set_linvel(force, true);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);
}
