use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0));
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    let (_handle, _) = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
