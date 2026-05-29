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
    let ground_size = 100.1;
    let ground_height = 0.1;

    for _ in 0..6 {
        let rigid_body =
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
        let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        let (_handle, _) = world.insert(rigid_body, collider);
    }

    // Build the dynamic box rigid body.
    for _ in 0..2 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.1, 0.0, 0.0))
            // .rotation(Vector::new(0.8, 0.2, 0.1))
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(2.0, 0.1, 1.0);
        let (_handle, _) = world.insert(rigid_body, collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
