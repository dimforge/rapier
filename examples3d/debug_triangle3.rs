use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    // Triangle ground.
    let vtx = [
        Vector::new(-10.0, 0.0, -10.0),
        Vector::new(10.0, 0.0, -10.0),
        Vector::new(0.0, 0.0, 10.0),
    ];

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0, 0.0));
    let collider = ColliderBuilder::triangle(vtx[0], vtx[1], vtx[2]);
    let (_handle, _) = world.insert(rigid_body, collider);

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(1.1, 0.01, 0.0))
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false);
    let collider = ColliderBuilder::cuboid(20.0, 0.1, 1.0);
    let (_handle, _) = world.insert(rigid_body, collider);

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
