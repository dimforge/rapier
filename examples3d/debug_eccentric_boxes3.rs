use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

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

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (handle, _) = world.insert(rigid_body, collider);

    // Build the dynamic box rigid body.
    let (mut vtx, idx) = Cuboid::new(Vector::new(1.0, 1.0, 1.0)).to_trimesh();
    vtx.iter_mut()
        .for_each(|pt| *pt += Vector::new(100.0, 100.0, 100.0));
    let shape = SharedShape::convex_mesh(vtx, &idx).unwrap();

    for _ in 0..2 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(-100.0, -100.0 + 10.0, -100.0))
            .can_sleep(false);
        let collider = ColliderBuilder::new(shape.clone());
        let (handle, _) = world.insert(rigid_body, collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
