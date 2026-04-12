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
    let ground_size = 10.0;
    let ground_thickness = 1.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let num = 6;
    let mut rad = 0.5;
    let mut y = rad;

    for _ in 0usize..num {
        // Build the rigid body.
        let rigid_body =
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y + ground_thickness));
        let collider = ColliderBuilder::cuboid(rad, rad);
        let _ = world.insert(rigid_body, collider);
        y += rad + rad * 2.0;
        rad *= 2.0;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
