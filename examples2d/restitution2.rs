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
    let ground_size = 20.;
    let ground_height = 1.0;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height).restitution(1.0);
    let _ = world.insert(rigid_body, collider);

    let num = 10;
    let rad = 0.5;

    for j in 0..2 {
        for i in 0..=num {
            let x = (i as f32) - num as f32 / 2.0;
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x * 2.0, 10.0 * (j as f32 + 1.0)));
            let collider = ColliderBuilder::ball(rad).restitution((i as f32) / (num as f32));
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 1.0), 25.0);
}
