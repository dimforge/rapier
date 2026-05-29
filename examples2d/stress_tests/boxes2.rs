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
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    let _ = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(Vec2::new(ground_size, ground_size * 2.0));
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    let _ = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(Vec2::new(-ground_size, ground_size * 2.0));
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let num = 26;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vec2::new(x, y));
            let collider = ColliderBuilder::cuboid(rad, rad);
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 50.0), 10.0);
}
