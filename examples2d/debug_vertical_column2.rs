use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let num = 80;
    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 1.0;
    let ground_thickness = 1.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness).friction(0.3);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */

    for i in 0..num {
        let y = i as f32 * rad * 2.0 + ground_thickness + rad;

        // Build the rigid body.
        let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y));
        let collider = ColliderBuilder::cuboid(rad, rad).friction(0.3);
        let _ = world.insert(rigid_body, collider);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    // testbed.harness_mut().physics.gravity.y = -981.0;
    testbed.look_at(Vec2::new(0.0, 2.5), 5.0);
}
