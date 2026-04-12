use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let friction = 0.7;

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -2.0));
    let collider = ColliderBuilder::cuboid(40.0, 2.0).friction(friction);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let scale = 10.0;
    let card_height = 0.2 * scale;
    let card_thickness = 0.001 * scale;
    let angle0 = 25.0 * std::f32::consts::PI / 180.0;
    let angle1 = -25.0 * std::f32::consts::PI / 180.0;
    let angle2 = 0.5 * std::f32::consts::PI;

    let card_box = ColliderBuilder::cuboid(card_thickness, card_height).friction(friction);

    let mut nb = 5;
    let mut z0 = 0.0;
    let mut y = card_height - 0.02 * scale;

    while nb != 0 {
        let mut z = z0;

        for i in 0..nb {
            if i != nb - 1 {
                let rigid_body = RigidBodyBuilder::dynamic()
                    .translation(Vector::new(
                        z + 0.25 * scale,
                        y + card_height - 0.015 * scale,
                    ))
                    .rotation(angle2);
                let _ = world.insert(rigid_body, card_box.clone());
            }

            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(z, y))
                .rotation(angle1);
            let _ = world.insert(rigid_body, card_box.clone());

            z += 0.175 * scale;

            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(z, y))
                .rotation(angle0);
            let _ = world.insert(rigid_body, card_box.clone());

            z += 0.175 * scale;
        }

        y += card_height * 2.0 - 0.03 * scale;
        z0 += 0.175 * scale;
        nb -= 1;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 2.5), 20.0);
}
