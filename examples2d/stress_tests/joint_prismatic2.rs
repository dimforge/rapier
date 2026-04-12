use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Create the balls
     */
    // Build the rigid body.
    let rad = 0.4;
    let num = 10;
    let shift = 1.0;

    for l in 0..25 {
        let y = l as f32 * shift * (num as f32 + 2.0) * 2.0;

        for j in 0..50 {
            let x = j as f32 * shift * 4.0;

            let ground = RigidBodyBuilder::fixed().translation(Vec2::new(x, y));
            let collider = ColliderBuilder::cuboid(rad, rad);
            let (mut curr_parent, _) = world.insert(ground, collider);

            for i in 0..num {
                let y = y - (i + 1) as f32 * shift;
                let density = 1.0;
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vec2::new(x, y));
                let collider = ColliderBuilder::cuboid(rad, rad).density(density);
                let (curr_child, _) = world.insert(rigid_body, collider);

                let axis = if i % 2 == 0 {
                    Vec2::new(1.0, 1.0).normalize()
                } else {
                    Vec2::new(-1.0, 1.0).normalize()
                };

                let prism = PrismaticJointBuilder::new(axis)
                    .local_anchor2(Vec2::new(0.0, shift))
                    .limits([-1.5, 1.5]);
                world.insert_impulse_joint(curr_parent, curr_child, prism);

                curr_parent = curr_child;
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(80.0, 80.0), 15.0);
}
