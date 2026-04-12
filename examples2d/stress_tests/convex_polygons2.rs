use rand::distr::{Distribution, StandardUniform};
use rand::{SeedableRng, rngs::StdRng};
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
    let ground_size = 30.0;

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
     * Create the convex polygons
     */
    let num = 26;
    let scale = 2.0;
    let border_rad = 0.0;

    let shift = border_rad * 2.0 + scale;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = StandardUniform;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift * 2.0 + centery + 2.0;

            let rigid_body = RigidBodyBuilder::dynamic().translation(Vec2::new(x, y));

            let mut points = Vec::new();

            for _ in 0..10 {
                let pt: [f32; 2] = distribution.sample(&mut rng);
                points.push(Vec2::from(pt) * scale);
            }

            let collider = ColliderBuilder::convex_hull(&points).unwrap();
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 50.0), 10.0);
}
