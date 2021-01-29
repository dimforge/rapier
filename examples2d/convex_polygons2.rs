use na::Point2;
use rand::distributions::{Distribution, Standard};
use rand::{rngs::StdRng, SeedableRng};
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let ground_size = 30.0;

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2).build();
    colliders.insert(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(ground_size, ground_size * 2.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2).build();
    colliders.insert(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(-ground_size, ground_size * 2.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the convex polygons
     */
    let num = 14;
    let scale = 4.0;
    let border_rad = 0.0;

    let shift = border_rad * 2.0 + scale;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0..num {
        for j in 0usize..num * 4 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift * 2.0 + centery + 2.0;

            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y).build();
            let handle = bodies.insert(rigid_body);

            let mut points = Vec::new();

            for _ in 0..10 {
                let pt: Point2<f32> = distribution.sample(&mut rng);
                points.push(pt * scale);
            }

            let collider = ColliderBuilder::convex_hull(&points).unwrap().build();
            colliders.insert(collider, handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 50.0), 10.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Balls", init_world)]);
    testbed.run()
}
