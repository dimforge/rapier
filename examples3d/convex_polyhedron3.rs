use rand::distr::{Distribution, StandardUniform};
use rand::{SeedableRng, rngs::StdRng};
use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 40.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the polyhedra
     */
    let num = 5;
    let scale = 2.0;
    let border_rad = 0.1;

    let shift = border_rad * 2.0 + scale;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = StandardUniform;

    for j in 0usize..25 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz;

                let mut points = Vec::new();
                for _ in 0..10 {
                    let pt: Point<f32> = distribution.sample(&mut rng);
                    points.push(pt * scale);
                }

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::round_convex_hull(&points, border_rad).unwrap();
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![30.0, 30.0, 30.0], Point::origin());
}
