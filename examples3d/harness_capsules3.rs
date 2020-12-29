extern crate nalgebra as na;

use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::harness::Harness;

pub fn init_world(harness: &mut Harness) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let shifty = rad * 4.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..47 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shifty + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::capsule_y(rad, rad).build();
                colliders.insert(collider, handle, &mut bodies);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Set up the harness.
     */
    harness.set_world(bodies, colliders, joints);
}

fn main() {
    let harness = &mut Harness::new_empty();
    init_world(harness);
    harness.set_max_steps(10000);
    harness.run();
    println!("{}", harness.state.timestep_id);
}
