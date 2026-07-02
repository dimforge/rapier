//! Headless benchmark: builds a capsule stack and steps it without any window
//! or testbed, printing the number of steps executed.
use rapier3d::prelude::*;

fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    let (_handle, _) = world.insert(rigid_body, collider);

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
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
                let collider = ColliderBuilder::capsule_y(rad, rad);
                let (_handle, _) = world.insert(rigid_body, collider);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    world
}

fn main() {
    let mut world = build_world();
    let num_steps = 10000;
    for _ in 0..num_steps {
        world.step();
    }
    println!("{num_steps}");
}
