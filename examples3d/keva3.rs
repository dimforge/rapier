use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn build_block(
    testbed: &mut Testbed,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    half_extents: Vector<f32>,
    shift: Vector<f32>,
    (mut numx, numy, mut numz): (usize, usize, usize),
) {
    let dimensions = [half_extents.xyz(), half_extents.zyx()];
    let block_width = 2.0 * half_extents.z * numx as f32;
    let block_height = 2.0 * half_extents.y * numy as f32;
    let spacing = (half_extents.z * numx as f32 - half_extents.x) / (numz as f32 - 1.0);
    let mut color0 = [0.7, 0.5, 0.9];
    let mut color1 = [0.6, 1.0, 0.6];

    for i in 0..numy {
        std::mem::swap(&mut numx, &mut numz);
        let dim = dimensions[i % 2];
        let y = dim.y * i as f32 * 2.0;

        for j in 0..numx {
            let x = if i % 2 == 0 {
                spacing * j as f32 * 2.0
            } else {
                dim.x * j as f32 * 2.0
            };

            for k in 0..numz {
                let z = if i % 2 == 0 {
                    dim.z * k as f32 * 2.0
                } else {
                    spacing * k as f32 * 2.0
                };

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![
                    x + dim.x + shift.x,
                    y + dim.y + shift.y,
                    z + dim.z + shift.z
                ]);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(dim.x, dim.y, dim.z);
                colliders.insert_with_parent(collider, handle, bodies);

                testbed.set_initial_body_color(handle, color0);
                std::mem::swap(&mut color0, &mut color1);
            }
        }
    }

    // Close the top.
    let dim = half_extents.zxy();

    for i in 0..(block_width / (dim.x * 2.0)) as usize {
        for j in 0..(block_width / (dim.z * 2.0)) as usize {
            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![
                i as f32 * dim.x * 2.0 + dim.x + shift.x,
                dim.y + shift.y + block_height,
                j as f32 * dim.z * 2.0 + dim.z + shift.z
            ]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(dim.x, dim.y, dim.z);
            colliders.insert_with_parent(collider, handle, bodies);
            testbed.set_initial_body_color(handle, color0);
            std::mem::swap(&mut color0, &mut color1);
        }
    }
}

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
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let half_extents = vector![0.02, 0.1, 0.4] / 2.0 * 10.0;
    let mut block_height = 0.0;
    // These should only be set to odd values otherwise
    // the blocks won't align in the nicest way.
    let numy = [0, 9, 13, 17, 21, 41];
    let mut num_blocks_built = 0;

    for i in (1..=5).rev() {
        let numx = i;
        let numy = numy[i];
        let numz = numx * 3 + 1;
        let block_width = numx as f32 * half_extents.z * 2.0;
        build_block(
            testbed,
            &mut bodies,
            &mut colliders,
            half_extents,
            vector![-block_width / 2.0, block_height, -block_width / 2.0],
            (numx, numy, numz),
        );
        block_height += numy as f32 * half_extents.y * 2.0 + half_extents.x * 2.0;
        num_blocks_built += numx * numy * numz;
    }

    println!("Num keva blocks: {num_blocks_built}");

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
