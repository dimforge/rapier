use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn create_pyramid(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector<f32>,
    stack_height: usize,
    half_extents: Vector<f32>,
) {
    let shift = half_extents * 2.5;
    for i in 0usize..stack_height {
        for j in i..stack_height {
            for k in i..stack_height {
                let fi = i as f32;
                let fj = j as f32;
                let fk = k as f32;
                let x = (fi * shift.x / 2.0) + (fk - fi) * shift.x + offset.x
                    - stack_height as f32 * half_extents.x;
                let y = fi * shift.y + offset.y;
                let z = (fi * shift.z / 2.0) + (fj - fi) * shift.z + offset.z
                    - stack_height as f32 * half_extents.z;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let rigid_body_handle = bodies.insert(rigid_body);

                let collider =
                    ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z);
                colliders.insert_with_parent(collider, rigid_body_handle, bodies);
            }
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
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    let cube_size = 1.0;
    let hext = Vector::repeat(cube_size);
    let bottomy = cube_size;
    create_pyramid(
        &mut bodies,
        &mut colliders,
        vector![0.0, bottomy, 0.0],
        24,
        hext,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
