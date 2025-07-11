use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

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
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(vector![ground_size, ground_size]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(vector![-ground_size, ground_size]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the trimeshes from a tessellated SVG.
     */
    let rapier_logo_buffers = crate::utils::svg::rapier_logo();

    for (ith, (vtx, idx)) in rapier_logo_buffers.into_iter().enumerate() {
        for k in 0..5 {
            let collider = ColliderBuilder::trimesh(vtx.clone(), idx.clone())
                .unwrap()
                .contact_skin(0.2);
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![ith as f32 * 8.0 - 20.0, 20.0 + k as f32 * 11.0]);
            let handle = bodies.insert(rigid_body);
            colliders.insert_with_parent(collider, handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 20.0], 17.0);
}
