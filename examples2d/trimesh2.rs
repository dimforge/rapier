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
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    let _ = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(Vector::new(ground_size, ground_size));
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    let _ = world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(Vector::new(-ground_size, ground_size));
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    let _ = world.insert(rigid_body, collider);

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
                .translation(Vector::new(ith as f32 * 8.0 - 20.0, 20.0 + k as f32 * 11.0));
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 20.0), 17.0);
}
