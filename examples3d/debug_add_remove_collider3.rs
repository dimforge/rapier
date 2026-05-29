use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4);
    let (ground_handle, mut ground_collider_handle) = world.insert(rigid_body, collider);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.2, 0.0));
    let collider = ColliderBuilder::ball(ball_rad).density(100.0);
    let (_ball_handle, _) = world.insert(rb, collider);

    testbed.add_callback(move |_, physics, _, _| {
        // Remove then re-add the ground collider.
        let removed_collider_handle = ground_collider_handle;
        let coll = physics
            .colliders
            .remove(
                removed_collider_handle,
                &mut physics.islands,
                &mut physics.bodies,
                true,
            )
            .unwrap();
        ground_collider_handle =
            physics
                .colliders
                .insert_with_parent(coll, ground_handle, &mut physics.bodies);
    });

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
