use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 10.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift / 2.0 + 3.04;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
            let collider = ColliderBuilder::cuboid(rad, rad);
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Setup a position-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_velocity_based()
        .translation(Vector::new(-10.0 * rad, 1.5 + 0.8));
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
    let (velocity_based_platform_handle, _) = world.insert(platform_body, collider);

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::kinematic_position_based()
        .translation(Vector::new(-10.0 * rad, 2.0 + 1.5 + 0.8));
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
    let (position_based_platform_handle, _) = world.insert(platform_body, collider);

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, physics, _, run_state| {
        let velocity = Vector::new(run_state.time.sin() * 5.0, (run_state.time * 5.0).sin());

        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(velocity_based_platform_handle) {
            platform.set_linvel(velocity, true);
        }

        // Update the position-based kinematic body by setting its next position.
        if let Some(platform) = physics.bodies.get_mut(position_based_platform_handle) {
            let mut next_tra = platform.translation();
            next_tra += velocity * physics.integration_parameters.dt;
            platform.set_next_kinematic_translation(next_tra);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 1.0), 40.0);
}
