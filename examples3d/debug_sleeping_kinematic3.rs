use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Setup a velocity-based kinematic rigid body.
     */
    let platform_body =
        RigidBodyBuilder::kinematic_velocity_based().translation(Vector::new(0.0, 1.5 + 0.8, 0.0));
    let collider = ColliderBuilder::cuboid(5.0, 0.5, 5.0);
    let (platform_handle, _) = world.insert(platform_body, collider);

    // A second velocity-based platform but this one will move super slow.
    let slow_platform_body =
        RigidBodyBuilder::kinematic_velocity_based().translation(Vector::new(0.0, 0.0, 0.0));
    let collider = ColliderBuilder::cuboid(5.0, 0.5, 5.0);
    let (slow_platform_handle, _) = world.insert(slow_platform_body, collider);

    /*
     * Setup a callback to control the platform.
     */
    let start_tick = 500;
    let stop_tick = 1000;

    testbed.add_callback(move |_, physics, _, run_state| {
        if run_state.timestep_id == stop_tick {
            println!("Both platforms should stop moving now and eventually fall asleep.");
            // The platforms moved until this time. They must not be sleeping.
            assert!(!physics.bodies[platform_handle].is_sleeping());
            assert!(!physics.bodies[slow_platform_handle].is_sleeping());

            if let Some(slow_platform) = physics.bodies.get_mut(slow_platform_handle) {
                slow_platform.set_linvel(Vector::ZERO, true);
            }
            if let Some(platform) = physics.bodies.get_mut(platform_handle) {
                platform.set_linvel(Vector::ZERO, true);
            }
        }

        if run_state.timestep_id > stop_tick + 500 {
            // Platforms should fall asleep shortly after we stopped moving them.
            assert!(physics.bodies[platform_handle].is_sleeping());
            assert!(physics.bodies[slow_platform_handle].is_sleeping());
        }

        if run_state.timestep_id < start_tick || run_state.timestep_id >= stop_tick {
            return;
        } else if run_state.timestep_id == start_tick {
            println!("Platforms should start moving now and never stop.");
            println!("The slow platform should move up and not sleep.");
            // Platforms should have had plenty of time to fall asleep before we start moving them.
            assert!(physics.bodies[platform_handle].is_sleeping());
            assert!(physics.bodies[slow_platform_handle].is_sleeping());

            let slow_velocity = Vector::new(0.0, 0.01, 0.0);
            if let Some(slow_platform) = physics.bodies.get_mut(slow_platform_handle) {
                slow_platform.set_linvel(slow_velocity, true);
            }
        }

        let velocity = Vector::new(
            0.0,
            (run_state.time * 2.0).cos(),
            run_state.time.sin() * 2.0,
        );

        // Update the velocity-based kinematic body by setting its velocity.
        if let Some(platform) = physics.bodies.get_mut(platform_handle) {
            platform.set_linvel(velocity, true);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 5.0, 10.0), Vec3::ZERO);
}
