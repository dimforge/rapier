use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
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
     * Setup the platform control parameters.
     */
    let start_tick = 500;
    let stop_tick = 1000;

    /*
     * Run the simulation.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 5.0, 10.0), Vec3::ZERO);

    let mut step_id = 0usize;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            step_id += 1;

            let timestep_id = step_id;
            let time = step_id as f32 * world.integration_parameters.dt as f32;

            if timestep_id == stop_tick {
                println!("Both platforms should stop moving now and eventually fall asleep.");
                // The platforms moved until this time. They must not be sleeping.
                assert!(!world.bodies[platform_handle].is_sleeping());
                assert!(!world.bodies[slow_platform_handle].is_sleeping());

                if let Some(slow_platform) = world.bodies.get_mut(slow_platform_handle) {
                    slow_platform.set_linvel(Vector::ZERO, true);
                }
                if let Some(platform) = world.bodies.get_mut(platform_handle) {
                    platform.set_linvel(Vector::ZERO, true);
                }
            }

            if timestep_id > stop_tick + 500 {
                // Platforms should fall asleep shortly after we stopped moving them.
                assert!(world.bodies[platform_handle].is_sleeping());
                assert!(world.bodies[slow_platform_handle].is_sleeping());
            }

            if !(timestep_id < start_tick || timestep_id >= stop_tick) {
                if timestep_id == start_tick {
                    println!("Platforms should start moving now and never stop.");
                    println!("The slow platform should move up and not sleep.");
                    // Platforms should have had plenty of time to fall asleep before we start moving them.
                    assert!(world.bodies[platform_handle].is_sleeping());
                    assert!(world.bodies[slow_platform_handle].is_sleeping());

                    let slow_velocity = Vector::new(0.0, 0.01, 0.0);
                    if let Some(slow_platform) = world.bodies.get_mut(slow_platform_handle) {
                        slow_platform.set_linvel(slow_velocity, true);
                    }
                }

                let velocity = Vector::new(0.0, (time * 2.0).cos(), time.sin() * 2.0);

                // Update the velocity-based kinematic body by setting its velocity.
                if let Some(platform) = world.bodies.get_mut(platform_handle) {
                    platform.set_linvel(velocity, true);
                }
            }
        }
    }
    Ok(())
}
