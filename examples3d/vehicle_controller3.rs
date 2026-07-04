use rapier_testbed3d::{Key, TestbedViewer};
use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    world.insert(rigid_body, collider);

    /*
     * Vehicle we will control manually.
     */
    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0, 0.0));
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
    let (vehicle_handle, _) = world.insert(rigid_body, collider);

    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let mut vehicle = DynamicRayCastVehicleController::new(vehicle_handle);
    let wheel_positions = [
        Vector::new(hw * 1.5, -hh, hw),
        Vector::new(hw * 1.5, -hh, -hw),
        Vector::new(-hw * 1.5, -hh, hw),
        Vector::new(-hw * 1.5, -hh, -hw),
    ];

    for pos in wheel_positions {
        vehicle.add_wheel(pos, -Vector::Y, Vector::Z, hh, hh / 4.0, &tuning);
    }

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = rad;

    for j in 0usize..1 {
        for k in 0usize..4 {
            for i in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift + centerx;

                let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                world.insert(rigid_body, collider);
            }
        }
    }

    /*
     * Create a slope we can climb.
     */
    let slope_angle = 0.2;
    let slope_size = 2.0;
    let collider = ColliderBuilder::cuboid(slope_size, ground_height, ground_size)
        .translation(Vector::new(
            ground_size + slope_size,
            -ground_height + 0.4,
            0.0,
        ))
        .rotation(Vector::Z * slope_angle);
    world.insert_collider(collider, None);

    /*
     * Create a slope we can’t climb.
     */
    let impossible_slope_angle = 0.9;
    let impossible_slope_size = 2.0;
    let collider = ColliderBuilder::cuboid(slope_size, ground_height, ground_size)
        .translation(Vector::new(
            ground_size + slope_size * 2.0 + impossible_slope_size - 0.9,
            -ground_height + 2.3,
            0.0,
        ))
        .rotation(Vector::Z * impossible_slope_angle);
    world.insert_collider(collider, None);

    /*
     * More complex ground.
     */
    let ground_size = Vector::new(10.0, 0.4, 10.0);
    let nsubdivs = 20;

    let heights = Array2::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        -(i as f32 * ground_size.x / (nsubdivs as f32) / 2.0).cos()
            - (j as f32 * ground_size.z / (nsubdivs as f32) / 2.0).cos()
    });

    let collider =
        ColliderBuilder::heightfield(heights, ground_size).translation(Vector::new(-7.0, 0.0, 0.0));
    world.insert_collider(collider, None);

    /*
     * Set up the viewer.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            /*
             * Vehicle controls.
             */
            let keys = viewer.keys();
            let mut engine_force = 0.0;
            let mut steering_angle = 0.0;

            if keys.pressed(Key::Right) {
                steering_angle += -0.7;
            }
            if keys.pressed(Key::Left) {
                steering_angle += 0.7;
            }
            if keys.pressed(Key::Up) {
                engine_force += 30.0;
            }
            if keys.pressed(Key::Down) {
                engine_force += -30.0;
            }

            let wheels = vehicle.wheels_mut();
            wheels[0].engine_force = engine_force;
            wheels[0].steering = steering_angle;
            wheels[1].engine_force = engine_force;
            wheels[1].steering = steering_angle;

            let q = world.broad_phase.as_query_pipeline_mut(
                world.narrow_phase.query_dispatcher(),
                &mut world.bodies,
                &mut world.colliders,
                QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle_handle),
            );
            vehicle.update_vehicle(world.integration_parameters.dt, q);

            world.step();
        }
    }
    Ok(())
}
