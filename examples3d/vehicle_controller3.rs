use rapier_testbed3d::Testbed;
use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

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
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Vehicle we will control manually.
     */
    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0, 0.0));
    let vehicle_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
    colliders.insert_with_parent(collider, vehicle_handle, &mut bodies);

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
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                colliders.insert_with_parent(collider, handle, &mut bodies);
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
    colliders.insert(collider);

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
    colliders.insert(collider);

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
    colliders.insert(collider);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.set_vehicle_controller(vehicle);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
