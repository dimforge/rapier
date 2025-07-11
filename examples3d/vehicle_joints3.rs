use rapier_testbed3d::{KeyCode, Testbed};
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground.
     */
    let ground_size = Vector::new(60.0, 0.4, 60.0);
    let nsubdivs = 100;

    let heights = DMatrix::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        -(i as f32 * ground_size.x / (nsubdivs as f32) / 2.0).cos()
            - (j as f32 * ground_size.z / (nsubdivs as f32) / 2.0).cos()
    });

    let collider = ColliderBuilder::heightfield(heights, ground_size)
        .translation(vector![-7.0, 0.0, 0.0])
        .friction(1.0);
    colliders.insert(collider);

    /*
     * Vehicle we will control manually, simulated using joints.
     * Strongly inspired from https://github.com/h3r2tic/cornell-mcray/blob/main/src/car.rs (MIT/Apache license).
     */
    const CAR_GROUP: Group = Group::GROUP_1;

    let wheel_params = [
        vector![0.6874, 0.2783, -0.7802],
        vector![-0.6874, 0.2783, -0.7802],
        vector![0.64, 0.2783, 1.0254],
        vector![-0.64, 0.2783, 1.0254],
    ];
    // TODO: lower center of mass?
    // let mut center_of_mass = wheel_params.iter().sum().unwrap() / 4.0;
    // center_of_mass.y = 0.0;

    let suspension_height = 0.12;
    let max_steering_angle = 35.0f32.to_radians();
    let drive_strength = 1.0;
    let wheel_radius = 0.28;
    let car_position = point![0.0, wheel_radius + suspension_height, 0.0];
    let body_position_in_car_space = vector![0.0, 0.4739, 0.0];

    let body_position = car_position + body_position_in_car_space;

    let body_co = ColliderBuilder::cuboid(0.65, 0.3, 0.9)
        .density(100.0)
        .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));
    let body_rb = RigidBodyBuilder::dynamic()
        .position(body_position.into())
        .build();
    let body_handle = bodies.insert(body_rb);
    colliders.insert_with_parent(body_co, body_handle, &mut bodies);

    let mut steering_joints = vec![];
    let mut motor_joints = vec![];

    for (wheel_id, wheel_pos_in_car_space) in wheel_params.into_iter().enumerate() {
        let is_front = wheel_id >= 2;
        let wheel_center = car_position + wheel_pos_in_car_space;

        let axle_mass_props = MassProperties::from_ball(100.0, wheel_radius);
        let axle_rb = RigidBodyBuilder::dynamic()
            .position(wheel_center.into())
            .additional_mass_properties(axle_mass_props);
        let axle_handle = bodies.insert(axle_rb);

        // This is a fake cylinder collider that we add only because our testbed can
        // only render colliders. Setting it as sensor makes it show up as wireframe.
        let wheel_fake_co = ColliderBuilder::cylinder(wheel_radius / 2.0, wheel_radius)
            .rotation(Vector::z() * std::f32::consts::FRAC_PI_2)
            .sensor(true)
            .density(0.0)
            .collision_groups(InteractionGroups::none());

        // The actual wheel collider. Simulating the wheel as a ball is interesting as it
        // is mathematically simpler than a cylinder and cheaper to compute for collision-detection.
        let wheel_co = ColliderBuilder::ball(wheel_radius)
            .density(100.0)
            .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP))
            .friction(1.0);
        let wheel_rb = RigidBodyBuilder::dynamic().position(wheel_center.into());
        let wheel_handle = bodies.insert(wheel_rb);
        colliders.insert_with_parent(wheel_co, wheel_handle, &mut bodies);
        colliders.insert_with_parent(wheel_fake_co, wheel_handle, &mut bodies);

        let suspension_attachment_in_body_space =
            wheel_pos_in_car_space - body_position_in_car_space;

        // Suspension between the body and the axle.
        let mut locked_axes = JointAxesMask::LIN_X
            | JointAxesMask::LIN_Z
            | JointAxesMask::ANG_X
            | JointAxesMask::ANG_Z;
        if !is_front {
            locked_axes |= JointAxesMask::ANG_Y;
        }

        let mut suspension_joint = GenericJointBuilder::new(locked_axes)
            .limits(JointAxis::LinY, [0.0, suspension_height])
            .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
            .local_anchor1(suspension_attachment_in_body_space.into());

        if is_front {
            suspension_joint =
                suspension_joint.limits(JointAxis::AngY, [-max_steering_angle, max_steering_angle]);
        }

        let body_axle_joint_handle =
            impulse_joints.insert(body_handle, axle_handle, suspension_joint, true);

        // Joint between the axle and the wheel.
        let wheel_joint = RevoluteJointBuilder::new(Vector::x_axis());
        let wheel_joint_handle =
            impulse_joints.insert(axle_handle, wheel_handle, wheel_joint, true);

        if is_front {
            steering_joints.push(body_axle_joint_handle);
            motor_joints.push(wheel_joint_handle);
        }
    }

    /*
     * Callback to control the wheels motors.
     */
    testbed.add_callback(move |gfx, physics, _, _| {
        let Some(gfx) = gfx else { return };

        let mut thrust = 0.0;
        let mut steering = 0.0;
        let mut boost = 1.0;

        for key in gfx.keys().get_pressed() {
            match *key {
                KeyCode::ArrowRight => {
                    steering = -1.0;
                }
                KeyCode::ArrowLeft => {
                    steering = 1.0;
                }
                KeyCode::ArrowUp => {
                    thrust = -drive_strength;
                }
                KeyCode::ArrowDown => {
                    thrust = drive_strength;
                }
                KeyCode::ShiftRight => {
                    boost = 1.5;
                }
                _ => {}
            }
        }
        let mut should_wake_up = false;
        if thrust != 0.0 || steering != 0.0 {
            should_wake_up = true;
        }

        // Apply steering to the axles.
        for steering_handle in &steering_joints {
            let steering_joint = physics
                .impulse_joints
                .get_mut(*steering_handle, should_wake_up)
                .unwrap();
            steering_joint.data.set_motor_position(
                JointAxis::AngY,
                max_steering_angle * steering,
                1.0e4,
                1.0e3,
            );
        }

        // Apply thrust.
        // Pseudo-differential adjusting speed of engines depending on steering arc
        // Higher values result in more drifty behavior.
        let differential_strength = 0.5;
        let sideways_shift = (max_steering_angle * steering).sin() * differential_strength;
        let speed_diff = if sideways_shift > 0.0 {
            f32::hypot(1.0, sideways_shift)
        } else {
            1.0 / f32::hypot(1.0, sideways_shift)
        };

        let ms = [1.0 / speed_diff, speed_diff];
        for (motor_handle, &ms) in motor_joints.iter().copied().zip(ms.iter()) {
            let motor_joint = physics
                .impulse_joints
                .get_mut(motor_handle, should_wake_up)
                .unwrap();
            motor_joint.data.set_motor_velocity(
                JointAxis::AngX,
                -30.0 * thrust * ms * boost,
                1.0e2,
            );
        }
    });

    /*
     * Create some cubes on the ground.
     */
    // let num = 8;
    // let rad = 0.1;
    //
    // let shift = rad * 2.0;
    // let centerx = shift * (num / 2) as f32;
    // let centery = rad;
    //
    // for j in 0usize..1 {
    //     for k in 0usize..4 {
    //         for i in 0..num {
    //             let x = i as f32 * shift - centerx;
    //             let y = j as f32 * shift + centery;
    //             let z = k as f32 * shift + centerx;
    //
    //             let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
    //             let handle = bodies.insert(rigid_body);
    //             let collider = ColliderBuilder::cuboid(rad, rad, rad);
    //             colliders.insert_with_parent(collider, handle, &mut bodies);
    //         }
    //     }
    // }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point!(10.0, 10.0, 10.0), Point::origin());
}
