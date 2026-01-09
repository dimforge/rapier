use kiss3d::color::Color;
use rapier_testbed3d::Testbed;
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
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 4000;
    let width = 1.0;
    let thickness = 0.1;

    let colors = [
        Color::new(0.7, 0.5, 0.9, 1.0),
        Color::new(0.6, 1.0, 0.6, 1.0),
    ];

    let mut curr_angle = 0.0;
    let mut curr_rad = 10.0;
    let mut prev_angle;
    let mut skip = 0;
    for i in 0..num {
        let perimeter = 2.0 * std::f32::consts::PI * curr_rad;
        let spacing = thickness * 4.0;
        prev_angle = curr_angle;
        curr_angle += 2.0 * std::f32::consts::PI * spacing / perimeter;
        let (x, z) = curr_angle.sin_cos();

        // Build the rigid body.
        let two_pi = 2.0 * std::f32::consts::PI;
        let nudged = curr_angle % two_pi < prev_angle % two_pi;
        let tilt = if nudged || i == num - 1 { 0.2 } else { 0.0 };

        if skip == 0 {
            let rot = Rotation::from_rotation_y(curr_angle);
            let tilt_axis = rot * Vector::Z;
            let tilt_rot = Rotation::from_axis_angle(tilt_axis, tilt);
            let position = Pose::from_parts(
                Vector::new(x * curr_rad, width * 2.0 + ground_height, z * curr_rad),
                tilt_rot * rot,
            );
            let rigid_body = RigidBodyBuilder::dynamic().pose(position);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(thickness, width * 2.0, width);
            colliders.insert_with_parent(collider, handle, &mut bodies);
            testbed.set_initial_body_color(handle, colors[i % 2]);
        } else {
            skip -= 1;
        }

        if nudged {
            skip = 5;
        }

        curr_rad += 1.5 / perimeter;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);
}
