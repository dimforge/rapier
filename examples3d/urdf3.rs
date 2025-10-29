use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: true,
        create_colliders_from_collision_shapes: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (mut robot, _) =
        UrdfRobot::from_file("assets/3d/T12/urdf/T12.URDF", options, None).unwrap();

    // The robot can be inserted using impulse joints.
    // (We clone because we want to insert the same robot once more afterward.)
    robot
        .clone()
        .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // Insert the robot a second time, but using multibody joints this time.
    robot.append_transform(&Isometry::translation(10.0, 0.0, 0.0));
    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![20.0, 20.0, 20.0], point![5.0, 0.0, 0.0]);
}
