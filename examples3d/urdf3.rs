use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: true,
        create_colliders_from_collision_shapes: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Pose::rotation(Vector::X * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (mut robot, _) =
        UrdfRobot::from_file("assets/3d/T12/urdf/T12.URDF", options, None).unwrap();

    // The robot can be inserted using impulse joints.
    // (We clone because we want to insert the same robot once more afterward.)
    robot
        .clone()
        .insert_using_impulse_joints(&mut world.bodies, &mut world.colliders, &mut world.impulse_joints);
    // Insert the robot a second time, but using multibody joints this time.
    robot.append_transform(&Pose::translation(10.0, 0.0, 0.0));
    robot.insert_using_multibody_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.multibody_joints,
        UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(20.0, 20.0, 20.0), Vec3::new(5.0, 0.0, 0.0));
}
