use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let options = MjcfLoaderOptions {
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Pose::rotation(Vector::X * -std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (mut robot, _) =
        MjcfRobot::from_file("assets/3d/agility_cassie/scene.xml", options).unwrap();

    // The robot can be inserted using impulse joints.
    // (We clone because we want to insert the same robot once more afterward.)
    robot.clone().insert_using_impulse_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.impulse_joints,
    );
    // Insert the robot a second time, but using multibody joints this time.
    robot.append_transform(&Pose::translation(0.0, 0.0, 1.0));
    robot.insert_using_multibody_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.multibody_joints,
        &mut world.impulse_joints,
        MjcfMultibodyOptions::SKIP_LOOP_CLOSURES | MjcfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(0.0, 0.0, 0.0));
}
