use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use rapier_urdf::{RapierRobot, UrdfLoaderOptions};

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
        apply_imported_mass_props: true,
        make_roots_fixed: true,
        // rigid_body_blueprint: RigidBodyBuilder::dynamic().gravity_scale(0.0),
        collider_blueprint: ColliderBuilder::ball(0.0)
            .density(0.0)
            .active_collision_types(ActiveCollisionTypes::empty()),
        ..Default::default()
    };
    let (mut robot, _) = RapierRobot::from_file("assets/3d/sample.urdf", options).unwrap();

    // robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    robot.insert_using_multibody_joints(&mut bodies, &mut colliders, &mut multibody_joints);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
