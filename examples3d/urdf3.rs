use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use rapier_urdf::{UrdfLoaderOptions, UrdfRobot};

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
        apply_imported_mass_props: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
        rigid_body_blueprint: RigidBodyBuilder::default().gravity_scale(1.0),
        collider_blueprint: ColliderBuilder::default()
            .density(1.0)
            .active_collision_types(ActiveCollisionTypes::empty()),
        ..Default::default()
    };
    let (mut robot, _) =
        UrdfRobot::from_file("assets/3d/T12/urdf/T12.URDF", options, None).unwrap();
    // let (mut robot, _) = UrdfRobot::from_file("assets/3d/sample.urdf", options).unwrap();

    // robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    robot.insert_using_multibody_joints(&mut bodies, &mut colliders, &mut multibody_joints);

    testbed.add_callback(move |mut graphics, physics, _, state| {
        for (_, body) in physics.bodies.iter() {
            println!("pose: {:?}", body.position());
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
