use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 1.0;
    let ground_height = 0.01;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    let _ = world.insert(rigid_body, collider);

    /*
     * Setup groups
     */
    let num_segments = 10;
    let body = RigidBodyBuilder::fixed();
    let mut last_body = world.insert_body(body);
    let mut last_link = MultibodyJointHandle::invalid();

    for i in 0..num_segments {
        let size = 1.0 / num_segments as f32;
        let body = RigidBodyBuilder::dynamic().can_sleep(false);
        // NOTE: we add a sensor collider just to make the destbed draw a rectangle to make
        //       the demo look nicer. IK could be used without cuboid.
        let collider = ColliderBuilder::cuboid(size / 8.0, size / 2.0)
            .density(0.0)
            .sensor(true);
        let (new_body, _) = world.insert(body, collider);

        let link_ab = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(0.0, size / 2.0 * (i != 0) as usize as f32))
            .local_anchor2(Vector::new(0.0, -size / 2.0))
            .build()
            .data;

        last_link = world
            .insert_multibody_joint(last_body, new_body, link_ab)
            .unwrap();

        last_body = new_body;
    }

    let mut displacements = DVector::zeros(0);

    testbed.add_callback(move |graphics, physics, _, _| {
        let Some(graphics) = graphics else { return };
        if let Some((multibody, link_id)) = physics.multibody_joints.get_mut(last_link) {
            // Ensure our displacement vector has the right number of elements.
            if displacements.nrows() < multibody.ndofs() {
                displacements = DVector::zeros(multibody.ndofs());
            } else {
                displacements.fill(0.0);
            }

            let Some(mouse_point) = graphics.mouse().point else {
                return;
            };

            // We will have the endpoint track the mouse position.
            let target_point = Vector::new(mouse_point.x, mouse_point.y);

            let options = InverseKinematicsOption {
                constrained_axes: JointAxesMask::LIN_AXES,
                ..Default::default()
            };

            multibody.inverse_kinematics(
                &physics.bodies,
                link_id,
                &options,
                &Pose::from_translation(target_point),
                |_| true,
                &mut displacements,
            );
            multibody.apply_displacements(displacements.as_slice());
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 300.0);
}
