use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 1.0;
    let ground_height = 0.01;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Setup groups
     */
    let num_segments = 10;
    let body = RigidBodyBuilder::fixed();
    let mut last_body = bodies.insert(body);
    let mut last_link = MultibodyJointHandle::invalid();

    for i in 0..num_segments {
        let size = 1.0 / num_segments as f32;
        let body = RigidBodyBuilder::dynamic().can_sleep(false);
        let new_body = bodies.insert(body);
        // NOTE: we add a sensor collider just to make the destbed draw a rectangle to make
        //       the demo look nicer. IK could be used without cuboid.
        let collider = ColliderBuilder::cuboid(size / 8.0, size / 2.0)
            .density(0.0)
            .sensor(true);
        colliders.insert_with_parent(collider, new_body, &mut bodies);

        let link_ab = RevoluteJointBuilder::new()
            .local_anchor1(point![0.0, size / 2.0 * (i != 0) as usize as f32])
            .local_anchor2(point![0.0, -size / 2.0])
            .build()
            .data;

        last_link = multibody_joints
            .insert(last_body, new_body, link_ab, true)
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
            let target_point = mouse_point.coords;

            let options = InverseKinematicsOption {
                constrained_axes: JointAxesMask::LIN_AXES,
                ..Default::default()
            };

            multibody.inverse_kinematics(
                &physics.bodies,
                link_id,
                &options,
                &Isometry::from(target_point),
                |_| true,
                &mut displacements,
            );
            multibody.apply_displacements(displacements.as_slice());
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 0.0], 300.0);
}
