use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

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
    let ground_size = 5.0;
    let ground_height = 0.1;

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
        let body = RigidBodyBuilder::dynamic().can_sleep(false);
        let new_body = bodies.insert(body);

        let link_ab = RevoluteJointBuilder::new()
            .local_anchor1(point![0.0, 0.5 / num_segments as f32])
            .local_anchor2(point![0.0, -0.5 / num_segments as f32])
            .build()
            .data;

        last_link = multibody_joints
            .insert(last_body, new_body, link_ab, true)
            .unwrap();

        last_body = new_body;
    }

    // TODO: reading the multibody ndofs at this point is incorrect (it will not account for the
    //       root being static yet). This should be improved.

    let mut displacements = DVector::zeros(0);

    testbed.add_callback(move |graphics, physics, _, state| {
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

            let target_point = mouse_point.coords;
            let mut delta = target_point - physics.bodies[last_body].position().translation.vector;
            let err_dist = delta.norm();

            if err_dist > 1.0e-5 {
                delta *= err_dist.min(0.4) / err_dist;

                // Push 0 for the angular target.
                let delta = delta.push(0.0);
                let options = JacobianIkOptions {
                    constrained_axes: JointAxesMask::LIN_AXES,
                    ..Default::default()
                };
                multibody.inverse_kinematics(
                    &physics.bodies,
                    link_id,
                    &options,
                    &Isometry::from(target_point),
                    &mut displacements,
                );
                // multibody.inverse_kinematics(link_id, &delta, 1.0, 100, &mut displacements);
                multibody.apply_displacements(displacements.as_slice());
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 1.0], 100.0);
}
