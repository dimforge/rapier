fn main() {
    let mut collider_set = ColliderSet::new();

    // DOCUSAURUS: Creation start
    use rapier3d::prelude::*;

    // The set that will contain our rigid-bodies.
    let mut rigid_body_set = RigidBodySet::new();

    // Builder for a fixed rigid-body.
    let _ = RigidBodyBuilder::fixed();
    // Builder for a dynamic rigid-body.
    let _ = RigidBodyBuilder::dynamic();
    // Builder for a kinematic rigid-body controlled at the velocity level.
    let _ = RigidBodyBuilder::kinematic_velocity_based();
    // Builder for a kinematic rigid-body controlled at the position level.
    let _ = RigidBodyBuilder::kinematic_position_based();
    // Builder for a body with a status specified by an enum.
    let rigid_body = RigidBodyBuilder::new(RigidBodyType::Dynamic)
        // The rigid body translation.
        // Default: zero vector.
        .translation(vector![0.0, 5.0, 1.0])
        // The rigid body rotation.
        // Default: no rotation.
        .rotation(vector![0.0, 0.0, 5.0])
        // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
        // Default: the identity isometry.
        .position(Isometry::new(
            vector![1.0, 3.0, 2.0],
            vector![0.0, 0.0, 0.4],
        ))
        // The linear velocity of this body.
        // Default: zero velocity.
        .linvel(vector![1.0, 3.0, 4.0])
        // The angular velocity of this body.
        // Default: zero velocity.
        .angvel(vector![3.0, 0.0, 1.0])
        // The scaling factor applied to the gravity affecting the rigid-body.
        // Default: 1.0
        .gravity_scale(0.5)
        // Whether or not this body can sleep.
        // Default: true
        .can_sleep(true)
        // Whether or not CCD is enabled for this rigid-body.
        // Default: false
        .ccd_enabled(false)
        // All done, actually build the rigid-body.
        .build();
    // Insert the rigid-body into the set.
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: Position1 start
    /* Set the position when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        // The rigid body translation.
        // Default: zero vector.
        .translation(vector![0.0, 5.0, 1.0])
        // The rigid body rotation.
        // Default: no rotation.
        .rotation(vector![0.2, 0.0, 0.0])
        // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
        // Default: the identity isometry.
        .position(Isometry::new(
            vector![1.0, 2.0, 3.0],
            vector![0.2, 0.0, 0.0],
        ))
        // All done, actually build the rigid-body.
        .build();
    // DOCUSAURUS: Position1 stop
    // Insert the rigid-body into the set.
    let rigid_body_handle = rigid_body_set.insert(rigid_body);

    // DOCUSAURUS: Position2 start
    /* Set the position after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_translation(vector![0.0, 5.0, 1.0], true);
    rigid_body.set_rotation(Rotation::from_scaled_axis(vector![0.2, 0.0, 0.0]), true);
    assert_eq!(*rigid_body.translation(), vector![0.0, 5.0, 1.0]);
    assert_eq!(rigid_body.rotation().scaled_axis(), vector![0.2, 0.0, 0.0]);

    rigid_body.set_position(
        Isometry::new(vector![1.0, 2.0, 3.0], vector![0.0, 0.4, 0.0]),
        true,
    );
    assert_eq!(
        *rigid_body.position(),
        Isometry::new(vector![1.0, 2.0, 3.0], vector![0.0, 0.4, 0.0])
    );
    // DOCUSAURUS: Position2 stop

    // DOCUSAURUS: Velocity1 start
    /* Set the velocities when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        // The linear velocity of this body.
        // Default: zero velocity.
        .linvel(vector![1.0, 3.0, 4.0])
        // The angular velocity of this body.
        // Default: zero velocity.
        .angvel(vector![3.0, 0.0, 0.0])
        // All done, actually build the rigid-body.
        .build();
    // DOCUSAURUS: Velocity1 stop
    // Insert the rigid-body into the set.
    let rigid_body_handle = rigid_body_set.insert(rigid_body);

    // DOCUSAURUS: Velocity2 start
    /* Set the velocities after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_linvel(vector![1.0, 3.0, 4.0], true);
    rigid_body.set_angvel(vector![3.0, 0.0, 0.0], true);
    assert_eq!(*rigid_body.linvel(), vector![1.0, 3.0, 4.0]);
    assert_eq!(*rigid_body.angvel(), vector![3.0, 0.0, 0.0]);
    // DOCUSAURUS: Velocity2 stop

    // DOCUSAURUS: Forces start
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();

    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.reset_forces(true); // Reset the forces to zero.
    rigid_body.reset_torques(true); // Reset the torques to zero.
    rigid_body.add_force(vector![0.0, 1000.0, 0.0], true);
    rigid_body.add_torque(vector![100.0, 0.0, 0.0], true);
    rigid_body.add_force_at_point(vector![0.0, 1000.0, 0.0], point![1.0, 2.0, 3.0], true);

    rigid_body.apply_impulse(vector![0.0, 1000.0, 0.0], true);
    rigid_body.apply_torque_impulse(vector![100.0, 0.0, 0.0], true);
    rigid_body.apply_impulse_at_point(vector![0.0, 1000.0, 0.0], point![1.0, 2.0, 3.0], true);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Mass2 start
    /* Set the mass-properties when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .additional_mass(0.5)
        // Sets both the mass and angular inertia at once.
        .additional_mass_properties(MassProperties::new(
            point![0.0, 1.0, 0.0],
            0.5,
            vector![0.3, 0.2, 0.1],
        ))
        .build();
    // DOCUSAURUS: Mass2 stop

    // DOCUSAURUS: Mass3 start
    /* Set the mass-properties after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_additional_mass_properties(
        MassProperties::new(point![0.0, 1.0, 0.0], 0.5, vector![0.3, 0.2, 0.1]),
        true,
    );
    // DOCUSAURUS: Mass3 stop

    // DOCUSAURUS: LockedAxes1 start
    /* Lock translations/rotations when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .lock_translations() // prevent translations along along all axes.
        .lock_rotations() // prevent rotations along all axes.
        .enabled_rotations(true, false, false) // only enable rotations along the X axis.
        .build();
    // DOCUSAURUS: LockedAxes1 stop

    // DOCUSAURUS: LockedAxes2 start
    /* Lock translations/rotations after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The last `true` argument makes sure the rigid-body is awake.
    rigid_body.lock_translations(true, true);
    rigid_body.lock_rotations(true, true);
    rigid_body.set_enabled_rotations(true, false, false, true);
    // DOCUSAURUS: LockedAxes2 stop
}
