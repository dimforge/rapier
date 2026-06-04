use rapier2d::prelude::*;

fn main() {
    let mut collider_set = ColliderSet::new();

    // DOCUSAURUS: Creation start
    use rapier2d::prelude::*;

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
        .translation(vector![0.0, 5.0])
        // The rigid body rotation.
        // Default: no rotation.
        .rotation(5.0)
        // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
        // Default: the identity isometry.
        .position(Isometry::new(vector![1.0, 2.0], 0.4))
        // The linear velocity of this body.
        // Default: zero velocity.
        .linvel(vector![1.0, 2.0])
        // The angular velocity of this body.
        // Default: zero velocity.
        .angvel(2.0)
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
        .translation(vector![0.0, 5.0])
        // The rigid body rotation.
        // Default: no rotation.
        .rotation(5.0)
        // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
        // Default: the identity isometry.
        .position(Isometry::new(vector![1.0, 2.0], 0.4))
        // All done, actually build the rigid-body.
        .build();
    // DOCUSAURUS: Position1 stop
    // Insert the rigid-body into the set.
    let rigid_body_handle = rigid_body_set.insert(rigid_body);

    // DOCUSAURUS: Position2 start
    use nalgebra::UnitComplex;
    /* Set the position after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_translation(vector![0.0, 5.0], true);
    rigid_body.set_rotation(UnitComplex::new(0.2), true);
    assert_eq!(*rigid_body.translation(), vector![0.0, 5.0]);
    assert_eq!(rigid_body.rotation().angle(), 0.2);

    rigid_body.set_position(Isometry::new(vector![1.0, 2.0], 0.4), true);
    assert_eq!(
        *rigid_body.position(),
        Isometry::new(vector![1.0, 2.0], 0.4)
    );
    // DOCUSAURUS: Position2 stop

    // DOCUSAURUS: Velocity1 start
    /* Set the velocities when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        // The linear velocity of this body.
        // Default: zero velocity.
        .linvel(vector![1.0, 3.0])
        // The angular velocity of this body.
        // Default: zero velocity.
        .angvel(3.0)
        // All done, actually build the rigid-body.
        .build();
    // DOCUSAURUS: Velocity1 stop
    // Insert the rigid-body into the set.
    let rigid_body_handle = rigid_body_set.insert(rigid_body);

    // DOCUSAURUS: Velocity2 start
    /* Set the velocities after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_linvel(vector![1.0, 3.0], true);
    rigid_body.set_angvel(3.0, true);
    assert_eq!(*rigid_body.linvel(), vector![1.0, 3.0]);
    assert_eq!(rigid_body.angvel(), 3.0);
    // DOCUSAURUS: Velocity2 stop

    // DOCUSAURUS: Gravity1 start
    /* Set the gravity scale when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        // Divide by 2 the strength of gravity for this rigid-body.
        .gravity_scale(0.5)
        .build();
    // DOCUSAURUS: Gravity1 stop

    // DOCUSAURUS: Gravity2 start
    /* Set the gravity scale after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.set_gravity_scale(0.5, true);
    assert_eq!(rigid_body.gravity_scale(), 0.5);
    // DOCUSAURUS: Gravity2 stop

    // DOCUSAURUS: Forces start
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();

    // The `true` argument makes sure the rigid-body is awake.
    rigid_body.reset_forces(true); // Reset the forces to zero.
    rigid_body.reset_torques(true); // Reset the torques to zero.
    rigid_body.add_force(vector![0.0, 1000.0], true);
    rigid_body.add_torque(100.0, true);
    rigid_body.add_force_at_point(vector![0.0, 1000.0], point![1.0, 2.0], true);

    rigid_body.apply_impulse(vector![0.0, 1000.0], true);
    rigid_body.apply_torque_impulse(100.0, true);
    rigid_body.apply_impulse_at_point(vector![0.0, 1000.0], point![1.0, 2.0], true);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Mass1 start
    let rigid_body = RigidBodyBuilder::dynamic().build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    // The default density is 1.0, we are setting 2.0 for this example.
    let collider = ColliderBuilder::ball(1.0).density(2.0).build();
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Mass1 stop
    // DOCUSAURUS: Mass2 start
    /* Set the mass-properties when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .additional_mass(0.5)
        // Sets both the mass and angular inertia at once.
        .additional_mass_properties(MassProperties::new(point![0.0, 1.0], 0.5, 0.3))
        .build();
    // DOCUSAURUS: Mass2 stop

    // DOCUSAURUS: Mass3 start
    /* Set the mass-properties after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The `true` argument makes sure the rigid-body is awake.
    rigid_body
        .set_additional_mass_properties(MassProperties::new(point![0.0, 1.0], 0.5, 0.3), true);
    // DOCUSAURUS: Mass3 stop

    // DOCUSAURUS: LockedAxes1 start
    /* Lock translations/rotations when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .lock_translations() // prevent translations along along all axes.
        .lock_rotations() // prevent rotations.
        .build();
    // DOCUSAURUS: LockedAxes1 stop

    // DOCUSAURUS: LockedAxes2 start
    /* Lock translations/rotations after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    // The last `true` argument makes sure the rigid-body is awake.
    rigid_body.lock_translations(true, true);
    rigid_body.lock_rotations(true, true);
    // DOCUSAURUS: LockedAxes2 stop

    // DOCUSAURUS: Damping1 start
    /* Set the damping coefficients when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .linear_damping(0.5)
        .angular_damping(1.0)
        .build();
    // DOCUSAURUS: Damping1 stop

    // DOCUSAURUS: Damping2 start
    /* Set the damping coefficients after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    rigid_body.set_linear_damping(0.5);
    rigid_body.set_angular_damping(1.0);
    assert_eq!(rigid_body.linear_damping(), 0.5);
    assert_eq!(rigid_body.angular_damping(), 1.0);
    // DOCUSAURUS: Damping2 stop

    // DOCUSAURUS: Dominance1 start
    /* Set the dominance group when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic().dominance_group(10).build();
    // DOCUSAURUS: Dominance1 stop

    // DOCUSAURUS: Dominance2 start
    /* Set the dominance group after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    rigid_body.set_dominance_group(10);
    assert_eq!(rigid_body.dominance_group(), 10);
    // DOCUSAURUS: Dominance2 stop

    // DOCUSAURUS: Ccd1 start
    /* Enable CCD when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic().ccd_enabled(true).build();
    // DOCUSAURUS: Ccd1 stop

    // DOCUSAURUS: Ccd2 start
    /* Enable CCD after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    rigid_body.enable_ccd(true);
    assert_eq!(rigid_body.is_ccd_enabled(), true);
    // DOCUSAURUS: Ccd2 stop

    // DOCUSAURUS: Userdata1 start
    /* Set the user-data when the rigid-body is created. */
    let rigid_body = RigidBodyBuilder::dynamic().user_data(42).build();
    // DOCUSAURUS: Userdata1 stop

    // DOCUSAURUS: Userdata2 start
    /* Set the user-data after the rigid-body creation. */
    let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
    rigid_body.user_data = 42;
    assert_eq!(rigid_body.user_data, 42);
    // DOCUSAURUS: Userdata2 stop
}
