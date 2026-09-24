# DOCUSAURUS: Creation start
import rapier3d as rp

# The world that will contain our rigid-bodies.
world = rp.PhysicsWorld()

# Builder for a fixed rigid-body.
_ = rp.RigidBody.fixed()
# Builder for a dynamic rigid-body.
_ = rp.RigidBody.dynamic()
# Builder for a kinematic rigid-body controlled at the velocity level.
_ = rp.RigidBody.kinematic_velocity_based()
# Builder for a kinematic rigid-body controlled at the position level.
_ = rp.RigidBody.kinematic_position_based()
# The properties of the builder can also be given as keyword arguments.
_ = rp.RigidBody.dynamic(translation=(0.0, 5.0, 1.0), gravity_scale=0.5)
# Builder for a body with a status specified by an enum.
rigid_body = (
    rp.RigidBody.new_body(rp.RigidBodyType.DYNAMIC)
    # The rigid body translation.
    # Default: zero vector.
    .translation((0.0, 5.0, 1.0))
    # The rigid body rotation, as a scaled rotation axis.
    # Default: no rotation.
    .rotation((0.0, 0.0, 5.0))
    # The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
    # Default: the identity isometry.
    .position(rp.Isometry3((1.0, 3.0, 2.0), rp.Rotation3.from_scaled_axis((0.0, 0.0, 0.4))))
    # The linear velocity of this body.
    # Default: zero velocity.
    .linvel((1.0, 3.0, 4.0))
    # The angular velocity of this body.
    # Default: zero velocity.
    .angvel((3.0, 0.0, 1.0))
    # The scaling factor applied to the gravity affecting the rigid-body.
    # Default: 1.0
    .gravity_scale(0.5)
    # Whether or not this body can sleep.
    # Default: True
    .can_sleep(True)
    # Whether or not CCD is enabled for this rigid-body.
    # Default: False
    .ccd_enabled(False)
    # All done, actually build the rigid-body.
    .build()
)
# Insert the rigid-body into the world.
rigid_body_handle = world.add_body(rigid_body)
# DOCUSAURUS: Creation stop

# DOCUSAURUS: Position1 start
# Set the position when the rigid-body is created.
rigid_body = (
    rp.RigidBody.dynamic()
    # The rigid body translation.
    # Default: zero vector.
    .translation((0.0, 5.0, 1.0))
    # The rigid body rotation, as a scaled rotation axis.
    # Default: no rotation.
    .rotation((0.2, 0.0, 0.0))
    # The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
    # Default: the identity isometry.
    .position(rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.2, 0.0, 0.0))))
    # All done, actually build the rigid-body.
    .build()
)
# DOCUSAURUS: Position1 stop
# Insert the rigid-body into the world.
rigid_body_handle = world.add_body(rigid_body)

# DOCUSAURUS: Position2 start
# Set the position after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
# Setting these properties automatically wakes the rigid-body up.
rigid_body.translation = (0.0, 5.0, 1.0)
rigid_body.rotation = rp.Rotation3.from_scaled_axis((0.2, 0.0, 0.0))
assert rigid_body.translation == rp.Vec3(0.0, 5.0, 1.0)
assert rigid_body.rotation.scaled_axis == rp.Vec3(0.2, 0.0, 0.0)

rigid_body.position = rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.0, 0.4, 0.0)))
assert rigid_body.position == rp.Isometry3(
    (1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.0, 0.4, 0.0))
)
# DOCUSAURUS: Position2 stop

# DOCUSAURUS: KinematicPosition start
platform_handle = world.add_body(rp.RigidBody.kinematic_position_based(translation=(0.0, 1.0, 0.0)))
platform = world.rigid_bodies[platform_handle]

# Move the platform up by 0.01 at each step.
for _ in range(10):
    next_translation = platform.translation + rp.Vec3(0.0, 0.01, 0.0)
    platform.set_next_kinematic_translation(next_translation)
    # The position isn't modified until the next step.
    assert platform.next_position.translation == next_translation
    world.step()
# DOCUSAURUS: KinematicPosition stop
assert abs(platform.translation.y - 1.1) < 1.0e-5

# DOCUSAURUS: Velocity1 start
# Set the velocities when the rigid-body is created.
rigid_body = (
    rp.RigidBody.dynamic()
    # The linear velocity of this body.
    # Default: zero velocity.
    .linvel((1.0, 3.0, 4.0))
    # The angular velocity of this body.
    # Default: zero velocity.
    .angvel((3.0, 0.0, 0.0))
    # All done, actually build the rigid-body.
    .build()
)
# DOCUSAURUS: Velocity1 stop
# Insert the rigid-body into the world.
rigid_body_handle = world.add_body(rigid_body)

# DOCUSAURUS: Velocity2 start
# Set the velocities after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
# Setting these properties automatically wakes the rigid-body up.
rigid_body.linvel = (1.0, 3.0, 4.0)
rigid_body.angvel = (3.0, 0.0, 0.0)
assert rigid_body.linvel == rp.Vec3(1.0, 3.0, 4.0)
assert rigid_body.angvel == rp.Vec3(3.0, 0.0, 0.0)
# DOCUSAURUS: Velocity2 stop

# DOCUSAURUS: Gravity1 start
# Set the gravity scale when the rigid-body is created.
rigid_body = (
    rp.RigidBody.dynamic()
    # Divide by 2 the strength of gravity for this rigid-body.
    .gravity_scale(0.5)
    .build()
)
# DOCUSAURUS: Gravity1 stop

# DOCUSAURUS: Gravity2 start
# Set the gravity scale after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
# Setting this property automatically wakes the rigid-body up.
rigid_body.gravity_scale = 0.5
assert rigid_body.gravity_scale == 0.5
# DOCUSAURUS: Gravity2 stop

# DOCUSAURUS: Forces start
rigid_body = world.rigid_bodies[rigid_body_handle]

# The rigid-body is woken up, unless `wake_up=False` is given.
rigid_body.reset_forces()  # Reset the forces to zero.
rigid_body.reset_torques()  # Reset the torques to zero.
rigid_body.add_force((0.0, 1000.0, 0.0))
rigid_body.add_torque((100.0, 0.0, 0.0))
rigid_body.add_force_at_point((0.0, 1000.0, 0.0), (1.0, 2.0, 3.0))

rigid_body.apply_impulse((0.0, 1000.0, 0.0))
rigid_body.apply_torque_impulse((100.0, 0.0, 0.0))
rigid_body.apply_impulse_at_point((0.0, 1000.0, 0.0), (1.0, 2.0, 3.0))
# DOCUSAURUS: Forces stop

# DOCUSAURUS: Mass1 start
rigid_body_handle = world.add_body(rp.RigidBody.dynamic())
# The default density is 1.0, we are setting 2.0 for this example.
collider = rp.Collider.ball(1.0).density(2.0)
# When the collider is attached, the rigid-body's mass and angular
# inertia is automatically updated to take the collider into account.
world.add_collider(collider, parent=rigid_body_handle)
# DOCUSAURUS: Mass1 stop

# DOCUSAURUS: Mass2 start
# Set the mass-properties when the rigid-body is created.
rigid_body = (
    rp.RigidBody.dynamic()
    .additional_mass(0.5)
    # Sets both the mass and angular inertia at once.
    .additional_mass_properties(
        rp.MassProperties(local_com=(0.0, 1.0, 0.0), mass=0.5, principal_inertia=(0.3, 0.2, 0.1))
    )
    .build()
)
# DOCUSAURUS: Mass2 stop

# DOCUSAURUS: Mass3 start
# Set the mass-properties after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
# The rigid-body is woken up, unless `wake_up=False` is given.
rigid_body.set_additional_mass_properties(
    rp.MassProperties(local_com=(0.0, 1.0, 0.0), mass=0.5, principal_inertia=(0.3, 0.2, 0.1))
)
# DOCUSAURUS: Mass3 stop

# DOCUSAURUS: LockedAxes1 start
# Lock translations/rotations when the rigid-body is created.
rigid_body = (
    rp.RigidBody.dynamic()
    # Prevent translations along all axes, and rotations around all axes.
    .locked_axes(rp.LockedAxes.TRANSLATION_LOCKED | rp.LockedAxes.ROTATION_LOCKED)
    # Only enable rotations around the X axis.
    .enabled_rotations((True, False, False))
    .build()
)
# DOCUSAURUS: LockedAxes1 stop
assert rigid_body.enabled_translations == (False, False, False)
assert rigid_body.enabled_rotations == (True, False, False)

# DOCUSAURUS: LockedAxes2 start
# Lock translations/rotations after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
# Setting these properties automatically wakes the rigid-body up.
rigid_body.locked_axes = rp.LockedAxes.TRANSLATION_LOCKED | rp.LockedAxes.ROTATION_LOCKED
# Only enable rotations around the X axis.
rigid_body.enabled_rotations = (True, False, False)
# DOCUSAURUS: LockedAxes2 stop
assert rigid_body.enabled_translations == (False, False, False)
assert rigid_body.enabled_rotations == (True, False, False)

# DOCUSAURUS: Damping1 start
# Set the damping coefficients when the rigid-body is created.
rigid_body = rp.RigidBody.dynamic().linear_damping(0.5).angular_damping(1.0).build()
# DOCUSAURUS: Damping1 stop

# DOCUSAURUS: Damping2 start
# Set the damping coefficients after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
rigid_body.linear_damping = 0.5
rigid_body.angular_damping = 1.0
assert rigid_body.linear_damping == 0.5
assert rigid_body.angular_damping == 1.0
# DOCUSAURUS: Damping2 stop

# DOCUSAURUS: Dominance1 start
# Set the dominance group when the rigid-body is created.
rigid_body = rp.RigidBody.dynamic().dominance_group(10).build()
# DOCUSAURUS: Dominance1 stop

# DOCUSAURUS: Dominance2 start
# Set the dominance group after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
rigid_body.dominance_group = 10
assert rigid_body.dominance_group == 10
# DOCUSAURUS: Dominance2 stop

# DOCUSAURUS: Ccd1 start
# Enable CCD when the rigid-body is created.
rigid_body = rp.RigidBody.dynamic().ccd_enabled(True).build()
# DOCUSAURUS: Ccd1 stop

# DOCUSAURUS: Ccd2 start
# Enable CCD after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
rigid_body.ccd_enabled = True
assert rigid_body.ccd_enabled
# DOCUSAURUS: Ccd2 stop

# DOCUSAURUS: Userdata1 start
# Set the user-data when the rigid-body is created.
rigid_body = rp.RigidBody.dynamic().user_data(42).build()
# DOCUSAURUS: Userdata1 stop

# DOCUSAURUS: Userdata2 start
# Set the user-data after the rigid-body creation.
rigid_body = world.rigid_bodies[rigid_body_handle]
rigid_body.user_data = 42
assert rigid_body.user_data == 42
# DOCUSAURUS: Userdata2 stop

# DOCUSAURUS: SolverSettings start
# Give a rigid-body more solver accuracy than the rest of the scene.
rigid_body = (
    rp.RigidBody.dynamic()
    # Extra substeps run for the whole island component this body belongs to.
    .additional_solver_iterations(4)
    # Extra internal PGS iterations run per substep for that same component.
    .additional_pgs_iterations(2)
    # Predictive contacts generated up to that distance ahead of the body's path: a cheaper
    # alternative to CCD for slow-but-thin or moderately fast objects.
    .soft_ccd_prediction(0.5)
    # Let the body exceed the angular speed cap, e.g. for a wheel.
    .allow_fast_rotation(True)
    # Gyroscopic forces give more realistic behaviors, e.g. the precession of a spinning top.
    .gyroscopic_forces(True)
    .build()
)
# DOCUSAURUS: SolverSettings stop
assert rigid_body.allow_fast_rotation

# Step once to make sure the scene built above is valid.
world.step()
