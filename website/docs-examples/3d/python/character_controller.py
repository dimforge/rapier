import math

import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

# Create the ground.
world.colliders.insert(rp.Collider.cuboid(100.0, 0.1, 100.0).build())

# Create the body to control.
collider = rp.Collider.ball(0.5).restitution(0.7).build()
rigid_body_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
    colliders=[collider],
)

character_shape = collider.shape
dt = world.integration_parameters.dt
# Run the game loop, stepping the simulation once per frame.
for _ in range(200):
    character_pos = world.rigid_bodies[rigid_body_handle].position
    # DOCUSAURUS: Setup start
    # The translation we would like to apply if there were no obstacles.
    desired_translation = (1.0, -2.0, 3.0)
    # Create the character controller, here with the default configuration.
    character_controller = rp.KinematicCharacterController()
    # Make sure the character we are trying to move isn’t considered an obstacle.
    query_filter = rp.QueryFilter().exclude_rigid_body(rigid_body_handle)
    # Calculate the possible movement.
    corrected_movement = character_controller.move_shape(
        dt,  # The timestep length (can be set to world.integration_parameters.dt).
        None,  # The rigid-body set, unused: the one of the query pipeline is used.
        None,  # The collider set, unused: the one of the query pipeline is used.
        world.query_pipeline,  # The query pipeline containing the obstacles.
        character_shape,  # The character’s shape.
        character_pos,  # The character’s initial position.
        desired_translation,
        query_filter,  # The obstacles to consider.
    )
    # TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described below.
    # DOCUSAURUS: Setup stop
    assert isinstance(corrected_movement, rp.EffectiveCharacterMovement)

    world.step()

    query_filter = rp.QueryFilter().exclude_rigid_body(rigid_body_handle)
    character_pos = world.rigid_bodies[rigid_body_handle].position
    desired_translation = (1.0, -2.0, 0.0)

    # DOCUSAURUS: Collisions1 start
    character_controller = rp.KinematicCharacterController()

    def on_collision(collision):
        # Handle or collect the collision in this callback.
        pass

    # Give a callback to handle or collect the collisions while
    # the character is being moved.
    character_controller.move_shape(
        dt,
        None,
        None,
        world.query_pipeline,
        character_shape,
        character_pos,
        desired_translation,
        query_filter,
        events_callback=on_collision,
    )
    # DOCUSAURUS: Collisions1 stop

    character_controller = rp.KinematicCharacterController()
    character_mass = 2.0

    # DOCUSAURUS: Collisions2 start
    # First, collect all the collisions.
    collisions = []
    character_controller.move_shape(
        dt,
        None,
        None,
        world.query_pipeline,
        character_shape,
        character_pos,
        desired_translation,
        query_filter,
        events_callback=collisions.append,
    )
    # Then, let the character controller solve (and apply) the collision impulses
    # to the dynamic rigid-bodies hit along its path.
    character_controller.solve_character_collision_impulses(
        dt,
        world.rigid_bodies,
        world.colliders,
        world.query_pipeline,
        character_shape,
        None,  # Unused: each collision stores the pose of the character when it happened.
        character_mass,
        collisions,
        query_filter,
    )
    # DOCUSAURUS: Collisions2 stop

    character_body = world.rigid_bodies[rigid_body_handle]
    print("Character body position:", character_body.translation)

# Once the ball rests on the ground, moving it down and sideways hits the ground.
assert all(isinstance(collision, rp.CharacterCollision) for collision in collisions)
assert all(collision.hit.time_of_impact >= 0.0 for collision in collisions)
assert len(collisions) > 0

character_controller = rp.KinematicCharacterController()
# DOCUSAURUS: Offset start
# The character offset is set to 0.01.
character_controller.offset = rp.CharacterLength.absolute(0.01)
# The character offset is set to 0.01 multiplied by the shape’s height.
character_controller.offset = rp.CharacterLength.relative(0.01)
# DOCUSAURUS: Offset stop
assert character_controller.offset == rp.CharacterLength.relative(0.01)

# DOCUSAURUS: UpVector start
# Set the up-vector to the positive X axis.
character_controller.up = (1.0, 0.0, 0.0)
# DOCUSAURUS: UpVector stop
assert character_controller.up == rp.Vec3(1.0, 0.0, 0.0)

# DOCUSAURUS: Slopes start
# Don’t allow climbing slopes larger than 45 degrees.
character_controller.max_slope_climb_angle = math.radians(45.0)
# Automatically slide down on slopes smaller than 30 degrees.
character_controller.min_slope_slide_angle = math.radians(30.0)
# DOCUSAURUS: Slopes stop
assert abs(character_controller.min_slope_slide_angle - math.radians(30.0)) < 1.0e-6

# DOCUSAURUS: Stairs start
# Set autostep to None to disable it.
character_controller.autostep = None
# Autostep if the step height is smaller than 0.5, and its width larger than 0.2.
character_controller.autostep = rp.CharacterAutostep(
    max_height=rp.CharacterLength.absolute(0.5),
    min_width=rp.CharacterLength.absolute(0.2),
    include_dynamic_bodies=True,
)
# Autostep if the step height is smaller than 0.3 multiplied by the character’s height,
# and its width larger than 0.5 multiplied by the character’s width (i.e. half the character’s
# width).
character_controller.autostep = rp.CharacterAutostep(
    max_height=rp.CharacterLength.relative(0.3),
    min_width=rp.CharacterLength.relative(0.5),
    include_dynamic_bodies=True,
)
# DOCUSAURUS: Stairs stop
assert character_controller.autostep.min_width == rp.CharacterLength.relative(0.5)

# DOCUSAURUS: Snap start
# Set snap-to-ground to None to disable it.
character_controller.snap_to_ground = None
# Snap to the ground if the vertical distance to the ground is smaller than 0.5.
character_controller.snap_to_ground = rp.CharacterLength.absolute(0.5)
# Snap to the ground if the vertical distance to the ground is smaller than 0.2 times the character’s height.
character_controller.snap_to_ground = rp.CharacterLength.relative(0.2)
# DOCUSAURUS: Snap stop
assert character_controller.snap_to_ground == rp.CharacterLength.relative(0.2)
