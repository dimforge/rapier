#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();

    /* Create the ground. */
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    /* Create the body to control. */
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = r2Vector(0.0, 10.0);
    R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &body);
    R2ColliderDesc collider = r2BallColliderDesc(0.5);
    collider.restitution = 0.7;
    R2ColliderHandle collider_handle = r2InsertCollider(rigid_body_handle, &collider);

    R2SharedShape *character_shape = r2Collider_CloneShape(collider_handle);
    R2Real dt = r2TimeStep(world);
    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        {
            R2Pose character_pos = r2RigidBody_Position(rigid_body_handle);
            // DOCUSAURUS: Setup start
            // The translation we would like to apply if there were no obstacles.
            R2Vector desired_translation = r2Vector(1.0, -2.0);
            // Create the character controller, here with the default configuration.
            R2KinematicCharacterController *character_controller = r2NewKinematicCharacterController();
            // Init the query options.
            R2QueryOptions options = r2DefaultQueryOptions();
            // Make sure the character we are trying to move isn't considered an obstacle.
            options.filter.exclude_rigid_body = rigid_body_handle;
            // Calculate the possible movement.
            R2CharacterMovement corrected_movement = r2KinematicCharacterController_MoveShape(
                world,                // The world containing the obstacles.
                &options,             // The query options (NULL for the default ones).
                character_controller, // The character controller.
                dt,                   // The timestep length (can be set to r2TimeStep(world)).
                character_shape,      // The character's shape.
                character_pos,        // The character's initial position.
                desired_translation);

            // TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described below.

            // Free the character controller once it is no longer needed.
            r2FreeKinematicCharacterController(character_controller);
            // DOCUSAURUS: Setup stop
            (void)corrected_movement;
        }

        r2Step(world, NULL, NULL);
        R2QueryOptions options = r2DefaultQueryOptions();
        // Make sure the character we are trying to move isn't considered an obstacle.
        options.filter.exclude_rigid_body = rigid_body_handle;

        {
            R2Pose character_pos = r2RigidBody_Position(rigid_body_handle);
            R2Vector desired_translation = r2Vector(1.0, -2.0);
            // DOCUSAURUS: Collisions1 start
            R2KinematicCharacterController *character_controller = r2NewKinematicCharacterController();
            // The collisions are recorded by the controller while the character is being moved.
            r2KinematicCharacterController_MoveShape(world, &options, character_controller, dt,
                                                     character_shape, character_pos,
                                                     desired_translation);
            // Read them after the movement (they remain available until the next movement
            // calculation of this controller).
            size_t num_collisions = r2KinematicCharacterController_Collisions(character_controller, NULL, 0);
            R2CharacterCollision *collisions = malloc(num_collisions * sizeof(R2CharacterCollision));
            r2KinematicCharacterController_Collisions(character_controller, collisions, num_collisions);
            for (size_t k = 0; k < num_collisions; k++) {
                R2CharacterCollision collision = collisions[k];
                /* Handle the collision with the collider `collision.collider`. */
                (void)collision;
            }
            free(collisions);
            // DOCUSAURUS: Collisions1 stop
            r2FreeKinematicCharacterController(character_controller);

            character_controller = r2NewKinematicCharacterController();
            R2Real character_mass = 2.0;

            // DOCUSAURUS: Collisions2 start
            // First, calculate the movement, which records all the collisions.
            r2KinematicCharacterController_MoveShape(world, &options, character_controller, dt,
                                                     character_shape, character_pos,
                                                     desired_translation);
            // Then, let the character controller solve (and apply) the collision impulses
            // to the dynamic rigid-bodies hit along its path. Note that this must be given the
            // same shape, timestep length, and query options as the movement calculation.
            r2KinematicCharacterController_SolveCharacterCollisionImpulses(
                character_controller, character_shape, dt, character_mass, &options);
            // DOCUSAURUS: Collisions2 stop
            r2FreeKinematicCharacterController(character_controller);
        }
        R2Vector translation = r2RigidBody_Translation(rigid_body_handle);
        printf("Character body position: (%f, %f)\n", (double)translation.x, (double)translation.y);
    }
    r2FreeSharedShape(character_shape);

    R2KinematicCharacterController *character_controller = r2NewKinematicCharacterController();
    // DOCUSAURUS: Offset start
    // The character offset is set to 0.01.
    r2KinematicCharacterController_SetOffset(character_controller,
                                             (R2CharacterLength){.value = 0.01, .relative = 0});
    // The character offset is set to 0.01 multiplied by the shape's height.
    r2KinematicCharacterController_SetOffset(character_controller,
                                             (R2CharacterLength){.value = 0.01, .relative = 1});
    // DOCUSAURUS: Offset stop

    // DOCUSAURUS: UpVector start
    // Set the up-vector to the positive X axis.
    r2KinematicCharacterController_SetUp(character_controller, r2Vector(1.0, 0.0));
    // DOCUSAURUS: UpVector stop

    // DOCUSAURUS: Slopes start
    r2KinematicCharacterController_SetSlopes(
        character_controller,
        // Don't allow climbing slopes larger than 45 degrees.
        45.0 * R2_PI / 180.0,
        // Automatically slide down on slopes smaller than 30 degrees.
        30.0 * R2_PI / 180.0);
    // DOCUSAURUS: Slopes stop

    // DOCUSAURUS: Stairs start
    // Set `enabled` (the second argument) to 0 to disable autostep (the other arguments are then ignored).
    r2KinematicCharacterController_SetAutostep(character_controller, 0, (R2CharacterLength){0},
                                               (R2CharacterLength){0}, 0);
    // Autostep if the step height is smaller than 0.5, and its width larger than 0.2.
    r2KinematicCharacterController_SetAutostep(
        character_controller, 1,
        (R2CharacterLength){.value = 0.5, .relative = 0}, // The maximum height.
        (R2CharacterLength){.value = 0.2, .relative = 0}, // The minimum width.
        1);                                               // Include dynamic bodies.
    // Autostep if the step height is smaller than 0.3 multiplied by the character's height,
    // and its width larger than 0.5 multiplied by the character's width (i.e. half the character's
    // width).
    r2KinematicCharacterController_SetAutostep(
        character_controller, 1,
        (R2CharacterLength){.value = 0.3, .relative = 1}, // The maximum height.
        (R2CharacterLength){.value = 0.5, .relative = 1}, // The minimum width.
        1);                                               // Include dynamic bodies.
    // DOCUSAURUS: Stairs stop

    // DOCUSAURUS: Snap start
    // Set `enabled` (the second argument) to 0 to disable snap-to-ground (the distance is then ignored).
    r2KinematicCharacterController_SetSnapToGround(character_controller, 0, (R2CharacterLength){0});
    // Snap to the ground if the vertical distance to the ground is smaller than 0.5.
    r2KinematicCharacterController_SetSnapToGround(character_controller, 1,
                                                   (R2CharacterLength){.value = 0.5, .relative = 0});
    // Snap to the ground if the vertical distance to the ground is smaller than 0.2 times the character's height.
    r2KinematicCharacterController_SetSnapToGround(character_controller, 1,
                                                   (R2CharacterLength){.value = 0.2, .relative = 1});
    // DOCUSAURUS: Snap stop

    // The settings can be read back.
    R2CharacterControllerSettings settings = r2KinematicCharacterController_Settings(character_controller);
    R2CharacterAutostep autostep = r2KinematicCharacterController_Autostep(character_controller);
    R2CharacterLength offset = r2KinematicCharacterController_Offset(character_controller);
    R2Vector up = r2KinematicCharacterController_Up(character_controller);
    printf("Max slope climb angle: %f, autostep: %u, offset: %f, up: (%f, %f)\n",
           (double)settings.max_slope_climb_angle, (unsigned)autostep.enabled, (double)offset.value,
           (double)up.x, (double)up.y);

    r2FreeKinematicCharacterController(character_controller);
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
