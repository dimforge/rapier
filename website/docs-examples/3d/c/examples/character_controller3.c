#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();

    /* Create the ground. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    /* Create the body to control. */
    R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
    body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle rigid_body_handle = r3InsertRigidBody(world, &body);
    R3ColliderDesc collider = r3BallColliderDesc(0.5);
    collider.restitution = 0.7;
    R3ColliderHandle collider_handle = r3InsertCollider(rigid_body_handle, &collider);

    R3SharedShape *character_shape = r3Collider_CloneShape(collider_handle);
    R3Real dt = r3TimeStep(world);
    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        {
            R3Pose character_pos = r3RigidBody_Position(rigid_body_handle);
            // DOCUSAURUS: Setup start
            // The translation we would like to apply if there were no obstacles.
            R3Vector desired_translation = r3Vector(1.0, -2.0, 3.0);
            // Create the character controller, here with the default configuration.
            R3KinematicCharacterController *character_controller = r3NewKinematicCharacterController();
            // Init the query options.
            R3QueryOptions options = r3DefaultQueryOptions();
            // Make sure the character we are trying to move isn't considered an obstacle.
            options.filter.exclude_rigid_body = rigid_body_handle;
            // Calculate the possible movement.
            R3CharacterMovement corrected_movement = r3KinematicCharacterController_MoveShape(
                world,                // The world containing the obstacles.
                &options,             // The query options (NULL for the default ones).
                character_controller, // The character controller.
                dt,                   // The timestep length (can be set to r3TimeStep(world)).
                character_shape,      // The character's shape.
                character_pos,        // The character's initial position.
                desired_translation);

            // TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described below.

            // Free the character controller once it is no longer needed.
            r3FreeKinematicCharacterController(character_controller);
            // DOCUSAURUS: Setup stop
            (void)corrected_movement;
        }

        r3Step(world, NULL, NULL);

        R3Vector translation = r3RigidBody_Translation(rigid_body_handle);
        printf("Character body position: (%f, %f, %f)\n", (double)translation.x,
               (double)translation.y, (double)translation.z);
    }

    r3FreeSharedShape(character_shape);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
