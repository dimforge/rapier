// DOCUSAURUS: basic_sim start
#include "rapier.h"
#include "rapier_math.h"
#include <stdio.h>
#include <stdlib.h>

/* Called whenever a Rapier function fails. */
static void RAPIER_CALL on_error(R3Status status, const char *message, void *user_data) {
    (void)user_data;
    fprintf(stderr, "Rapier error %u: %s\n", (unsigned)status, message);
    exit(EXIT_FAILURE);
}

int main(void) {
    /* Abort as soon as any Rapier function reports an error. */
    R3ErrorHandler error_handler = {on_error, NULL};
    r3SetErrorHandler(error_handler);

    /* The world owns every structure needed by the simulation. */
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0.0, -9.81, 0.0));

    /* Create the ground. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle ball_body_handle = r3InsertRigidBody(world, &ball_body);

    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r3InsertCollider(ball_body_handle, &ball);

    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        r3Step(world, NULL, NULL);

        R3Vector translation = r3RigidBody_Translation(ball_body_handle);
        printf("Ball altitude: %f\n", (double)translation.y);
    }

    /* Freeing the world frees everything it contains. */
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
// DOCUSAURUS: basic_sim stop

// DOCUSAURUS: CheckAbi start
/* Checks that the headers match the Rapier library actually loaded. */
void check_rapier_abi(void) {
    if (r3CheckAbi(R3_ABI_VERSION, R3_DIMENSION, sizeof(R3Real), sizeof(R3Vector), sizeof(R3Pose), R3_ABI_FEATURES) != R3_OK) {
        fprintf(stderr, "Incompatible Rapier library: %s\n", r3LastError());
        exit(EXIT_FAILURE);
    }
}
// DOCUSAURUS: CheckAbi stop
