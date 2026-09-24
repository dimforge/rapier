// DOCUSAURUS: basic_sim start
#include "rapier.h"
#include "rapier_math.h"
#include <stdio.h>
#include <stdlib.h>

/* Called whenever a Rapier function fails. */
static void RAPIER_CALL on_error(R2Status status, const char *message, void *user_data) {
    (void)user_data;
    fprintf(stderr, "Rapier error %u: %s\n", (unsigned)status, message);
    exit(EXIT_FAILURE);
}

int main(void) {
    /* Abort as soon as any Rapier function reports an error. */
    R2ErrorHandler error_handler = {on_error, NULL};
    r2SetErrorHandler(error_handler);

    /* The world owns every structure needed by the simulation. */
    R2World *world = r2NewWorld();
    r2SetGravity(world, r2Vector(0.0, -9.81));

    /* Create the ground. */
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R2RigidBodyDesc ball_body = r2DynamicRigidBodyDesc();
    ball_body.position.translation = r2Vector(0.0, 10.0);
    R2RigidBodyHandle ball_body_handle = r2InsertRigidBody(world, &ball_body);

    R2ColliderDesc ball = r2BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r2InsertCollider(ball_body_handle, &ball);

    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        r2Step(world, NULL, NULL);

        R2Vector translation = r2RigidBody_Translation(ball_body_handle);
        printf("Ball altitude: %f\n", (double)translation.y);
    }

    /* Freeing the world frees everything it contains. */
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
// DOCUSAURUS: basic_sim stop

// DOCUSAURUS: CheckAbi start
/* Checks that the headers match the Rapier library actually loaded. */
void check_rapier_abi(void) {
    if (r2CheckAbi(R2_ABI_VERSION, R2_DIMENSION, sizeof(R2Real), sizeof(R2Vector), sizeof(R2Pose), R2_ABI_FEATURES) != R2_OK) {
        fprintf(stderr, "Incompatible Rapier library: %s\n", r2LastError());
        exit(EXIT_FAILURE);
    }
}
// DOCUSAURUS: CheckAbi stop
