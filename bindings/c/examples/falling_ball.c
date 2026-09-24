#include "rapier_helpers.h"
#include "rapier_math.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(call)                                                                                \
    do {                                                                                           \
        if ((call) != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            exit(EXIT_FAILURE);                                                                    \
        }                                                                                          \
    } while (0)

int main(void) {
    CHECK(RAPIER_FN(CheckAbi)(RAPIER_CONST(ABI_VERSION), RAPIER_CONST(DIMENSION),
                              sizeof(RAPIER_TYPE(Real)), sizeof(RAPIER_TYPE(Vector)),
                              sizeof(RAPIER_TYPE(Pose)), RAPIER_CONST(ABI_FEATURES)));

    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    CHECK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(RigidBodyDesc) rigid_body = RAPIER_FN(DynamicRigidBodyDesc)();
#if defined(RAPIER_DIM2)
    RAPIER_TYPE(Vector) position = RAPIER_FN(Vector)(0.0, 5.0);
#else
    RAPIER_TYPE(Vector) position = RAPIER_FN(Vector)(0.0, 5.0, 0.0);
#endif
    rigid_body.position.translation = position;

    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)(0.5);
    RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_FN(InsertRigidBody)(world, &rigid_body);
    CHECK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(handle, &collider);
    CHECK(RAPIER_FN(LastStatus)());

    /* Descriptions own no resources. The world owns the inserted objects. */

    for (int i = 0; i < 60; i++) {
        CHECK(RAPIER_FN(Step)(world, NULL, NULL));
    }

    position = RAPIER_FN(RigidBody_Translation)(handle);
    CHECK(RAPIER_FN(LastStatus)());
    printf("Ball y after one second: %.3f\n", (double)position.y);

    /* Only the world owns native resources. */
    CHECK(RAPIER_FN(FreeWorld)(world));
    return EXIT_SUCCESS;
}
