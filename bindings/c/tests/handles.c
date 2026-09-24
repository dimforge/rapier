#include "rapier.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define OK(call)                                                                                   \
    do {                                                                                           \
        RAPIER_TYPE(Status) status = (call);                                                       \
        if (status != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

static void RAPIER_CALL on_error(RAPIER_TYPE(Status) status, const char *message, void *data) {
    assert(status != RAPIER_CONST(OK) && message);
    assert(RAPIER_FN(LastStatus)() == status);
    ++*(unsigned *)data;
}

int main(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(RigidBodyDesc) desc = RAPIER_FN(DynamicRigidBodyDesc)();
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)(0.5);
    desc.canSleep = 0;
    RAPIER_TYPE(RigidBodyHandle) body;
    RAPIER_TYPE(ColliderHandle) shape;
    body = RAPIER_FN(InsertRigidBody)(world, &desc);
    OK(RAPIER_FN(LastStatus)());
    shape = RAPIER_FN(InsertCollider)(body, &collider);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(Vector) position = {0};
    position.y = 10;
    OK(RAPIER_FN(RigidBody_SetTranslation)(body, position, 1));
    OK(RAPIER_FN(Collider_SetSensor)(shape, 1));
    RAPIER_TYPE(Bool) sensor = RAPIER_FN(Collider_IsSensor)(shape);
    OK(RAPIER_FN(LastStatus)());
    assert(sensor);
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    position = RAPIER_FN(RigidBody_Translation)(body);
    OK(RAPIER_FN(LastStatus)());
    assert(position.y < 10);

    RAPIER_TYPE(Vector) throughSet = RAPIER_FN(RigidBody_Translation)(body);
    OK(RAPIER_FN(LastStatus)());
    assert(throughSet.y == position.y);
    /* Retained handles survive storage growth and stepping; no element pointer is kept. */
    for (int i = 0; i < 512; ++i) {
        desc.position.translation.x = (RAPIER_TYPE(Real))(10 + i);
        RAPIER_FN(InsertRigidBody)(world, &desc);
        OK(RAPIER_FN(LastStatus)());
    }
    RAPIER_TYPE(RigidBodyHandle) order[] = {body, body};
    RAPIER_TYPE(RigidBodyState) states[2];
    size_t count = 99;
    count = RAPIER_FN(RigidBodyReadStates)(world, order, 2, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 2);
    states[0].position.translation.x = 123;
    count = RAPIER_FN(RigidBodyReadStates)(world, order, 2, states, 1);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(BUFFER_TOO_SMALL));
    assert(count == 2 && states[0].position.translation.x == 123);
    count = RAPIER_FN(RigidBodyReadStates)(world, order, 2, states, 2);
    OK(RAPIER_FN(LastStatus)());
    assert(states[0].position.translation.y == position.y &&
           states[1].position.translation.y == position.y);
    RAPIER_TYPE(Vector) invalid = position;
    invalid.y = (RAPIER_TYPE(Real))NAN;
    assert(RAPIER_FN(RigidBody_SetTranslation)(body, invalid, 1) ==
           RAPIER_CONST(INVALID_ARGUMENT));
    throughSet = RAPIER_FN(RigidBody_Translation)(body);
    OK(RAPIER_FN(LastStatus)());
    assert(throughSet.y == position.y);

    RAPIER_TYPE(SoftBodyDesc) soft = RAPIER_FN(DefaultSoftBodyDesc)();
    soft.kind = RAPIER_CONST(SOFT_DESC_ROPE);
    soft.nx = 3;
    RAPIER_TYPE(SoftBodyHandle) softHandle = RAPIER_FN(InsertSoftBody)(world, &soft);
    OK(RAPIER_FN(LastStatus)());
    position = RAPIER_FN(SoftBody_ParticlePosition)(softHandle, 0);
    OK(RAPIER_FN(LastStatus)());
    position.y = 20;
    OK(RAPIER_FN(SoftBody_SetParticlePosition)(softHandle, 0, position));
    throughSet = RAPIER_FN(SoftBody_ParticlePosition)(softHandle, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(throughSet.y == 20);

    RAPIER_TYPE(RigidBodyHandle) root;
    OK(RAPIER_FN(SoftBody_ValidateHandle)(softHandle));
    root = RAPIER_FN(SoftBody_RootBody)(softHandle);
    OK(RAPIER_FN(LastStatus)());
    assert(RAPIER_FN(RigidBody_SetTranslation)(root, position, 1) ==
           RAPIER_CONST(INVALID_ARGUMENT));

    OK(RAPIER_FN(RemoveRigidBody)(body, 1));
    RAPIER_TYPE(RigidBodyHandle) replacement = RAPIER_FN(InsertRigidBody)(world, &desc);
    OK(RAPIER_FN(LastStatus)());
    assert(replacement.index == body.index && replacement.generation != body.generation);
    unsigned errors = 0;
    RAPIER_TYPE(ErrorHandler)
    previous = RAPIER_FN(SetErrorHandler)((RAPIER_TYPE(ErrorHandler)){on_error, &errors});
    position.y = 123;
    position = RAPIER_FN(RigidBody_Translation)(body);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_HANDLE));
    assert(errors == 1 && position.y == 0);
    RAPIER_FN(SetErrorHandler)(previous);
    sensor = RAPIER_FN(Collider_IsSensor)(shape);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_HANDLE));
    order[0] = replacement;
    count = 999;
    states[0].position.translation.x = 123;
    count = RAPIER_FN(RigidBodyReadStates)(world, order, 2, states, 2);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_HANDLE));
    assert(count == 0 && states[0].position.translation.x == 123);
    count = RAPIER_FN(RigidBodyReadStates)(world, NULL, 0, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 0);
    OK(RAPIER_FN(FreeWorld)(world));
    return 0;
}
