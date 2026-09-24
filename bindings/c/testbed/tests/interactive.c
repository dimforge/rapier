/* Exercise the real example loops with synthetic input; no renderer is involved. */
#include "testbed.h"
#include "rapier_math.h"
#include <assert.h>

#define CHECK(call)                                                                                \
    do {                                                                                           \
        if ((call) != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

typedef struct InputRun {
    int kind, frame, mode;
    RAPIER_TYPE(RigidBodyHandle) tracked;
    RAPIER_TYPE(Vector) start, middle;
    RAPIER_TYPE(Real) initialDistance;
    uintptr_t initialShape;
    RAPIER_TYPE(SharedShape) *retainedShape;
    int verified;
} InputRun;

static RAPIER_TYPE(Vector) bodyPosition(Testbed *t, RAPIER_TYPE(RigidBodyHandle) handle) {
    assert(handle.world == t->world);
    RAPIER_TYPE(Vector) position;
    CHECK(RAPIER_FN(RigidBody_ValidateHandle)(handle));
    position = RAPIER_FN(RigidBody_Translation)(handle);
    CHECK(RAPIER_FN(LastStatus)());
    return position;
}

static int inputFrame(Testbed *t, void *context) {
    InputRun *run = context;
    size_t bodies, colliders, soft;
    assert(tbValidate(t, &bodies, &colliders, &soft));
    if (run->kind == 0) {
        t->cursorValid = 1;
        t->cursor = V(.3, .65, 0);
        if (!run->frame) {
            RAPIER_TYPE(RigidBodyHandle) *handles = malloc(bodies * sizeof(*handles));
            assert(handles);
            bodies = RAPIER_FN(RigidBodyHandles)(t->world, handles, bodies);
            CHECK(RAPIER_FN(LastStatus)());
            run->tracked = handles[bodies - 1];
            free(handles);
        }
        RAPIER_TYPE(Vector) delta = RAPIER_FN(VectorSub)(bodyPosition(t, run->tracked), t->cursor);
        RAPIER_TYPE(Real) distance = RAPIER_FN(VectorLength)(delta);
        if (run->frame == 1) {
            run->initialDistance = distance;
        }
        if (run->frame == 60) {
            assert(distance < .03);
            assert(distance < run->initialDistance * .25);
            run->verified = 1;
            return 0;
        }
    } else if (run->kind == 1) {
        if (!run->frame) {
            RAPIER_TYPE(RigidBodyHandle) *handles = malloc(bodies * sizeof(*handles));
            assert(handles);
            bodies = RAPIER_FN(RigidBodyHandles)(t->world, handles, bodies);
            CHECK(RAPIER_FN(LastStatus)());
            int found = 0;
            for (size_t i = 0; i < bodies; ++i) {
                uint32_t type;
                CHECK(RAPIER_FN(RigidBody_ValidateHandle)(handles[i]));
                type = RAPIER_FN(RigidBody_BodyType)(handles[i]);
                CHECK(RAPIER_FN(LastStatus)());
                if (type == RAPIER_CONST(KINEMATIC_POSITION_BASED)) {
                    run->tracked = handles[i];
                    found = 1;
                    break;
                }
            }
            assert(found);
            free(handles);
            run->start = bodyPosition(t, run->tracked);
        }
        t->inputDirection = V(1, 0, 0);
        t->jump = 1;
        if (run->frame == 20) {
            run->middle = bodyPosition(t, run->tracked);
            assert(run->middle.x > run->start.x + .2);
            for (size_t i = 0; i < t->settingCount; ++i) {
                if (!strcmp(t->settings[i].name, "Control mode")) {
                    t->settings[i].value = 1;
                }
            }
        }
        if (run->frame == 40) {
            uint32_t type;
            CHECK(RAPIER_FN(RigidBody_ValidateHandle)(run->tracked));
            type = RAPIER_FN(RigidBody_BodyType)(run->tracked);
            CHECK(RAPIER_FN(LastStatus)());
            assert(type == RAPIER_CONST(DYNAMIC));
            assert(bodyPosition(t, run->tracked).x > run->middle.x + .1);
            run->verified = 1;
            return 0;
        }
    }
#if defined(RAPIER_DIM3)
    else if (run->kind == 2) {

        RAPIER_TYPE(ColliderHandle) handle = {t->world, 0, 0};
        CHECK(r3Collider_ValidateHandle(handle));
        uintptr_t identity;
        identity = r3Collider_ShapeIdentity(handle);
        CHECK(r3LastStatus());
        if (!run->frame) {
            run->initialShape = identity;
            run->retainedShape = r3Collider_CloneShape(handle);
            CHECK(r3LastStatus());
        }
        t->rayValid = 1;
        t->rayOrigin = r3Vector(100, 100, 100);
        t->rayDirection = r3Vector(0, -1, 0);
        t->jump = 1;
        t->removeVoxel = run->mode;
        if (run->frame == 2) {
            assert(identity != run->initialShape);
            CHECK(r3FreeSharedShape(run->retainedShape));
            run->verified = 1;
            return 0;
        }
    }
#endif
    ++run->frame;
    t->simulating = 1;
    return 1;
}

static void checkExample(const char *id, int kind, int mode) {
    Testbed t = {0};
    InputRun input = {.kind = kind, .mode = mode};
    t.renderFrame = inputFrame;
    t.viewer = &input;
    t.noSleep = 1;
    t.requestedThreads = 1;
    t.assetRoot = TB_ASSET_ROOT;
    const TbExample *example = NULL;
    for (size_t i = 0; i < tbExampleCount; ++i) {
        if (!strcmp(tbExamples[i].id, id)) {
            example = &tbExamples[i];
        }
    }
    assert(example && tbRun(&t, example, 0));
    assert(input.verified);
    tbDestroy(&t);
    printf("PASS %s interaction mode %d\n", id, mode);
}

int main(void) {
#if defined(RAPIER_DIM2)
    checkExample("inverse_kinematics2", 0, 0);
    checkExample("character_controller2", 1, 0);
#else
    checkExample("inverse_kinematics3", 0, 0);
    checkExample("character_controller3", 1, 0);
    checkExample("voxels3", 2, 0);
    checkExample("voxels3", 2, 1);
#endif
    return 0;
}
