/* Drive the same picking and spring joints as the viewer, without a GPU. */
#include "grab.h"
#include "rapier_math.h"
#include <assert.h>
#define CHECK(call)                                                                                \
    do {                                                                                           \
        RAPIER_TYPE(Status) status = (call);                                                       \
        if (status != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

static void pointCursor(Testbed *t, RAPIER_TYPE(Vector) point) {
    t->cursorValid = t->rayValid = 1;
    t->cursor = point;
    t->rayOrigin = RAPIER_FN(VectorAdd)(point, V(0, 0, 10));
    t->rayDirection = V(0, 0, -1);
}

static void init(Testbed *t) {
    t->world = RAPIER_FN(NewWorld)();
    CHECK(RAPIER_FN(LastStatus)());
    t->requestedThreads = 1;
    tbRefreshWorld(t);
    CHECK(RAPIER_FN(SetGravity)(t->world, V(0, 0, 0)));
}

static RAPIER_TYPE(RigidBodyHandle) addBall(Testbed *t, RAPIER_TYPE(Vector) center, uint32_t kind,
                                            bool sensor) {
    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
    body.bodyType = kind;
    body.position.translation = center;
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(DefaultColliderDesc)();
    collider.isSensor = sensor;
    RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_FN(InsertRigidBody)(t->world, &body);
    RAPIER_FN(InsertCollider)(handle, &collider);
    CHECK(RAPIER_FN(LastStatus)());
    return handle;
}

static void checkCounts(Testbed *t, size_t bodies, size_t joints) {
    size_t actual = RAPIER_FN(RigidBodyCount)(t->world);
    CHECK(RAPIER_FN(LastStatus)());
    assert(actual == bodies);
    actual = RAPIER_FN(ImpulseJointHandles)(t->world, NULL, 0);
    CHECK(RAPIER_FN(LastStatus)());
    assert(actual == joints);
}

static void rigidDrag(void) {
    Testbed t = {0};
    init(&t);
    TbGrab grab = {0};
    RAPIER_TYPE(RigidBodyHandle) picked = addBall(&t, V(0, 0, 0), RAPIER_CONST(DYNAMIC), false);
    addBall(&t, V(-3, 0, 0), RAPIER_CONST(DYNAMIC), true);
    addBall(&t, V(3, 0, 0), RAPIER_CONST(FIXED), false);
    CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));
    pointCursor(&t, V(-3, 0, 0));
    CHECK(tbGrabBegin(&t, &grab, .1));
    assert(!grab.active);
    pointCursor(&t, V(3, 0, 0));
    CHECK(tbGrabBegin(&t, &grab, .1));
    assert(!grab.active);
    pointCursor(&t, V(0, 0, 0));
    CHECK(tbGrabBegin(&t, &grab, .1));
    assert(grab.active && !grab.soft && grab.body.index == picked.index);
    checkCounts(&t, 4, 1);
    pointCursor(&t, V(2, 1, 0));
    for (int i = 0; i < 120; ++i) {
        CHECK(tbGrabUpdate(&t, &grab, V(0, 0, -1)));
        CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));
    }

    RAPIER_TYPE(Vector) position;
    CHECK(RAPIER_FN(RigidBody_ValidateHandle)(picked));
    position = RAPIER_FN(RigidBody_Translation)(picked);
    CHECK(RAPIER_FN(LastStatus)());
    assert(position.x > 1.5 && position.y > .5);
    CHECK(tbGrabRelease(&t, &grab));
    assert(!grab.active);
    checkCounts(&t, 3, 0);
    pointCursor(&t, position);
    CHECK(tbGrabBegin(&t, &grab, .1));
    assert(grab.active);
    RAPIER_TYPE(Bool) removed = RAPIER_FN(RemoveRigidBody)(picked, 1);
    CHECK(RAPIER_FN(LastStatus)());
    assert(removed);
    CHECK(tbGrabUpdate(&t, &grab, V(0, 0, -1)));
    assert(!grab.active);
    checkCounts(&t, 2, 0);
    CHECK(RAPIER_FN(FreeWorld)(t.world));
}

static void articulatedDrag(void) {
    Testbed t = {0};
    init(&t);
    TbGrab grab = {0};
    RAPIER_TYPE(RigidBodyDesc) builder = RAPIER_FN(FixedRigidBodyDesc)();
    RAPIER_TYPE(RigidBodyHandle) fixed = RAPIER_FN(InsertRigidBody)(t.world, &builder);
    CHECK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(RigidBodyHandle) link = addBall(&t, V(0, 0, 0), RAPIER_CONST(DYNAMIC), false);

    RAPIER_TYPE(JointDesc) joint = RAPIER_FN(PrismaticJointDesc)(V(1, 0, 0));
    RAPIER_FN(InsertMultibodyJoint)(fixed, link, &joint);
    CHECK(RAPIER_FN(LastStatus)());
    CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));
    pointCursor(&t, V(0, 0, 0));
    CHECK(tbGrabBegin(&t, &grab, .1));
    assert(grab.active && grab.body.index == link.index);
    pointCursor(&t, V(2, 0, 0));
    for (int i = 0; i < 120; ++i) {
        CHECK(tbGrabUpdate(&t, &grab, V(0, 0, -1)));
        CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));
    }

    RAPIER_TYPE(Vector) position;
    CHECK(RAPIER_FN(RigidBody_ValidateHandle)(link));
    position = RAPIER_FN(RigidBody_Translation)(link);
    CHECK(RAPIER_FN(LastStatus)());
    assert(position.x > 1.5 && fabs(position.y) < .01);
    CHECK(tbGrabRelease(&t, &grab));
    checkCounts(&t, 2, 0);
    CHECK(RAPIER_FN(FreeWorld)(t.world));
}

static void softDrag(void) {
    Testbed t = {0};
    init(&t);
    TbGrab grab = {0};

#if defined(RAPIER_DIM2)
    RAPIER_TYPE(SoftBodyDesc) builder = RAPIER_FN(GridSoftBodyDesc)(V(0, 0, 0), V(.5, .5, 0), 3, 3);
#else
    RAPIER_TYPE(SoftBodyDesc)
    builder = RAPIER_FN(ClothSoftBodyDesc)(V(-.5, -.5, 0), V(.5, 0, 0), V(0, .5, 0), 3, 3);
#endif

    RAPIER_TYPE(ColliderDesc) surface = RAPIER_FN(BallColliderDesc)(.05);
    builder.collider = surface;
    RAPIER_TYPE(SoftBodyHandle) handle = RAPIER_FN(InsertSoftBody)(t.world, &builder);
    CHECK(RAPIER_FN(LastStatus)());
    CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));

    size_t before, after, bodyCount;
    CHECK(RAPIER_FN(SoftBody_ValidateHandle)(handle));
    before = RAPIER_FN(SoftBody_Clusters)(handle, NULL, 0);
    CHECK(RAPIER_FN(LastStatus)());
    bodyCount = RAPIER_FN(RigidBodyCount)(t.world);
    CHECK(RAPIER_FN(LastStatus)());
    pointCursor(&t, V(-.4, -.4, 0));
    CHECK(tbGrabBegin(&t, &grab, .2));
    assert(grab.active && grab.soft);
    CHECK(RAPIER_FN(SoftBody_ValidateHandle)(handle));
    after = RAPIER_FN(SoftBody_Clusters)(handle, NULL, 0);
    CHECK(RAPIER_FN(LastStatus)());
    assert(after == before + 1);
    RAPIER_TYPE(Vector) start = RAPIER_FN(SoftBody_ParticlePosition)(handle, 0);
    CHECK(RAPIER_FN(LastStatus)());
    pointCursor(&t, V(-1.5, 1, 0));
    for (int i = 0; i < 60; ++i) {
        CHECK(tbGrabUpdate(&t, &grab, V(0, 0, -1)));
        CHECK(RAPIER_FN(Step)(t.world, NULL, NULL));
    }
    RAPIER_TYPE(Vector) end;
    CHECK(RAPIER_FN(SoftBody_ValidateHandle)(handle));
    end = RAPIER_FN(SoftBody_ParticlePosition)(handle, 0);
    CHECK(RAPIER_FN(LastStatus)());
    assert(RAPIER_FN(VectorLength)(RAPIER_FN(VectorSub)(end, start)) > .3);
    CHECK(tbGrabRelease(&t, &grab));
    CHECK(RAPIER_FN(SoftBody_ValidateHandle)(handle));
    after = RAPIER_FN(SoftBody_Clusters)(handle, NULL, 0);
    CHECK(RAPIER_FN(LastStatus)());
    assert(after == before);
    checkCounts(&t, bodyCount, 0);
    CHECK(RAPIER_FN(FreeWorld)(t.world));
}

int main(void) {
    rigidDrag();
    articulatedDrag();
    softDrag();
    puts("Mouse picking, rigid/soft spring dragging, release, and deletion passed");
    return 0;
}
