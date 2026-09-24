#include "rapier.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define OK(expr)                                                                                   \
    do {                                                                                           \
        RAPIER_TYPE(Status) status_ = (expr);                                                      \
        if (status_ != RAPIER_CONST(OK)) {                                                         \
            fprintf(stderr, "%s:%d: %s: %s (%u)\n", __FILE__, __LINE__, #expr,                     \
                    RAPIER_FN(LastError)(), status_);                                              \
            abort();                                                                               \
        }                                                                                          \
    } while (0)
#define EXPECT(expr, code)                                                                         \
    do {                                                                                           \
        RAPIER_TYPE(Status) status_ = (expr);                                                      \
        RAPIER_TYPE(Status) expected_ = (code);                                                    \
        if (status_ != expected_) {                                                                \
            fprintf(stderr, "%s:%d: expected %u, got %u: %s\n", __FILE__, __LINE__,                \
                    expected_, status_, RAPIER_FN(LastError)());                                   \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

static RAPIER_TYPE(Vector) vector(RAPIER_TYPE(Real) x, RAPIER_TYPE(Real) y, RAPIER_TYPE(Real) z) {
    RAPIER_TYPE(Vector) v;
    v.x = x;
    v.y = y;
#ifdef RAPIER_DIM3
    v.z = z;
#else
    (void)z;
#endif
    return v;
}

static RAPIER_TYPE(Pose) pose(RAPIER_TYPE(Real) x, RAPIER_TYPE(Real) y, RAPIER_TYPE(Real) z) {
    RAPIER_TYPE(Pose) p = {0};
    p.translation = vector(x, y, z);
#ifdef RAPIER_DIM3
    p.rotation.w = 1;
#endif
    return p;
}

static RAPIER_TYPE(RigidBodyHandle) add_body(RAPIER_TYPE(World) *world, uint32_t kind,
                                             RAPIER_TYPE(Real) y) {
    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
    body.bodyType = kind;
    body.position = pose(0, y, 0);
    body.canSleep = 0;
    RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    return handle;
}

static RAPIER_TYPE(ColliderHandle) add_ball(RAPIER_TYPE(RigidBodyHandle) body) {
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(DefaultColliderDesc)();
    collider.activeEvents = RAPIER_CONST(COLLISION_EVENTS) | RAPIER_CONST(CONTACT_FORCE_EVENTS);
    collider.activeHooks = 1 | 4;
    RAPIER_TYPE(ColliderHandle) handle = RAPIER_FN(InsertCollider)(body, &collider);
    OK(RAPIER_FN(LastStatus)());
    return handle;
}

static int filter_calls = 0, modify_calls = 0;

static int32_t RAPIER_CALL contact_filter(void *user, const RAPIER_TYPE(ReadContext) *read,
                                          RAPIER_TYPE(ColliderHandle) a,
                                          RAPIER_TYPE(ColliderHandle) b,
                                          RAPIER_TYPE(RigidBodyHandle) x,
                                          RAPIER_TYPE(RigidBodyHandle) y) {
    RAPIER_TYPE(Vector) position = RAPIER_FN(ReadCollider_Translation)(read, a);
    OK(RAPIER_FN(LastStatus)());
    assert(isfinite(position.y));
    (void)a;
    (void)b;
    (void)x;
    (void)y;
    assert(user == &filter_calls);
    ++filter_calls;
    return 1;
}

static void RAPIER_CALL modify_contact(void *user, const RAPIER_TYPE(ReadContext) *read,
                                       RAPIER_TYPE(ColliderHandle) a, RAPIER_TYPE(ColliderHandle) b,
                                       RAPIER_TYPE(ContactModification) *contact) {
    (void)read;
    (void)user;
    (void)a;
    (void)b;
    ++modify_calls;
    contact->friction = (RAPIER_TYPE(Real))0.7;
}

static void test_world_pipeline(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(BuildFeatures) profiling = RAPIER_FN(BuildFeatures)();
    EXPECT(RAPIER_FN(SetCountersEnabled)(world, 1),
           profiling.profiling ? RAPIER_CONST(OK) : RAPIER_CONST(UNSUPPORTED));
    EXPECT(RAPIER_FN(SetCountersEnabled)(world, 2), RAPIER_CONST(INVALID_ARGUMENT));
    RAPIER_TYPE(RigidBodyHandle) body = add_body(world, RAPIER_CONST(DYNAMIC), 10);
    (void)add_ball(body);
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    double step_ms = RAPIER_FN(StepTimeMs)(world);
    OK(RAPIER_FN(LastStatus)());
    assert(isfinite(step_ms) && (profiling.profiling ? step_ms > 0 : step_ms == 0));
    OK(RAPIER_FN(SetCountersEnabled)(world, 0));
    RAPIER_FN(StepTimeMs)(NULL);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(NULL_POINTER));
    RAPIER_TYPE(Vector) position = RAPIER_FN(RigidBody_Translation)(body);
    OK(RAPIER_FN(LastStatus)());
    assert(position.y < 10);
    OK(RAPIER_FN(DetectCollisions)(world, NULL, NULL));
    RAPIER_TYPE(Vector) after = RAPIER_FN(RigidBody_Translation)(body);
    OK(RAPIER_FN(LastStatus)());
    assert(after.y == position.y);
    OK(RAPIER_FN(FreeWorld)(world));
}

int main(void) {
    OK(RAPIER_FN(CheckAbi)(RAPIER_CONST(ABI_VERSION), RAPIER_CONST(DIMENSION),
                           sizeof(RAPIER_TYPE(Real)), sizeof(RAPIER_TYPE(Vector)),
                           sizeof(RAPIER_TYPE(Pose))));
    EXPECT(RAPIER_FN(CheckAbi)(RAPIER_CONST(ABI_VERSION), 99, sizeof(RAPIER_TYPE(Real)),
                               sizeof(RAPIER_TYPE(Vector)), sizeof(RAPIER_TYPE(Pose))),
           RAPIER_CONST(INVALID_ARGUMENT));
    const char *version = RAPIER_FN(Version)();
    assert(version && strstr(version, "+c."));
    assert(version == RAPIER_FN(Version)());
#ifdef RAPIER_EXPECTED_VERSION
    assert(!strcmp(version, RAPIER_EXPECTED_VERSION));
#endif
    const char *profile = RAPIER_FN(BuildProfile)();
    assert(profile && (!strcmp(profile, "release") || !strcmp(profile, "debug")));
#ifdef RAPIER_EXPECTED_PROFILE
    assert(!strcmp(profile, RAPIER_EXPECTED_PROFILE));
#endif
    RAPIER_TYPE(BuildFeatures) features = RAPIER_FN(BuildFeatures)();
    assert(features.simd_lanes == 4 || features.simd_lanes == 8);
#ifdef RAPIER_EXPECTED_SIMD_LANES
    assert(features.simd_lanes == RAPIER_EXPECTED_SIMD_LANES);
#endif
#ifdef RAPIER_PARALLEL
    assert(features.parallel == 1);
#else
    assert(features.parallel == 0);
#endif
    RAPIER_TYPE(World) *pool_world = NULL, *other_pool_world = NULL;
    pool_world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    other_pool_world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    size_t workers = 99;
    if (features.parallel) {
        workers = RAPIER_FN(NumThreads)(pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 0);
        OK(RAPIER_FN(SetNumThreads)(pool_world, 1));
        OK(RAPIER_FN(SetNumThreads)(other_pool_world, 2));
        workers = RAPIER_FN(NumThreads)(pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 1);
        OK(RAPIER_FN(SetNumThreads)(pool_world, 4));
        workers = RAPIER_FN(NumThreads)(pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 4);
        workers = RAPIER_FN(NumThreads)(other_pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 2);
        OK(RAPIER_FN(Step)(pool_world, NULL, NULL));
        OK(RAPIER_FN(ClearThreadPool)(pool_world));
        workers = RAPIER_FN(NumThreads)(pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 0);
    } else {
        EXPECT(RAPIER_FN(SetNumThreads)(pool_world, 2), RAPIER_CONST(UNSUPPORTED));
        EXPECT(RAPIER_FN(ClearThreadPool)(pool_world), RAPIER_CONST(UNSUPPORTED));
        workers = RAPIER_FN(NumThreads)(pool_world);
        OK(RAPIER_FN(LastStatus)());
        assert(workers == 1);
    }
    OK(RAPIER_FN(FreeWorld)(pool_world));
    OK(RAPIER_FN(FreeWorld)(other_pool_world));
    RAPIER_TYPE(BuildInfo) info = RAPIER_FN(BuildInfo)();
    assert(info.dimension == RAPIER_CONST(DIMENSION) &&
           info.real_size == sizeof(RAPIER_TYPE(Real)));
    EXPECT(RAPIER_FN(SetGravity)(NULL, vector(0, 0, 0)), RAPIER_CONST(NULL_POINTER));
    assert(strstr(RAPIER_FN(LastError)(), "null"));
    OK(RAPIER_FN(FreeWorld)(NULL));
    RAPIER_TYPE(SharedShape) *bad = (RAPIER_TYPE(SharedShape) *)(uintptr_t)1;
    bad = RAPIER_FN(BallSharedShape)(-1);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));
    assert(bad == NULL);
    bad = RAPIER_FN(BallSharedShape)((RAPIER_TYPE(Real))NAN);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));
    RAPIER_TYPE(Vector) triangle[3] = {vector(0, 0, 0), vector(1, 0, 0), vector(0, 1, 0)};
    uint32_t bad_indices[3] = {0, 1, 3};
    bad = RAPIER_FN(TrimeshSharedShape)(
        (RAPIER_TYPE(VectorView)){triangle, 3},
        (RAPIER_TYPE(TriangleView)){(const RAPIER_TYPE(Triangle) *)bad_indices, 1});
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));
    test_world_pipeline();
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(ColliderHandle) floor_h;
    RAPIER_TYPE(ColliderDesc)
    floor = RAPIER_FN(CuboidColliderDesc)(vector(10, (RAPIER_TYPE(Real))0.5, 10));
    floor.position = pose(0, (RAPIER_TYPE(Real))-0.5, 0);
    floor_h = RAPIER_FN(InsertColliderWithoutParent)(world, &floor);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(RigidBodyHandle) ball = add_body(world, RAPIER_CONST(DYNAMIC), 5);
    RAPIER_TYPE(ColliderHandle) ball_collider = add_ball(ball);
    RAPIER_TYPE(EventCollector) *events = RAPIER_FN(NewEventCollector)();
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(PhysicsHooks) hooks = {0};
    hooks.user_data = &filter_calls;
    hooks.filter_contact_pair = contact_filter;
    hooks.modify_solver_contacts = modify_contact;
    for (int i = 0; i < 240; ++i) {
        OK(RAPIER_FN(Step)(world, &hooks, events));
    }
    RAPIER_TYPE(Vector) position;
    OK(RAPIER_FN(RigidBody_ValidateHandle)(ball));
    position = RAPIER_FN(RigidBody_Translation)(ball);
    OK(RAPIER_FN(LastStatus)());
    assert(position.y > 0.45 && position.y < 0.6);
    assert(filter_calls > 0 && modify_calls > 0);
    size_t count = RAPIER_FN(EventCollector_CollisionEvents)(events, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_TYPE(CollisionEvent) *collisions = malloc(count * sizeof(*collisions));
    assert(collisions);
    count = RAPIER_FN(EventCollector_CollisionEvents)(events, collisions, count);
    OK(RAPIER_FN(LastStatus)());
    assert(collisions[0].started == 1);
    free(collisions);
    count = RAPIER_FN(EventCollector_ContactForceEvents)(events, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    count = RAPIER_FN(ContactPairs)(world, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_TYPE(ContactPair) pair = RAPIER_FN(ContactPair)(floor_h, ball_collider);
    OK(RAPIER_FN(LastStatus)());
    assert(pair.has_any_active_contact);
    count = RAPIER_FN(ContactPoints)(floor_h, ball_collider, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_TYPE(QueryOptions) query = RAPIER_FN(DefaultQueryOptions)();
    RAPIER_TYPE(RayHit)
    hit = RAPIER_FN(CastRay)(world, &query, vector(0, 10, 0), vector(0, -1, 0), 20, 1);
    OK(RAPIER_FN(LastStatus)());
    assert(hit.collider.index == ball_collider.index && hit.normal.y > 0.9);
    hit = RAPIER_FN(CastRay)(world, &query, vector(100, 10, 0), vector(0, -1, 0), 20, 1);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(NOT_FOUND));
    count = RAPIER_FN(IntersectPoint)(world, &query, vector(0, (RAPIER_TYPE(Real))0.5, 0), NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 1);
    RAPIER_TYPE(ColliderHandle) sentinel = {NULL, 123, 456};
    count = RAPIER_FN(IntersectPoint)(world, &query, vector(0, (RAPIER_TYPE(Real))0.5, 0),
                                      &sentinel, 0);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(BUFFER_TOO_SMALL));
    assert(sentinel.index == 123 && count == 1);
    RAPIER_TYPE(SharedShape) *cast_shape = RAPIER_FN(BallSharedShape)((RAPIER_TYPE(Real))0.25);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ShapeCastOptions) cast_options = RAPIER_FN(DefaultShapeCastOptions)();
    cast_options.max_time_of_impact = 10;
    RAPIER_TYPE(ShapeCastHit)
    shape_hit = RAPIER_FN(CastShape)(world, &query, pose(0, 4, 0), vector(0, -1, 0), cast_shape,
                                     cast_options);
    OK(RAPIER_FN(LastStatus)());
    assert(shape_hit.collider.index == ball_collider.index && shape_hit.time_of_impact > 2 &&
           shape_hit.time_of_impact < 4);
    count = RAPIER_FN(IntersectShape)(world, &query, pose(0, (RAPIER_TYPE(Real))0.5, 0), cast_shape,
                                      NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 1);
    RAPIER_TYPE(Aabb) aabb = {vector(-1, -1, -1), vector(1, 1, 1)};
    count = RAPIER_FN(IntersectAabbConservative)(world, &query, aabb, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count >= 2);
    RAPIER_TYPE(MassProperties)
    mass_properties = RAPIER_FN(SharedShape_MassProperties)(cast_shape, 1);
    OK(RAPIER_FN(LastStatus)());
    assert(mass_properties.mass > 0);
    OK(RAPIER_FN(FreeSharedShape)(cast_shape));
    RAPIER_TYPE(Real) heights[4] = {0, 0, 0, 0};
#ifdef RAPIER_DIM3
    cast_shape = RAPIER_FN(HeightfieldSharedShape)((RAPIER_TYPE(RealView)){heights, 4}, 2, 2,
                                                   vector(2, 1, 2));
    OK(RAPIER_FN(LastStatus)());
#else
    cast_shape = RAPIER_FN(HeightfieldSharedShape)((RAPIER_TYPE(RealView)){heights, 2}, 2, 1,
                                                   vector(2, 1, 0));
    OK(RAPIER_FN(LastStatus)());
#endif
    aabb = RAPIER_FN(SharedShape_ComputeAabb)(cast_shape, pose(0, 0, 0));
    OK(RAPIER_FN(LastStatus)());
    assert(aabb.maxs.x > 0);
    OK(RAPIER_FN(FreeSharedShape)(cast_shape));
    RAPIER_TYPE(PointProjection)
    projection = RAPIER_FN(ProjectPoint)(world, &query, vector(0, 3, 0), 10, 1);
    OK(RAPIER_FN(LastStatus)());
    assert(projection.point.y < 1.1);
    RAPIER_TYPE(QueryFilter) filter = RAPIER_FN(DefaultQueryFilter)();
    filter.exclude_collider = ball_collider;
    query.filter = filter;
    hit = RAPIER_FN(CastRay)(world, &query, vector(0, 10, 0), vector(0, -1, 0), 20, 1);
    OK(RAPIER_FN(LastStatus)());
    assert(hit.collider.index == floor_h.index);
    RAPIER_TYPE(Bytes) *snapshot;
    const uint8_t *data;
    size_t length;
    snapshot = RAPIER_FN(SerializeWorld)(world);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ByteView) bytesDataResult = RAPIER_FN(Bytes_Data)(snapshot);
    data = bytesDataResult.data;
    length = bytesDataResult.count;
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(World) *restored = RAPIER_FN(DeserializeWorld)(data, length);
    OK(RAPIER_FN(LastStatus)());

    OK(RAPIER_FN(RigidBody_ValidateHandle)(ball));
    RAPIER_TYPE(RigidBodyHandle) restored_ball = ball;
    restored_ball.world = restored; /* Same snapshot indices, new owning allocation. */
    RAPIER_TYPE(Vector) restored_position = RAPIER_FN(RigidBody_Translation)(restored_ball);
    OK(RAPIER_FN(LastStatus)());
    assert(fabs((double)(restored_position.y - position.y)) < 1e-6);
    OK(RAPIER_FN(FreeBytes)(snapshot));
    OK(RAPIER_FN(Step)(restored, NULL, NULL));
    OK(RAPIER_FN(FreeWorld)(restored));
    uint8_t malformed[6] = {'R',
                            'P',
                            'R',
                            RAPIER_CONST(ABI_VERSION),
                            RAPIER_CONST(DIMENSION),
                            sizeof(RAPIER_TYPE(Real))};
    restored = RAPIER_FN(DeserializeWorld)(malformed, sizeof(malformed));
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));

    RAPIER_TYPE(ImpulseJointHandle) jh;
    RAPIER_TYPE(RigidBodyHandle) anchor = add_body(world, RAPIER_CONST(FIXED), 5);
    RAPIER_TYPE(JointDesc) joint = RAPIER_FN(RopeJointDesc)(5);
    jh = RAPIER_FN(InsertImpulseJoint)(anchor, ball, &joint);
    OK(RAPIER_FN(LastStatus)());
    OK(RAPIER_FN(ImpulseJoint_SetContactsEnabled)(jh, 0, 1));
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    OK(RAPIER_FN(RemoveImpulseJoint)(jh, 1));
    EXPECT(RAPIER_FN(RemoveImpulseJoint)(jh, 1), RAPIER_CONST(INVALID_HANDLE));
    RAPIER_TYPE(MultibodyJointHandle) mh;
    joint = RAPIER_FN(FixedJointDesc)();
    mh = RAPIER_FN(InsertMultibodyJoint)(anchor, ball, &joint);
    OK(RAPIER_FN(LastStatus)());
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    OK(RAPIER_FN(RemoveMultibodyJoint)(mh, 1));
    RAPIER_TYPE(SoftBodyDesc)
    rope = RAPIER_FN(RopeSoftBodyDesc)(vector(3, 4, 0), vector(3, 2, 0), 8);
    uint32_t pin = 0;
    OK(RAPIER_FN(SoftBodyDesc_SetPinnedParticles)(&rope, (RAPIER_TYPE(IndexView)){&pin, 1}));
    RAPIER_TYPE(SoftBodyHandle) sh = RAPIER_FN(InsertSoftBody)(world, &rope);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(Vector) particles[8];
    count = RAPIER_FN(SoftBody_ParticlePositions)(sh, particles, 8);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 8 && particles[0].y == 4);
    for (int i = 0; i < 10; ++i) {
        OK(RAPIER_FN(Step)(world, NULL, events));
    }
    count = RAPIER_FN(SoftBody_ParticlePositions)(sh, particles, 8);
    OK(RAPIER_FN(LastStatus)());
    assert(fabs((double)(particles[0].y - 4)) < 1e-5);
    EXPECT(RAPIER_FN(SoftBody_SetParticlePosition)(sh, 99, vector(0, 0, 0)),
           RAPIER_CONST(INVALID_ARGUMENT));
    RAPIER_TYPE(RigidBodyHandle) root = RAPIER_FN(SoftBody_RootBody)(sh);
    OK(RAPIER_FN(LastStatus)());
    EXPECT(RAPIER_FN(RigidBody_SetTranslation)(root, vector(0, 0, 0), 1),
           RAPIER_CONST(INVALID_ARGUMENT));
    position = RAPIER_FN(RigidBody_Translation)(root);
    OK(RAPIER_FN(LastStatus)());
    count = RAPIER_FN(SoftBody_MeshColliders)(sh, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_TYPE(ColliderHandle) mesh;
    count = RAPIER_FN(SoftBody_MeshColliders)(sh, &mesh, 1);
    OK(RAPIER_FN(LastStatus)());
    count = RAPIER_FN(SoftBody_MeshVertices)(sh, mesh, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_TYPE(Vector) blade[RAPIER_CONST(DIMENSION)];
    blade[0] = vector(2, 3, 0);
    blade[1] = vector(4, 3, 0);
#ifdef RAPIER_DIM3
    blade[0] = vector(2, 3, -1);
    blade[1] = vector(4, 3, -1);
    blade[2] = vector(3, 3, 1);
#endif
    RAPIER_TYPE(SoftBodyTearEvent) *tear = RAPIER_FN(CutSoftBody)(sh, blade);
    OK(RAPIER_FN(LastStatus)());
    assert(tear);
    count = RAPIER_FN(SoftBodyTearEvent_Bodies)(tear, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count >= 1);
    OK(RAPIER_FN(FreeSoftBodyTearEvent)(tear));
    OK(RAPIER_FN(RemoveSoftBody)(sh));
    EXPECT(RAPIER_FN(SoftBody_ValidateHandle)(sh), RAPIER_CONST(INVALID_HANDLE));
    RAPIER_TYPE(SharedShape) *character_shape = RAPIER_FN(BallSharedShape)((RAPIER_TYPE(Real))0.3);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(KinematicCharacterController) *character =
        RAPIER_FN(NewKinematicCharacterController)();
    OK(RAPIER_FN(LastStatus)());
    query = RAPIER_FN(DefaultQueryOptions)();
    RAPIER_TYPE(CharacterMovement)
    movement = RAPIER_FN(KinematicCharacterController_MoveShape)(
        world, &query, character, (RAPIER_TYPE(Real))(1.0 / 60.0), character_shape, pose(5, 2, 0),
        vector(0, -5, 0));
    OK(RAPIER_FN(LastStatus)());
    assert(movement.translation.y > -2 && movement.grounded);
    OK(RAPIER_FN(FreeKinematicCharacterController)(character));
    OK(RAPIER_FN(FreeSharedShape)(character_shape));
#ifdef RAPIER_DIM3
    RAPIER_TYPE(DynamicRayCastVehicleController) *vehicle =
        RAPIER_FN(NewDynamicRayCastVehicleController)(ball);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(WheelTuning) tuning = RAPIER_FN(DefaultWheelTuning)();
    size_t wheel = RAPIER_FN(DynamicRayCastVehicleController_AddWheel)(
        vehicle, vector(0, 0, 0), vector(0, -1, 0), vector(1, 0, 0), (RAPIER_TYPE(Real))0.4,
        (RAPIER_TYPE(Real))0.3, &tuning);
    OK(RAPIER_FN(LastStatus)());
    assert(wheel == 0);
    OK(RAPIER_FN(DynamicRayCastVehicleController_UpdateVehicle)(
        vehicle, (RAPIER_TYPE(Real))(1.0 / 60.0), NULL));
    OK(RAPIER_FN(FreeDynamicRayCastVehicleController)(vehicle));
#endif
    count = RAPIER_FN(DebugRender)(world, 1, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count > 0);
    RAPIER_FN(RemoveRigidBody)(ball, 1);
    OK(RAPIER_FN(LastStatus)());
    EXPECT(RAPIER_FN(RigidBody_ValidateHandle)(ball), RAPIER_CONST(INVALID_HANDLE));
    EXPECT(RAPIER_FN(Collider_ValidateHandle)(ball_collider), RAPIER_CONST(INVALID_HANDLE));
    RAPIER_TYPE(RigidBodyHandle) replacement = add_body(world, RAPIER_CONST(DYNAMIC), 3);
    assert(replacement.index != ball.index || replacement.generation != ball.generation);
    OK(RAPIER_FN(EventCollector_Clear)(events));
    count = RAPIER_FN(EventCollector_CollisionEvents)(events, NULL, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(count == 0);
    OK(RAPIER_FN(FreeEventCollector)(events));
    OK(RAPIER_FN(FreeWorld)(world));
    printf("Rapier C integration passed: %uD, %zu-bit real\n", info.dimension,
           8 * sizeof(RAPIER_TYPE(Real)));
    return 0;
}
