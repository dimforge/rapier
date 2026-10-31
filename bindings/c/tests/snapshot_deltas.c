/* Smoke test of the snapshot-delta C API; snapshot_delta_tests.rs covers the edge cases. */
#include "rapier.h"
#include <assert.h>
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

/* An owned copy of native bytes. */
typedef struct Buffer {
    uint8_t *data;
    size_t count;
} Buffer;

static Buffer copy(const uint8_t *data, size_t count) {
    Buffer result = {malloc(count ? count : 1), count};
    assert(result.data);
    memcpy(result.data, data, count);
    return result;
}

/* Copies and releases owned bytes. */
static Buffer take(RAPIER_TYPE(Bytes) *bytes) {
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ByteView) view = RAPIER_FN(Bytes_Data)(bytes);
    OK(RAPIER_FN(LastStatus)());
    Buffer result = copy(view.data, view.count);
    OK(RAPIER_FN(FreeBytes)(bytes));
    return result;
}

static int same(Buffer a, Buffer b) {
    return a.count == b.count && memcmp(a.data, b.data, a.count) == 0;
}

static int same_snapshots(const RAPIER_TYPE(World) *a, const RAPIER_TYPE(World) *b) {
    Buffer x = take(RAPIER_FN(SerializeWorld)(a)), y = take(RAPIER_FN(SerializeWorld)(b));
    int result = same(x, y);
    free(x.data);
    free(y.data);
    return result;
}

/* Delta and tick bytes since `since` (NULL: a full delta). */
static void delta(RAPIER_TYPE(World) *world, const Buffer *since, Buffer *delta, Buffer *tick) {
    RAPIER_TYPE(WorldDelta)
    result = RAPIER_FN(SerializeWorldDelta)(world, since ? since->data : NULL,
                                            since ? since->count : 0);
    OK(RAPIER_FN(LastStatus)());
    *delta = take(result.delta);
    *tick = take(result.tick);
}

/* Applies `delta`, checks it acknowledges `tick`, and returns that tick. */
static Buffer apply(RAPIER_TYPE(World) *world, Buffer delta, Buffer tick) {
    Buffer acked = take(RAPIER_FN(ApplyWorldDelta)(world, delta.data, delta.count));
    assert(same(acked, tick));
    free(acked.data);
    free(delta.data);
    return tick;
}

/* A ground and a few falling balls, stepped until they touch. */
static RAPIER_TYPE(World) *pile(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(Vector) half = {0};
    half.x = 10;
    half.y = (RAPIER_TYPE(Real))0.5;
#ifdef RAPIER_DIM3
    half.z = 10;
#endif
    RAPIER_TYPE(ColliderDesc) ground = RAPIER_FN(CuboidColliderDesc)(half);
    RAPIER_FN(InsertColliderWithoutParent)(world, &ground);
    OK(RAPIER_FN(LastStatus)());
    for (int i = 0; i < 4; ++i) {
        RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
        body.position.translation.x = (RAPIER_TYPE(Real))0.1 * i;
        body.position.translation.y = 1 + (RAPIER_TYPE(Real))1.1 * i;
        RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_FN(InsertRigidBody)(world, &body);
        OK(RAPIER_FN(LastStatus)());
        RAPIER_TYPE(ColliderDesc) ball = RAPIER_FN(BallColliderDesc)((RAPIER_TYPE(Real))0.5);
        RAPIER_FN(InsertCollider)(handle, &ball);
        OK(RAPIER_FN(LastStatus)());
    }
    for (int i = 0; i < 10; ++i) {
        OK(RAPIER_FN(Step)(world, NULL, NULL));
    }
    return world;
}

int main(void) {
    RAPIER_TYPE(World) *sender = pile();
    RAPIER_TYPE(WorldDelta) untracked = RAPIER_FN(SerializeWorldDelta)(sender, NULL, 0);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(UNSUPPORTED));
    assert(!untracked.delta && !untracked.tick);
    OK(RAPIER_FN(EnableChangeTracking)(sender));

    /* A receiver restored from a snapshot declares the tick issued right before it. */
    Buffer restored_tick = take(RAPIER_FN(ChangeTick)(sender));
    assert(memcmp(restored_tick.data, "RPRT", 4) == 0);
    Buffer snapshot = take(RAPIER_FN(SerializeWorld)(sender));
    RAPIER_TYPE(World) *restored = RAPIER_FN(DeserializeWorld)(snapshot.data, snapshot.count);
    OK(RAPIER_FN(LastStatus)());
    free(snapshot.data);
    EXPECT(RAPIER_FN(SetDeltaBaseline)(restored, NULL, 0), RAPIER_CONST(INVALID_ARGUMENT));
    OK(RAPIER_FN(SetDeltaBaseline)(restored, restored_tick.data, restored_tick.count));

    /* A fresh receiver starts from a full delta. */
    RAPIER_TYPE(World) *fresh = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    Buffer d, t;
    delta(sender, NULL, &d, &t);
    Buffer fresh_tick = apply(fresh, d, t);

    /* Both then accept incremental deltas from their own baseline. */
    OK(RAPIER_FN(Step)(sender, NULL, NULL));
    delta(sender, &restored_tick, &d, &t);
    free(restored_tick.data);
    restored_tick = apply(restored, d, t);
    delta(sender, &fresh_tick, &d, &t);
    free(fresh_tick.data);
    fresh_tick = apply(fresh, d, t);
    assert(same_snapshots(restored, sender) && same_snapshots(fresh, sender));

    /* A stepped receiver no longer holds its baseline, and recovers with a full delta. */
    OK(RAPIER_FN(Step)(sender, NULL, NULL));
    OK(RAPIER_FN(Step)(fresh, NULL, NULL));
    delta(sender, &fresh_tick, &d, &t);
    assert(!RAPIER_FN(ApplyWorldDelta)(fresh, d.data, d.count));
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));
    assert(strstr(RAPIER_FN(LastError)(), "baseline"));
    free(d.data);
    free(t.data);
    delta(sender, NULL, &d, &t);
    free(apply(fresh, d, t).data);
    assert(same_snapshots(fresh, sender));

    free(restored_tick.data);
    free(fresh_tick.data);
    OK(RAPIER_FN(FreeWorld)(fresh));
    OK(RAPIER_FN(FreeWorld)(restored));
    OK(RAPIER_FN(FreeWorld)(sender));
    puts("snapshot deltas passed");
    return 0;
}
