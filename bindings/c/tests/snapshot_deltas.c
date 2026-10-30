/* Replicate a stepping world through snapshot deltas, and check rejected inputs. */
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

/* Copies and releases owned bytes. */
static Buffer take(RAPIER_TYPE(Bytes) *bytes) {
    RAPIER_TYPE(ByteView) view = RAPIER_FN(Bytes_Data)(bytes);
    OK(RAPIER_FN(LastStatus)());
    Buffer result = {malloc(view.count ? view.count : 1), view.count};
    assert(result.data);
    memcpy(result.data, view.data, view.count);
    OK(RAPIER_FN(FreeBytes)(bytes));
    return result;
}

static int same(Buffer a, Buffer b) {
    return a.count == b.count && memcmp(a.data, b.data, a.count) == 0;
}

static Buffer snapshot(const RAPIER_TYPE(World) *world) {
    RAPIER_TYPE(Bytes) *bytes = RAPIER_FN(SerializeWorld)(world);
    OK(RAPIER_FN(LastStatus)());
    return take(bytes);
}

static int same_snapshots(const RAPIER_TYPE(World) *a, const RAPIER_TYPE(World) *b) {
    Buffer x = snapshot(a), y = snapshot(b);
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

static void reject_delta(RAPIER_TYPE(World) *world, const uint8_t *since, size_t count,
                         RAPIER_TYPE(Status) status) {
    RAPIER_TYPE(WorldDelta) result = RAPIER_FN(SerializeWorldDelta)(world, since, count);
    EXPECT(RAPIER_FN(LastStatus)(), status);
    assert(!result.delta && !result.tick);
}

static Buffer apply(RAPIER_TYPE(World) *world, Buffer delta) {
    RAPIER_TYPE(Bytes) *tick = RAPIER_FN(ApplyWorldDelta)(world, delta.data, delta.count);
    OK(RAPIER_FN(LastStatus)());
    return take(tick);
}

static void reject_apply(RAPIER_TYPE(World) *world, const uint8_t *data, size_t count) {
    RAPIER_TYPE(Bytes) *tick = RAPIER_FN(ApplyWorldDelta)(world, data, count);
    assert(!tick);
    EXPECT(RAPIER_FN(LastStatus)(), RAPIER_CONST(INVALID_ARGUMENT));
}

static RAPIER_TYPE(RigidBodyHandle)
    add_ball(RAPIER_TYPE(World) *world, RAPIER_TYPE(Real) x, RAPIER_TYPE(Real) y) {
    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(DynamicRigidBodyDesc)();
    body.position.translation = vector(x, y, 0);
    RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)((RAPIER_TYPE(Real))0.5);
    RAPIER_FN(InsertCollider)(handle, &collider);
    OK(RAPIER_FN(LastStatus)());
    return handle;
}

/* A ground and a few falling balls, stepped until they touch. */
static RAPIER_TYPE(World) *pile(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(ColliderDesc) ground = RAPIER_FN(CuboidColliderDesc)(vector(10, (RAPIER_TYPE(Real))0.5, 10));
    RAPIER_FN(InsertColliderWithoutParent)(world, &ground);
    OK(RAPIER_FN(LastStatus)());
    for (int i = 0; i < 4; ++i) {
        add_ball(world, (RAPIER_TYPE(Real))0.1 * i, 1 + (RAPIER_TYPE(Real))1.1 * i);
    }
    for (int i = 0; i < 10; ++i) {
        OK(RAPIER_FN(Step)(world, NULL, NULL));
    }
    return world;
}

typedef struct Receiver {
    RAPIER_TYPE(World) *world;
    Buffer acked; /* data == NULL: nothing acknowledged yet. */
    int every;
} Receiver;

static void sync(Receiver *receiver, RAPIER_TYPE(World) *sender) {
    Buffer d, t, applied;
    delta(sender, receiver->acked.data ? &receiver->acked : NULL, &d, &t);
    applied = apply(receiver->world, d);
    assert(same(applied, t));
    free(receiver->acked.data);
    free(applied.data);
    free(d.data);
    receiver->acked = t;
    assert(same_snapshots(receiver->world, sender));
}

static Buffer copy(Buffer source) {
    Buffer result = {malloc(source.count), source.count};
    assert(result.data);
    memcpy(result.data, source.data, source.count);
    return result;
}

static void test_replication(void) {
    RAPIER_TYPE(World) *sender = pile();
    OK(RAPIER_FN(EnableChangeTracking)(sender));
    /* Restored receivers start from a snapshot taken at the instant of their baseline. */
    Buffer snap = snapshot(sender), full, baseline;
    delta(sender, NULL, &full, &baseline);
    assert(memcmp(full.data, "RPRD", 4) == 0 && memcmp(baseline.data, "RPRT", 4) == 0);
    free(full.data);
    Receiver receivers[3] = {
        {RAPIER_FN(DeserializeWorld)(snap.data, snap.count), copy(baseline), 1},
        {RAPIER_FN(NewWorld)(), {NULL, 0}, 1},
        /* Lags behind: each delta is against its own older acknowledged tick. */
        {RAPIER_FN(DeserializeWorld)(snap.data, snap.count), baseline, 4},
    };
    OK(RAPIER_FN(LastStatus)());
    free(snap.data);

    RAPIER_TYPE(RigidBodyHandle) spawned = {0};
    for (int i = 0; i < 30; ++i) {
        if (i == 5) {
            spawned = add_ball(sender, 2, 3);
        } else if (i == 13) {
            OK(RAPIER_FN(RemoveRigidBody)(spawned, 1));
        }
        OK(RAPIER_FN(Step)(sender, NULL, NULL));
        for (int r = 0; r < 3; ++r) {
            if ((i + 1) % receivers[r].every == 0) {
                sync(&receivers[r], sender);
            }
        }
    }

    /* Once caught up, receivers keep simulating identically. */
    sync(&receivers[2], sender);
    for (int i = 0; i < 20; ++i) {
        OK(RAPIER_FN(Step)(sender, NULL, NULL));
        for (int r = 0; r < 3; ++r) {
            OK(RAPIER_FN(Step)(receivers[r].world, NULL, NULL));
            assert(same_snapshots(receivers[r].world, sender));
        }
    }
    for (int r = 0; r < 3; ++r) {
        OK(RAPIER_FN(FreeWorld)(receivers[r].world));
        free(receivers[r].acked.data);
    }
    OK(RAPIER_FN(FreeWorld)(sender));
}

static void test_errors(void) {
    RAPIER_TYPE(World) *sender = pile();
    Buffer full, tick0, old_tick, other_tick, delta1, tick1, stale, ignored;

    /* Without change tracking. */
    reject_delta(sender, NULL, 0, RAPIER_CONST(UNSUPPORTED));
    OK(RAPIER_FN(EnableChangeTracking)(sender));
    delta(sender, NULL, &ignored, &old_tick);
    free(ignored.data);
    OK(RAPIER_FN(DisableChangeTracking)(sender));
    reject_delta(sender, old_tick.data, old_tick.count, RAPIER_CONST(UNSUPPORTED));
    /* A restarted tracking session rejects the ticks issued before. */
    OK(RAPIER_FN(EnableChangeTracking)(sender));
    reject_delta(sender, old_tick.data, old_tick.count, RAPIER_CONST(INVALID_ARGUMENT));
    assert(strstr(RAPIER_FN(LastError)(), "baseline"));
    delta(sender, NULL, &full, &tick0);

    /* Ticks of another world. */
    RAPIER_TYPE(World) *other = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(EnableChangeTracking)(other));
    delta(other, NULL, &ignored, &other_tick);
    free(ignored.data);
    reject_delta(sender, other_tick.data, other_tick.count, RAPIER_CONST(INVALID_ARGUMENT));

    /* Malformed ticks: NULL with a length, damaged header, truncated, or delta bytes. */
    reject_delta(sender, NULL, 1, RAPIER_CONST(INVALID_ARGUMENT));
    for (size_t i = 0; i < 16; ++i) {
        Buffer damaged = copy(tick0);
        damaged.data[i] ^= 1;
        reject_delta(sender, damaged.data, damaged.count, RAPIER_CONST(INVALID_ARGUMENT));
        free(damaged.data);
    }
    for (size_t len = 0; len < tick0.count; ++len) {
        reject_delta(sender, tick0.data, len, RAPIER_CONST(INVALID_ARGUMENT));
    }
    reject_delta(sender, full.data, full.count, RAPIER_CONST(INVALID_ARGUMENT));
    reject_delta(NULL, NULL, 0, RAPIER_CONST(NULL_POINTER));

    /* Malformed deltas leave the receiver untouched. */
    RAPIER_TYPE(World) *receiver = RAPIER_FN(NewWorld)();
    Buffer empty = snapshot(receiver);
    const uint8_t garbage[] = "not a snapshot delta";
    reject_apply(receiver, garbage, sizeof(garbage));
    reject_apply(receiver, NULL, 0);
    reject_apply(receiver, tick0.data, tick0.count);
    reject_apply(receiver, full.data, full.count - 1);
    for (size_t i = 0; i < 16; ++i) {
        Buffer damaged = copy(full);
        damaged.data[i] ^= 1;
        reject_apply(receiver, damaged.data, damaged.count);
        free(damaged.data);
    }
    Buffer now = snapshot(receiver);
    assert(same(now, empty));
    free(now.data);
    free(empty.data);

    /* A delta against a baseline the receiver moved past is rejected. */
    Buffer applied = apply(receiver, full);
    assert(same(applied, tick0));
    free(applied.data);
    OK(RAPIER_FN(Step)(sender, NULL, NULL));
    delta(sender, &tick0, &delta1, &tick1);
    applied = apply(receiver, delta1);
    assert(same(applied, tick1));
    free(applied.data);
    OK(RAPIER_FN(Step)(sender, NULL, NULL));
    delta(sender, &tick0, &stale, &ignored);
    free(ignored.data);
    reject_apply(receiver, stale.data, stale.count);
    assert(strstr(RAPIER_FN(LastError)(), "baseline"));

    /* A full delta resynchronizes it. */
    free(full.data);
    free(tick0.data);
    delta(sender, NULL, &full, &tick0);
    applied = apply(receiver, full);
    assert(same(applied, tick0));
    assert(same_snapshots(receiver, sender));

    Buffer buffers[] = {full, tick0, old_tick, other_tick, delta1, tick1, stale, applied};
    for (size_t i = 0; i < sizeof(buffers) / sizeof(buffers[0]); ++i) {
        free(buffers[i].data);
    }
    OK(RAPIER_FN(FreeWorld)(receiver));
    OK(RAPIER_FN(FreeWorld)(other));
    OK(RAPIER_FN(FreeWorld)(sender));
}

int main(void) {
    test_replication();
    test_errors();
    puts("snapshot deltas passed");
    return 0;
}
