#include "testbed.h"
#include <assert.h>

#define CHECK(call)                                                                                \
    do {                                                                                           \
        if ((call) != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

typedef struct Viewer {
    unsigned frame;
    uint64_t expectedStep, snapshotStep;
    double snapshotTime;
    RAPIER_TYPE(Bytes) *snapshot;
    RAPIER_TYPE(World) *original;
} Viewer;

static const TbExample *example(const char *name) {
    for (size_t i = 0; i < tbExampleCount; i++) {
        if (!strcmp(tbExamples[i].name, name)) {
            return &tbExamples[i];
        }
    }
    abort();
}

static int renderFrame(Testbed *t, void *context) {
    Viewer *viewer = context;
    assert(t->step == viewer->expectedStep);
    assert(t->activeThreads == (t->requestedThreads ? t->requestedThreads : t->activeThreads));
    if (viewer->frame == 0) {
        viewer->original = t->world;
        CHECK(RAPIER_FN(SetTimeStep)(t->world, (RAPIER_TYPE(Real))(1.0 / 120)));
    }
    if (viewer->frame == 3) {
        if (t->buildFeatures.parallel) {
            CHECK(tbSetThreads(t, 2));
            assert(t->activeThreads == 2);
            CHECK(tbSetThreads(t, 0));
            assert(t->activeThreads > 0);
            CHECK(tbSetThreads(t, 2));
        } else {
            assert(tbSetThreads(t, 2) == RAPIER_CONST(UNSUPPORTED));
            assert(t->activeThreads == 1);
        }
        assert(t->world == viewer->original);
        RAPIER_TYPE(Real) dt = RAPIER_FN(TimeStep)(t->world);
        CHECK(RAPIER_FN(LastStatus)());
        assert(fabs(dt - 1.0 / 120) < 1.0e-6);
    }
    if (viewer->frame == 10) {
        viewer->snapshot = RAPIER_FN(SerializeWorld)(t->world);
        CHECK(RAPIER_FN(LastStatus)());
        viewer->snapshotStep = t->step;
        viewer->snapshotTime = t->time;
    }
    if (viewer->frame == 15) {
        const uint8_t *data;
        size_t size;
        RAPIER_TYPE(World) *restored = NULL;
        RAPIER_TYPE(ByteView) bytesDataResult = RAPIER_FN(Bytes_Data)(viewer->snapshot);
        data = bytesDataResult.data;
        size = bytesDataResult.count;
        CHECK(RAPIER_FN(LastStatus)());
        restored = RAPIER_FN(DeserializeWorld)(data, size);
        CHECK(RAPIER_FN(LastStatus)());
        CHECK(RAPIER_FN(FreeWorld)(t->world));
        t->world = restored;
        t->step = viewer->snapshotStep;
        t->time = viewer->snapshotTime;
        viewer->expectedStep = t->step;
        tbRefreshWorld(t);
        assert(t->activeThreads == t->requestedThreads);
    }
    if (viewer->frame == 30) {
        size_t bodies, colliders, softBodies;
        assert(tbValidate(t, &bodies, &colliders, &softBodies));
        assert(bodies > 0 && colliders > 0);
        CHECK(RAPIER_FN(FreeBytes)(viewer->snapshot));
        viewer->snapshot = NULL;
        return 0;
    }
    /* Paused frames still return to the example. One requested step advances once. */
    t->simulating = viewer->frame != 7 && viewer->frame != 9;
    viewer->expectedStep += t->simulating;
    viewer->frame++;
    return 1;
}

int main(void) {
    size_t parsed;
    assert(tbParseThreads("0", &parsed) && parsed == 0);
    assert(tbParseThreads("1", &parsed) && parsed == 1);
    assert(tbParseThreads("256", &parsed) && parsed == 256);
    assert(!tbParseThreads("257", &parsed));
    assert(!tbParseThreads("-1", &parsed));
    assert(!tbParseThreads("", &parsed));
    assert(!tbParseThreads("2x", &parsed));
    assert(!tbParseThreads("999999999999999999999999999999999999999", &parsed));
    Testbed testbed = {0};
    testbed.requestedThreads = 1;
    testbed.noSleep = 1;
    testbed.assetRoot = TB_ASSET_ROOT;
    testbed.renderFrame = renderFrame;
    const char *names[] = {"Restitution", "Restitution", "Damping"};
    for (size_t i = 0; i < TB_COUNT(names); i++) {
        Viewer viewer = {0};
        testbed.viewer = &viewer;
        size_t requested = testbed.requestedThreads;
        assert(tbRun(&testbed, example(names[i]), i == 1));
        assert(!testbed.world); /* The example freed its world after its loop. */
        assert(viewer.frame == 30);
        if (i) {
            assert(testbed.requestedThreads == requested);
        }
    }
    tbDestroy(&testbed);
    puts("Example-owned loops, pause/step, worker changes, restart/switch, and snapshot restore "
         "passed");
}
