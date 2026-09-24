/* Manual no-sleep benchmark, linked to the actual C testbed and loaded library.
 * Writes trusted snapshots for compare_steps.rs; excludes setup and
 * serialization. Only rigid-body scenes without local animation state are accepted so native
 * Rust can replay them.
 */
#include "testbed.h"
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif

static double nowMs(void) {
#ifdef _WIN32
    LARGE_INTEGER value, frequency;
    assert(QueryPerformanceCounter(&value));
    assert(QueryPerformanceFrequency(&frequency));
    return (double)value.QuadPart * 1000.0 / (double)frequency.QuadPart;
#else
    struct timespec t;
    assert(clock_gettime(CLOCK_MONOTONIC, &t) == 0);
    return t.tv_sec * 1000.0 + t.tv_nsec * 1.0e-6;
#endif
}

static void save(Testbed *t, const char *path) {
    RAPIER_TYPE(Bytes) *bytes = NULL;
    const uint8_t *data = NULL;
    size_t size = 0;
    bytes = RAPIER_FN(SerializeWorld)(t->world);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    RAPIER_TYPE(ByteView) bytesDataResult = RAPIER_FN(Bytes_Data)(bytes);
    data = bytesDataResult.data;
    size = bytesDataResult.count;
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    FILE *file = fopen(path, "wb");
    assert(file && fwrite(data, 1, size, file) == size);
    assert(fclose(file) == 0);
    assert(RAPIER_FN(FreeBytes)(bytes) == RAPIER_CONST(OK));
}

typedef struct Benchmark {
    const char *initialPath, *finalPath;
    double start, wall, engine;
} Benchmark;

static int renderFrame(Testbed *t, void *context) {
    Benchmark *benchmark = context;
    if (t->step == 0) {
        assert(t->snapshotSupported);
        size_t softCount = RAPIER_FN(SoftBodyCount)(t->world);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
        assert(softCount == 0);
        save(t, benchmark->initialPath);
    } else if (t->step > 120) {
        benchmark->wall += nowMs() - benchmark->start;
        benchmark->engine += t->physicsStepMs;
    }
    if (t->step < 420) {
        t->simulating = 1;
        benchmark->start = nowMs();
        return 1;
    }
    size_t n = RAPIER_FN(RigidBodyHandles)(t->world, NULL, 0);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    RAPIER_TYPE(RigidBodyHandle) *handles = malloc(n * sizeof(*handles));
    n = RAPIER_FN(RigidBodyHandles)(t->world, handles, n);
    assert(handles && RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    size_t active = 0;
    for (size_t i = 0; i < n; i++) {
        RAPIER_TYPE(Bool) dynamic = 0, sleeping = 0;
        assert(RAPIER_FN(RigidBody_ValidateHandle)(handles[i]) == RAPIER_CONST(OK));
        dynamic = RAPIER_FN(RigidBody_IsDynamic)(handles[i]);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
        sleeping = RAPIER_FN(RigidBody_IsSleeping)(handles[i]);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
        if (dynamic) {
            assert(!sleeping);
            active++;
        }
    }
    save(t, benchmark->finalPath);
    printf("C %s bodies=%zu awake_dynamic=%zu SIMD=%u workers=%zu warmup=120 "
           "measured=300 wall_ms=%.6f engine_ms=%.6f\n",
           t->example->id, n, active, t->buildFeatures.simd_lanes, t->activeThreads,
           benchmark->wall / 300, benchmark->engine / 300);
    free(handles);
    return 0;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s SCENE INITIAL_SNAPSHOT FINAL_SNAPSHOT\n", argv[0]);
        return 2;
    }
    Testbed testbed = {0};
    testbed.noSleep = 1;
    testbed.requestedThreads = 1;
    testbed.assetRoot = TB_ASSET_ROOT;
    testbed.renderFrame = renderFrame;
    Benchmark benchmark = {.initialPath = argv[2], .finalPath = argv[3]};
    testbed.viewer = &benchmark;
    const TbExample *scene = NULL;
    for (size_t i = 0; i < tbExampleCount; i++) {
        if (!strcmp(tbExamples[i].id, argv[1])) {
            scene = &tbExamples[i];
        }
    }
    assert(scene && tbRun(&testbed, scene, 0));
    tbDestroy(&testbed);
}
