/* Exercise the real soft-mesh renderer without creating a window or GPU context. */
#include "testbed.h"
#include "raylib.h"
#include "rlgl.h"
#include <assert.h>

static size_t triangles, lineCount, translucent;
static bool depthWrite = true, pendingTranslucent;
static int culling = 1, flushed;

static void begin3d(Camera3D camera) {
    (void)camera;
}

static void end3d(void) {
    /* raylib flushes buffered triangles here, using the current culling state. */
    assert(!culling);
    ++flushed;
}

static void enableCulling(void) {
    culling = 1;
}

static void disableCulling(void) {
    culling = 0;
}

static void flushBatch(void) {
    if (pendingTranslucent) {
        assert(!depthWrite);
        pendingTranslucent = false;
    }
}

static void disableDepthWrite(void) {
    assert(!pendingTranslucent);
    depthWrite = false;
}

static void enableDepthWrite(void) {
    assert(!pendingTranslucent);
    depthWrite = true;
}

static void checkPoint(Vector3 point) {
    assert(isfinite(point.x) && isfinite(point.y) && isfinite(point.z));
}

static void recordTriangle(Vector3 a, Vector3 b, Vector3 c, Color color) {
    if (color.a < 255) {
        assert(color.a == 102);
        assert(!depthWrite);
        ++translucent;
        pendingTranslucent = true;
    }
    checkPoint(a);
    checkPoint(b);
    checkPoint(c);
    ++triangles;
}

static void recordLine(Vector3 a, Vector3 b, Color color) {
    (void)color;
    checkPoint(a);
    checkPoint(b);
    ++lineCount;
}

static void *boundedRealloc(void *pointer, size_t bytes) {
    /* Catch a sentinel used as a cache index before allocating gigabytes. */
    if (bytes > 64 * 1024 * 1024) {
        fprintf(stderr, "Unexpected renderer allocation: %zu bytes\n", bytes);
        abort();
    }
    return realloc(pointer, bytes);
}

#define BeginMode3D begin3d
#define EndMode3D end3d
#define rlEnableBackfaceCulling enableCulling
#define rlDisableBackfaceCulling disableCulling
#define DrawTriangle3D recordTriangle
#define DrawLine3D recordLine
#define rlDrawRenderBatchActive flushBatch
#define rlDisableDepthMask disableDepthWrite
#define rlEnableDepthMask enableDepthWrite
#define realloc boundedRealloc
#include "../graphics.c"
#undef BeginMode3D
#undef EndMode3D
#undef rlEnableBackfaceCulling
#undef rlDisableBackfaceCulling
#undef rlDrawRenderBatchActive
#undef rlDisableDepthMask
#undef rlEnableDepthMask
#undef realloc
#undef DrawLine3D
#undef DrawTriangle3D

typedef struct RenderTest {
    TbGraphics graphics;
    size_t frames;
    bool forceSensors, expectTranslucent;
} RenderTest;

static int renderFrame(Testbed *t, void *context) {
    RenderTest *test = context;
    if (!test->frames && test->forceSensors) {
        size_t count = RAPIER_FN(ColliderHandles)(t->world, NULL, 0);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
        RAPIER_TYPE(ColliderHandle) *handles = malloc(count * sizeof(*handles));
        assert(handles);
        count = RAPIER_FN(ColliderHandles)(t->world, handles, count);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
        for (size_t i = 0; i < count; ++i) {
            assert(RAPIER_FN(Collider_ValidateHandle)(handles[i]) == RAPIER_CONST(OK));
            assert(RAPIER_FN(Collider_SetSensor)(handles[i], 1) == RAPIER_CONST(OK));
        }
        free(handles);
    }
    triangles = lineCount = translucent = 0;
    test->graphics.transparentCount = 0;
    drawSoft(&test->graphics, t, true, (Camera3D){.position = {0, 0, 100}, .fovy = 20});
    assert(translucent == 0); /* No sensor geometry in the opaque pass. */
    size_t queued = test->graphics.transparentCount;
    drawTransparent(&test->graphics);
    assert(translucent == queued && depthWrite && !pendingTranslucent);
    assert((translucent > 0) == test->expectTranslucent);
    for (size_t i = 1; i < queued; ++i) {
        assert(test->graphics.transparent[i - 1].depth >= test->graphics.transparent[i].depth);
    }
    assert(triangles + lineCount > 0);
    assert(test->graphics.entryCapacity < 1024);
    /* Turning surfaces off must also avoid indexing render-only colliders. */
    drawSoft(&test->graphics, t, false, (Camera3D){0});
    /* The full frame must flush before restoring culling, even on an empty
     * surface pass. Mesh draw calls above are recorded without a GPU. */
    int previousFlushes = flushed;
    assert(tbGraphicsDraw(&test->graphics, t, (Camera3D){0}, 0, false));
    assert(flushed == previousFlushes + 1 && culling);
    if (++test->frames == 4) {
        return 0;
    }
    t->simulating = 1;
    return 1;
}

static void checkScene(const char *id, bool forceSensors, bool expectTranslucent) {
    const TbExample *example = NULL;
    for (size_t i = 0; i < tbExampleCount; ++i) {
        if (!strcmp(tbExamples[i].id, id)) {
            example = &tbExamples[i];
        }
    }
    assert(example && !example->requires);
    Testbed t = {0};
    RenderTest test = {.forceSensors = forceSensors, .expectTranslucent = expectTranslucent};
    t.renderFrame = renderFrame;
    t.viewer = &test;
    t.requestedThreads = 1;
    t.assetRoot = TB_ASSET_ROOT;
    t.noSleep = 1;
    assert(tbRun(&t, example, 0));
    assert(test.frames == 4 && t.step == 3);
    free(test.graphics.transparent);
    free(test.graphics.entries);
    free(test.graphics.softHandles);
    free(test.graphics.softMeshes);
    free(test.graphics.vertices);
    free(test.graphics.indices);
    tbDestroy(&t);
    printf("PASS soft renderer: %s\n", id);
}

int main(void) {
#if defined(RAPIER_DIM3)
    checkScene("soft_meshes3", false, true);
    checkScene("soft_trimesh3", false, false);
    checkScene("soft_surface3", false, false);
    checkScene("soft_cloth3", false, false);
#else
    checkScene("soft_letters2", false, false);
    checkScene("soft_surface2", false, false);
    checkScene("soft_surface2", true, true);
#endif
    return 0;
}
