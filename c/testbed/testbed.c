#include "testbed_internal.h"
#include "testbed.h"
#include <stdlib.h>
#include <inttypes.h>
#include <errno.h>

void tbCheck(Testbed *t, RAPIER_TYPE(Status) s, const char *call, const char *file, int line) {
    if (s == RAPIER_CONST(OK)) {
        return;
    }
    snprintf(t->error, sizeof(t->error), "%s:%d: %s: %s", file, line, call, RAPIER_FN(LastError)());
    longjmp(t->failure, 1);
}

void tbDestroy(Testbed *t) {
    for (size_t i = 0; i < t->renderMeshCount; ++i) {
        TbRenderMesh *mesh = &t->renderMeshes[i];
        free(mesh->vertices);
        free(mesh->indices);
        free(mesh->uvs);
        free(mesh->normals);
        free(mesh->texture);
    }
    free(t->renderMeshes);
    t->renderMeshes = NULL;
    t->renderMeshCount = 0;
    free(t->bodyColors);
    free(t->colliderColors);
    t->bodyColors = t->colliderColors = NULL;
    t->bodyColorCount = t->colliderColorCount = 0;
    t->world = NULL;
    t->activeThreads = 0;
}

int tbParseThreads(const char *text, size_t *out) {
    if (!text || !*text || !out) {
        return 0;
    }
    for (const char *p = text; *p; p++) {
        if (*p < '0' || *p > '9') {
            return 0;
        }
    }
    char *end;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (errno || *end || value > TB_MAX_THREADS) {
        return 0;
    }
    *out = (size_t)value;
    return 1;
}

RAPIER_TYPE(Status) tbSetThreads(Testbed *t, size_t count) {
    RAPIER_TYPE(BuildFeatures) features = RAPIER_FN(BuildFeatures)();
    RAPIER_TYPE(Status) status;
    size_t active = 1;
    if (features.parallel || count > 1) {
        status = RAPIER_FN(SetNumThreads)(t->world, count);
        if (status != RAPIER_CONST(OK)) {
            return status;
        }
        active = RAPIER_FN(NumThreads)(t->world);
        status = RAPIER_FN(LastStatus)();
        if (status != RAPIER_CONST(OK)) {
            return status;
        }
    }
    t->requestedThreads = count;
    t->activeThreads = active;
    t->buildFeatures = features;
    return RAPIER_CONST(OK);
}

void tbRefreshWorld(Testbed *t) {
    t->buildFeatures = RAPIER_FN(BuildFeatures)();
    TB(t, tbSetThreads(t, t->requestedThreads));
    TB(t, RAPIER_FN(SetCountersEnabled)(t->world, 1));
    t->physicsStepMs = 0;
}

static void sceneError(RAPIER_TYPE(Status) status, const char *message, void *userData) {
    const Testbed *t = userData;
    fprintf(stderr, "Rapier error in %s (status %u): %s\n", t->example->id, (unsigned)status,
            message);
    /* Do not longjmp out of a Rust frame. Stop before using a failed call's outputs. */
    exit(EXIT_FAILURE);
}

int tbRun(Testbed *t, const TbExample *e, int preserve) {
    tbDestroy(t);
    t->error[0] = 0;
    t->example = e;
    t->step = 0;
    t->time = 0;
    t->randomState = 42;
    t->framePending = 0;
    t->snapshotSupported = 1;
    t->inputDirection = V(0, 0, 0);
    t->action = 0;
    t->cutting = 0;
    t->cursorValid = 0;
    t->jump = t->descend = t->slow = t->boost = 0;
    t->cameraRight = V(1, 0, 0);
    t->cameraForward = V(0, 0, -1);
    t->initialDebug = 0;
    t->collidersVisible = 1;
    t->frameAll = 0;
    t->preserveCamera = 0;
    t->up[0] = 0;
    t->up[1] = 1;
    t->up[2] = 0;
    t->rayValid = t->removeVoxel = 0;
    t->lineCount = 0;
    t->labelCount = 0;
    if (!preserve) {
        t->settingCount = 0;
    }
    if (e->requires) {
        snprintf(t->error, sizeof(t->error), "%s", e->requires);
        return 0;
    }
    t->previousErrorHandler =
        RAPIER_FN(SetErrorHandler)((RAPIER_TYPE(ErrorHandler)){sceneError, t});
    if (setjmp(t->failure)) {
        RAPIER_FN(SetErrorHandler)(t->previousErrorHandler);
        (void)RAPIER_FN(FreeWorld)(t->world);
        t->world = NULL;
        return 0;
    }
    tbCamera(t, 20, 15, 25, 0, 3, 0);
    t->viewWidth = 30;
    e->run(t);
    RAPIER_FN(SetErrorHandler)(t->previousErrorHandler);
    t->world = NULL;
    return !t->error[0];
}

int tbRenderFrame(Testbed *t, RAPIER_TYPE(World) **world) {
    /* UI operations handle recoverable errors themselves. The example resumes
     * with its fail-fast handler after rendering and input have completed. */
    RAPIER_TYPE(ErrorHandler) sceneHandler = RAPIER_FN(SetErrorHandler)(t->previousErrorHandler);
    if (setjmp(t->failure)) {
        RAPIER_FN(SetErrorHandler)(sceneHandler);
        *world = t->world;
        return 0;
    }
    if (t->framePending) {
        t->step++;
        t->time += t->frameDt;
        t->physicsStepMs = RAPIER_FN(StepTimeMs)(t->world);
        TB(t, RAPIER_FN(LastStatus)());
    }
    t->framePending = 0;
    t->simulating = 0;
    int keepOpen = t->renderFrame && t->renderFrame(t, t->viewer);
    *world = t->world;
    if (keepOpen && t->simulating && !t->error[0]) {
        t->frameDt = RAPIER_FN(TimeStep)(t->world);
        RAPIER_TYPE(Status) status = RAPIER_FN(LastStatus)();
        if (status != RAPIER_CONST(OK)) {
            snprintf(t->error, sizeof(t->error), "%s", RAPIER_FN(LastError)());
            keepOpen = 0;
        }
    }
    RAPIER_FN(SetErrorHandler)(sceneHandler);
    return keepOpen;
}

int tbSimulating(Testbed *t) {
    t->framePending = t->simulating && !t->error[0];
    return t->framePending;
}

void tbCamera(Testbed *t, float x, float y, float z, float tx, float ty, float tz) {
    t->eye[0] = x;
    t->eye[1] = y;
    t->eye[2] = z;
    t->target[0] = tx;
    t->target[1] = ty;
    t->target[2] = tz;
}

void tbCamera2(Testbed *t, float x, float y, float zoom) {
    tbCamera(t, x, y, 100, x, y, 0);
    t->viewWidth = 1440.0f / zoom;
}

double tbSetting(Testbed *t, const char *name, double initial, double min, double max,
                 int integer) {
    for (size_t i = 0; i < t->settingCount; i++) {
        if (!strcmp(t->settings[i].name, name)) {
            return t->settings[i].value;
        }
    }
    if (t->settingCount == TB_COUNT(t->settings)) {
        snprintf(t->error, sizeof(t->error), "too many settings");
        longjmp(t->failure, 1);
    }
    t->settings[t->settingCount++] = (TbSetting){.name = name,
                                                 .value = initial,
                                                 .initial = initial,
                                                 .min = min,
                                                 .max = max,
                                                 .integer = integer};
    return initial;
}

double tbLiveSetting(Testbed *t, const char *name, double initial, double min, double max,
                     int integer) {
    double value = tbSetting(t, name, initial, min, max, integer);
    for (size_t i = 0; i < t->settingCount; ++i) {
        if (!strcmp(t->settings[i].name, name)) {
            t->settings[i].live = 1;
        }
    }
    return value;
}

static void tint(TbTint **colors, size_t *count, uint32_t index, uint32_t generation, float r,
                 float g, float b, float a) {
    if (index == UINT32_MAX) {
        return;
    }
    if (index >= *count) {
        size_t n = (size_t)index + 16;
        TbTint *p = realloc(*colors, n * sizeof(*p));
        if (!p) {
            abort();
        }
        memset(p + *count, 0, (n - *count) * sizeof(*p));
        *colors = p;
        *count = n;
    }
    (*colors)[index] = (TbTint){generation, {r, g, b, a}, 1};
}

void tbBodyColor(Testbed *t, RAPIER_TYPE(RigidBodyHandle) h, float r, float g, float b, float a) {
    tint(&t->bodyColors, &t->bodyColorCount, h.index, h.generation, r, g, b, a);
}

void tbColliderColor(Testbed *t, RAPIER_TYPE(ColliderHandle) h, float r, float g, float b,
                     float a) {
    tint(&t->colliderColors, &t->colliderColorCount, h.index, h.generation, r, g, b, a);
}

const float *tbFindColor(Testbed *t, RAPIER_TYPE(RigidBodyHandle) b,
                         RAPIER_TYPE(ColliderHandle) c) {
    if (c.index < t->colliderColorCount && t->colliderColors[c.index].valid &&
        t->colliderColors[c.index].generation == c.generation) {
        return t->colliderColors[c.index].rgba;
    }
    if (b.index < t->bodyColorCount && t->bodyColors[b.index].valid &&
        t->bodyColors[b.index].generation == b.generation) {
        return t->bodyColors[b.index].rgba;
    }
    return NULL;
}

void tbSetWorld(Testbed *testbed, RAPIER_TYPE(World) *world) {
    testbed->world = world;
    tbRefreshWorld(testbed);
}

void tbLine(Testbed *t, RAPIER_TYPE(Vector) a, RAPIER_TYPE(Vector) b, float r, float g, float blue,
            float alpha) {
    if (t->lineCount == TB_COUNT(t->lines)) {
        return;
    }
    size_t i = t->lineCount++;
    t->lines[i].a = a;
    t->lines[i].b = b;
    t->lines[i].rgba[0] = r;
    t->lines[i].rgba[1] = g;
    t->lines[i].rgba[2] = blue;
    t->lines[i].rgba[3] = alpha;
}

void tbLabel(Testbed *t, const char *name, const char *value) {
    size_t i = 0;
    for (; i < t->labelCount; ++i) {
        if (!strcmp(t->labels[i].name, name)) {
            break;
        }
    }
    if (i == TB_COUNT(t->labels)) {
        return;
    }
    if (i == t->labelCount) {
        ++t->labelCount;
    }
    t->labels[i].name = name;
    snprintf(t->labels[i].value, sizeof(t->labels[i].value), "%s", value);
}
#ifdef _WIN32
#include <windows.h>

double tbClock(void) {
    LARGE_INTEGER frequency, counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart / frequency.QuadPart;
}
#else
#include <time.h>

double tbClock(void) {
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time.tv_sec + time.tv_nsec * 1e-9;
}
#endif

size_t tbChoice(Testbed *t, const char *name, size_t initial, const char *const *choices,
                size_t count, int live, int reset) {
    tbSetting(t, name, initial, 0, count ? count - 1 : 0, 1);
    for (size_t i = 0; i < t->settingCount; ++i) {
        TbSetting *s = &t->settings[i];
        if (strcmp(s->name, name)) {
            continue;
        }
        s->choices = choices;
        s->choiceCount = count;
        s->live = live;
        s->max = count ? count - 1 : 0;
        if (reset || s->value > s->max) {
            s->value = initial;
        }
        return (size_t)s->value;
    }
    return initial;
}

static void *copyRenderData(const void *data, size_t bytes) {
    if (!data || !bytes) {
        return NULL;
    }
    void *copy = malloc(bytes);
    if (!copy) {
        abort();
    }
    memcpy(copy, data, bytes);
    return copy;
}

void tbAddBodyRenderMesh(Testbed *t, const TbRenderMesh *source) {
    TbRenderMesh *meshes = realloc(t->renderMeshes, (t->renderMeshCount + 1) * sizeof(*meshes));
    if (!meshes) {
        abort();
    }
    t->renderMeshes = meshes;
    TbRenderMesh *mesh = &meshes[t->renderMeshCount++];
    *mesh = *source;
    mesh->vertices =
        copyRenderData(source->vertices, source->vertexCount * sizeof(*source->vertices));
    mesh->indices = copyRenderData(source->indices, source->indexCount * sizeof(*source->indices));
    mesh->uvs = copyRenderData(source->uvs, source->vertexCount * 2 * sizeof(float));
    mesh->normals = copyRenderData(source->normals, source->vertexCount * 3 * sizeof(float));
    mesh->texture =
        source->texture ? copyRenderData(source->texture, strlen(source->texture) + 1) : NULL;
}
