#ifndef RAPIER_TESTBED_H
#define RAPIER_TESTBED_H
#include "rapier.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define TB_PI ((RAPIER_TYPE(Real))3.14159265358979323846)
#define TB_MAX_THREADS 256
#define TB_COUNT(a) (sizeof(a) / sizeof((a)[0]))
#if defined(RAPIER_DIM2)
#define V(x, y, ...) ((RAPIER_TYPE(Vector)){(RAPIER_TYPE(Real))(x), (RAPIER_TYPE(Real))(y)})
#else
#define V(x, y, z) ((RAPIER_TYPE(Vector)){(RAPIER_TYPE(Real))(x), (RAPIER_TYPE(Real))(y), (RAPIER_TYPE(Real))(z)})
#endif
typedef struct Testbed Testbed;

typedef struct TbSetting {
    const char *name;
    double value, initial, min, max;
    int integer, live;
    const char *const *choices;
    size_t choiceCount;
} TbSetting;

typedef struct TbExample {
    const char *id, *group, *name, *source;
    void (*run)(Testbed *);
    const char *
        requires;
} TbExample;

typedef struct TbTint {
    uint32_t generation;
    float rgba[4];
    int valid;
} TbTint;

/* Render-only body geometry. Arrays are copied by tbAddBodyRenderMesh. */
typedef struct TbRenderMesh {
    RAPIER_TYPE(RigidBodyHandle) body;
    RAPIER_TYPE(Pose) localPose;
    RAPIER_TYPE(Vector) *vertices;
    uint32_t *indices;
    size_t vertexCount, indexCount;
    float *uvs, *normals;
    char *texture;
    float rgba[4], metallic, roughness, reflectance, emissive[3];
} TbRenderMesh;

#ifdef _MSC_VER
#pragma warning(push)
/* Windows jmp_buf requires 16-byte alignment; padding here is intentional. */
#pragma warning(disable : 4324)
#endif
struct Testbed {
    TbTint *bodyColors, *colliderColors;
    size_t bodyColorCount, colliderColorCount;
    RAPIER_TYPE(World) *world;
    double physicsStepMs;
    /* The renderer returns control to the example once per frame. */
    int (*renderFrame)(Testbed *, void *);
    void *viewer;
    int simulating, framePending, snapshotSupported;
    RAPIER_TYPE(Real) frameDt;
    RAPIER_TYPE(ErrorHandler) previousErrorHandler;
    jmp_buf failure;
    char error[1024];
    const TbExample *example;
    uint64_t step, randomState;
    double time;
    int noSleep;
    size_t requestedThreads, activeThreads;
    RAPIER_TYPE(BuildFeatures) buildFeatures;
    float eye[3], target[3], up[3], viewWidth;
    int collidersVisible, frameAll, preserveCamera;
    TbRenderMesh *renderMeshes;
    size_t renderMeshCount;
    RAPIER_TYPE(Vector) inputDirection;
    int action, cutting, cursorValid, jump, descend, slow, boost;
    RAPIER_TYPE(Vector) cameraRight, cameraForward;
    RAPIER_TYPE(Vector) cursor, rayOrigin, rayDirection;
    int rayValid, removeVoxel;
    uint32_t initialDebug;

    struct {
        RAPIER_TYPE(Vector) a, b;
        float rgba[4];
    } lines[256];

    size_t lineCount;

    struct {
        const char *name;
        char value[256];
    } labels[32];

    size_t labelCount;
    TbSetting settings[64];
    size_t settingCount;
    const char *assetRoot;
};
#ifdef _MSC_VER
#pragma warning(pop)
#endif

double tbClock(void);
size_t tbChoice(Testbed *, const char *, size_t, const char *const *, size_t, int live, int reset);
void tbAddBodyRenderMesh(Testbed *, const TbRenderMesh *);
void tbLabel(Testbed *, const char *, const char *);
void tbLine(Testbed *, RAPIER_TYPE(Vector), RAPIER_TYPE(Vector), float, float, float, float);
void tbBodyColor(Testbed *, RAPIER_TYPE(RigidBodyHandle), float, float, float, float);
void tbColliderColor(Testbed *, RAPIER_TYPE(ColliderHandle), float, float, float, float);
const float *tbFindColor(Testbed *, RAPIER_TYPE(RigidBodyHandle), RAPIER_TYPE(ColliderHandle));
extern const TbExample tbExamples[];
extern const size_t tbExampleCount;
void tbDestroy(Testbed *);
int tbRun(Testbed *, const TbExample *, int preserveSettings);
/* Render one frame; update *world if the viewer restored a snapshot. */
int tbRenderFrame(Testbed *, RAPIER_TYPE(World) **world);
int tbSimulating(Testbed *);
void tbRefreshWorld(Testbed *);
/* Borrow a scene world for rendering and controls; the example owns its lifetime. */
void tbSetWorld(Testbed *, RAPIER_TYPE(World) *);
RAPIER_TYPE(Status) tbSetThreads(Testbed *, size_t);
int tbParseThreads(const char *, size_t *);
void tbCamera(Testbed *, float, float, float, float, float, float);
void tbCamera2(Testbed *, float, float, float);
double tbSetting(Testbed *, const char *, double, double, double, int);

/* A setting read each frame by the example, applied without restarting. */
double tbLiveSetting(Testbed *, const char *, double, double, double, int);

int tbValidate(Testbed *, size_t *, size_t *, size_t *);
int tbHeadless(int, char **);
int tbGui(int, char **);
#endif
