#include "testbed_internal.h"
#include "testbed.h"
#include <stdlib.h>
#include <errno.h>
#include <inttypes.h>

static int finiteVector(RAPIER_TYPE(Vector) p) {
#if defined(RAPIER_DIM3)
    if (!isfinite(p.z)) {
        return 0;
    }
#endif
    return isfinite(p.x) && isfinite(p.y);
}

int tbValidate(Testbed *t, size_t *nb, size_t *nc, size_t *ns) {
    if (setjmp(t->failure)) {
        return 0;
    }
    *nb = RAPIER_FN(RigidBodyCount)(t->world);
    TB(t, RAPIER_FN(LastStatus)());
    *nc = RAPIER_FN(ColliderCount)(t->world);
    TB(t, RAPIER_FN(LastStatus)());
    *ns = RAPIER_FN(SoftBodyCount)(t->world);
    TB(t, RAPIER_FN(LastStatus)());
    RAPIER_TYPE(RigidBodyHandle) *b = malloc((*nb ? *nb : 1) * sizeof(*b));
    if (!b) {
        return 0;
    }
    *nb = RAPIER_FN(RigidBodyHandles)(t->world, b, *nb);
    RAPIER_TYPE(Status) status = RAPIER_FN(LastStatus)();
    if (status) {
        free(b);
        TB(t, status);
    }
    for (size_t i = 0; i < *nb; i++) {
        RAPIER_TYPE(Vector) p, v;
        status = RAPIER_FN(RigidBody_ValidateHandle)(b[i]);
        if (!status) {
            p = RAPIER_FN(RigidBody_Translation)(b[i]);
            status = RAPIER_FN(LastStatus)();
        }
        if (!status) {
            v = RAPIER_FN(RigidBody_Linvel)(b[i]);
            status = RAPIER_FN(LastStatus)();
        }
        if (status || !finiteVector(p) || !finiteVector(v)) {
            free(b);
            snprintf(t->error, sizeof(t->error), "non-finite or invalid rigid body");
            return 0;
        }
    }
    free(b);
    RAPIER_TYPE(SoftBodyHandle) *s = malloc((*ns ? *ns : 1) * sizeof(*s));
    if (!s) {
        return 0;
    }
    *ns = RAPIER_FN(SoftBodyHandles)(t->world, s, *ns);
    status = RAPIER_FN(LastStatus)();
    if (status) {
        free(s);
        TB(t, status);
    }
    for (size_t i = 0; i < *ns; i++) {
        size_t n = 0;
        status = RAPIER_FN(SoftBody_ValidateHandle)(s[i]);
        if (!status) {
            n = RAPIER_FN(SoftBody_ParticlePositions)(s[i], NULL, 0);
            status = RAPIER_FN(LastStatus)();
        }
        if (status) {
            free(s);
            TB(t, status);
        }
        RAPIER_TYPE(Vector) *p = malloc((n ? n : 1) * sizeof(*p));
        if (!p) {
            free(s);
            return 0;
        }
        n = RAPIER_FN(SoftBody_ParticlePositions)(s[i], p, n);
        status = RAPIER_FN(LastStatus)();
        for (size_t j = 0; j < n && !status; j++) {
            if (!finiteVector(p[j])) {
                status = RAPIER_CONST(INVALID_ARGUMENT);
            }
        }
        free(p);
        if (status) {
            free(s);
            snprintf(t->error, sizeof(t->error), "non-finite or invalid soft body");
            return 0;
        }
    }
    free(s);
    return 1;
}

typedef struct HeadlessViewer {
    unsigned long steps;
    size_t bodies, colliders, softBodies;
    int valid;
} HeadlessViewer;

static int headlessFrame(Testbed *t, void *context) {
    HeadlessViewer *viewer = context;
    if (t->step == 0) {
        printf("CONFIG %s SIMD=%u parallel=%s workers=%zu requested=%zu\n", t->example->id,
               t->buildFeatures.simd_lanes, t->buildFeatures.parallel ? "on" : "off",
               t->activeThreads, t->requestedThreads);
    }
    if (t->step < viewer->steps && !t->error[0]) {
        t->simulating = 1;
        return 1;
    }
    viewer->valid = tbValidate(t, &viewer->bodies, &viewer->colliders, &viewer->softBodies);
    return 0;
}

static void usage(const char *name) {
    printf("Usage: %s [--list] [--example ID] [--all] [--steps N] [--no-sleep] [--threads N] "
           "[--assets PATH]\n",
           name);
}

int tbHeadless(int argc, char **argv) {
    const char *id = NULL, *assets = TB_ASSET_ROOT;
    int all = 0, noSleep = 0;
    unsigned long steps = 120;
    size_t threads = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--headless")) {
            continue;
        }
        if (!strcmp(argv[i], "--list")) {
            for (size_t j = 0; j < tbExampleCount; j++) {
                printf("%s\t%s / %s%s%s\n", tbExamples[j].id, tbExamples[j].group,
                       tbExamples[j].name, tbExamples[j].requires ? "\tUNAVAILABLE: " : "",
                       tbExamples[j].requires ? tbExamples[j].requires : "");
            }
            return 0;
        }
        if (!strcmp(argv[i], "--all")) {
            all = 1;
            continue;
        }
        if (!strcmp(argv[i], "--no-sleep")) {
            noSleep = 1;
            continue;
        }
        if (!strcmp(argv[i], "--threads") && i + 1 < argc) {
            if (!tbParseThreads(argv[++i], &threads)) {
                fprintf(stderr, "--threads expects an integer from 0 (automatic) to %d\n",
                        TB_MAX_THREADS);
                return 2;
            }
            continue;
        }
        if (!strcmp(argv[i], "--example") && i + 1 < argc) {
            id = argv[++i];
            continue;
        }
        if (!strcmp(argv[i], "--assets") && i + 1 < argc) {
            assets = argv[++i];
            continue;
        }
        if (!strcmp(argv[i], "--steps") && i + 1 < argc) {
            char *end;
            errno = 0;
            const char *s = argv[++i];
            steps = strtoul(s, &end, 10);
            if (errno || *end || s == end || s[0] == '-') {
                usage(argv[0]);
                return 2;
            }
            continue;
        }
        usage(argv[0]);
        return !strcmp(argv[i], "--help") ? 0 : 2;
    }
    Testbed *t = calloc(1, sizeof(*t));
    if (!t) {
        return 1;
    }
    t->noSleep = noSleep;
    t->assetRoot = assets;
    t->requestedThreads = threads;
    size_t ran = 0, failed = 0, skipped = 0;
    for (size_t i = 0; i < tbExampleCount; i++) {
        const TbExample *e = &tbExamples[i];
        if (!all && ((id && strcmp(e->id, id)) || (!id && i))) {
            continue;
        }
        if (e->requires) {
            printf("SKIP %s: %s\n", e->id, e->requires);
            skipped++;
            continue;
        }
        HeadlessViewer viewer = {.steps = steps};
        t->renderFrame = headlessFrame;
        t->viewer = &viewer;
        int ok = tbRun(t, e, 0) && viewer.valid;
        printf("%s %s steps=%" PRIu64 " bodies=%zu colliders=%zu soft=%zu%s%s\n",
               ok ? "PASS" : "FAIL", e->id, t->step, viewer.bodies, viewer.colliders,
               viewer.softBodies, ok ? "" : " ", ok ? "" : t->error);
        fflush(stdout);
        ran++;
        failed += !ok;
    }
    tbDestroy(t);
    free(t);
    if (!ran && !skipped) {
        fprintf(stderr, "unknown example: %s\n", id ? id : "");
        return 2;
    }
    printf("%zu passed, %zu failed, %zu unavailable\n", ran - failed, failed, skipped);
    return failed || (!all && skipped) ? 1 : 0;
}
#ifdef TB_HEADLESS_ONLY
int main(int argc, char **argv) {
    return tbHeadless(argc, argv);
}
#endif
