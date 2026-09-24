#include "testbed.h"
#include "rapier_helpers.h"

static int renderFrame(Testbed *testbed, void *context) {
    (void)context;
    testbed->simulating = 1;
    return 1;
}

static void invalidCall(void) {
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(BallColliderDesc)(-1);
    RAPIER_FN(ShapeDesc_Build)(&collider.shape);
    fputs("continued after failed call\n", stderr);
    abort();
}

static void run(Testbed *testbed) {
    if (testbed->action) {
        invalidCall();
    }
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    tbSetWorld(testbed, world);
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            RAPIER_FN(Step)(world, NULL, NULL);
            invalidCall();
        }
    }
    RAPIER_FN(FreeWorld)(world);
}

static void failSetup(Testbed *testbed) {
    (void)testbed;
    invalidCall();
}

int main(int argc, char **argv) {
    Testbed testbed = {0};
    testbed.requestedThreads = 1;
    testbed.renderFrame = renderFrame;
    TbExample example = {"error-test", "", "", "", run, NULL};
    if (argc == 2 && !strcmp(argv[1], "setup")) {
        example.run = failSetup;
    }
    return tbRun(&testbed, &example, 0) ? 0 : 2;
}
