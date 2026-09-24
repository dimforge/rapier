/* Port of examples3d/debug_deserialize3.rs. */
#include "testbed.h"
#include <errno.h>

void tbDebugDeserialize3(Testbed *testbed) {
    const unsigned frameId = (unsigned)tbSetting(testbed, "frame", 0, 0, 1400, 1);
    const char *frameDirs = getenv("RAPIER_SNAPSHOT_DIR");
    if (!frameDirs) {
        frameDirs = "/Users/sebcrozet/work/hytopia/sdk/examples/bug-demo";
    }
    char path[4096];
    snprintf(path, sizeof(path), "%s/snapshot%u.bincode", frameDirs, frameId);
    FILE *file = fopen(path, "rb");
    if (!file) {
        snprintf(testbed->error, sizeof(testbed->error),
                 "Cannot open snapshot%u.bincode: %s. Set RAPIER_SNAPSHOT_DIR to the snapshot "
                 "directory.",
                 frameId, strerror(errno));
        return;
    }
    if (fseek(file, 0, SEEK_END)) {
        fclose(file);
        return;
    }
    long length = ftell(file);
    if (length < 0 || length > 256L * 1024 * 1024 || fseek(file, 0, SEEK_SET)) {
        fclose(file);
        return;
    }
    unsigned char *bytes = malloc(length ? (size_t)length : 1);
    if (!bytes) {
        abort();
    }
    size_t count = fread(bytes, 1, (size_t)length, file);
    fclose(file);
    if (count != (size_t)length) {
        free(bytes);
        return;
    }
    R3World *world = r3DeserializeRigidState(bytes, count);
    free(bytes);
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);
    tbSetWorld(testbed, world);
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
