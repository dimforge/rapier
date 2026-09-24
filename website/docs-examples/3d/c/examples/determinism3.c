#include "snippets.h"
#include <math.h>

int main(void) {
    snippets_init();
    R3World *world = r3NewWorld();

    {
        // DOCUSAURUS: DeterminismWrong start
        /* WRONG version:
         * The following will not work cross-platform-deterministically because the values
         * given to the collider translation are computed by the math library of the platform. */
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.position.translation = r3Vector(sqrtf(1.0f), sinf(2.0f), cosf(3.0f));
        // DOCUSAURUS: DeterminismWrong stop
        r3InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Determinism start
        /* CORRECT version:
         * The following will work cross-platform-deterministically because we use the
         * math functions of Rapier (the square root is exactly rounded on every platform). */
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.position.translation = r3Vector(sqrtf(1.0f), r3Sin(2.0), r3Cos(3.0));
        // DOCUSAURUS: Determinism stop
        r3InsertColliderWithoutParent(world, &collider);
    }

    // DOCUSAURUS: CheckDeterminism start
    /* Check that the loaded library is built with the enhanced-determinism feature. */
    if (!r3BuildFeatures().enhanced_determinism) {
        fprintf(stderr, "This Rapier library isn't cross-platform deterministic.\n");
    }
    // DOCUSAURUS: CheckDeterminism stop

    // DOCUSAURUS: SnapshotHash start
    /* Two worlds are in the exact same state if their snapshots are identical. */
    R3Bytes *snapshot = r3SerializeWorld(world);
    R3ByteView bytes = r3Bytes_Data(snapshot);
    uint64_t hash = 14695981039346656037ull; /* FNV-1a, or any other hash function. */
    for (size_t i = 0; i < bytes.count; i++) {
        hash = (hash ^ bytes.data[i]) * 1099511628211ull;
    }
    printf("World hash: %016llx\n", (unsigned long long)hash);
    r3FreeBytes(snapshot);
    // DOCUSAURUS: SnapshotHash stop

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
