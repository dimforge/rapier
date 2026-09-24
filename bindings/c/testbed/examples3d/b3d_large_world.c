/* Port of examples3d/b3d_large_world.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB3dLargeWorld(Testbed *testbed) {
    R3World *world = r3NewWorld();
    /* One million parentless floor colliders, matching the Rust benchmark. */
    r3SetGravity(world, r3Vector(0, -10, 0));
    const R3Real cell = 10;
    const int grid = 1000, spheres = 100, dropInterval = 5;
    const R3Real halfSpan = .5 * cell * grid;
    for (int i = 0; i < grid; ++i) {
        const R3Real x = -halfSpan + (i + .5) * cell;
        for (int j = 0; j < grid; ++j) {
            const R3Real z = -halfSpan + (j + .5) * cell;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.5 * cell, .25, .5 * cell));
            collider.position.translation = r3Vector(x, 0, z);
            r3InsertColliderWithoutParent(world, &collider);
        }
    }
    tbCamera(testbed, 0, 60, 250, 0, 0, 0);

    tbSetWorld(testbed, world);
    int side = 1;
    while (side * side < spheres) {
        ++side;
    }
    int stepCount = 0, dropped = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            if (dropped < spheres && stepCount > 0 && stepCount % dropInterval == 0) {
                const int gi = dropped % side, gj = dropped / side;
                const R3Real inset = .1 * 2 * halfSpan;
                const R3Real usable = 2 * halfSpan - 2 * inset;
                const R3Real x = -halfSpan + inset + (gi + .5) * (usable / side);
                const R3Real z = -halfSpan + inset + (gj + .5) * (usable / side);
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(x, 1.5, z);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3BallColliderDesc(.5);
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
                ++dropped;
            }
            ++stepCount;
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
