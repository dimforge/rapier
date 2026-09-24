/* Port of examples3d/stress_tests/keva3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void buildBlock(Testbed *, R3World *, R3Vector, R3Vector, int, int, int);

void tbStressTestsKeva3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    /* Ground. */
    const R3Real groundSize = 50.0;
    const R3Real groundHeight = 0.1;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0.0, -groundHeight, 0.0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(groundSize, groundHeight, groundSize));
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    /* Create the cubes. Odd layer counts keep adjacent blocks aligned. */
    const R3Vector halfExtents = r3Vector(0.1, 0.5, 2.0);
    R3Real blockHeight = 0.0;
    const int layers[] = {0, 13, 17, 21, 41, 83};

    for (int i = 5; i >= 1; i--) {
        const int numx = i * 2;
        const int numy = layers[i];
        const int numz = numx * 3 + 1;
        const R3Real blockWidth = numx * halfExtents.z * 2.0;
        buildBlock(testbed, world, halfExtents,
                   r3Vector(-blockWidth / 2.0, blockHeight, -blockWidth / 2.0), numx, numy, numz);
        blockHeight += numy * halfExtents.y * 2.0 + halfExtents.x * 2.0;
    }

    /* Set up the viewer. */
    tbCamera(testbed, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
