/* Port of examples2d/stress_tests/balls2.rs. */
#include "testbed.h"
#include "rapier_math.h"

void tbStressTestsBalls2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int i = 0; i < 50; i++) {
        for (int j = 0; j < 250; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.bodyType = j ? R2_DYNAMIC : R2_FIXED;
            rigidBody.position.translation = r2Vector(i * 2.5 - 62.5, j * 2 + 1);
            R2ColliderDesc collider = r2BallColliderDesc(1);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 5);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
