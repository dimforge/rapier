/* Port of examples3d/stress_tests/many_sleep3.rs. */
#include "testbed.h"
#include "rapier_math.h"

void tbStressTestsManySleep3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 50; i++) {
        for (int j = 0; j < 50; j++) {
            for (int k = 0; k < 50; k++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.bodyType = j ? R3_DYNAMIC : R3_FIXED;
                rigidBody.position.translation = r3Vector(i * 3 - 75.0, j * 3 + 1.5, k * 3 - 75.0);
                rigidBody.sleeping = 1;
                R3ColliderDesc collider = r3BallColliderDesc(1);
                collider.density = 0.477;
                if (testbed->noSleep) {
                    rigidBody.canSleep = 0;
                    rigidBody.sleeping = 0;
                }
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
