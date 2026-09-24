/* Port of examples2d/stress_tests/large_pyramids2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsLargePyramids2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, -1);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(520, 1));
    groundBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle groundBodyHandle = r2InsertRigidBody(world, &groundBody);
    r2InsertCollider(groundBodyHandle, &collider);
    for (int p = 0; p < 8; p++) {
        for (int i = 0; i < 55; i++) {
            for (int j = i; j < 55; j++) {
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r2Vector(p * 65 - 260 + i * 0.5 + j - i, i * 1.001 + 0.5);
                rigidBody.canSleep = 0;
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
                if (testbed->noSleep) {
                    rigidBody.canSleep = 0;
                    rigidBody.sleeping = 0;
                }
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 27.5, 3);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
