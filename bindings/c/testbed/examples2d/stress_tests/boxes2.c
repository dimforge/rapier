/* Port of examples2d/stress_tests/boxes2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsBoxes2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(25, 1.2));
    groundBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle groundBodyHandle = r2InsertRigidBody(world, &groundBody);
    r2InsertCollider(groundBodyHandle, &collider);
    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 25, 50);
        rigidBody.position = r2Pose(r2Vector(side * 25, 50), r2Rotation(R2_PI / 2));
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(50, 1.2));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 26; i++) {
        for (int j = 0; j < 130; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i - 13, j * 1 + 2.5);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 50, 10);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
