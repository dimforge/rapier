/* Port of examples3d/stress_tests/stacks3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsStacks3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(200, 0.1, 200));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &collider);
    for (int p = 0; p < 4; p++) {
        for (int i = 0; i < 12; i++) {
            for (int j = i; j < 12; j++) {
                for (int k = i; k < 12; k++) {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(i + (k - i) * 2 - 110 + p * 30 - 12,
                                                              i * 2 + 50, i + (j - i) * 2 - 12);
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    rigidBody.canSleep = !testbed->noSleep;
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    for (int w = 0; w < 3; w++) {
        for (int i = 0; i < 12; i++) {
            for (int j = i; j < 12; j++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(-2 + w * 6, i * 2 + 50, i + (j - i) * 2 - 12);
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    R3Real radius = 1.3 * 24 / R3_PI;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 24; j++) {
            R3Real angle = (i / 2.0 + j) * R3_PI * 2 / 24;
            R3Vector position = r3Vector(25 + sin(angle) * radius, 50 + i * 2, cos(angle) * radius);
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = position;
            rigidBody.position =
                r3Pose(position, r3RotationFromAxisAngle(r3Vector(0, 1, 0), angle));
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
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
