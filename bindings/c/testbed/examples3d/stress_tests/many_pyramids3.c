/* Port of examples3d/stress_tests/many_pyramids3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsManyPyramids3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(50, 0.1, 130));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    for (int p = 0; p < 40; p++) {
        for (int i = 0; i < 20; i++) {
            for (int j = i; j < 20; j++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 0.5 + j - i, i + 0.5, (p - 20) * 4);
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
                rigidBody.canSleep = !testbed->noSleep;
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
