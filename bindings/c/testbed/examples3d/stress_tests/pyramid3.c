/* Port of examples3d/stress_tests/pyramid3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsPyramid3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -9.81, 0));
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(100, 1, 100));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &boxCollider);
    for (int i = 0; i < 50; i++) {
        for (int j = i / 2; j < 50 - (i + 1) / 2; j++) {
            for (int k = i / 2; k < 50 - (i + 1) / 2; k++) {
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.975, 0.975, 0.975));
                collider.density = 1000;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(-50 + 2.25 * j + (i & 1), 1 + 2.5 * i, -50 + 2.25 * k + (i & 1));
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 200, 130, 200, 5, 50, 5);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
