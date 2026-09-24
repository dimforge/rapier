/* Port of examples3d/stress_tests/compound3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsCompound3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(200.1, 0.1, 200.1));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &boxCollider);
    R3Real offset = -2.4;
    for (int j = 0; j < 25; j++, offset -= 0.07) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                R3RigidBodyHandle handle;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 5 - 4 + offset, j * 5 + 3.5, k * 2 - 4 + offset);
                R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(2, 0.2, 0.2));
                rigidBody.canSleep = !testbed->noSleep;
                handle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(handle, &boxCollider);
                for (int side = -1; side <= 1; side += 2) {
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 2, 0.2));
                    collider.position.translation = r3Vector(side * 2, 2, 0);
                    r3InsertCollider(handle, &collider);
                }
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
