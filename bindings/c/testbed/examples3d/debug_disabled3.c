/* Port of examples3d/debug_disabled3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugDisabled3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3ColliderHandle platformCollider = {0};
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -2.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(10.1, 2.1, 10.1));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3RigidBodyHandle handle;
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(0, 5, 0);
    dynamicBody.canSleep = !testbed->noSleep;
    handle = r3InsertRigidBody(world, &dynamicBody);

    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(5, 1, 5));
    platformCollider = r3InsertCollider(handle, &boxCollider);

    /* Set up the viewer. */
    tbCamera(testbed, 30, 4, 30, 0, 1, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            uint64_t step = testbed->step + 1;
            if (step % 250 == 0) {
                R3Bool enabled = r3Collider_IsEnabled(platformCollider);
                r3Collider_SetEnabled(platformCollider, !enabled);
            }
            if (step % 25 == 0) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(0, 20, 0);
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    r3FreeWorld(world);
}
