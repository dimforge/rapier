/* Port of examples3d/debug_add_remove_collider3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugAddRemoveCollider3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle groundHandle = {0};
    R3ColliderHandle groundCollider = {0};
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    rigidBody.canSleep = !testbed->noSleep;
    groundHandle = r3InsertRigidBody(world, &rigidBody);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(3, 0.1, 0.4));
    groundCollider = r3InsertCollider(groundHandle, &boxCollider);
    R3ColliderDesc collider = r3BallColliderDesc(0.1);
    collider.density = 100;
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(0, 0.2, 0);
    dynamicBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle dynamicBodyHandle = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(dynamicBodyHandle, &collider);
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            r3RemoveCollider(groundCollider, 1);
            groundCollider = r3InsertCollider(groundHandle, &boxCollider);
        }
    }
    r3FreeWorld(world);
}
