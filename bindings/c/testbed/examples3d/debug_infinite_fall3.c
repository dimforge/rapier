/* Port of examples3d/debug_infinite_fall3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugInfiniteFall3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 4, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(100.1, 2.1, 100.1));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &collider);
    for (int i = 0; i < 2; i++) {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, i ? 2 : 7, 0);
        rigidBody.canSleep = 0;
        R3ColliderDesc collider = r3BallColliderDesc(1);
        if (testbed->noSleep) {
            rigidBody.canSleep = 0;
            rigidBody.sleeping = 0;
        }
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, -10, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
