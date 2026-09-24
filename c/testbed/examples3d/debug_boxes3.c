/* Port of examples3d/debug_boxes3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugBoxes3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 6; i++) {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(100.1, 0.1, 100.1));
        rigidBody.canSleep = !testbed->noSleep;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 2; i++) {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(1.1, 0, 0);
        rigidBody.canSleep = 0;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, 0.1, 1));
        if (testbed->noSleep) {
            rigidBody.canSleep = 0;
            rigidBody.sleeping = 0;
        }
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
