/* Port of examples2d/debug_total_overlap2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugTotalOverlap2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int i = 0; i < 100; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 0);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 50);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
