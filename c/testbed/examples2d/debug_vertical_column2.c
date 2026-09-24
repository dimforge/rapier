/* Port of examples2d/debug_vertical_column2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugVerticalColumn2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2ColliderDesc floor = r2CuboidColliderDesc(r2Vector(1, 1));
    floor.friction = 0.3;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);
    for (int i = 0; i < 80; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
        collider.friction = 0.3;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, i + 1.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 5);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
