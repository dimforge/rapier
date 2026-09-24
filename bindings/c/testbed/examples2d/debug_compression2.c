/* Port of examples2d/debug_compression2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugCompression2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, side * 32);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(75, 2));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2RigidBodyHandle handle[2] = {0};
    for (int i = 0; i < 2; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(i ? 73 : -73, 0);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2, 30));
        rigidBody.canSleep = !testbed->noSleep;
        handle[i] = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(handle[i], &collider);
    }
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 7.5 - 30, j * 7.5 - 26.25);
            R2ColliderDesc collider = r2BallColliderDesc(3.75);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 50);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);

            for (int i = 0; i < 2; i++) {
                r2RigidBody_ResetForces(handle[i], 1);
                r2RigidBody_AddForce(handle[i],
                                    r2Vector((i ? -1 : 1) * (R2Real)testbed->step * 10000, 0), 1);
            }
        }
    }
    r2FreeWorld(world);
}
