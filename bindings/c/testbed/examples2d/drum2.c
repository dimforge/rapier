/* Port of examples2d/drum2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDrum2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int i = 0; i < 30; i++) {
        for (int j = 0; j < 30; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 0.4 - 6, j * 0.4 - 6);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R2RigidBodyHandle drum = {0};
    R2RigidBodyDesc platformBody = r2KinematicVelocityBasedRigidBodyDesc();
    platformBody.position.translation = r2Vector(0, 0);
    platformBody.canSleep = !testbed->noSleep;
    drum = r2InsertRigidBody(world, &platformBody);

    for (int side = -1; side <= 1; side += 2) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(10, 0.25));
        collider.position.translation = r2Vector(0, side * 10);
        r2InsertCollider(drum, &collider);

        collider = r2CuboidColliderDesc(r2Vector(0.25, 10));
        collider.position.translation = r2Vector(side * 10, 0);
        r2InsertCollider(drum, &collider);
        for (int x = -1; x <= 1; x += 2) {
            collider = r2BallColliderDesc(1.25);
            collider.position.translation = r2Vector(x * 6, side * 6);
            r2InsertCollider(drum, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 1, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);

            r2RigidBody_SetAngvel(drum, -0.15, 1);
        }
    }
    r2FreeWorld(world);
}
