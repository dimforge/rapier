/* Port of examples2d/platform2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbPlatform2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.1);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(10, 0.1));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 0.4 - 1.2, j * 0.4 + 3.24);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R2RigidBodyHandle velocityBasedPlatformHandle = {0};
    R2RigidBodyHandle positionBasedPlatformHandle = {0};
    R2RigidBodyDesc platformBody = r2KinematicVelocityBasedRigidBodyDesc();
    platformBody.position.translation = r2Vector(-2, 2.3);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(2, 0.2));
    platformBody.canSleep = !testbed->noSleep;
    velocityBasedPlatformHandle = r2InsertRigidBody(world, &platformBody);
    r2InsertCollider(velocityBasedPlatformHandle, &boxCollider);
    R2RigidBodyDesc positionBasedPlatform = r2KinematicPositionBasedRigidBodyDesc();
    positionBasedPlatform.position.translation = r2Vector(-2, 4.3);
    R2ColliderDesc positionBasedCollider = r2CuboidColliderDesc(r2Vector(2, 0.2));
    positionBasedPlatform.canSleep = !testbed->noSleep;
    positionBasedPlatformHandle = r2InsertRigidBody(world, &positionBasedPlatform);
    r2InsertCollider(positionBasedPlatformHandle, &positionBasedCollider);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 1, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);

            R2Real dt = r2TimeStep(world);
            R2Real time = (R2Real)(testbed->step + 1) * dt;
            R2Vector velocity = r2Vector(sin(time) * 5, sin(time * 5));

            r2RigidBody_SetLinvel(velocityBasedPlatformHandle, velocity, 1);

            R2Vector position = r2RigidBody_Translation(positionBasedPlatformHandle);
            r2RigidBody_SetNextKinematicTranslation(
                positionBasedPlatformHandle,
                r2VectorAdd(position, r2VectorScale(velocity, dt)));
        }
    }
    r2FreeWorld(world);
}
