/* Port of examples3d/platform3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbPlatform3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(10, 0.1, 10));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 0.4 - 1.2, j * 0.4 + (j >= 3 ? 5 : 3), k * 0.4 - 1.2);
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    R3RigidBodyHandle velocityBasedPlatformHandle = {0};
    R3RigidBodyHandle positionBasedPlatformHandle = {0};
    R3RigidBodyDesc platformBody = r3KinematicVelocityBasedRigidBodyDesc();
    platformBody.position.translation = r3Vector(0, 2.3, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(2, 0.2, 2));
    platformBody.canSleep = !testbed->noSleep;
    velocityBasedPlatformHandle = r3InsertRigidBody(world, &platformBody);
    r3InsertCollider(velocityBasedPlatformHandle, &boxCollider);
    R3RigidBodyDesc positionBasedPlatform = r3KinematicPositionBasedRigidBodyDesc();
    positionBasedPlatform.position.translation = r3Vector(0, 5.3, 0);
    R3ColliderDesc positionBasedCollider = r3CuboidColliderDesc(r3Vector(2, 0.2, 2));
    positionBasedPlatform.canSleep = !testbed->noSleep;
    positionBasedPlatformHandle = r3InsertRigidBody(world, &positionBasedPlatform);
    r3InsertCollider(positionBasedPlatformHandle, &positionBasedCollider);
    /* Set up the viewer. */
    tbCamera(testbed, 10, 5, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            R3Real dt = r3TimeStep(world);
            R3Real time = (R3Real)(testbed->step + 1) * dt;
            R3Vector velocity = r3Vector(0, cos(time * 2), sin(time) * 2);

            r3RigidBody_SetLinvel(velocityBasedPlatformHandle, velocity, 1);
            r3RigidBody_SetAngvel(velocityBasedPlatformHandle, r3Vector(0, 1, 0), 1);

            R3Vector position = r3RigidBody_Translation(positionBasedPlatformHandle);
            r3RigidBody_SetNextKinematicTranslation(
                positionBasedPlatformHandle,
                r3VectorAdd(position, r3VectorScale(velocity, -dt)));
            R3Rotation rotation = r3RigidBody_Rotation(positionBasedPlatformHandle);
            r3RigidBody_SetNextKinematicRotation(
                positionBasedPlatformHandle,
                r3RotationMul(r3RotationFromAxisAngle(r3Vector(0, 1, 0), -0.5 * dt), rotation));
        }
    }
    r3FreeWorld(world);
}
