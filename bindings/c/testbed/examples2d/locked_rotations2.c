/* Port of examples2d/locked_rotations2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbLockedRotations2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.1);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(5, 0.1));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    R2RigidBodyDesc rectangle = r2DynamicRigidBodyDesc();
    rectangle.position.translation = r2Vector(0, 3);
    rectangle.lockedAxes = R2_LOCK_TRANSLATION_X | R2_LOCK_TRANSLATION_Y;
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(2, 0.6));
    rectangle.canSleep = !testbed->noSleep;
    rigidBodyHandle = r2InsertRigidBody(world, &rectangle);
    r2InsertCollider(rigidBodyHandle, &boxCollider);

    R2RigidBodyDesc capsule = r2DynamicRigidBodyDesc();
    capsule.position.translation = r2Vector(0, 5);
    capsule.position = r2Pose(r2Vector(0, 5), r2Rotation(1));
    capsule.lockedAxes = R2_LOCK_ROTATION_Z;
    R2ColliderDesc capsuleCollider = r2CapsuleYColliderDesc(0.6, 0.4);
    capsule.canSleep = !testbed->noSleep;
    rigidBodyHandle = r2InsertRigidBody(world, &capsule);
    r2InsertCollider(rigidBodyHandle, &capsuleCollider);

    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
