/* Port of examples3d/locked_rotations3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbLockedRotations3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(5, 0.1, 5));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3RigidBodyDesc rectangle = r3DynamicRigidBodyDesc();
    rectangle.position.translation = r3Vector(0, 3, 0);
    rectangle.lockedAxes = R3_LOCK_TRANSLATION_X | R3_LOCK_TRANSLATION_Y | R3_LOCK_TRANSLATION_Z |
                           R3_LOCK_ROTATION_Y | R3_LOCK_ROTATION_Z;
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(0.2, 0.6, 2));
    rectangle.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &rectangle);
    r3InsertCollider(rigidBodyHandle, &boxCollider);

    R3RigidBodyDesc capsule = r3DynamicRigidBodyDesc();
    capsule.position.translation = r3Vector(0, 5, 0);
    capsule.position = r3Pose(r3Vector(0, 5, 0), r3RotationFromAxisAngle(r3Vector(1, 0, 0), 1));
    capsule.lockedAxes = R3_LOCK_ROTATION_X | R3_LOCK_ROTATION_Y | R3_LOCK_ROTATION_Z;
    R3RigidBodyHandle handle;
    R3ColliderDesc capsuleCollider = r3CapsuleYColliderDesc(0.6, 0.4);
    capsule.canSleep = !testbed->noSleep;
    handle = r3InsertRigidBody(world, &capsule);
    r3InsertCollider(handle, &capsuleCollider);

    R3SharedShape *shape = r3CapsuleSharedShape(r3Vector(-0.6, 0, 0), r3Vector(0.6, 0, 0), 0.4);
    R3ColliderDesc shapeCollider = r3DefaultColliderDesc();
    shapeCollider.shape.kind = R3_SHAPE_DESC_SHARED;
    shapeCollider.shape.sharedShape = shape;
    r3InsertCollider(handle, &shapeCollider);

    /* Set up the viewer. */
    tbCamera(testbed, 10, 3, 0, 0, 3, 0);
    r3FreeSharedShape(shape);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
