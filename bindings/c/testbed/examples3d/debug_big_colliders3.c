/* Port of examples3d/debug_big_colliders3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugBigColliders3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3SharedShape *shape = r3HalfspaceSharedShape(r3Vector(0, 1, 0));
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc collider = r3DefaultColliderDesc();
    collider.shape.kind = R3_SHAPE_DESC_SHARED;
    collider.shape.sharedShape = shape;
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3Real y = 0;
    R3Real width = 10000;
    for (int i = 0; i < 12; i++) {
        R3Real height = (R3Real)fmin(0.1, width);
        y += height * 4;
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, y, 0);
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(width, height, width));
        rigidBody.canSleep = !testbed->noSleep;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);

        width /= 5;
    }
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);
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
