/* Port of examples3d/debug_triangle3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugTriangle3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3SharedShape *shape =
        r3TriangleSharedShape(r3Vector(-10, 0, -10), r3Vector(10, 0, -10), r3Vector(0, 0, 10));
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc collider = r3DefaultColliderDesc();
    collider.shape.kind = R3_SHAPE_DESC_SHARED;
    collider.shape.sharedShape = shape;
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &collider);

    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(1.1, 0.01, 0);
    rigidBody.canSleep = 0;
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(20, 0.1, 1));
    if (testbed->noSleep) {
        rigidBody.canSleep = 0;
        rigidBody.sleeping = 0;
    }
    groundBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);

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
