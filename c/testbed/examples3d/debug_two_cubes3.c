/* Port of examples3d/debug_two_cubes3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugTwoCubes3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 2, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
    groundBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(rigidBodyHandle, &boxCollider);
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
