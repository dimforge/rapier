/* Port of examples3d/debug_cylinder3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugCylinder3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(100.1, 0.1, 100.1));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(-1.5, 4.5, -1.5);
    R3ColliderDesc objectCollider = r3CylinderColliderDesc(1, 1);
    dynamicBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(rigidBodyHandle, &objectCollider);

    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
