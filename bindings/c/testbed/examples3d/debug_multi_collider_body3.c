/* Port of examples3d/debug_multi_collider_body3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugMultiColliderBody3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -9.81, 0));
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(100, 0.5, 100));
    r3InsertColliderWithoutParent(world, &boxCollider);
    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 40, 0);
    rigidBody.position =
        r3Pose(r3Vector(0, 40, 0), r3RotationFromAxisAngle(r3Vector(1, 2, 3), (R3Real)sqrt(14)));
    R3RigidBodyHandle handle;
    rigidBody.canSleep = !testbed->noSleep;
    handle = r3InsertRigidBody(world, &rigidBody);
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 20; j++) {
            for (int k = 0; k < 20; k++) {
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
                collider.position.translation = r3Vector(i - 10, j - 10, k - 10);
                r3InsertCollider(handle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 40, 30, 40, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
