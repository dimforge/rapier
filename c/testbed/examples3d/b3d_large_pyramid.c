/* Port of examples3d/b3d_large_pyramid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB3dLargePyramid(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -10, 0));
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(400, 1, 400));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);
    for (int i = 0; i < 200; i++) {
        for (int j = i; j < 200; j++) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r3Vector((i + 1) * 0.5 + j - i - 100, (2 * i + 1) * 0.5, 0);
            rigidBody.canSleep = 0;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
            collider.density = 100;
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 0, 40, 110, 0, 20, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
