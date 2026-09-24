/* Port of examples3d/b3d_many_pyramids.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB3dManyPyramids(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -10, 0));
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(77, 1, 77));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);
    for (int row = 0; row < 14; row++) {
        for (int col = 0; col < 14; col++) {
            for (int i = 0; i < 10; i++) {
                for (int j = i; j < 10; j++) {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r3Vector((i + 1) * 0.5 + j - i - 77 + col * 11 + 1 - 0.5, (2 * i + 1) * 0.5,
                                 -76 + row * (152.0 / 13));
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
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 0, 30, 120, 0, 5, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
