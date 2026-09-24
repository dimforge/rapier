/* Port of examples2d/b2d_large_pyramid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dLargePyramid(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, -1);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(120, 1));
    groundBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle groundBodyHandle = r2InsertRigidBody(world, &groundBody);
    r2InsertCollider(groundBodyHandle, &boxCollider);
    for (int i = 0; i < 200; i++) {
        for (int j = i; j < 200; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r2Vector((i + 1) * 0.5 + j - i - 100, (2 * i + 1) * 0.5);
            rigidBody.canSleep = 0;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            collider.density = 1;
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 50, 2.5);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
