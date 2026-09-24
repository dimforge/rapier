/* Port of examples2d/s2d_high_mass_ratio_3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dHighMassRatio3(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2ColliderDesc floor = r2CuboidColliderDesc(r2Vector(40, 2));
    floor.friction = 0.6;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -2);
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);
    for (int i = 0; i < 3; i++) {
        R2ColliderDesc collider =
            r2CuboidColliderDesc(i == 2 ? r2Vector(10, 10) : r2Vector(0.5, 0.5));
        collider.friction = 0.6;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(i == 2 ? 0 : i ? 9 : -9, i == 2 ? 26 : 0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
