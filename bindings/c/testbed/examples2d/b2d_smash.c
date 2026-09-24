/* Port of examples2d/b2d_smash.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dSmash(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, 0));
    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
    rigidBody.position.translation = r2Vector(-20, 0);
    rigidBody.linvel = r2Vector(40, 0);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(4, 4));
    collider.density = 8;
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
    for (int i = 0; i < 120; i++) {
        for (int j = 0; j < 80; j++) {
            rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 0.4 + 30, (j - 40) * 0.4);
            rigidBody.sleeping = 1;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 20, 0, 8);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
