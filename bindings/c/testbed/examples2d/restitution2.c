/* Port of examples2d/restitution2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbRestitution2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2ColliderDesc floor = r2CuboidColliderDesc(r2Vector(20, 1));
    floor.restitution = 1;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -1);
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);
    const int num = 10;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i <= num; i++) {
            R2ColliderDesc collider = r2BallColliderDesc(0.5);
            collider.restitution = (R2Real)i / num;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector((i - num / 2.0) * 2, 10 * (j + 1));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 1, 25);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
