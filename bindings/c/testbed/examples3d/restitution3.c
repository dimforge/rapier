/* Port of examples3d/restitution3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbRestitution3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3ColliderDesc floor = r3CuboidColliderDesc(r3Vector(20, 1, 2));
    floor.restitution = 1;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -1, 0);
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &floor);
    const int num = 10;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i <= num; i++) {
            R3ColliderDesc collider = r3BallColliderDesc(0.5);
            collider.restitution = (R3Real)i / num;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector((i - num / 2.0) * 2, 10 * (j + 1), 0);
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 0, 3, 30, 0, 3, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
