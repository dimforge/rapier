/* Port of examples3d/debug_balls3.rs. */
#include "testbed.h"
#include "rapier_math.h"

void tbDebugBalls3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 10; k++) {
                int fixed = j == 0 || i == 0 || k == 0 || i == 9 || k == 9;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
                rigidBody.position.translation = r3Vector(i - 5, j * (fixed ? 1 : 2) + 0.5, k - 5);
                rigidBody.canSleep = 0;
                R3ColliderDesc collider = r3BallColliderDesc(0.5);
                collider.friction = 0;
                if (testbed->noSleep) {
                    rigidBody.canSleep = 0;
                    rigidBody.sleeping = 0;
                }
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
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
