/* Port of examples3d/stress_tests/heightfield3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsHeightfield3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3Real heights[441];
    for (int j = 0; j <= 20; j++) {
        for (int i = 0; i <= 20; i++) {
            heights[i + j * 21] =
                i == 0 || i == 20 || j == 0 || j == 20 ? 10 : (R3Real)(sin(i * 10) + cos(j * 10));
        }
    }
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc groundCollider = r3DefaultColliderDesc();
    groundCollider.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    groundCollider.shape.heights = (R3RealView){heights, (21) * (21)};
    groundCollider.shape.rows = 21;
    groundCollider.shape.columns = 21;
    groundCollider.shape.scale = r3Vector(200, 1, 200);
    groundCollider.shape.flags = 0;
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &groundCollider);

    for (int j = 0; j < 47; j++) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                R3ColliderDesc collider;
                if (j % 2) {
                    collider = r3BallColliderDesc(1);
                } else {
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                }
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 3 - 12, j * 3 + 4.5, k * 3 - 12);
                rigidBody.canSleep = !testbed->noSleep;
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
