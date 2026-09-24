/* Port of examples2d/stress_tests/heightfield2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsHeightfield2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2Real heights[2001];
    for (int i = 0; i <= 2000; i++) {
        heights[i] = i == 0 || i == 2000 ? 80 : (R2Real)cos(i * 50.0 / 2000) * 2;
    }
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    R2ColliderDesc collider = r2DefaultColliderDesc();
    collider.shape.kind = R2_SHAPE_DESC_HEIGHTFIELD;
    collider.shape.heights = (R2RealView){heights, (2001) * (1)};
    collider.shape.rows = 2001;
    collider.shape.columns = 1;
    collider.shape.scale = r2Vector(50, 1);
    collider.shape.flags = 0;
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    for (int i = 0; i < 26; i++) {
        for (int j = 0; j < 130; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i - 13, j + 3.5);
            R2ColliderDesc collider;
            if (j % 2) {
                collider = r2BallColliderDesc(0.5);
            } else {
                collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            }
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 50, 10);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
