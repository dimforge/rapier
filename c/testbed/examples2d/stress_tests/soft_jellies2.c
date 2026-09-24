/* Port of examples2d/stress_tests/soft_jellies2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftJellies2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(16, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 16, 30);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 30));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    int k = 0;
    for (int layer = 0; layer < 40; layer++) {
        for (int i = 0; i < 12; i++, k++) {
            R2SoftBodyDesc softBody =
                r2GridSoftBodyDesc(r2Vector(-14 + i * 2.5 + layer % 2 * 0.8, 2 + layer * 2.4),
                                   r2Vector(0.6, 0.6), 5, 5);
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            softBody.cellModel = k % 3 ? R2_SOFT_CELL_COROTATIONAL : R2_SOFT_CELL_NEO_HOOKEAN;
            material.youngModulus = 2.0e3 * (1 + k % 5 * 3);
            material.poissonRatio = 0.4;
            material.elasticDampingRatio = 0.5;
            softBody.material = material;
            softBody.particleMass = 0.1;
            softBody.particleRadius = (R2OptionalReal){1, 0.08};
            R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.08);
            surfaceCollider.friction = 0.7;
            softBody.collider = surfaceCollider;

            if (testbed->noSleep) {
                softBody.canSleep = 0;
            }
            r2InsertSoftBody(world, &softBody);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 8, 25);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
