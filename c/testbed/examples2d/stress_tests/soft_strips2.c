/* Port of examples2d/stress_tests/soft_strips2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftStrips2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(10, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 10, 30);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 30));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int row = 0; row < 4; row++) {
        for (int i = 0; i < 6; i++) {
            R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-7.5 + i * 3 + row % 2 * 1.5, 4 + row * 3);
            R2ColliderDesc collider = r2BallColliderDesc(0.25);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    for (int layer = 0; layer < 30; layer++) {
        for (int i = 0; i < 5; i++) {
            R2SoftBodyDesc softBody =
                r2GridSoftBodyDesc(r2Vector(-7.8 + i * 3.7 + layer % 2 * 0.7, 17 + layer * 1.5),
                                   r2Vector(1.6, 0.1), 33, 2);
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
            material.youngModulus = 2.0e4;
            material.poissonRatio = 0.4;
            material.elasticDampingRatio = 0.5;
            softBody.material = material;
            softBody.selfContacts = 1;
            softBody.particleMass = 0.05;
            softBody.particleRadius = (R2OptionalReal){1, 0.06};
            R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.06);
            surfaceCollider.friction = 0.6;
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
