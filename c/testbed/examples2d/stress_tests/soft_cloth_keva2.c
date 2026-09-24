/* Port of examples2d/stress_tests/soft_cloth_keva2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftClothKeva2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(40, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    R2Real y = 0;
    for (int pair = 0; pair < 18; pair++) {
        int n = 20 - pair;
        for (int k = 0; k <= n; k++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-n + k * 2, y + 1);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 1));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
        for (int k = 0; k < n; k++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-n + k * 2 + 1, y + 2.1);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 0.1));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
        y += 2.2;
    }
    R2SoftBodyDesc softBody = r2GridSoftBodyDesc(r2Vector(0, y + 6), r2Vector(24, 0.1), 241, 2);
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
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 20, 12);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
