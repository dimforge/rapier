/* Port of examples2d/stress_tests/soft_slab2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftSlab2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(14, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 14, 30);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 30));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 12; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-11 + i * 2, 0.5);
        R2ColliderDesc collider;
        if (i % 2) {
            collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
        } else {
            collider = r2BallColliderDesc(0.5);
        }
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyDesc softBody = r2GridSoftBodyDesc(r2Vector(0, 3), r2Vector(12, 1.5), 161, 21);
    R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
    softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
    material.youngModulus = 4.0e4;
    material.poissonRatio = 0.4;
    material.elasticDampingRatio = 0.5;
    softBody.material = material;
    softBody.particleMass = 0.2;
    R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.1);
    surfaceCollider.friction = 0.7;
    softBody.collider = surfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r2InsertSoftBody(world, &softBody);
    for (int j = 0; j < 40; j++) {
        for (int i = 0; i < 24; i++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-11.5 + i + j % 2 * 0.5, 7.5 + j * 1.2);
            R2ColliderDesc collider;
            if ((i + j) % 2) {
                collider = r2CuboidColliderDesc(r2Vector(0.3, 0.3));
            } else {
                collider = r2BallColliderDesc(0.3);
            }
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
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
