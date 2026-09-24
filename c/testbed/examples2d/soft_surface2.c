/* Port of examples2d/soft_surface2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftSurface2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(22, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &boxCollider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 22, 2);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 3));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyDesc hammock = r2GridSoftBodyDesc(r2Vector(-9, 5), r2Vector(3, 0.15), 21, 2);
    hammock.cellModel = R2_SOFT_CELL_VOLUME;
    uint32_t hp[] = {0, 1, 40, 41};
    r2SoftBodyDesc_SetPinnedParticles(&hammock, (R2IndexView){(const uint32_t *)hp, 4});
    hammock.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    hammock.particleMass = 0.05;
    hammock.particleRadius = (R2OptionalReal){1, 0.06};
    R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.06);
    surfaceCollider.friction = 0.6;
    hammock.collider = surfaceCollider;

    if (testbed->noSleep) {
        hammock.canSleep = 0;
    }
    r2InsertSoftBody(world, &hammock);
    for (int i = 0; i < 12; i++) {
        for (int h = 0; h < 4; h++) {
            R2ColliderDesc collider = r2BallColliderDesc(0.08);
            collider.density = 2;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-11.2 + i * 0.4 + h % 2 * 0.15, 7 + h * 0.5);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R2SoftBodyDesc bridge = r2GridSoftBodyDesc(r2Vector(2, 3), r2Vector(3, 0.2), 31, 3);
    bridge.cellModel = R2_SOFT_CELL_VOLUME;
    uint32_t bp[] = {0, 1, 2, 90, 91, 92};
    r2SoftBodyDesc_SetPinnedParticles(&bridge, (R2IndexView){(const uint32_t *)bp, 6});
    bridge.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    bridge.particleMass = 0.05;
    bridge.particleRadius = (R2OptionalReal){1, 0.05};
    R2ColliderDesc clothSurfaceCollider = r2BallColliderDesc(0.05);
    clothSurfaceCollider.friction = 0.8;
    bridge.collider = clothSurfaceCollider;

    if (testbed->noSleep) {
        bridge.canSleep = 0;
    }
    r2InsertSoftBody(world, &bridge);
    for (int i = 0; i < 8; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.03, 0.4));
        collider.density = 3;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-0.4 + i * 0.65, 4.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 3; i++) {
        R2SoftBodyDesc softBody =
            r2GridSoftBodyDesc(r2Vector(8, 0.75 + i * 1.6), r2Vector(0.7, 0.7), 5, 5);
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
        material.youngModulus = 1.0e4;
        material.poissonRatio = 0.4;
        material.elasticDampingRatio = 0.5;
        softBody.material = material;
        softBody.particleMass = 0.1;
        softBody.particleRadius = (R2OptionalReal){1, 0.05};
        R2ColliderDesc jellySurfaceCollider = r2BallColliderDesc(0.05);
        jellySurfaceCollider.friction = 0.8;
        softBody.collider = jellySurfaceCollider;

        if (testbed->noSleep) {
            softBody.canSleep = 0;
        }
        r2InsertSoftBody(world, &softBody);
    }
    R2SoftBodyDesc blob = r2DiskSoftBodyDesc(r2Vector(8, 5), 0.8, 24);
    blob.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20, 1});
    blob.volumeFactor = 1.1;
    blob.particleMass = 0.05;
    blob.particleRadius = (R2OptionalReal){1, 0.05};
    R2ColliderDesc blobSurfaceCollider = r2BallColliderDesc(0.05);
    blobSurfaceCollider.friction = 0.6;
    blob.collider = blobSurfaceCollider;

    if (testbed->noSleep) {
        blob.canSleep = 0;
    }
    r2InsertSoftBody(world, &blob);
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(14, 1);
    R2ColliderDesc supportCollider = r2CuboidColliderDesc(r2Vector(0.15, 1));
    groundBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r2InsertRigidBody(world, &groundBody);
    r2InsertCollider(rigidBodyHandle, &supportCollider);

    R2SoftBodyDesc strip = r2GridSoftBodyDesc(r2Vector(14, 9), r2Vector(4, 0.1), 60, 2);
    strip.cellModel = R2_SOFT_CELL_VOLUME;
    strip.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    strip.selfContacts = 1;
    strip.particleMass = 0.02;
    strip.particleRadius = (R2OptionalReal){1, 0.05};
    R2ColliderDesc stripSurfaceCollider = r2BallColliderDesc(0.05);
    stripSurfaceCollider.friction = 0.6;
    strip.collider = stripSurfaceCollider;

    if (testbed->noSleep) {
        strip.canSleep = 0;
    }
    r2InsertSoftBody(world, &strip);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 4, 18);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
