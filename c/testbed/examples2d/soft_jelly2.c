/* Port of examples2d/soft_jelly2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftJelly2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(20, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
    for (int level = 0; level < 4; level++) {
        for (int i = 0; i < 4 - level; i++) {
            R2SoftBodyDesc softBody = r2GridSoftBodyDesc(
                r2Vector(-8 + (i - (4 - level) * 0.5 + 0.5) * 1.6, 0.75 + level * 1.5),
                r2Vector(0.75, 0.75), 5, 5);
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
            material.youngModulus = 2.0e4 / (1 + level * 1.5);
            material.poissonRatio = 0.4;
            material.elasticDampingRatio = 0.5;
            softBody.material = material;
            softBody.particleMass = 0.1;
            if (testbed->noSleep) {
                softBody.canSleep = 0;
            }
            r2InsertSoftBody(world, &softBody);
        }
    }
    R2SoftBodyDesc bridge = r2GridSoftBodyDesc(r2Vector(3, 3), r2Vector(4, 0.2), 41, 3);
    bridge.cellModel = R2_SOFT_CELL_VOLUME;
    bridge.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    uint32_t pins[] = {0, 1, 2, 120, 121, 122};
    r2SoftBodyDesc_SetPinnedParticles(&bridge, (R2IndexView){(const uint32_t *)pins, 6});
    bridge.particleMass = 0.05;
    if (testbed->noSleep) {
        bridge.canSleep = 0;
    }
    r2InsertSoftBody(world, &bridge);
    for (int i = 0; i < 6; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0.5 + i, 5 + i);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.25, 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyDesc driven = r2DiskSoftBodyDesc(r2Vector(8, 4), 0.8, 24);
    driven.shapeMatching = (R2OptionalBool){1, 1};
    driven.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){15, 1});
    driven.volumePreservation = 0;
    driven.gravityScale = 0;
    driven.particleMass = 0.2;
    driven.canSleep = 0;
    R2SoftBodyHandle softBodyHandle = {0};
    if (testbed->noSleep) {
        driven.canSleep = 0;
    }
    softBodyHandle = r2InsertSoftBody(world, &driven);
    for (int i = 0; i < 8; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(6.5 + 0.5 * i, 0.25);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 3, 30);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            R2Real time = (R2Real)testbed->time + dt;
            R2Pose target = r2Pose(r2Vector(8 + 2.5 * cos(time), 1 + 1.5 * fabs(sin(2 * time))),
                                   r2Rotation(time));

            r2SoftBody_SetClusterShapeMatchingTarget(softBodyHandle, 0, &target);

            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
