/* Port of examples3d/soft_surface3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftSurface3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(40, 1, 40));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &boxCollider);

    uint32_t pins[76];
    size_t np = 0;
    for (uint32_t i = 0; i < 400; i++) {
        if (i / 20 == 0 || i / 20 == 19 || i % 20 == 0 || i % 20 == 19) {
            pins[np++] = i;
        }
    }
    R3SoftBodyDesc softBody =
        r3ClothSoftBodyDesc(r3Vector(-2, 3, -2), r3Vector(0.2, 0, 0), r3Vector(0, 0, 0.2), 20, 20);
    r3SoftBodyDesc_SetPinnedParticles(&softBody, (R3IndexView){(const uint32_t *)pins, np});
    softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    softBody.particleMass = 0.05;
    softBody.particleRadius = (R3OptionalReal){1, 0.05};
    R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.05);
    surfaceCollider.friction = 0.6;
    softBody.collider = surfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int h = 0; h < 3; h++) {
                R3ColliderDesc collider = r3BallColliderDesc(0.08);
                collider.density = 2;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(-1.2 + i * 0.5 + h % 2 * 0.2, 5 + h * 0.6,
                                                          -1.2 + j * 0.5 + h % 2 * 0.2);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    for (int i = 0; i < 5; i++) {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-3 + i * 1.5, 0.5, 6);
        R3ColliderDesc collider = r3CapsuleYColliderDesc(0.5, 0.08);
        rigidBody.canSleep = !testbed->noSleep;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3RigidBodyDesc bar = r3FixedRigidBodyDesc();
    bar.position.translation = r3Vector(0, 1, 8);
    bar.position = r3Pose(r3Vector(0, 1, 8), r3RotationFromAxisAngle(r3Vector(1, 0, 0), R3_PI / 4));
    R3ColliderDesc barCollider = r3CuboidColliderDesc(r3Vector(3.5, 0.4, 0.4));
    bar.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &bar);
    r3InsertCollider(rigidBodyHandle, &barCollider);

    softBody =
        r3ClothSoftBodyDesc(r3Vector(-4, 3, 5), r3Vector(0.2, 0, 0), r3Vector(0, 0, 0.2), 40, 25);
    softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){30, 1};
    material.volumeSoftness = (R3SpringCoefficients){30, 1};
    material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    softBody.material = material;
    softBody.particleMass = 0.02;
    softBody.particleRadius = (R3OptionalReal){1, 0.04};
    R3ColliderDesc drapedClothCollider = r3BallColliderDesc(0.04);
    drapedClothCollider.friction = 0.8;
    softBody.collider = drapedClothCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
    for (int i = 0; i < 3; i++) {
        softBody = r3CuboidSoftBodyDesc(r3Vector(9, 0.8 + i * 1.7, 6), r3Vector(0.75, 0.75, 0.75),
                                        4, 4, 4);
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        softBody.cellModel = R3_SOFT_CELL_COROTATIONAL;
        material.youngModulus = 6.0e3;
        material.poissonRatio = 0.4;
        material.elasticDampingRatio = 0.5;
        softBody.material = material;
        softBody.particleMass = 0.1;
        softBody.particleRadius = (R3OptionalReal){1, 0.05};
        R3ColliderDesc jellySurfaceCollider = r3BallColliderDesc(0.05);
        jellySurfaceCollider.friction = 0.8;
        softBody.collider = jellySurfaceCollider;

        if (testbed->noSleep) {
            softBody.canSleep = 0;
        }
        r3InsertSoftBody(world, &softBody);
    }
    R3ColliderDesc colliderC = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
    colliderC.density = 2;
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(9, 6.5, 6);
    dynamicBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(rigidBodyHandle, &colliderC);

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(-9, 0.75, 6);
    R3ColliderDesc obstacleCollider = r3CuboidColliderDesc(r3Vector(0.4, 0.75, 0.4));
    groundBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(rigidBodyHandle, &obstacleCollider);

    softBody =
        r3ClothSoftBodyDesc(r3Vector(-13, 4, 5.6), r3Vector(0.1, 0, 0), r3Vector(0, 0, 0.1), 80, 8);
    softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    R3SoftBodyMaterial materialValue = r3DefaultSoftBodyMaterial();
    materialValue.edgeSoftness = (R3SpringCoefficients){30, 1};
    materialValue.volumeSoftness = (R3SpringCoefficients){30, 1};
    materialValue.shapeMatchingSoftness = (R3SpringCoefficients){30, 1};
    materialValue.bendSoftness = (R3SpringCoefficients){2, 1};
    softBody.material = materialValue;
    softBody.selfContacts = 1;
    softBody.particleMass = 0.02;
    softBody.particleRadius = (R3OptionalReal){1, 0.05};
    R3ColliderDesc stripSurfaceCollider = r3BallColliderDesc(0.05);
    stripSurfaceCollider.friction = 0.6;
    softBody.collider = stripSurfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
    /* Set up the viewer. */
    tbCamera(testbed, 0, 12, 24, 0, 1.5, 3);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
