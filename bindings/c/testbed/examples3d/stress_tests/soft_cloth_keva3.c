/* Port of examples3d/stress_tests/soft_cloth_keva3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

extern void buildBlock(Testbed *, R3World *, R3Vector, R3Vector, int, int, int);

static void cloth(Testbed *testbed, R3World *world, R3Real side, R3Real spacing, R3Real y) {
    size_t n = (size_t)round(side / spacing) + 1;
    R3Real width = (n - 1) * spacing;
    R3SoftBodyDesc softBody =
        r3ClothSoftBodyDesc(r3Vector(-width * 0.5, y, -width * 0.5), r3Vector(spacing, 0, 0),
                            r3Vector(0, 0, spacing), n, n);
    softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){30, 1};
    material.volumeSoftness = (R3SpringCoefficients){30, 1};
    material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    softBody.material = material;
    softBody.selfContacts = 1;
    softBody.particleMass = 0.05;
    softBody.particleRadius = (R3OptionalReal){1, 0.15};
    R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.15);
    surfaceCollider.friction = 0.6;
    softBody.collider = surfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
}

void tbStressTestsSoftClothKeva3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(50, 0.1, 50));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3Real height = 0;
    const int layers[] = {0, 9, 13, 17};
    for (int i = 3; i >= 1; i--) {
        R3Real width = i * 4;
        buildBlock(testbed, world, r3Vector(0.1, 0.5, 2), r3Vector(-width / 2, height, -width / 2),
                   i, layers[i], i * 3 + 1);
        height += layers[i] + 0.2;
        if (i > 1) {
            cloth(testbed, world, width + 4, 0.5, height + 0.15);
            height += 0.3;
        }
    }
    cloth(testbed, world, 18, 0.75, height + 6);
    /* Set up the viewer. */
    tbCamera(testbed, 50, 50, 50, 0, 15, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
