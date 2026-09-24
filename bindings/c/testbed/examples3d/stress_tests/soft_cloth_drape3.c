/* Port of examples3d/stress_tests/soft_cloth_drape3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftClothDrape3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(12, 0.1, 12));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &boxCollider);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int shape = (i + j) % 3;
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(-4.5 + i * 3, 1, -4.5 + j * 3);
            R3ColliderDesc collider;
            if (shape == 0) {
                collider = r3BallColliderDesc(0.8);
            } else {
                if (shape == 1) {
                    collider = r3CuboidColliderDesc(r3Vector(0.6, 1, 0.6));
                } else {
                    collider = r3CapsuleYColliderDesc(0.6, 0.4);
                }
            }
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R3SoftBodyDesc softBody = r3ClothSoftBodyDesc(r3Vector(-6, 3.5, -6), r3Vector(0.1, 0, 0),
                                                  r3Vector(0, 0, 0.1), 101, 101);
    softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){30, 1};
    material.volumeSoftness = (R3SpringCoefficients){30, 1};
    material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    softBody.material = material;
    softBody.selfContacts = 1;
    softBody.particleMass = 0.02;
    softBody.particleRadius = (R3OptionalReal){1, 0.05};
    R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.05);
    surfaceCollider.friction = 0.6;
    softBody.collider = surfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            R3ColliderDesc collider = r3BallColliderDesc(0.4);
            collider.density = 2;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r3Vector(-4 + i * 2 + j % 2 * 0.5, 6 + (i + j) * 0.5, -4 + j * 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 14, 10, 14, 0, 1, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
