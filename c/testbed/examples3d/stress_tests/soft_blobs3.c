/* Port of examples3d/stress_tests/soft_blobs3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftBlobs3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle floor;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.5, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(7.5, 0.5, 7.5));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(floor, &collider);

    tbBodyColor(testbed, floor, 0.6, 0.7, 1, 0.3);
    const int dx[] = {1, -1, 0, 0};
    const int dz[] = {0, 0, 1, -1};
    for (int k = 0; k < 4; k++) {
        R3RigidBodyHandle handle;
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(dx[k] * 7.25, 4, dz[k] * 7.25);
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(dx[k] ? 0.25 : 7.5, 4, dx[k] ? 7.5 : 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.6, 0.7, 1, 0.3);
    }
    for (int layer = 0; layer < 8; layer++) {
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                R3SoftBodyDesc softBody =
                    r3SphereSoftBodyDesc(r3Vector(-5.7 + i * 1.9 + layer % 2 * 0.4, 2 + layer * 2,
                                                  -5.7 + j * 1.9 + layer % 3 * 0.3),
                                         0.45 + 0.1 * ((i + j + layer) % 3), 1);
                softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){20, 1});
                softBody.volumeFactor = 1.1;
                softBody.particleMass = 0.03;
                softBody.particleRadius = (R3OptionalReal){1, 0.08};
                R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.08);
                surfaceCollider.friction = 0.6;
                softBody.collider = surfaceCollider;

                if (testbed->noSleep) {
                    softBody.canSleep = 0;
                }
                r3InsertSoftBody(world, &softBody);
            }
        }
    }
    R3SoftBodyDesc softBodyValue = r3ClothSoftBodyDesc(r3Vector(-4, 28, -0.6), r3Vector(0.15, 0, 0),
                                                       r3Vector(0, 0, 0.15), 54, 9);
    softBodyValue.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){30, 1};
    material.volumeSoftness = (R3SpringCoefficients){30, 1};
    material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    softBodyValue.material = material;
    softBodyValue.selfContacts = 1;
    softBodyValue.particleMass = 0.02;
    softBodyValue.particleRadius = (R3OptionalReal){1, 0.05};
    R3ColliderDesc surfaceCollider2 = r3BallColliderDesc(0.05);
    surfaceCollider2.friction = 0.5;
    softBodyValue.collider = surfaceCollider2;

    if (testbed->noSleep) {
        softBodyValue.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBodyValue);
    /* Set up the viewer. */
    tbCamera(testbed, 20, 18, 20, 0, 6, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
