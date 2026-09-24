/* Port of examples3d/stress_tests/soft_jellies3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftJellies3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle floor;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.5, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(6.5, 0.5, 6.5));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(floor, &collider);

    tbBodyColor(testbed, floor, 0.6, 0.7, 1, 0.3);
    const int dx[] = {1, -1, 0, 0};
    const int dz[] = {0, 0, 1, -1};
    for (int k = 0; k < 4; k++) {
        R3RigidBodyHandle handle;
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(dx[k] * 6.25, 3.5, dz[k] * 6.25);
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(dx[k] ? 0.25 : 6.5, 3.5, dx[k] ? 6.5 : 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.6, 0.7, 1, 0.3);
    }
    int k = 0;
    for (int layer = 0; layer < 8; layer++) {
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++, k++) {
                R3SoftBodyDesc softBody =
                    r3CuboidSoftBodyDesc(r3Vector(-4.8 + i * 2.4 + layer % 2 * 0.6, 2 + layer * 2.2,
                                                  -4.8 + j * 2.4 + layer % 3 * 0.4),
                                         r3Vector(0.55, 0.55, 0.55), 4, 4, 4);
                R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
                softBody.cellModel = k % 3 ? R3_SOFT_CELL_COROTATIONAL : R3_SOFT_CELL_NEO_HOOKEAN;
                material.youngModulus = 2.0e3 * (1 + k % 5 * 3);
                material.poissonRatio = 0.4;
                material.elasticDampingRatio = 0.5;
                softBody.material = material;
                softBody.particleMass = 0.1;
                softBody.particleRadius = (R3OptionalReal){1, 0.08};
                R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.08);
                surfaceCollider.friction = 0.7;
                softBody.collider = surfaceCollider;

                if (testbed->noSleep) {
                    softBody.canSleep = 0;
                }
                r3InsertSoftBody(world, &softBody);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 18, 16, 18, 0, 5, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
