/* Port of examples3d/stress_tests/soft_slab3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftSlab3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle floor;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.5, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(9.5, 0.5, 9.5));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(floor, &collider);

    tbBodyColor(testbed, floor, 0.6, 0.7, 1, 0.3);
    const int dx[] = {1, -1, 0, 0};
    const int dz[] = {0, 0, 1, -1};
    for (int k = 0; k < 4; k++) {
        R3RigidBodyHandle handle;
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(dx[k] * 9.25, 3.5, dz[k] * 9.25);
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(dx[k] ? 0.25 : 9.5, 3.5, dx[k] ? 9.5 : 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.6, 0.7, 1, 0.3);
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(-6 + i * 3, 0.5, -6 + j * 3);
            R3ColliderDesc collider;
            if ((i + j) % 2) {
                collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
            } else {
                collider = r3BallColliderDesc(0.5);
            }
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R3SoftBodyDesc softBody =
        r3CuboidSoftBodyDesc(r3Vector(0, 2.2, 0), r3Vector(8, 1, 8), 25, 4, 25);
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    softBody.cellModel = R3_SOFT_CELL_COROTATIONAL;
    material.youngModulus = 4.0e4;
    material.poissonRatio = 0.4;
    material.elasticDampingRatio = 0.5;
    softBody.material = material;
    softBody.particleMass = 0.2;
    R3ColliderDesc surfaceCollider = r3BallColliderDesc(0.3);
    surfaceCollider.friction = 0.7;
    softBody.collider = surfaceCollider;

    if (testbed->noSleep) {
        softBody.canSleep = 0;
    }
    r3InsertSoftBody(world, &softBody);
    for (int wave = 0; wave < 3; wave++) {
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(-7.5 + i + wave % 2 * 0.5, 5.5 + wave * 3 + (i + j) % 3 * 0.7,
                             -7.5 + j + wave % 3 * 0.3);
                R3ColliderDesc collider;
                if ((i + j + wave) % 2) {
                    collider = r3CuboidColliderDesc(r3Vector(0.35, 0.35, 0.35));
                } else {
                    collider = r3BallColliderDesc(0.35);
                }
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 20, 14, 20, 0, 2, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
