/* Port of examples3d/soft_pile3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftPile3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle floor;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.5, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(5.5, 0.5, 5.5));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(floor, &boxCollider);

    tbBodyColor(testbed, floor, 0.6, 0.7, 1, 0.3);
    const int dx[] = {1, -1, 0, 0};
    const int dz[] = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++) {
        R3RigidBodyHandle handle;
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(dx[i] * 5.25, 4, dz[i] * 5.25);
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(dx[i] ? 0.25 : 5.5, 4, dx[i] ? 5.5 : 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.6, 0.7, 1, 0.3);
    }
    int k = 0;
    for (int layer = 0; layer < 40; layer++) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++, k++) {
                R3Vector position = r3Vector(-3 + i * 3 + layer % 2 * 0.7, 3 + layer * 2.5,
                                             -3 + j * 3 + layer % 3 * 0.5);
                if (k % 5 == 4) {
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
                    collider.density = 0.5;
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = position;
                    rigidBody.canSleep = !testbed->noSleep;
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                } else {
                    int large = k % 5 == 0 || k % 5 == 2;
                    R3Real radius = large ? 0.08 : 0.07;
                    R3SoftBodyDesc softBody = r3SphereSoftBodyDesc(position, large ? 0.6 : 0.45, 1);
                    softBody.material =
                        r3UniformSoftBodyMaterial((R3SpringCoefficients){large ? 15 : 10, 1});
                    softBody.volumeFactor = large ? 1.15 : 1.3;
                    softBody.particleMass = 0.03;
                    softBody.particleRadius = (R3OptionalReal){1, radius};
                    R3ColliderDesc surfaceCollider = r3BallColliderDesc(radius);
                    surfaceCollider.friction = 0.6;
                    softBody.collider = surfaceCollider;

                    if (testbed->noSleep) {
                        softBody.canSleep = 0;
                    }
                    r3InsertSoftBody(world, &softBody);
                }
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        R3SoftBodyDesc softBody = r3RopeSoftBodyDesc(r3Vector(-3.5, 24 + i, -2 + i * 2),
                                                     r3Vector(3.5, 24 + i, -1.5 + i * 2), 30);
        softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
        softBody.particleMass = 0.03;
        R3ColliderDesc surfaceCollider2 = r3BallColliderDesc(0.08);
        surfaceCollider2.friction = 0.6;
        softBody.collider = surfaceCollider2;

        if (testbed->noSleep) {
            softBody.canSleep = 0;
        }
        r3InsertSoftBody(world, &softBody);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 14, 12, 14, 0, 4, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
