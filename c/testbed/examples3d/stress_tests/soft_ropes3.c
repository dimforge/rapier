/* Port of examples3d/stress_tests/soft_ropes3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftRopes3(Testbed *testbed) {
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
        rigidBody.position.translation = r3Vector(dx[k] * 6.25, 3, dz[k] * 6.25);
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(dx[k] ? 0.25 : 6.5, 3, dx[k] ? 6.5 : 0.25));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.6, 0.7, 1, 0.3);
    }
    for (int k = 0; k < 4; k++) {
        R3SharedShape *shape = r3CapsuleSharedShape(r3Vector(-5, 0, 0), r3Vector(5, 0, 0), 0.12);
        R3RigidBodyHandle handleH;
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 4, -4.5 + k * 3);
        R3ColliderDesc collider = r3DefaultColliderDesc();
        collider.shape.kind = R3_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        rigidBody.canSleep = !testbed->noSleep;
        handleH = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handleH, &collider);

        tbBodyColor(testbed, handleH, 0.6, 0.7, 1, 0.3);
        r3FreeSharedShape(shape);
    }
    for (int layer = 0; layer < 10; layer++) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                R3Real across = -5 + i * 2 + layer % 2 * 0.5;
                R3Real along = -5.7 + j * 3.8;
                R3Real y = 8 + layer * 1.5;
                R3Vector start =
                    layer % 2 ? r3Vector(across, y, along) : r3Vector(along, y, across);
                R3Vector end = layer % 2 ? r3Vector(across + 0.4, y + 0.3, along + 3.5)
                                         : r3Vector(along + 3.5, y + 0.3, across + 0.4);
                R3SoftBodyDesc softBody = r3RopeSoftBodyDesc(start, end, 40);
                softBody.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
                softBody.particleMass = 0.03;
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
    /* Set up the viewer. */
    tbCamera(testbed, 16, 14, 16, 0, 4, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
