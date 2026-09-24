/* Port of examples3d/stress_tests/convex_polyhedron3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "example_math.h"

static R3SharedShape *randomHull(Testbed *testbed) {
    R3Vector points[10];
    for (size_t i = 0; i < 10; i++) {
        R3Real x = exampleRandom(&testbed->randomState) * 2;
        R3Real y = exampleRandom(&testbed->randomState) * 2;
        R3Real z = exampleRandom(&testbed->randomState) * 2;
        points[i] = r3Vector(x, y, z);
    }
    R3SharedShape *shape = r3RoundConvexHullSharedShape((R3VectorView){points, 10}, 0.1);
    return shape;
}

void tbStressTestsConvexPolyhedron3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(200.1, 0.1, 200.1));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3SharedShape *shapes[5];
    for (int i = 0; i < 5; i++) {
        shapes[i] = randomHull(testbed);
    }
    R3Real offset = -8.8;
    for (int j = 0; j < 47; j++) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 2.2 - 8.8 + offset, j * 2.2 + 4.1, k * 2.2 - 8.8 + offset);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3DefaultColliderDesc();
                collider.shape.kind = R3_SHAPE_DESC_SHARED;
                collider.shape.sharedShape = shapes[(i + k) % 5];
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
        offset -= 0.35;
    }
    for (size_t i = 0; i < TB_COUNT(shapes); ++i) {
        r3FreeSharedShape(shapes[i]);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
