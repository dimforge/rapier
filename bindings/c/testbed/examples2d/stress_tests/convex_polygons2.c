/* Port of examples2d/stress_tests/convex_polygons2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "example_math.h"

void tbStressTestsConvexPolygons2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 1.2));
    groundBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle groundBodyHandle = r2InsertRigidBody(world, &groundBody);
    r2InsertCollider(groundBodyHandle, &collider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 30, 60);
        rigidBody.position = r2Pose(r2Vector(side * 30, 60), r2Rotation(R2_PI / 2));
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(60, 1.2));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SharedShape *shapes[5];
    for (size_t i = 0; i < 5; i++) {
        R2Vector points[10];
        for (size_t k = 0; k < 10; k++) {
            R2Real x = exampleRandom(&testbed->randomState) * 2;
            R2Real y = exampleRandom(&testbed->randomState) * 2;
            points[k] = r2Vector(x, y);
        }
        shapes[i] = r2ConvexHullSharedShape((R2VectorView){points, 10});
    }
    for (int i = 0; i < 26; i++) {
        for (int j = 0; j < 130; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 2 - 26, j * 4 + 3);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2DefaultColliderDesc();
            collider.shape.kind = R2_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = shapes[i % 5];
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    for (size_t i = 0; i < TB_COUNT(shapes); ++i) {
        r2FreeSharedShape(shapes[i]);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 50, 10);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
