/* Port of examples2d/stress_tests/many_pyramids2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsManyPyramids2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle floor;
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    floor = r2InsertRigidBody(world, &groundBody);

    for (int i = 0; i < 20; i++) {
        R2SharedShape *shape = r2SegmentSharedShape(r2Vector(-110, i * 11), r2Vector(110, i * 11));
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        r2InsertCollider(floor, &collider);

        r2FreeSharedShape(shape);
    }
    for (int row = 0; row < 20; row++) {
        for (int col = 0; col < 20; col++) {
            for (int i = 0; i < 10; i++) {
                for (int j = i; j < 10; j++) {
                    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r2Vector((i + 1) * 0.5 + (j - i) - 110 + col * 11 + 1 - 0.5,
                                 (2 * i + 1) * 0.5 + row * 11);
                    rigidBody.canSleep = 0;
                    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
                    if (testbed->noSleep) {
                        rigidBody.canSleep = 0;
                        rigidBody.sleeping = 0;
                    }
                    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                    r2InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 110, 3);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
