/* Port of examples2d/s2d_confined.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dConfined(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    const R2Vector ends[][2] = {{r2Vector(-10.5, 0), r2Vector(10.5, 0)},
                                {r2Vector(-10.5, 0), r2Vector(-10.5, 20.5)},
                                {r2Vector(10.5, 0), r2Vector(10.5, 20.5)},
                                {r2Vector(-10.5, 20.5), r2Vector(10.5, 20.5)}};
    for (int i = 0; i < 4; i++) {
        R2SharedShape *shape = r2CapsuleSharedShape(ends[i][0], ends[i][1], 0.5);
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        collider.friction = 0.6;
        r2InsertColliderWithoutParent(world, &collider);

        r2FreeSharedShape(shape);
    }
    for (int col = 0; col < 25; col++) {
        for (int row = 0; row < 25; row++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r2Vector(-8.75 + col * 18.0 / 25, 1.5 + row * 18.0 / 25);
            rigidBody.gravityScale = 0;
            R2ColliderDesc collider = r2BallColliderDesc(0.5);
            collider.friction = 0.6;
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
