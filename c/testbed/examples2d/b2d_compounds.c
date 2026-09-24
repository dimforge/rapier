/* Port of examples2d/b2d_compounds.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dCompounds(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyHandle ground;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    rigidBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &rigidBody);

    for (int i = 0; i <= 80; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.55, 0.5));
        collider.position.translation = r2Vector(-40 + i, 0);
        r2InsertCollider(ground, &collider);
    }
    for (int side = -1; side <= 1; side += 2) {
        for (int i = 0; i < 100; i++) {
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.55));
            collider.position.translation = r2Vector(side * 40, i + 1);
            r2InsertCollider(ground, &collider);
        }
    }
    R2SharedShape *segment = r2SegmentSharedShape(r2Vector(-800, -80), r2Vector(800, -80));
    R2ColliderDesc shapeCollider = r2DefaultColliderDesc();
    shapeCollider.shape.kind = R2_SHAPE_DESC_SHARED;
    shapeCollider.shape.sharedShape = segment;
    r2InsertCollider(ground, &shapeCollider);

    const R2Vector left[] = {r2Vector(-1, 0), r2Vector(0.5, 1), r2Vector(0, 2)};
    const R2Vector right[] = {r2Vector(1, 0), r2Vector(-0.5, 1), r2Vector(0, 2)};
    R2Real side = 0.25;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 150; j++) {
            R2RigidBodyHandle handle;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 2 - 19 + side, j * 2.25 + 5.575);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r2InsertRigidBody(world, &rigidBody);

            side = -side;
            for (int p = 0; p < 2; p++) {
                R2ColliderDesc collider = r2DefaultColliderDesc();
                r2ShapeDesc_SetConvexHull(&collider.shape, (R2VectorView){p ? right : left, 3});
                collider.friction = 0.5;
                r2InsertCollider(handle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 120, 2);
    r2FreeSharedShape(segment);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
