/* Port of examples2d/s2d_high_mass_ratio_1.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dHighMassRatio1(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2SharedShape *shape = r2SegmentSharedShape(r2Vector(-66, 0), r2Vector(66, 0));
    R2ColliderDesc floor = r2DefaultColliderDesc();
    floor.shape.kind = R2_SHAPE_DESC_SHARED;
    floor.shape.sharedShape = shape;
    floor.friction = 0.5;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);

    for (int j = 0; j < 3; j++) {
        for (int count = 10; count > 0; count--) {
            for (int i = 0; i < count; i++) {
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 1));
                collider.density = count == 1 ? (j + 1) * 100 : 1;
                collider.friction = 0.5;
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r2Vector(2 * (i - count * 0.5) - 20 + 22 * j,
                             1 + (10 - count) * 2 + (count == 1 ? 2 : 0));
                rigidBody.canSleep = !testbed->noSleep;
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    r2FreeSharedShape(shape);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
