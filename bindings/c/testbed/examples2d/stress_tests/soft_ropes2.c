/* Port of examples2d/stress_tests/soft_ropes2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsSoftRopes2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(10, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 10, 5);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 10));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 5; i++) {
        R2SharedShape *shape = r2CapsuleSharedShape(r2Vector(-0.6, 0), r2Vector(0.6, 0), 0.1);
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-6 + i * 3, 5);
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);

        r2FreeSharedShape(shape);
    }
    for (int layer = 0; layer < 80; layer++) {
        for (int i = 0; i < 4; i++) {
            R2Real x = -8.6 + i * 4.4 + layer % 2 * 0.6;
            R2Real y = 8 + layer * 1.2;
            R2SoftBodyDesc softBody =
                r2RopeSoftBodyDesc(r2Vector(x, y), r2Vector(x + 4, y + 0.4), 40);
            softBody.particleMass = 0.03;
            R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.4);
            surfaceCollider.friction = 0.6;
            softBody.collider = surfaceCollider;

            if (testbed->noSleep) {
                softBody.canSleep = 0;
            }
            r2InsertSoftBody(world, &softBody);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 8, 25);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
