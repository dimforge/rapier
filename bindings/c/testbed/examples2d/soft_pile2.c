/* Port of examples2d/soft_pile2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftPile2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(9, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &boxCollider);

    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 9, 12);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 12));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int i = 0; i < 5; i++) {
        R2SharedShape *shape = r2CapsuleSharedShape(r2Vector(-0.6, 0), r2Vector(0.6, 0), 0.1);
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-6 + i * 3, 6);
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);

        r2FreeSharedShape(shape);
    }
    int k = 0;
    for (int layer = 0; layer < 40; layer++) {
        for (int i = 0; i < 6; i++, k++) {
            R2Vector position = r2Vector(-7 + i * 2.8 + layer % 2, 9 + layer * 2.6);
            R2SoftBodyDesc softBody;
            if (k % 5 == 0 || k % 5 == 3) {
                softBody = r2GridSoftBodyDesc(position, r2Vector(0.6, 0.6), 4, 4);
                R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
                softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
                material.youngModulus = 3.0e3 * (1 + k % 7 * 4);
                material.poissonRatio = 0.4;
                material.elasticDampingRatio = 0.5;
                softBody.material = material;
                softBody.particleMass = 0.1;
                softBody.particleRadius = (R2OptionalReal){1, 0.08};
                R2ColliderDesc surfaceCollider = r2BallColliderDesc(0.08);
                surfaceCollider.friction = 0.7;
                softBody.collider = surfaceCollider;
            } else if (k % 5 == 1) {
                softBody = r2DiskSoftBodyDesc(position, 0.6, 20);
                softBody.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20, 1});
                softBody.volumeFactor = 1.1;
                softBody.selfContacts = 1;
                softBody.particleMass = 0.05;
                softBody.particleRadius = (R2OptionalReal){1, 0.06};
                R2ColliderDesc surfaceCollider2 = r2BallColliderDesc(0.06);
                surfaceCollider2.friction = 0.6;
                softBody.collider = surfaceCollider2;
            } else if (k % 5 == 2) {
                softBody = r2GridSoftBodyDesc(position, r2Vector(1.2, 0.12), 13, 2);
                R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
                softBody.cellModel = R2_SOFT_CELL_COROTATIONAL;
                material.youngModulus = 2.0e4;
                material.poissonRatio = 0.4;
                material.elasticDampingRatio = 0.5;
                softBody.material = material;
                softBody.particleMass = 0.1;
                softBody.particleRadius = (R2OptionalReal){1, 0.06};
                R2ColliderDesc surfaceCollider3 = r2BallColliderDesc(0.06);
                surfaceCollider3.friction = 0.6;
                softBody.collider = surfaceCollider3;
            } else {
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
                collider.density = 0.5;
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation = position;
                rigidBody.canSleep = !testbed->noSleep;
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }
            {
                if (testbed->noSleep) {
                    softBody.canSleep = 0;
                }
                r2InsertSoftBody(world, &softBody);
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        R2SoftBodyDesc softBody =
            r2RopeSoftBodyDesc(r2Vector(-6 + i, 36 + i), r2Vector(4 + i, 37 + i), 30);
        softBody.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
        softBody.particleMass = 0.03;
        R2ColliderDesc surfaceCollider4 = r2BallColliderDesc(0.08);
        surfaceCollider4.friction = 0.6;
        softBody.collider = surfaceCollider4;

        if (testbed->noSleep) {
            softBody.canSleep = 0;
        }
        r2InsertSoftBody(world, &softBody);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 10, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
