/* Port of examples2d/soft_stress2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftStress2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(8, 9.3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(3, 0.3));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Bridge under a crate. */
    {
        uint32_t pinned[155];
        size_t pinnedCount = 0;
        for (uint32_t i = 0; i < 31; ++i) {
            for (uint32_t j = 0; j < 5; ++j) {
                if (i == 0 || i == 30) {
                    pinned[pinnedCount++] = i * 5 + j;
                }
            }
        }
        R2SoftBodyDesc bridge = r2GridSoftBodyDesc(r2Vector(-7, 3), r2Vector(4.5, 0.6), 31, 5);
        r2SoftBodyDesc_SetPinnedParticles(&bridge,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        bridge.particleMass = .05;
        bridge.particleRadius = (R2OptionalReal){1, .15};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.edgeSoftness = (R2SpringCoefficients){100, 1};
            material.volumeSoftness = (R2SpringCoefficients){100, 1};
            material.tearForce = (R2OptionalReal){1, 65};
            material.tearSmoothing = .5;
            bridge.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.15);
            surface.friction = .8;
            bridge.collider = surface;
        }
        bridge.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &bridge);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-7, 4.4);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.6, 0.6));
        collider.density = 1.5;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Bar hanging a crate. */
    R2SoftBodyHandle bar;
    {
        uint32_t pinned[51];
        size_t pinnedCount = 0;
        for (uint32_t i = 0; i < 3; ++i) {
            for (uint32_t j = 0; j < 17; ++j) {
                if (j == 16) {
                    pinned[pinnedCount++] = i * 17 + j;
                }
            }
        }
        R2SoftBodyDesc builder = r2GridSoftBodyDesc(r2Vector(8, 6.8), r2Vector(0.3, 2.2), 3, 17);
        r2SoftBodyDesc_SetPinnedParticles(&builder,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        builder.particleMass = .05;
        builder.particleRadius = (R2OptionalReal){1, .1};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.edgeSoftness = (R2SpringCoefficients){100, 1};
            material.volumeSoftness = (R2SpringCoefficients){100, 1};
            material.tearForce = (R2OptionalReal){1, 16};
            material.tearSmoothing = .5;
            builder.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = .8;
            builder.collider = surface;
        }
        builder.canSleep = !testbed->noSleep;
        bar = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle crateBody;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(8, 4);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.6, 0.5));
        collider.density = 1;
        crateBody = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(crateBody, &collider);
    }

    for (size_t i = 0; i < 3; ++i) {
        r2SoftBody_AttachParticle(bar, i * 17, crateBody);
    }
    /* Block under a crate. */
    R2SoftBodyDesc block = r2GridSoftBodyDesc(r2Vector(2, 1.2), r2Vector(1.2, 1.2), 9, 9);
    block.particleMass = .05;
    block.particleRadius = (R2OptionalReal){1, .15};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.edgeSoftness = (R2SpringCoefficients){100, 1};
        material.volumeSoftness = (R2SpringCoefficients){100, 1};
        material.tearForce = (R2OptionalReal){1, 10};
        material.tearSmoothing = .5;
        block.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.15);
        surface.friction = .8;
        block.collider = surface;
    }
    block.canSleep = !testbed->noSleep;
    r2InsertSoftBody(world, &block);

    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(2, 3.2);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.8, 0.6));
        collider.density = 2;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Blob colored by stretch instead of a tear threshold. */
    R2SoftBodyDesc blob = r2DiskSoftBodyDesc(r2Vector(13, 1.6), 1.5, 36);
    blob.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){8, 1});
    blob.particleMass = .05;
    {
        R2ColliderDesc surface = r2BallColliderDesc(.1);
        surface.friction = .8;
        blob.collider = surface;
    }
    blob.canSleep = !testbed->noSleep;
    r2InsertSoftBody(world, &blob);

    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(13, 4);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.6, 0.4));
        collider.density = 1;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera2(testbed, 2, 4, 40);
    testbed->initialDebug = R2_DEBUG_SOFT_BODIES | R2_DEBUG_SOFT_BODY_STRESS;

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
