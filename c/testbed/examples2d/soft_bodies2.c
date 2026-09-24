/* Port of examples2d/soft_bodies2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftBodies2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    /* Ground and walls. */
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(15, 0.5));

        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-15, 5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 5));

        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(15, 5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 5));

        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }

    /* Pressurized blobs of various sizes. */
    for (int i = 0; i < 5; i++) {
        const R2Real radius = 0.6 + 0.15 * i;
        R2SoftBodyDesc blob = r2DiskSoftBodyDesc(r2Vector(-10.0 + i * 2.5, 2.0 + i), radius, 24);
        blob.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20.0, 1.0});
        blob.volumeFactor = 1.1;
        blob.selfContacts = 1;
        blob.particleMass = 0.05;
        blob.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &blob);
    }

    /* Jelly bodies: corotational, Neo-Hookean, and per-cell volume constraints. */
    {
        R2SoftBodyDesc jelly = r2GridSoftBodyDesc(r2Vector(2, 1.2), r2Vector(1, 1), 6, 6);
        jelly.cellModel = R2_SOFT_CELL_COROTATIONAL;
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = 3.0e3;
        material.poissonRatio = 0.35;
        material.elasticDampingRatio = 0.5;
        jelly.material = material;
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &jelly);
    }
    {
        R2SoftBodyDesc jelly = r2GridSoftBodyDesc(r2Vector(8, 1.2), r2Vector(1, 1), 6, 6);
        jelly.cellModel = R2_SOFT_CELL_NEO_HOOKEAN;
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = 3.0e3;
        material.poissonRatio = 0.35;
        material.elasticDampingRatio = 0.5;
        jelly.material = material;
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &jelly);
    }
    {
        R2SoftBodyDesc jelly = r2GridSoftBodyDesc(r2Vector(5, 1.2), r2Vector(1, 1), 6, 6);
        jelly.cellModel = R2_SOFT_CELL_VOLUME;
        jelly.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20.0, 1.0});
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &jelly);
    }

    /* A rope hanging from a fixed anchor, holding a rigid weight. */
    const uint32_t pinnedParticle = 0;
    R2SoftBodyDesc rope = r2DefaultSoftBodyDesc();
    rope.kind = R2_SOFT_DESC_ROPE;
    rope.a = r2Vector(8, 9);
    rope.b = r2Vector(12, 9);
    rope.nx = 25;
    rope.pinned = (R2IndexView){&pinnedParticle, 1};
    rope.material.edgeSoftness = rope.material.bendSoftness = rope.material.volumeSoftness =
        rope.material.shapeMatchingSoftness = (R2SpringCoefficients){40.0, 1.0};
    rope.particleMass = 0.05;
    rope.canSleep = !testbed->noSleep;
    R2SoftBodyHandle ropeHandle = r2InsertSoftBody(world, &rope);

    R2Vector lastPos = r2SoftBody_ParticlePosition(ropeHandle, 24);
    R2RigidBodyHandle weight;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2VectorAdd(lastPos, r2Vector(0, -0.4));
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.3, 0.3));
        collider.density = 2.0;
        weight = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(weight, &collider);
    }

    r2SoftBody_AttachParticle(ropeHandle, 24, weight);

    /* A stack of rigid boxes for the blobs to knock down. */
    for (int i = 0; i < 6; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-4.0, 0.3 + 0.6 * i);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.3, 0.3));

        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }

    /* Set up the viewer. */
    tbCamera2(testbed, 0.0, 4.0, 30.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
