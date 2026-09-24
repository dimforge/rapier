/* Port of examples2d/soft_thin_features2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R2SoftBodyDesc jelly(R2Vector center, R2Real half, size_t n, R2Real young) {
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(center, r2Vector(half, half), n, n);
    builder.cellModel = R2_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .1;
    builder.particleRadius = (R2OptionalReal){1, .06};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = young;
        material.poissonRatio = .4;
        material.elasticDampingRatio = .5;
        builder.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.06);
        surface.friction = .7;
        builder.collider = surface;
    }
    return builder;
}

static R2SoftBodyDesc strip(R2Vector center, R2Vector half, size_t nx) {
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(center, half, nx, 2);
    builder.cellModel = R2_SOFT_CELL_VOLUME;
    builder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    builder.particleMass = .03;
    builder.particleRadius = (R2OptionalReal){1, .05};
    {
        R2ColliderDesc surface = r2BallColliderDesc(.05);
        surface.friction = .6;
        builder.collider = surface;
    }
    return builder;
}

static R2SoftBodyDesc blob(R2Vector center, R2Real radius) {
    R2SoftBodyDesc builder = r2DiskSoftBodyDesc(center, radius, 24);
    builder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20, 1});
    builder.volumeFactor = 1.2;
    builder.particleMass = .05;
    builder.particleRadius = (R2OptionalReal){1, .06};
    {
        R2ColliderDesc surface = r2BallColliderDesc(.06);
        surface.friction = .6;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftThinFeatures2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(40, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Bed of nails. */
    for (int i = 0; i < 24; ++i) {
        {
            R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-16 + i * .5, 0.6);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2CapsuleYColliderDesc(.6, .03);
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    {
        R2SoftBodyDesc body = jelly(r2Vector(-14.5, 3.5), .75, 5, 3.0e3);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    {
        R2SoftBodyDesc body = jelly(r2Vector(-11.5, 3.5), .75, 5, 5.0e4);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    {
        R2SoftBodyDesc body = blob(r2Vector(-8.5, 3.5), .8);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    {
        R2SoftBodyDesc body = strip(r2Vector(-6, 6), r2Vector(2.5, .1), 31);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    /* Needle rain on a hammock and a jelly block. */
    const uint32_t pinned[] = {0, 1, 60, 61};
    {
        R2SoftBodyDesc hammock = strip(r2Vector(0, 4), r2Vector(3, .1), 31);
        r2SoftBodyDesc_SetPinnedParticles(&hammock,
                                         (R2IndexView){(const uint32_t *)pinned, TB_COUNT(pinned)});
        hammock.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &hammock);
    }
    {
        R2SoftBodyDesc body = jelly(r2Vector(6.5, .9), .9, 6, 2.0e4);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    const R2Real centers[] = {0, 6.5};
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (size_t k = 0; k < TB_COUNT(centers); ++k) {
                const R2Real cx = centers[k];
                {
                    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                    rigidBody.position.translation = r2Vector(cx - 2 + i * .45, 8 + j * 1.2);
                    rigidBody.position.rotation = r2Rotation(.4 * (i + j));
                    rigidBody.canSleep = !testbed->noSleep;
                    R2ColliderDesc collider = r2CapsuleYColliderDesc(.5, .02);
                    collider.density = 3;
                    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                    r2InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    /* Thin plates falling on a blob. */
    {
        R2SoftBodyDesc body = blob(r2Vector(11, .9), .9);
        body.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &body);
    }
    for (int i = 0; i < 4; ++i) {
        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(11, 3.5 + i * .5);
            rigidBody.position.rotation = r2Rotation(.15 * i);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.9, 0.015));
            collider.density = 1;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* A strip stretched at its right end with a rod resting on it. */
    R2SoftBodyHandle stretched;
    {
        R2SoftBodyDesc body = strip(r2Vector(16, 3), r2Vector(2.5, .1), 31);
        r2SoftBodyDesc_SetPinnedParticles(&body,
                                         (R2IndexView){(const uint32_t *)pinned, TB_COUNT(pinned)});
        body.canSleep = !testbed->noSleep;
        stretched = r2InsertSoftBody(world, &body);
    }

    R2Vector rightRest[2];
    rightRest[0] = r2SoftBody_ParticlePosition(stretched, 60);
    rightRest[1] = r2SoftBody_ParticlePosition(stretched, 61);
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(16, 4);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CapsuleXColliderDesc(.8, .03);
        collider.density = 2;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera2(testbed, 0, 4, 22);

    tbSetWorld(testbed, world);
    R2Real t = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            t += dt;
            const R2Real stretch = 2.5 * (1 - cos(.5 * t));

            for (size_t i = 0; i < 2; ++i) {
                r2SoftBody_SetParticleKinematicTarget(
                    stretched, 60 + i, r2VectorAdd(rightRest[i], r2Vector(stretch, 0)));
            }
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
