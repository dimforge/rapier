/* Port of examples2d/soft_cutting2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftCutting2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Jelly block to slice by hand. */
    R2SoftBodyDesc block = r2GridSoftBodyDesc(r2Vector(-8, 2), r2Vector(2, 2), 13, 13);
    block.cellModel = R2_SOFT_CELL_COROTATIONAL;
    block.particleMass = .05;
    block.particleRadius = (R2OptionalReal){1, .15};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = 2.0e4;
        material.poissonRatio = .35;
        material.elasticDampingRatio = 1;
        block.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.15);
        surface.friction = .8;
        block.collider = surface;
    }
    block.canSleep = !testbed->noSleep;
    r2InsertSoftBody(world, &block);

    /* Pressurized ring: a cut opens it and it falls limp. */
    R2SoftBodyDesc blob = r2DiskSoftBodyDesc(r2Vector(-2, 2), 1.6, 40);
    blob.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    blob.particleMass = .05;
    {
        R2ColliderDesc surface = r2BallColliderDesc(.1);
        surface.friction = .8;
        blob.collider = surface;
    }
    blob.canSleep = !testbed->noSleep;
    r2InsertSoftBody(world, &blob);

    /* Curtain. */
    {
        uint32_t pinned[279];
        size_t pinnedCount = 0;
        for (uint32_t i = 0; i < 9; ++i) {
            for (uint32_t j = 0; j < 31; ++j) {
                if (j == 30) {
                    pinned[pinnedCount++] = i * 31 + j;
                }
            }
        }
        R2SoftBodyDesc curtain = r2GridSoftBodyDesc(r2Vector(3, 4.5), r2Vector(0.6, 3), 9, 31);
        curtain.cellModel = R2_SOFT_CELL_COROTATIONAL;
        r2SoftBodyDesc_SetPinnedParticles(&curtain,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        curtain.particleMass = .05;
        curtain.particleRadius = (R2OptionalReal){1, .1};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 3.0e4;
            material.poissonRatio = .35;
            material.elasticDampingRatio = 1;
            curtain.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = .8;
            curtain.collider = surface;
        }
        curtain.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &curtain);
    }
    /* Slab. */
    {
        uint32_t pinned[125];
        size_t pinnedCount = 0;
        for (uint32_t i = 0; i < 25; ++i) {
            for (uint32_t j = 0; j < 5; ++j) {
                if (i == 0 || i == 24) {
                    pinned[pinnedCount++] = i * 5 + j;
                }
            }
        }
        R2SoftBodyDesc slab = r2GridSoftBodyDesc(r2Vector(10, 4), r2Vector(3, 0.5), 25, 5);
        slab.cellModel = R2_SOFT_CELL_COROTATIONAL;
        r2SoftBodyDesc_SetPinnedParticles(&slab,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        slab.particleMass = .05;
        slab.particleRadius = (R2OptionalReal){1, .1};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 5.0e4;
            material.poissonRatio = .35;
            material.elasticDampingRatio = 1;
            slab.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = .8;
            slab.collider = surface;
        }
        slab.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &slab);
    }
    /* Sensor blade: the cut does the work, without pushing the slab. */
    const R2Vector sawStart = r2Vector(10, 1.5);
    R2RigidBodyHandle saw;
    {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = sawStart;
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.05, 1));
        collider.isSensor = 1;
        saw = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(saw, &collider);
    }

    size_t handleCount = r2SoftBodyCount(world);
    R2SoftBodyHandle *handles = malloc(handleCount * sizeof(*handles));
    if (!handles) {
        abort();
    }
    handleCount = r2SoftBodyHandles(world, handles, handleCount);
    tbCamera2(testbed, 1, 3.5, 40);

    tbSetWorld(testbed, world);
    R2Real t = 0;
    R2Vector bladeStart = {0};
    int bladeActive = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        /* Hold C to position a hand blade, then release C to cut, even while paused. */
        if (testbed->cutting) {
            if (!bladeActive && testbed->cursorValid) {
                bladeStart = testbed->cursor;
                bladeActive = 1;
            }
            if (bladeActive && testbed->cursorValid) {
                tbLine(testbed, bladeStart, testbed->cursor, 1, .35f, .25f, 1);
            }
        } else if (bladeActive) {
            bladeActive = 0;
            if (testbed->cursorValid) {
                const R2Vector edge[] = {bladeStart, testbed->cursor};
                for (size_t i = 0; i < handleCount; ++i) {
                    R2SoftBodyTearEvent *event = r2CutSoftBody(handles[i], edge);
                    r2FreeSoftBodyTearEvent(event);
                }
            }
        }

        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            t += dt;
            const R2Real rise = fmin(fmax(t - 1, 0) * .2, 4.5);
            const R2Vector position = r2VectorAdd(sawStart, r2Vector(0, rise));

            r2RigidBody_SetNextKinematicTranslation(saw, position);
            const R2Vector edge[] = {r2VectorSub(position, r2Vector(0, 1)),
                                     r2VectorAdd(position, r2Vector(0, 1))};
            for (size_t i = 0; i < handleCount; ++i) {
                R2SoftBodyTearEvent *event = r2CutSoftBody(handles[i], edge);
                r2FreeSoftBodyTearEvent(event);
            }
            r2Step(world, NULL, NULL);
        }
    }
    free(handles);
    r2FreeWorld(world);
}
