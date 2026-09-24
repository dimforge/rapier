/* Port of examples2d/soft_force_tearing2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct DrivenParticle {
    R2SoftBodyHandle body;
    uint32_t index;
    R2Vector rest;
    int right;
} DrivenParticle;

/* Follow driven particle indices through compaction and newly split bodies. */
static void followTears(R2EventCollector *events, DrivenParticle *ends, size_t endCount) {
    size_t count = r2EventCollector_TearEventCount(events);
    for (size_t i = 0; i < count; ++i) {
        R2SoftBodyTearEvent *event = r2EventCollector_TearEvent(events, i);
        R2SoftBodyHandle origin = r2SoftBodyTearEvent_SoftBody(event);
        for (size_t j = 0; j < endCount; ++j) {
            if (ends[j].body.index == origin.index &&
                ends[j].body.generation == origin.generation) {
                R2Bool found;
                R2SoftBodyHandle destination;
                uint32_t index;
                R2OptionalParticleDestination softBodyTearEventTryParticleDestinationResult =
                    r2SoftBodyTearEvent_TryParticleDestination(event, ends[j].index);
                destination = softBodyTearEventTryParticleDestinationResult.body;
                index = softBodyTearEventTryParticleDestinationResult.index;
                found = softBodyTearEventTryParticleDestinationResult.found;
                if (found) {
                    ends[j].body = destination;
                    ends[j].index = index;
                }
            }
        }
        r2FreeSoftBodyTearEvent(event);
    }
}

static R2SoftBodyDesc hangingBar(R2Real x, R2SoftBodyMaterial *material) {
    R2SoftBodyDesc bar = r2GridSoftBodyDesc(r2Vector(x, 7.5), r2Vector(.3, 1.5), 3, 13);
    static const uint32_t pinned[] = {12, 25, 38};
    r2SoftBodyDesc_SetPinnedParticles(&bar,
                                     (R2IndexView){(const uint32_t *)pinned, TB_COUNT(pinned)});
    bar.material = *material;
    bar.particleMass = .05;
    bar.particleRadius = (R2OptionalReal){1, .1};
    R2ColliderDesc surface = r2BallColliderDesc(.1);
    surface.friction = .8;
    bar.collider = surface;

    return bar;
}

static void stiff(R2SoftBodyMaterial *material) {
    material->edgeSoftness = (R2SpringCoefficients){150, 1};
    material->volumeSoftness = (R2SpringCoefficients){150, 1};
}

void tbSoftForceTearing2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    const R2Real tearForce = tbSetting(testbed, "Tear force (crate bar)", 8, 2, 30, 0);
    const R2Real interiorStrength = tbSetting(testbed, "Interior strength (slab)", 4, 1, 8, 0);
    const R2Real smoothing = tbSetting(testbed, "Tear smoothing (right bar, s)", 1, 0, 3, 0);

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
        rigidBody.position.translation = r2Vector(0, 9.3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 0.3));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int column = 0; column < 2; ++column) {
        const R2Real x = column == 0 ? -13 : -10;
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        stiff(&material);
        if (column == 0) {
            material.tearStrain = (R2OptionalReal){1, .5};
        } else {
            material.tearForce = (R2OptionalReal){1, tearForce};
            material.tearSmoothing = .5;
        }
        R2SoftBodyDesc builder = hangingBar(x, &material);
        builder.canSleep = !testbed->noSleep;
        R2SoftBodyHandle bar = r2InsertSoftBody(world, &builder);

        R2RigidBodyHandle crateBody;
        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, 5.4);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.6, 0.5));
            collider.density = 1.2;
            crateBody = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(crateBody, &collider);
        }

        for (size_t i = 0; i < 3; ++i) {
            r2SoftBody_AttachParticle(bar, i * 13, crateBody);
        }
    }
    const size_t sx = 25, sy = 7;
    uint32_t pinned[28];
    for (size_t j = 0; j < sy; ++j) {
        pinned[4 * j] = j;
        pinned[4 * j + 1] = sy + j;
        pinned[4 * j + 2] = (sx - 2) * sy + j;
        pinned[4 * j + 3] = (sx - 1) * sy + j;
    }
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(r2Vector(-1, 2), r2Vector(3, 0.75), sx, sy);
    r2SoftBodyDesc_SetPinnedParticles(&builder,
                                     (R2IndexView){(const uint32_t *)pinned, TB_COUNT(pinned)});
    builder.particleMass = .05;
    builder.particleRadius = (R2OptionalReal){1, .1};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.edgeSoftness = (R2SpringCoefficients){60, 1};
        material.volumeSoftness = (R2SpringCoefficients){60, 1};
        material.tearForce = (R2OptionalReal){1, 60};
        material.tearSmoothing = .05;
        material.interiorStrength = interiorStrength;
        builder.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.1);
        surface.friction = .8;
        builder.collider = surface;
    }
    builder.canSleep = !testbed->noSleep;
    R2SoftBodyHandle slab = r2InsertSoftBody(world, &builder);

    size_t indexCount = r2SoftBody_Edges(slab, NULL, 0);
    uint32_t *edges = malloc(indexCount * sizeof(*edges));
    if (!edges) {
        abort();
    }
    indexCount = r2SoftBody_Edges(slab, edges, indexCount);
    for (size_t i = 0; i < indexCount / 2; ++i) {
        const uint32_t a = edges[2 * i], b = edges[2 * i + 1];
        if (a % sy == sy - 1 && b % sy == sy - 1 && a / sy >= 11 && a / sy <= 13 && b / sy >= 11 &&
            b / sy <= 13) {
            r2SoftBody_SetEdgeTearResistance(slab, i, .4);
        }
    }
    free(edges);
    DrivenParticle rightEnd[14];
    for (size_t j = 0; j < sy; ++j) {
        for (size_t side = 0; side < 2; ++side) {
            const uint32_t index = (sx - 2 + side) * sy + j;
            R2Vector rest = r2SoftBody_ParticlePosition(slab, index);
            rightEnd[j * 2 + side] = (DrivenParticle){slab, index, rest, 1};
        }
    }
    R2RigidBodyHandle disks[2];
    for (int column = 0; column < 2; ++column) {
        const R2Real x = column == 0 ? 8 : 13;
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.tearForce = (R2OptionalReal){1, 25};
        material.tearSmoothing = column == 0 ? 0 : smoothing;
        stiff(&material);
        R2SoftBodyDesc builder = hangingBar(x, &material);
        builder.canSleep = !testbed->noSleep;
        R2SoftBodyHandle bar = r2InsertSoftBody(world, &builder);

        R2RigidBodyHandle crateBody;
        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, 5.4);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1.2, 0.4));
            collider.density = .75;
            crateBody = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(crateBody, &collider);
        }

        for (size_t i = 0; i < 3; ++i) {
            r2SoftBody_AttachParticle(bar, i * 13, crateBody);
        }
        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x + .8, 8.6);
            rigidBody.enabled = 0;
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2BallColliderDesc(.2);
            collider.density = 3;
            disks[column] = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(disks[column], &collider);
        }
    }
    R2EventCollector *events = r2NewEventCollector();
    tbCamera2(testbed, 0, 4.5, 40);
    testbed->snapshotSupported = 0;
    testbed->initialDebug = R2_DEBUG_SOFT_BODIES | R2_DEBUG_SOFT_BODY_STRESS;
    tbSetWorld(testbed, world);
    R2Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            if (t < 1 && t + dt >= 1) {
                for (size_t i = 0; i < TB_COUNT(disks); ++i) {
                    r2RigidBody_SetEnabled(disks[i], 1);
                    r2RigidBody_SetLinvel(disks[i], r2Vector(0, -15), 1);
                }
            }
            t += dt;
            const R2Real shift = fmin(fmax(t - 2, 0) * .25, 3);
            for (size_t i = 0; i < TB_COUNT(rightEnd); ++i) {
                r2SoftBody_SetParticleKinematicTarget(
                    rightEnd[i].body, rightEnd[i].index,
                    r2VectorAdd(rightEnd[i].rest, r2Vector(shift, 0)));
            }
            r2EventCollector_Clear(events);
            r2Step(world, NULL, events);
            followTears(events, rightEnd, TB_COUNT(rightEnd));
        }
    }
    r2FreeEventCollector(events);
    r2FreeWorld(world);
}
