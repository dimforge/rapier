/* Port of examples2d/soft_tearing2.rs. */
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

void tbSoftTearing2(Testbed *testbed) {
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
        rigidBody.position.translation = r2Vector(16, 3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.3, 3));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Bridge. */
    {
        const size_t nx = 31, ny = 6;
        uint32_t pinned[186];
        size_t pinnedCount = 0;
        for (size_t i = 0; i < nx; ++i) {
            for (size_t j = 0; j < ny; ++j) {
                if (i == 0 || i == nx - 1) {
                    pinned[pinnedCount++] = (uint32_t)(i * ny + j);
                }
            }
        }
        R2SoftBodyDesc bridge = r2GridSoftBodyDesc(r2Vector(-8, 4), r2Vector(3, 0.5), nx, ny);
        bridge.cellModel = R2_SOFT_CELL_COROTATIONAL;
        r2SoftBodyDesc_SetPinnedParticles(&bridge,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        bridge.particleMass = .05;
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 1.0e6;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            material.tearStrain = (R2OptionalReal){1, .35};
            bridge.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = .8;
            bridge.collider = surface;
        }
        bridge.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &bridge);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-8, 9);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2BallColliderDesc(.6);
        collider.density = 20;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Curtain. */
    {
        const size_t nx = 7, ny = 41;
        uint32_t pinned[287];
        size_t pinnedCount = 0;
        for (size_t i = 0; i < nx; ++i) {
            for (size_t j = 0; j < ny; ++j) {
                if (j == ny - 1) {
                    pinned[pinnedCount++] = (uint32_t)(i * ny + j);
                }
            }
        }
        R2SoftBodyDesc curtain = r2GridSoftBodyDesc(r2Vector(2, 5), r2Vector(0.45, 3), nx, ny);
        curtain.cellModel = R2_SOFT_CELL_COROTATIONAL;
        r2SoftBodyDesc_SetPinnedParticles(&curtain,
                                         (R2IndexView){(const uint32_t *)pinned, pinnedCount});
        curtain.particleMass = .05;
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 3.0e4;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            material.tearStrain = (R2OptionalReal){1, .35};
            curtain.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.08);
            surface.friction = .8;
            curtain.collider = surface;
        }
        curtain.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &curtain);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-4, 4.5);
        rigidBody.linvel = r2Vector(20, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2BallColliderDesc(.4);
        collider.density = 10;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Jelly bar pulled apart by its two pinned ends. */
    R2SoftBodyHandle bar;
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(r2Vector(-3, 1), r2Vector(2, 0.4), 21, 5);
    builder.cellModel = R2_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .05;
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = 5.0e4;
        material.poissonRatio = .3;
        material.elasticDampingRatio = 1;
        material.tearStrain = (R2OptionalReal){1, .4};
        builder.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.1);
        surface.friction = .8;
        builder.collider = surface;
    }
    builder.canSleep = !testbed->noSleep;
    bar = r2InsertSoftBody(world, &builder);

    size_t count = r2SoftBody_NumParticles(bar);
    DrivenParticle *ends = malloc(count * sizeof(*ends));
    if (!ends) {
        abort();
    }
    size_t endCount = 0;
    for (size_t i = 0; i < count; ++i) {
        R2Vector position = r2SoftBody_ParticlePosition(bar, i);
        int left = position.x < -4.99;
        int right = position.x > -1.01;
        if (left || right) {
            ends[endCount++] = (DrivenParticle){bar, (uint32_t)i, position, right};
            r2SoftBody_SetParticlePinned(bar, i, 1);
        }
    }
    R2EventCollector *events = r2NewEventCollector();

    tbCamera2(testbed, 0, 4, 40);

    tbSetWorld(testbed, world);
    R2Real t = 0;
    uint32_t previousMinPiece = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        const int useDefault = (int)tbLiveSetting(testbed, "Default minimum piece", 1, 0, 1, 1);
        const uint32_t selected =
            (uint32_t)tbLiveSetting(testbed, "Minimum piece (elements)", 3, 1, 40, 1);
        const uint32_t minPiece = useDefault ? 0 : selected;
        if (minPiece != previousMinPiece) {
            size_t bodyCount = r2SoftBodyCount(world);
            R2SoftBodyHandle *handles = malloc(bodyCount * sizeof(*handles));
            if (bodyCount && !handles) {
                abort();
            }
            bodyCount = r2SoftBodyHandles(world, handles, bodyCount);
            for (size_t i = 0; i < bodyCount; ++i) {
                R2SoftBodyMaterial material = r2SoftBody_Material(handles[i]);
                material.minPiece = (R2OptionalU32){minPiece != 0, minPiece};
                r2SoftBody_SetMaterial(handles[i], &material);
            }
            free(handles);
            previousMinPiece = minPiece;
        }
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            t += dt;
            /* Move the right clamp after one second, stopping at twice the bar's length. */
            const R2Real shift = fmin(fmax(t - 1, 0) * .5, 4);
            for (size_t i = 0; i < endCount; ++i) {
                if (ends[i].right) {
                    r2SoftBody_SetParticleKinematicTarget(
                        ends[i].body, ends[i].index,
                        r2VectorAdd(ends[i].rest, r2Vector(shift, 0)));
                }
            }
            r2EventCollector_Clear(events);
            r2Step(world, NULL, events);
            followTears(events, ends, endCount);
        }
    }
    free(ends);
    r2FreeEventCollector(events);
    r2FreeWorld(world);
}
