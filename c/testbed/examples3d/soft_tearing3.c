/* Port of examples3d/soft_tearing3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct DrivenParticle {
    R3SoftBodyHandle body;
    uint32_t index;
    R3Vector rest;
    int right;
} DrivenParticle;

/* Follow driven particle indices through compaction and newly split bodies. */
static void followTears(R3EventCollector *events, DrivenParticle *ends, size_t endCount) {
    size_t count = r3EventCollector_TearEventCount(events);
    for (size_t i = 0; i < count; ++i) {
        R3SoftBodyTearEvent *event = r3EventCollector_TearEvent(events, i);
        R3SoftBodyHandle origin = r3SoftBodyTearEvent_SoftBody(event);
        for (size_t j = 0; j < endCount; ++j) {
            if (ends[j].body.index == origin.index &&
                ends[j].body.generation == origin.generation) {
                R3Bool found;
                R3SoftBodyHandle destination;
                uint32_t index;
                R3OptionalParticleDestination softBodyTearEventTryParticleDestinationResult =
                    r3SoftBodyTearEvent_TryParticleDestination(event, ends[j].index);
                destination = softBodyTearEventTryParticleDestinationResult.body;
                index = softBodyTearEventTryParticleDestinationResult.index;
                found = softBodyTearEventTryParticleDestinationResult.found;
                if (found) {
                    ends[j].body = destination;
                    ends[j].index = index;
                }
            }
        }
        r3FreeSoftBodyTearEvent(event);
    }
}

void tbSoftTearing3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(30, 0.1, 30));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Sheet. */
    {
        const size_t nu = 40, nv = 40;
        uint32_t pinned[1600];
        size_t pinnedCount = 0;
        for (size_t i = 0; i < nu; ++i) {
            for (size_t j = 0; j < nv; ++j) {
                if (i == 0 || j == 0 || i == nu - 1 || j == nv - 1) {
                    pinned[pinnedCount++] = (uint32_t)(i * nv + j);
                }
            }
        }
        R3SoftBodyDesc sheet = r3ClothSoftBodyDesc(r3Vector(-6, 3, -2), r3Vector(0.1, 0, 0),
                                                   r3Vector(0, 0, 0.1), nu, nv);
        r3SoftBodyDesc_SetPinnedParticles(&sheet,
                                         (R3IndexView){(const uint32_t *)pinned, pinnedCount});
        sheet.particleMass = .02;
        sheet.canSleep = !testbed->noSleep;
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){100, 1.0};
        material.bendSoftness = (R3SpringCoefficients){100, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){100, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){100, 1.0};
        material.bendSoftness = (R3SpringCoefficients){3, 1};
        material.tearStrain = (R3OptionalReal){1, .4};
        sheet.material = material;
        {
            R3ColliderDesc surface = r3BallColliderDesc(.05);
            surface.friction = .8;
            sheet.collider = surface;
        }
        r3InsertSoftBody(world, &sheet);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-4, 6, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(.6);
        collider.density = 30;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Curtain. */
    {
        const size_t nu = 50, nv = 40;
        uint32_t pinned[2000];
        size_t pinnedCount = 0;
        for (size_t i = 0; i < nu; ++i) {
            for (size_t j = 0; j < nv; ++j) {
                if (j == 0) {
                    pinned[pinnedCount++] = (uint32_t)(i * nv + j);
                }
            }
        }
        R3SoftBodyDesc curtain = r3ClothSoftBodyDesc(r3Vector(0, 4.5, 4), r3Vector(0.1, 0, 0),
                                                     r3Vector(0, -0.1, 0), nu, nv);
        r3SoftBodyDesc_SetPinnedParticles(&curtain,
                                         (R3IndexView){(const uint32_t *)pinned, pinnedCount});
        curtain.particleMass = .02;
        curtain.canSleep = !testbed->noSleep;
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){100, 1.0};
        material.bendSoftness = (R3SpringCoefficients){100, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){100, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){100, 1.0};
        material.bendSoftness = (R3SpringCoefficients){3, 1};
        material.tearStrain = (R3OptionalReal){1, .2};
        curtain.material = material;
        r3InsertSoftBody(world, &curtain);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(2.5, 2.5, 12);
        rigidBody.linvel = r3Vector(0, 0, -25);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.3, 0.3, 0.3));
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(.5, .5, .5), sqrt(.75));
        collider.density = 20;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Jelly bar pulled apart by its two pinned ends. */
    R3SoftBodyHandle bar;
    R3SoftBodyDesc builder =
        r3CuboidSoftBodyDesc(r3Vector(3, 1, -4), r3Vector(2, 0.4, 0.4), 21, 5, 5);
    builder.cellModel = R3_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .05;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = 5.0e4;
        material.poissonRatio = .3;
        material.elasticDampingRatio = 1;
        material.tearStrain = (R3OptionalReal){1, .4};
        builder.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.1);
        surface.friction = .8;
        builder.collider = surface;
    }
    builder.canSleep = !testbed->noSleep;
    bar = r3InsertSoftBody(world, &builder);

    size_t count = r3SoftBody_NumParticles(bar);
    DrivenParticle *ends = malloc(count * sizeof(*ends));
    if (!ends) {
        abort();
    }
    size_t endCount = 0;
    for (size_t i = 0; i < count; ++i) {
        R3Vector position = r3SoftBody_ParticlePosition(bar, i);
        int left = position.x < 1.01;
        int right = position.x > 4.99;
        if (left || right) {
            ends[endCount++] = (DrivenParticle){bar, (uint32_t)i, position, right};
            r3SoftBody_SetParticlePinned(bar, i, 1);
        }
    }
    R3EventCollector *events = r3NewEventCollector();

    tbCamera(testbed, 9, 8, 16, 0, 1.5, 1);

    tbSetWorld(testbed, world);
    R3Real t = 0;
    uint32_t previousMinPiece = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        const int useDefault = (int)tbLiveSetting(testbed, "Default minimum piece", 1, 0, 1, 1);
        const uint32_t selected =
            (uint32_t)tbLiveSetting(testbed, "Minimum piece (elements)", 6, 1, 40, 1);
        const uint32_t minPiece = useDefault ? 0 : selected;
        if (minPiece != previousMinPiece) {
            size_t bodyCount = r3SoftBodyCount(world);
            R3SoftBodyHandle *handles = malloc(bodyCount * sizeof(*handles));
            if (bodyCount && !handles) {
                abort();
            }
            bodyCount = r3SoftBodyHandles(world, handles, bodyCount);
            for (size_t i = 0; i < bodyCount; ++i) {
                R3SoftBodyMaterial material = r3SoftBody_Material(handles[i]);
                material.minPiece = (R3OptionalU32){minPiece != 0, minPiece};
                r3SoftBody_SetMaterial(handles[i], &material);
            }
            free(handles);
            previousMinPiece = minPiece;
        }
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            /* Move the right clamp after one second, stopping at twice the bar's length. */
            const R3Real shift = fmin(fmax(t - 1, 0) * .5, 4);
            for (size_t i = 0; i < endCount; ++i) {
                if (ends[i].right) {
                    r3SoftBody_SetParticleKinematicTarget(
                        ends[i].body, ends[i].index,
                        r3VectorAdd(ends[i].rest, r3Vector(shift, 0, 0)));
                }
            }
            r3EventCollector_Clear(events);
            r3Step(world, NULL, events);
            followTears(events, ends, endCount);
        }
    }
    free(ends);
    r3FreeEventCollector(events);
    r3FreeWorld(world);
}
