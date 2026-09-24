/* Port of examples3d/debug_self_intersect3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct Shot {
    R3SoftBodyHandle handle;
    R3Vector *rest;
    size_t count;
    R3Real speed;
    int onlyParticle;
} Shot;

static void reset(const Shot *shot) {
    for (size_t i = 0; i < shot->count; ++i) {
        r3SoftBody_SetParticlePosition(shot->handle, i, shot->rest[i]);
        const R3Real speed =
            shot->onlyParticle < 0 || i == (size_t)shot->onlyParticle ? shot->speed : 0;
        r3SoftBody_SetParticleVelocity(shot->handle, i, r3Vector(0, speed, 0));
    }
}

static void refire(const Shot shots[3]) {
    for (size_t i = 0; i < 3; ++i) {
        reset(&shots[i]);
    }
}

void tbDebugSelfIntersect3(Testbed *testbed) {
    R3World *world = r3NewWorld();

    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -1.5, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(8, 0.5, 4));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Slab pinned at both ends, with its top-middle vertex below the bottom face. */
    R3SoftBodyDesc builder =
        r3CuboidSoftBodyDesc(r3Vector(-3, 0.15, 0), r3Vector(1.5, 0.15, 0.15), 3, 2, 2);
    builder.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){5, 1});
    builder.selfContacts = 1;
    builder.canSleep = !testbed->noSleep;
    uint32_t *builderPins = NULL;
    {
        size_t count = r3SoftBodyDesc_ParticlePositions(&builder, NULL, 0);
        R3Vector *positions = malloc(count * sizeof(*positions));
        builderPins = malloc(count * sizeof(*builderPins));
        if (!positions || !builderPins) {
            abort();
        }
        count = r3SoftBodyDesc_ParticlePositions(&builder, positions, count);
        size_t builderPinsCount = 0;
        for (size_t i = 0; i < count; ++i) {
            if (fabs(positions[i].x + 3) > 1.4) {
                builderPins[builderPinsCount++] = (uint32_t)i;
            }
        }
        r3SoftBodyDesc_SetPinnedParticles(
            &builder, (R3IndexView){(const uint32_t *)builderPins, builderPinsCount});

        free(positions);
    }
    R3Vector slabPositions[12];
    size_t slabCount = r3SoftBodyDesc_ParticlePositions(&builder, slabPositions, 12);
    size_t captured = 0;
    R3Real closest = INFINITY;
    for (size_t i = 0; i < slabCount; ++i) {
        R3Real distance = r3VectorLength(r3VectorSub(slabPositions[i], r3Vector(-3, .3, .15)));
        if (distance < closest) {
            captured = i;
            closest = distance;
        }
    }
    R3SoftBodyHandle slab = r3InsertSoftBody(world, &builder);
    free(builderPins);

    r3SoftBody_SetParticlePosition(slab, captured, r3Vector(-2.7, -.4, 0));
    const size_t nu = 8, nv = 3;
    uint32_t endPins[] = {0, 1, 2, 21, 22, 23};
    Shot shots[3] = {0};
    R3SoftBodyHandle clothHandle;
    const R3Vector clothOrigin = r3Vector(1.5, 0, -0.25);
    R3SoftBodyDesc cloth =
        r3ClothSoftBodyDesc(clothOrigin, r3Vector(0.25, 0, 0), r3Vector(0, 0, 0.25), nu, nv);
    cloth.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    cloth.particleMass = .02;
    cloth.particleRadius = (R3OptionalReal){1, .05};
    r3SoftBodyDesc_SetPinnedParticles(&cloth,
                                     (R3IndexView){(const uint32_t *)endPins, TB_COUNT(endPins)});
    cloth.selfContacts = 1;
    cloth.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){40, 1.0};
        material.bendSoftness = (R3SpringCoefficients){40, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){40, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){40, 1.0};
        cloth.material = material;
    }
    clothHandle = r3InsertSoftBody(world, &cloth);

    for (size_t i = 4; i < nu; ++i) {
        for (size_t j = 0; j < nv; ++j) {
            r3SoftBody_SetParticlePosition(
                clothHandle, i * nv + j,
                r3VectorAdd(clothOrigin, r3Vector(.25 * (7 - i), 0.15, .25 * j)));
        }
    }
    r3SoftBody_SetParticlePosition(clothHandle, 5 * nv + 1,
                                  r3VectorAdd(clothOrigin, r3Vector(.5, -.15, .25)));
    R3SoftBodyHandle flickHandle;
    const R3Vector flickOrigin = r3Vector(-6.5, 0, -0.25);
    R3SoftBodyDesc flick =
        r3ClothSoftBodyDesc(flickOrigin, r3Vector(0.25, 0, 0), r3Vector(0, 0, 0.25), nu, nv);
    flick.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    flick.particleMass = .02;
    flick.particleRadius = (R3OptionalReal){1, .05};
    r3SoftBodyDesc_SetPinnedParticles(&flick,
                                     (R3IndexView){(const uint32_t *)endPins, TB_COUNT(endPins)});
    flick.selfContacts = 1;
    flick.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){40, 1.0};
        material.bendSoftness = (R3SpringCoefficients){40, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){40, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){40, 1.0};
        flick.material = material;
    }
    flickHandle = r3InsertSoftBody(world, &flick);

    for (size_t i = 4; i < nu; ++i) {
        for (size_t j = 0; j < nv; ++j) {
            r3SoftBody_SetParticlePosition(
                flickHandle, i * nv + j,
                r3VectorAdd(flickOrigin, r3Vector(.25 * (7 - i), 0.2, .25 * j)));
        }
    }
    shots[0].handle = flickHandle;
    shots[0].speed = -40;
    shots[0].onlyParticle = 16;
    const size_t n = 12;
    const R3Real extent = .12 * (n - 1) / 2;
    uint32_t edgePins[144];
    size_t edgeCount = 0;
    for (uint32_t k = 0; k < n * n; ++k) {
        size_t i = k / n, j = k % n;
        if (i == 0 || j == 0 || i == n - 1 || j == n - 1) {
            edgePins[edgeCount++] = k;
        }
    }
    R3SoftBodyDesc net = r3ClothSoftBodyDesc(r3Vector(5 - extent, 1.2, -extent),
                                             r3Vector(0.12, 0, 0), r3Vector(0, 0, 0.12), n, n);
    net.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    net.particleMass = .02;
    net.particleRadius = (R3OptionalReal){1, .05};
    r3SoftBodyDesc_SetPinnedParticles(&net, (R3IndexView){(const uint32_t *)edgePins, edgeCount});
    net.selfContacts = 1;
    net.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){40, 1.0};
        material.bendSoftness = (R3SpringCoefficients){40, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){40, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){40, 1.0};
        net.material = material;
    }
    r3InsertSoftBody(world, &net);

    R3SoftBodyDesc bullet = r3SphereSoftBodyDesc(r3Vector(5, 2.6, 0), .22, 1);
    bullet.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){20, 1});
    bullet.particleMass = .05;
    bullet.canSleep = !testbed->noSleep;
    shots[1].handle = r3InsertSoftBody(world, &bullet);

    shots[1].speed = -60;
    shots[1].onlyParticle = -1;
    R3SoftBodyDesc bottomStrip = r3ClothSoftBodyDesc(r3Vector(7.4, 0.3, -0.1), r3Vector(0.2, 0, 0),
                                                     r3Vector(0, 0, 0.2), 12, 2);
    bottomStrip.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    bottomStrip.particleMass = .02;
    bottomStrip.particleRadius = (R3OptionalReal){1, .05};
    bottomStrip.selfContacts = 1;
    bottomStrip.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){40, 1.0};
        material.bendSoftness = (R3SpringCoefficients){40, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){40, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){40, 1.0};
        bottomStrip.material = material;
    }
    const uint32_t stripPins[] = {0, 1, 22, 23};
    r3SoftBodyDesc_SetPinnedParticles(&bottomStrip, (R3IndexView){(const uint32_t *)stripPins, 4});
    r3InsertSoftBody(world, &bottomStrip);

    R3SoftBodyDesc topStrip = r3ClothSoftBodyDesc(r3Vector(8.4, 1.3, -1.1), r3Vector(0, 0, 0.2),
                                                  r3Vector(0.2, 0, 0), 12, 2);
    topStrip.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
    topStrip.particleMass = .02;
    topStrip.particleRadius = (R3OptionalReal){1, .05};
    topStrip.selfContacts = 1;
    topStrip.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){40, 1.0};
        material.bendSoftness = (R3SpringCoefficients){40, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){40, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){40, 1.0};
        topStrip.material = material;
    }
    shots[2].handle = r3InsertSoftBody(world, &topStrip);

    shots[2].speed = -30;
    shots[2].onlyParticle = -1;
    for (size_t i = 0; i < TB_COUNT(shots); ++i) {
        shots[i].count = r3SoftBody_NumParticles(shots[i].handle);
        shots[i].rest = malloc(shots[i].count * sizeof(*shots[i].rest));
        if (!shots[i].rest) {
            abort();
        }
        shots[i].count =
            r3SoftBody_ParticlePositions(shots[i].handle, shots[i].rest, shots[i].count);
    }
    refire(shots);
    tbCamera(testbed, 1, 3.5, 11, 1, .5, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R3Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            if (fmod(t, 4) < dt && t > 0) {
                refire(shots);
            }
            r3Step(world, NULL, NULL);
        }
    }
    for (size_t i = 0; i < TB_COUNT(shots); ++i) {
        free(shots[i].rest);
    }
    r3FreeWorld(world);
}
