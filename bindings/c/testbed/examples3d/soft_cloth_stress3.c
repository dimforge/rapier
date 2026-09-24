/* Port of examples3d/soft_cloth_stress3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R3SoftBodyDesc cloth(R3Vector origin, R3Vector du, R3Vector dv, size_t nu, size_t nv) {
    R3SoftBodyDesc builder = r3ClothSoftBodyDesc(origin, du, dv, nu, nv);
    builder.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    builder.particleMass = .02;
    builder.particleRadius = (R3OptionalReal){1, .05};
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){30, 1.0};
    material.bendSoftness = (R3SpringCoefficients){30, 1.0};
    material.volumeSoftness = (R3SpringCoefficients){30, 1.0};
    material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1.0};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    builder.material = material;
    {
        R3ColliderDesc surface = r3BallColliderDesc(.05);
        surface.friction = .5;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftClothStress3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.5, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(30, 0.5, 30));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Sheets slide down a slope into a stopper. */
    const R3Real slope = .5;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-6, 2, 0);
        rigidBody.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), -slope);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(5, 0.2, 4));
        collider.friction = .4;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-0.6, 0.6, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.6, 4));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    const R3Rotation rot = r3RotationFromAxisAngle(r3Vector(0, 0, 1), -slope);
    for (int k = 0; k < 6; ++k) {
        const R3Vector origin = r3VectorAdd(
            r3Vector(-6, 2, 0),
            r3RotationTransformVector(rot, r3Vector(-3.5 + k * .3, .3 + k * .12, -1.5)));
        {
            R3SoftBodyDesc sheet =
                cloth(origin, r3RotationTransformVector(rot, r3Vector(.15, 0, 0)),
                      r3Vector(0, 0, .15), 21, 21);
            sheet.canSleep = !testbed->noSleep;
            r3InsertSoftBody(world, &sheet);
        }
    }
    /* Banner pinned along both ends. */
    const size_t nu = 31, nv = 13;
    uint32_t right[13], bannerPinned[26];
    for (size_t j = 0; j < nv; ++j) {
        right[j] = (uint32_t)((nu - 1) * nv + j);
        bannerPinned[j] = (uint32_t)j;
        bannerPinned[nv + j] = right[j];
    }
    R3SoftBodyHandle banner;
    {
        R3SoftBodyDesc body =
            cloth(r3Vector(2, 4, -4), r3Vector(.15, 0, 0), r3Vector(0, -.15, 0), nu, nv);
        r3SoftBodyDesc_SetPinnedParticles(&body, (R3IndexView){(const uint32_t *)bannerPinned, 26});
        body.canSleep = !testbed->noSleep;
        banner = r3InsertSoftBody(world, &body);
    }

    R3Vector bannerRightRest[13];
    for (size_t j = 0; j < nv; ++j) {
        bannerRightRest[j] = r3SoftBody_ParticlePosition(banner, right[j]);
    }
    /* Strip pinned top and bottom; its bottom clamp rotates. */
    const size_t su = 11, sv = 41;
    uint32_t bottom[11], stripPinned[22];
    for (size_t i = 0; i < su; ++i) {
        bottom[i] = (uint32_t)(i * sv + sv - 1);
        stripPinned[i] = (uint32_t)(i * sv);
        stripPinned[su + i] = bottom[i];
    }
    R3SoftBodyHandle strip;
    {
        R3SoftBodyDesc body =
            cloth(r3Vector(9, 6.5, -.75), r3Vector(0, 0, .15), r3Vector(0, -.15, 0), su, sv);
        r3SoftBodyDesc_SetPinnedParticles(&body, (R3IndexView){(const uint32_t *)stripPinned, 22});
        body.selfContacts = 1;
        body.canSleep = !testbed->noSleep;
        strip = r3InsertSoftBody(world, &body);
    }
    const R3Vector stripAxis = r3Vector(9, 0, 0);

    R3Vector stripBottomRest[11];
    for (size_t i = 0; i < su; ++i) {
        stripBottomRest[i] = r3SoftBody_ParticlePosition(strip, bottom[i]);
    }

    tbCamera(testbed, 2, 8, 18, 1.5, 3, 0);

    tbSetWorld(testbed, world);
    R3Real t = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            /* Stretch/release the banner and twist the strip. */
            const R3Real stretch = 1.5 * (1 - cos(.5 * t));

            for (size_t j = 0; j < nv; ++j) {
                r3SoftBody_SetParticleKinematicTarget(
                    banner, right[j],
                    r3VectorAdd(bannerRightRest[j], r3Vector(stretch, 0, 0)));
            }
            const R3Rotation twist = r3RotationFromAxisAngle(r3Vector(0, 1, 0), .8 * t);

            for (size_t i = 0; i < su; ++i) {
                const R3Vector target = r3VectorAdd(
                    stripAxis,
                    r3RotationTransformVector(twist, r3VectorSub(stripBottomRest[i], stripAxis)));
                r3SoftBody_SetParticleKinematicTarget(strip, bottom[i], target);
            }
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
