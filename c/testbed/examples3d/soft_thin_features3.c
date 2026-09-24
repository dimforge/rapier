/* Port of examples3d/soft_thin_features3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R3SoftBodyDesc jelly(R3Vector center, R3Real half, size_t n, R3Real young) {
    R3SoftBodyDesc builder = r3CuboidSoftBodyDesc(center, r3Vector(half, half, half), n, n, n);
    builder.cellModel = R3_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .1;
    builder.particleRadius = (R3OptionalReal){1, .06};
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = young;
        material.poissonRatio = .4;
        material.elasticDampingRatio = .5;
        builder.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.06);
        surface.friction = .7;
        builder.collider = surface;
    }
    return builder;
}

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
        surface.friction = .6;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftThinFeatures3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.5, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(30, 0.5, 30));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Bed of nails. */
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            {
                R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
                rigidBody.position.translation = r3Vector(-8 + i * .45, 0.6, -2 + j * .45);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3CapsuleYColliderDesc(.6, .03);
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    {
        R3SoftBodyDesc body = jelly(r3Vector(-7, 3.5, -1), .75, 5, 2.0e4);
        body.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &body);
    }
    {
        R3SoftBodyDesc balloon = r3SphereSoftBodyDesc(r3Vector(-4.8, 3.5, 1.2), .7, 2);
        balloon.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){15, 1});
        balloon.volumeFactor = 1.2;
        balloon.particleMass = .03;
        balloon.particleRadius = (R3OptionalReal){1, .06};
        {
            R3ColliderDesc surface = r3BallColliderDesc(.06);
            surface.friction = .6;
            balloon.collider = surface;
        }
        balloon.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &balloon);
    }
    {
        R3SoftBodyDesc sheet =
            cloth(r3Vector(-8.5, 5.5, -2.5), r3Vector(.12, 0, 0), r3Vector(0, 0, .12), 30, 30);
        sheet.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &sheet);
    }
    /* Knife edges. */
    for (int i = 0; i < 4; ++i) {
        {
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(-1.5, 0.75, -3 + i * .8);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1.2, 0.75, 0.015));
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    {
        R3SoftBodyDesc body = jelly(r3Vector(-1.5, 3, -1.8), .9, 5, 5.0e3);
        body.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &body);
    }
    /* Wire grid. */
    for (int i = 0; i < 7; ++i) {
        const R3Real t = -1.8 + i * .6;
        {
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(t, 2.5, 5);
            rigidBody.position.rotation = r3RotationFromAxisAngle(r3Vector(1, 0, 0), R3_PI / 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CapsuleYColliderDesc(1.8, .02);
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
        {
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(0, 2.5, 5 + t);
            rigidBody.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI / 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CapsuleYColliderDesc(1.8, .02);
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    {
        R3SoftBodyDesc sheet =
            cloth(r3Vector(-1.5, 4.5, 3.5), r3Vector(.12, 0, 0), r3Vector(0, 0, .12), 26, 26);
        sheet.selfContacts = 1;
        sheet.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &sheet);
    }
    /* Needle rain on a trampoline and a jelly block. */
    const size_t n = 26;
    uint32_t pinned[26 * 26];
    size_t pinnedCount = 0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (i == 0 || j == 0 || i == n - 1 || j == n - 1) {
                pinned[pinnedCount++] = (uint32_t)(i * n + j);
            }
        }
    }
    {
        R3SoftBodyDesc trampoline =
            cloth(r3Vector(3, 2.5, -3.5), r3Vector(.12, 0, 0), r3Vector(0, 0, .12), n, n);
        r3SoftBodyDesc_SetPinnedParticles(&trampoline,
                                         (R3IndexView){(const uint32_t *)pinned, pinnedCount});
        trampoline.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40, 1});
        trampoline.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &trampoline);
    }
    {
        R3SoftBodyDesc body = jelly(r3Vector(4.5, .9, 3), .9, 5, 3.0e4);
        body.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &body);
    }
    const R3Real locations[][3] = {{4.5, -2, 6}, {4.5, 3, 5}};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (size_t k = 0; k < TB_COUNT(locations); ++k) {
                const R3Real cx = locations[k][0], cz = locations[k][1], h = locations[k][2];
                const R3Vector rotation = r3Vector(.3 * i, 0, .2 * j);
                const R3Real angle = sqrt(rotation.x * rotation.x + rotation.z * rotation.z);
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r3Vector(cx - 1 + i * .4, h + (i + j) * .3, cz - 1 + j * .4);
                    rigidBody.position.rotation = r3RotationFromAxisAngle(rotation, angle);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3CapsuleYColliderDesc(.5, .02);
                    collider.density = 3;
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    /* Thin plates and a long rod. */
    {
        R3SoftBodyDesc body = jelly(r3Vector(9, .9, -2), .9, 5, 1.0e4);
        body.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &body);
    }
    for (int i = 0; i < 4; ++i) {
        const R3Vector rotation = r3Vector(.1 * i, .5 * i, .05);
        const R3Real angle =
            sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z);
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(9, 3.5 + i * .5, -2);
            rigidBody.position.rotation = r3RotationFromAxisAngle(rotation, angle);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.8, 0.015, 0.8));
            collider.density = 1;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    {
        R3SoftBodyDesc balloon = r3SphereSoftBodyDesc(r3Vector(9, 1, 3), .9, 2);
        balloon.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){15, 1});
        balloon.volumeFactor = 1.2;
        balloon.particleMass = .03;
        balloon.particleRadius = (R3OptionalReal){1, .06};
        {
            R3ColliderDesc surface = r3BallColliderDesc(.06);
            surface.friction = .6;
            balloon.collider = surface;
        }
        balloon.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &balloon);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(9, 3.5, 3);
        rigidBody.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI / 2);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CapsuleYColliderDesc(2.5, .03);
        collider.density = 2;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera(testbed, 2, 10, 18, 1, 1.5, 1);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
