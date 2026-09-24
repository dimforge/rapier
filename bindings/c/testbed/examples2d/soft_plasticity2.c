/* Port of examples2d/soft_plasticity2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R2SoftBodyMaterial clay(R2Real young, R2Real plasticYield, R2Real plasticCreep) {
    R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
    material.youngModulus = young;
    material.poissonRatio = .35;
    material.elasticDampingRatio = 1;
    material.plasticYield = plasticYield;
    material.plasticCreep = plasticCreep;
    material.deformationDamping = 4;
    return material;
}

void tbSoftPlasticity2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(16, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(13, 2);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 2));
        collider.friction = .8;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Yield ladder: elastic to increasingly plastic, hit by identical balls. */
    const R2Real yields[] = {0, .2, .08, .02};
    for (size_t i = 0; i < TB_COUNT(yields); ++i) {
        const R2Real x = -13 + i * 2.4;
        R2SoftBodyDesc square = r2GridSoftBodyDesc(r2Vector(x, 0.75), r2Vector(0.75, 0.75), 6, 6);
        square.cellModel = R2_SOFT_CELL_COROTATIONAL;
        square.particleMass = .1;
        square.canSleep = !testbed->noSleep;
        {
            R2SoftBodyMaterial material = clay(1.0e4, yields[i], 20);
            square.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = .8;
            square.collider = surface;
        }
        r2InsertSoftBody(world, &square);

        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, 6);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2BallColliderDesc(.4);
            collider.density = 5;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Clay slab stamped by a kinematic press. */
    R2SoftBodyDesc slab = r2GridSoftBodyDesc(r2Vector(0, 0.5), r2Vector(3, 0.5), 25, 5);
    slab.cellModel = R2_SOFT_CELL_COROTATIONAL;
    slab.particleMass = .1;
    slab.canSleep = !testbed->noSleep;
    {
        R2SoftBodyMaterial material = clay(3.0e4, .02, 50);
        slab.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.12);
        surface.friction = .8;
        slab.collider = surface;
    }
    r2InsertSoftBody(world, &slab);

    const R2Vector pressRest = r2Vector(-2, 2.4);
    R2RigidBodyHandle press;
    {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = pressRest;
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
        collider.position.rotation = r2Rotation(R2_PI / 4);
        collider.friction = .5;
        press = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(press, &collider);
    }
    /* Elastic and creeping columns under their own weight. */
    for (int i = 0; i < 2; ++i) {
        const R2Real x = 5.5 + i * 1.5;
        R2SoftBodyDesc column = r2GridSoftBodyDesc(r2Vector(x, 1.2), r2Vector(0.3, 1.2), 3, 12);
        column.cellModel = R2_SOFT_CELL_COROTATIONAL;
        column.particleMass = .2;
        column.canSleep = !testbed->noSleep;
        {
            R2SoftBodyMaterial material = clay(2.0e3, i == 0 ? 0 : .04, .5);
            column.material = material;
        }
        {
            R2ColliderDesc surface = r2BallColliderDesc(.1);
            surface.friction = 1;
            column.collider = surface;
        }
        r2InsertSoftBody(world, &column);
    }
    /* Volumetric clay disks thrown at the wall. */
    const size_t n = 24;
    uint32_t indices[24][2];
    for (uint32_t i = 0; i < n; ++i) {
        indices[i][0] = i;
        indices[i][1] = (i + 1) % n;
    }
    for (int i = 0; i < 3; ++i) {
        const R2Vector center = r2Vector(11.5 - i * 1.5, 1 + i * .5);
        R2Vector vertices[24];
        for (size_t k = 0; k < n; ++k) {
            const R2Real a = (R2Real)k / n * 2 * R2_PI;
            vertices[k] = r2VectorAdd(center, r2Vector(.5 * cos(a), .5 * sin(a)));
        }

        R2VolumeMeshParameters ballMeshing = r2NewVolumeMeshParameters(.15);
        R2SoftBodyDesc ball = r2VolumetricSoftBodyDesc(
            (R2VectorView){vertices, n}, (R2SurfaceElementView){(const R2Edge *)&indices[0][0], n},
            ballMeshing);
        {
            ball.cellModel = R2_SOFT_CELL_COROTATIONAL;
            R2SoftBodyMaterial material = clay(1.0e4, .03, 60);
            ball.material = material;
            ball.particleMass = .05;
            ball.canSleep = !testbed->noSleep;
            {
                R2ColliderDesc surface = r2BallColliderDesc(.075);
                surface.friction = .8;
                ball.collider = surface;
            }
            R2SoftBodyHandle handle = r2InsertSoftBody(world, &ball);

            size_t count = r2SoftBody_NumParticles(handle);
            for (size_t k = 0; k < count; ++k) {
                r2SoftBody_SetParticleVelocity(handle, k, r2Vector(10, 1));
            }
        }
    }

    tbCamera2(testbed, 0, 3, 30);

    tbSetWorld(testbed, world);
    R2Real t = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            t += dt;
            /* One second down, one second up, then move to the next stamp. */
            const R2Real period = 3;
            const R2Real cycle = floor(t / period);
            const R2Real phase = t - cycle * period;
            const R2Real x = pressRest.x + fmod(cycle, 5);
            const R2Real depth = 1;
            const R2Real y = pressRest.y - (phase < 1   ? depth * phase
                                            : phase < 2 ? depth * (2 - phase)
                                                        : 0);

            r2RigidBody_SetNextKinematicTranslation(press, r2Vector(x, y));
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
