/* Port of examples2d/soft_fem2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#ifdef RAPIER_FEM

void tbSoftFem2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Compare FEM (1) with the constraint solver (0). */
    const uint32_t solvers[] = {R2_SOFT_SOLVER_FEM, R2_SOFT_SOLVER_CONSTRAINTS};
    for (size_t row = 0; row < TB_COUNT(solvers); ++row) {
        const R2Real y = 2.0 + row * 6.0;
        /* Cantilever bolted to a wall. */
        R2SoftBodyDesc beam = r2GridSoftBodyDesc(r2Vector(-6, y), r2Vector(2, 0.2), 17, 3);
        beam.cellModel = R2_SOFT_CELL_COROTATIONAL;
        beam.totalMass = (R2OptionalReal){1, 8.0};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 2.0e6;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            beam.material = material;
        }
        beam.canSleep = !testbed->noSleep;
        uint32_t *beamPins = NULL;
        {
            size_t count = r2SoftBodyDesc_ParticlePositions(&beam, NULL, 0);
            R2Vector *positions = malloc(count * sizeof(*positions));
            beamPins = malloc(count * sizeof(*beamPins));
            if (!positions || !beamPins) {
                abort();
            }
            count = r2SoftBodyDesc_ParticlePositions(&beam, positions, count);
            size_t beamPinsCount = 0;
            for (size_t i = 0; i < count; ++i) {
                if (positions[i].x < -8.0 + 1.0e-4) {
                    beamPins[beamPinsCount++] = (uint32_t)i;
                }
            }
            r2SoftBodyDesc_SetPinnedParticles(
                &beam, (R2IndexView){(const uint32_t *)beamPins, beamPinsCount});

            free(positions);
        }
        beam.solver = solvers[row];
        r2InsertSoftBody(world, &beam);
        free(beamPins);

        /* Plank pinned at both ends. */
        R2SoftBodyDesc plank = r2GridSoftBodyDesc(r2Vector(2, y), r2Vector(3, 0.25), 21, 3);
        plank.cellModel = R2_SOFT_CELL_COROTATIONAL;
        plank.totalMass = (R2OptionalReal){1, 20.0};
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 4.0e6;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            plank.material = material;
        }
        plank.canSleep = !testbed->noSleep;
        uint32_t *plankPins = NULL;
        {
            size_t count = r2SoftBodyDesc_ParticlePositions(&plank, NULL, 0);
            R2Vector *positions = malloc(count * sizeof(*positions));
            plankPins = malloc(count * sizeof(*plankPins));
            if (!positions || !plankPins) {
                abort();
            }
            count = r2SoftBodyDesc_ParticlePositions(&plank, positions, count);
            size_t plankPinsCount = 0;
            for (size_t i = 0; i < count; ++i) {
                if (fabs(positions[i].x - 2.0) > 2.9) {
                    plankPins[plankPinsCount++] = (uint32_t)i;
                }
            }
            r2SoftBodyDesc_SetPinnedParticles(
                &plank, (R2IndexView){(const uint32_t *)plankPins, plankPinsCount});

            free(positions);
        }
        plank.solver = solvers[row];
        r2InsertSoftBody(world, &plank);
        free(plankPins);

        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(2, y + 2.5);
            rigidBody.canSleep = !testbed->noSleep;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            collider.density = 20;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
        /* Neo-Hookean jelly. */
        R2SoftBodyDesc jelly = r2GridSoftBodyDesc(r2Vector(9, y + 1), r2Vector(0.8, 0.8), 5, 5);
        jelly.cellModel = R2_SOFT_CELL_NEO_HOOKEAN;
        jelly.particleMass = .1;
        jelly.solver = solvers[row];
        {
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = 2.0e4;
            material.poissonRatio = .4;
            material.elasticDampingRatio = .5;
            jelly.material = material;
        }
        jelly.canSleep = !testbed->noSleep;
        r2InsertSoftBody(world, &jelly);
    }
    tbCamera2(testbed, 1, 5, 25);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
#endif
