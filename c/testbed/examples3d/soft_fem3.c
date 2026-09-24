/* Port of examples3d/soft_fem3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#ifdef RAPIER_FEM

void tbSoftFem3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(20, 0.1, 20));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Compare FEM (1) with the constraint solver (0). */
    const uint32_t solvers[] = {R3_SOFT_SOLVER_FEM, R3_SOFT_SOLVER_CONSTRAINTS};
    for (size_t row = 0; row < TB_COUNT(solvers); ++row) {
        const R3Real z = row == 0 ? -3.5 : 3.5;
        /* Cantilever bolted to a wall. */
        R3SoftBodyDesc beam =
            r3CuboidSoftBodyDesc(r3Vector(-3.5, 3, z), r3Vector(1.5, 0.15, 0.15), 13, 3, 3);
        beam.cellModel = R3_SOFT_CELL_COROTATIONAL;
        beam.totalMass = (R3OptionalReal){1, 12.0};
        {
            R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
            material.youngModulus = 2.0e6;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            beam.material = material;
        }
        beam.canSleep = !testbed->noSleep;
        uint32_t *beamPins = NULL;
        {
            size_t count = r3SoftBodyDesc_ParticlePositions(&beam, NULL, 0);
            R3Vector *positions = malloc(count * sizeof(*positions));
            beamPins = malloc(count * sizeof(*beamPins));
            if (!positions || !beamPins) {
                abort();
            }
            count = r3SoftBodyDesc_ParticlePositions(&beam, positions, count);
            size_t beamPinsCount = 0;
            for (size_t i = 0; i < count; ++i) {
                if (positions[i].x < -5.0 + 1.0e-4) {
                    beamPins[beamPinsCount++] = (uint32_t)i;
                }
            }
            r3SoftBodyDesc_SetPinnedParticles(
                &beam, (R3IndexView){(const uint32_t *)beamPins, beamPinsCount});

            free(positions);
        }
        beam.solver = solvers[row];
        r3InsertSoftBody(world, &beam);
        free(beamPins);

        /* Plank pinned at both ends. */
        R3SoftBodyDesc plank =
            r3CuboidSoftBodyDesc(r3Vector(1, 2, z), r3Vector(2, 0.15, 0.6), 17, 3, 5);
        plank.cellModel = R3_SOFT_CELL_COROTATIONAL;
        plank.totalMass = (R3OptionalReal){1, 20.0};
        {
            R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
            material.youngModulus = 4.0e6;
            material.poissonRatio = .3;
            material.elasticDampingRatio = 1;
            plank.material = material;
        }
        plank.canSleep = !testbed->noSleep;
        uint32_t *plankPins = NULL;
        {
            size_t count = r3SoftBodyDesc_ParticlePositions(&plank, NULL, 0);
            R3Vector *positions = malloc(count * sizeof(*positions));
            plankPins = malloc(count * sizeof(*plankPins));
            if (!positions || !plankPins) {
                abort();
            }
            count = r3SoftBodyDesc_ParticlePositions(&plank, positions, count);
            size_t plankPinsCount = 0;
            for (size_t i = 0; i < count; ++i) {
                if (fabs(positions[i].x - 1.0) > 1.9) {
                    plankPins[plankPinsCount++] = (uint32_t)i;
                }
            }
            r3SoftBodyDesc_SetPinnedParticles(
                &plank, (R3IndexView){(const uint32_t *)plankPins, plankPinsCount});

            free(positions);
        }
        plank.solver = solvers[row];
        r3InsertSoftBody(world, &plank);
        free(plankPins);

        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(1, 4.5, z);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.35, 0.35, 0.35));
            collider.density = 30;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
        /* Neo-Hookean jelly. */
        R3SoftBodyDesc jelly =
            r3CuboidSoftBodyDesc(r3Vector(6, 2, z), r3Vector(0.7, 0.7, 0.7), 5, 5, 5);
        jelly.cellModel = R3_SOFT_CELL_NEO_HOOKEAN;
        jelly.particleMass = .1;
        jelly.solver = solvers[row];
        {
            R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
            material.youngModulus = 2.0e4;
            material.poissonRatio = .4;
            material.elasticDampingRatio = .5;
            jelly.material = material;
        }
        jelly.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &jelly);
    }
    tbCamera(testbed, 2, 8, 16, .5, 2, 0);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
#endif
