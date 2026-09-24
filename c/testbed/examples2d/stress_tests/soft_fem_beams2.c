/* Port of examples2d/stress_tests/soft_fem_beams2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#ifdef RAPIER_FEM

void tbStressTestsSoftFemBeams2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(40, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Cantilevers bolted at their left end, stiffer on successive rows. */
    const R2Real length = 4.0, thickness = 0.4;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 10; ++col) {
            const R2Real x0 = -30.0 + col * 6.0;
            const R2Real y = 3.0 + row * 5.0;
            R2SoftBodyDesc beam = r2GridSoftBodyDesc(
                r2Vector(x0 + length * 0.5, y), r2Vector(length * 0.5, thickness * 0.5), 33, 5);
            beam.cellModel = R2_SOFT_CELL_COROTATIONAL;
            beam.totalMass = (R2OptionalReal){1, 8.0};
            beam.solver = R2_SOFT_SOLVER_FEM;
            {
                R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
                material.youngModulus = 1.0e6 * (1.0 + row);
                material.poissonRatio = 0.3;
                material.elasticDampingRatio = 1.0;
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
                    if (positions[i].x < x0 + 1.0e-4) {
                        beamPins[beamPinsCount++] = (uint32_t)i;
                    }
                }
                r2SoftBodyDesc_SetPinnedParticles(
                    &beam, (R2IndexView){(const uint32_t *)beamPins, beamPinsCount});

                free(positions);
            }
            r2InsertSoftBody(world, &beam);
            free(beamPins);

            /* Load dropped on the free end. */
            {
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation = r2Vector(x0 + length - 0.5, y + 2.0);
                rigidBody.canSleep = !testbed->noSleep;
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
                collider.density = 20.0;
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    tbCamera2(testbed, 0, 12, 15);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
#endif
