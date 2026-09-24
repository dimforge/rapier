/* Port of examples3d/stress_tests/soft_fem_beams3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#ifdef RAPIER_FEM

void tbStressTestsSoftFemBeams3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(30, 0.1, 30));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Cantilevers bolted at their left end, stiffer on successive rows. */
    const R3Real length = 3.0, thickness = 0.3;
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            const R3Real x0 = -20.0 + col * 5.0;
            const R3Real z = -17.5 + row * 5.0;
            R3SoftBodyDesc beam = r3CuboidSoftBodyDesc(
                r3Vector(x0 + length * 0.5, 2.5, z),
                r3Vector(length * 0.5, thickness * 0.5, thickness * 0.5), 13, 3, 3);
            beam.cellModel = R3_SOFT_CELL_COROTATIONAL;
            beam.totalMass = (R3OptionalReal){1, 12.0};
            beam.solver = R3_SOFT_SOLVER_FEM;
            {
                R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
                material.youngModulus = 1.0e6 * (1.0 + row);
                material.poissonRatio = 0.3;
                material.elasticDampingRatio = 1.0;
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
                    if (positions[i].x < x0 + 1.0e-4) {
                        beamPins[beamPinsCount++] = (uint32_t)i;
                    }
                }
                r3SoftBodyDesc_SetPinnedParticles(
                    &beam, (R3IndexView){(const uint32_t *)beamPins, beamPinsCount});

                free(positions);
            }
            r3InsertSoftBody(world, &beam);
            free(beamPins);

            /* Load dropped on the free end. */
            {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(x0 + length - 0.4, 4, z);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.3, 0.3, 0.3));
                collider.density = 30.0;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    tbCamera(testbed, 0, 28, 34, 0, 1, 0);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
#endif
