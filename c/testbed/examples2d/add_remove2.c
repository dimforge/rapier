/* Port of examples2d/add_remove2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "example_math.h"

void tbAddRemove2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    const R2Real rad = 0.5;
    const R2Vector positions[] = {r2Vector(5.0, -1.0), r2Vector(-5.0, -1.0)};

    R2RigidBodyHandle platformHandles[2] = {0};

    for (size_t i = 0; i < TB_COUNT(positions); i++) {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = positions[i];
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(rad * 10.0, rad));
        platformHandles[i] = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(platformHandles[i], &collider);
    }

    /* Set up the viewer. */
    tbCamera2(testbed, 0.0, 0.0, 20.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);

            const uint64_t stepId = testbed->step + 1;
            R2Real dt = r2TimeStep(world);

            const R2Real rot = -(R2Real)stepId * dt;
            for (size_t i = 0; i < TB_COUNT(platformHandles); i++) {
                r2RigidBody_SetNextKinematicRotation(platformHandles[i], r2Rotation(rot));
            }

            if (stepId % 10 == 0) {
                const R2Real rad = 0.5;
                const R2Real x = exampleRandom(&testbed->randomState) * 10.0 - 5.0;
                const R2Real y = exampleRandom(&testbed->randomState) * 10.0 + 10.0;
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation = r2Vector(x, y);
                rigidBody.canSleep = !testbed->noSleep;

                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(rad, rad));
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }

            /* Copy the handles before removing bodies from their set. */
            size_t numBodies = r2RigidBodyHandles(world, NULL, 0);
            R2RigidBodyHandle *handles = malloc(numBodies * sizeof(*handles));
            if (!handles && numBodies != 0) {
                abort();
            }
            numBodies = r2RigidBodyHandles(world, handles, numBodies);

            for (size_t i = 0; i < numBodies; i++) {
                R2Vector position = r2RigidBody_Translation(handles[i]);
                if (position.y < -10.0) {
                    r2RemoveRigidBody(handles[i], 1);
                }
            }
            free(handles);
        }
    }
    r2FreeWorld(world);
}
