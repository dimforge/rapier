/* Port of examples3d/stress_tests/many_kinematics3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "example_math.h"

static R3Real bounce(R3Real p, R3Real v) {
    return (v > 0 && p > 105) || (v < 0 && p < -105) ? -v : v;
}

void tbStressTestsManyKinematics3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 30; i++) {
        for (int j = 0; j < 30; j++) {
            for (int k = 0; k < 30; k++) {
                R3RigidBodyDesc rigidBody = r3KinematicVelocityBasedRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 7 - 105, j * 7 - 105, k * 7 - 105);
                R3Real vx = (exampleRandom(&testbed->randomState) - 0.5) * 30;
                R3Real vy = (exampleRandom(&testbed->randomState) - 0.5) * 30;
                R3Real vz = (exampleRandom(&testbed->randomState) - 0.5) * 30;
                rigidBody.linvel = r3Vector(vx, vy, vz);
                R3ColliderDesc collider = r3BallColliderDesc(1);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            size_t n = r3RigidBodyHandles(world, NULL, 0);
            R3RigidBodyHandle *handle = malloc(n * sizeof(*handle));
            if (!handle) {
                abort();
            }
            n = r3RigidBodyHandles(world, handle, n);
            for (size_t i = 0; i < n; i++) {
                R3Vector position;
                R3Vector translation;
                position = r3RigidBody_Translation(handle[i]);
                translation = r3RigidBody_Linvel(handle[i]);
                r3RigidBody_SetLinvel(handle[i],
                                     r3Vector(bounce(position.x, translation.x),
                                              bounce(position.y, translation.y),
                                              bounce(position.z, translation.z)),
                                     0);
            }
            free(handle);
        }
    }
    r3FreeWorld(world);
}
