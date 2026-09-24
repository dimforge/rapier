/* Port of examples3d/debug_sleeping_kinematic3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugSleepingKinematic3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle handles[2] = {0};
    for (int i = 0; i < 2; i++) {
        R3RigidBodyDesc rigidBody = r3KinematicVelocityBasedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, i ? 0 : 2.3, 0);
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(5, 0.5, 5));
        rigidBody.canSleep = !testbed->noSleep;
        handles[i] = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handles[i], &collider);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 10, 5, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            uint64_t step = testbed->step + 1;
            R3Real dt = r3TimeStep(world);
            R3Real time = (R3Real)step * dt;
            if (!testbed->noSleep && (step == 500 || step == 1000 || step > 1500)) {
                for (int i = 0; i < 2; i++) {
                    R3Bool sleeping = r3RigidBody_IsSleeping(handles[i]);
                    if (sleeping != 0 != (step != 1000)) {
                        snprintf(testbed->error, sizeof(testbed->error),
                                 "kinematic sleeping assertion failed at step %llu",
                                 (unsigned long long)step);
                        fprintf(stderr, "%s\n", testbed->error);
                        abort();
                    }
                }
            }
            if (step == 1000) {
                for (int i = 0; i < 2; i++) {
                    r3RigidBody_SetLinvel(handles[i], r3Vector(0, 0, 0), 1);
                }
            }
            if (step >= 500 && step < 1000) {
                if (step == 500) {
                    r3RigidBody_SetLinvel(handles[1], r3Vector(0, 0.01, 0), 1);
                }

                r3RigidBody_SetLinvel(handles[0], r3Vector(0, cos(time * 2), sin(time) * 2),
                                     1);
            }
        }
    }
    r3FreeWorld(world);
}
