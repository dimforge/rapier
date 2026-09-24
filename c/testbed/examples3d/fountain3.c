/* Port of examples3d/fountain3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbFountain3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -2.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(40, 2.1, 40));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    /* Set up the viewer. */
    tbCamera(testbed, 30, 4, 30, 0, 1, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            unsigned k = (unsigned)((testbed->step + 1) % 3);
            R3ColliderDesc collider;
            if (k == 0) {
                collider = r3RoundCylinderColliderDesc(0.5, 0.5, 0.05);
            } else {
                if (k == 1) {
                    collider = r3ConeColliderDesc(0.5, 0.5);
                } else {
                    collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
                }
            }
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(0, 10, 0);
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);

            size_t n = r3RigidBodyCount(world);
            if (n > 2001) {
                R3RigidBodyHandle *handle = malloc(n * sizeof(*handle));
                if (!handle) {
                    abort();
                }
                n = r3RigidBodyHandles(world, handle, n);
                R3Real largest = -1;
                R3RigidBodyHandle remove = {NULL, UINT32_MAX, UINT32_MAX};
                for (size_t i = 0; i < n; i++) {
                    R3Bool dynamic = r3RigidBody_IsDynamic(handle[i]);
                    if (dynamic) {
                        R3Vector position = r3RigidBody_Translation(handle[i]);
                        R3Real distance = (R3Real)(fabs(position.x) + fabs(position.z));
                        if (distance > largest) {
                            largest = distance;
                            remove = handle[i];
                        }
                    }
                }
                free(handle);
                if (remove.index != UINT32_MAX) {
                    r3RemoveRigidBody(remove, 1);
                }
            }
        }
    }
    r3FreeWorld(world);
}
