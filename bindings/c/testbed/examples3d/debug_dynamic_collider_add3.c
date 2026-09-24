/* Port of examples3d/debug_dynamic_collider_add3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugDynamicColliderAdd3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle ballHandle = {0};
    R3RigidBodyHandle groundHandle = {0};
    R3Vector savedLinvel = {0};
    R3AngVector savedAngvel = {0};
    R3Pose savedPosition = {0};
    unsigned stepId = {0};
    R3ColliderHandle addedColliders[200] = {0};
    size_t numAddedColliders = {0};
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(20, 0.1, 0.4));
    collider.friction = 0.15;
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -0.1, 0);
    groundBody.canSleep = !testbed->noSleep;
    groundHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundHandle, &collider);

    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0.2, 0);
    rigidBody.linvel = r3Vector(10, 0, 0);

    collider = r3BallColliderDesc(0.1);
    collider.density = 100;
    rigidBody.canSleep = !testbed->noSleep;
    ballHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(ballHandle, &collider);
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            stepId++;
            R3ColliderDesc collider = r3BallColliderDesc(0.1 + 0.01 * stepId);
            collider.density = 100;
            addedColliders[numAddedColliders++] = r3InsertCollider(ballHandle, &collider);

            if (stepId == 51) {
                savedLinvel = r3RigidBody_Linvel(ballHandle);
                savedAngvel = r3RigidBody_Angvel(ballHandle);
                savedPosition = r3RigidBody_Position(ballHandle);
            }
            if (stepId == 100) {
                r3RigidBody_SetLinvel(ballHandle, savedLinvel, 1);
                r3RigidBody_SetAngvel(ballHandle, savedAngvel, 1);
                r3RigidBody_SetPosition(ballHandle, savedPosition, 1);
                stepId = 51;
                for (size_t i = 0; i < numAddedColliders; i++) {
                    r3RemoveCollider(addedColliders[i], 1);
                }
                numAddedColliders = 0;
            }
            R3ColliderDesc floor = r3CuboidColliderDesc(r3Vector(20, 0.1 + stepId * 0.01, 0.4));
            floor.friction = 0.15;
            addedColliders[numAddedColliders++] = r3InsertCollider(groundHandle, &floor);
        }
    }
    r3FreeWorld(world);
}
