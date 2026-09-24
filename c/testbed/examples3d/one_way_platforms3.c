/* Port of examples3d/one_way_platforms3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct OneWayPlatformHook {
    R3ColliderHandle platform1, platform2;
} OneWayPlatformHook;

static int sameCollider(R3ColliderHandle a, R3ColliderHandle b) {
    return a.world == b.world && a.index == b.index && a.generation == b.generation;
}

static void RAPIER_CALL modifySolverContacts(void *userData, const R3ReadContext *read,
                                             R3ColliderHandle collider1, R3ColliderHandle collider2,
                                             R3ContactModificationContext *context) {
    (void)read;
    const OneWayPlatformHook *hook = userData;
    R3Vector allowedLocalN1 = r3Vector(0, 0, 0);
    /* Flip the allowed normal when the platform is collider2. */
    if (sameCollider(collider1, hook->platform1)) {
        allowedLocalN1 = r3Vector(0, 1, 0);
    } else if (sameCollider(collider2, hook->platform1)) {
        allowedLocalN1 = r3Vector(0, -1, 0);
    }
    if (sameCollider(collider1, hook->platform2)) {
        allowedLocalN1 = r3Vector(0, -1, 0);
    } else if (sameCollider(collider2, hook->platform2)) {
        allowedLocalN1 = r3Vector(0, 1, 0);
    }
    r3ContactModificationContext_UpdateAsOnewayPlatform(context, allowedLocalN1, .1);
    const R3Real tangentVelocity =
        sameCollider(collider1, hook->platform1) || sameCollider(collider2, hook->platform2) ? -12
                                                                                             : 12;
    r3ContactModificationContext_SetTangentVelocity(context, r3Vector(0, 0, tangentVelocity));
}

void tbOneWayPlatforms3(Testbed *testbed) {
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(9, 0.5, 25));
    collider.position.translation = r3Vector(0, 2, 30);
    collider.activeHooks = R3_MODIFY_SOLVER_CONTACTS;
    R3RigidBodyHandle handle;
    OneWayPlatformHook platformHook;
    handle = r3InsertRigidBody(world, &rigidBody);
    platformHook.platform1 = r3InsertCollider(handle, &collider);
    collider = r3CuboidColliderDesc(r3Vector(9, 0.5, 25));
    collider.position.translation = r3Vector(0, -2, -30);
    collider.activeHooks = R3_MODIFY_SOLVER_CONTACTS;
    platformHook.platform2 = r3InsertCollider(handle, &collider);
    R3PhysicsHooks physicsHooks = {0};
    physicsHooks.user_data = &platformHook;
    physicsHooks.modify_solver_contacts_context = modifySolverContacts;
    tbCamera(testbed, 100, 0, 0, 0, 0, 0);

    tbSetWorld(testbed, world);
    size_t stepId = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, &physicsHooks, NULL);
            ++stepId;
            size_t bodyCount = r3RigidBodyCount(world);
            /* Spawn cubes periodically and reverse gravity below the lower platform. */
            if (stepId % 200 == 0 && bodyCount <= 7) {
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(0, 6, 20);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 2, 1.5));
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
            size_t activeCount = r3ActiveRigidBodies(world, NULL, 0);
            R3RigidBodyHandle *active = malloc(activeCount * sizeof(*active));
            if (activeCount && !active) {
                abort();
            }
            activeCount = r3ActiveRigidBodies(world, active, activeCount);
            for (size_t i = 0; i < activeCount; ++i) {
                R3Vector position = r3RigidBody_Translation(active[i]);
                if (position.y > 1) {
                    r3RigidBody_SetGravityScale(active[i], 1, 0);
                } else if (position.y < -1) {
                    r3RigidBody_SetGravityScale(active[i], -1, 0);
                }
            }
            free(active);
        }
    }
    r3FreeWorld(world);
}
