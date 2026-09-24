/* Port of examples2d/one_way_platforms2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct OneWayPlatformHook {
    R2ColliderHandle platform1, platform2;
} OneWayPlatformHook;

static int sameCollider(R2ColliderHandle a, R2ColliderHandle b) {
    return a.world == b.world && a.index == b.index && a.generation == b.generation;
}

static void RAPIER_CALL modifySolverContacts(void *userData, const R2ReadContext *read,
                                             R2ColliderHandle collider1, R2ColliderHandle collider2,
                                             R2ContactModificationContext *context) {
    (void)read;
    const OneWayPlatformHook *hook = userData;
    R2Vector allowedLocalN1 = r2Vector(0, 0);
    /* Flip the allowed normal when the platform is collider2. */
    if (sameCollider(collider1, hook->platform1)) {
        allowedLocalN1 = r2Vector(0, 1);
    } else if (sameCollider(collider2, hook->platform1)) {
        allowedLocalN1 = r2Vector(0, -1);
    }
    if (sameCollider(collider1, hook->platform2)) {
        allowedLocalN1 = r2Vector(0, -1);
    } else if (sameCollider(collider2, hook->platform2)) {
        allowedLocalN1 = r2Vector(0, 1);
    }
    r2ContactModificationContext_UpdateAsOnewayPlatform(context, allowedLocalN1, .1);
    const R2Real tangentVelocity =
        sameCollider(collider1, hook->platform1) || sameCollider(collider2, hook->platform2) ? -12
                                                                                             : 12;
    r2ContactModificationContext_SetTangentVelocity(context, r2Vector(tangentVelocity, 0));
}

void tbOneWayPlatforms2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(25, 0.5));
    collider.position.translation = r2Vector(30, 2);
    collider.activeHooks = R2_MODIFY_SOLVER_CONTACTS;
    R2RigidBodyHandle handle;
    OneWayPlatformHook platformHook;
    handle = r2InsertRigidBody(world, &rigidBody);
    platformHook.platform1 = r2InsertCollider(handle, &collider);
    collider = r2CuboidColliderDesc(r2Vector(25, 0.5));
    collider.position.translation = r2Vector(-30, -2);
    collider.activeHooks = R2_MODIFY_SOLVER_CONTACTS;
    platformHook.platform2 = r2InsertCollider(handle, &collider);
    R2PhysicsHooks physicsHooks = {0};
    physicsHooks.user_data = &platformHook;
    physicsHooks.modify_solver_contacts_context = modifySolverContacts;
    tbCamera2(testbed, 0, 0, 20);

    tbSetWorld(testbed, world);
    size_t stepId = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, &physicsHooks, NULL);
            ++stepId;
            size_t bodyCount = r2RigidBodyCount(world);
            /* Spawn cubes periodically and reverse gravity below the lower platform. */
            if (stepId % 200 == 0 && bodyCount <= 7) {
                {
                    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                    rigidBody.position.translation = r2Vector(20, 10);
                    rigidBody.canSleep = !testbed->noSleep;
                    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1.5, 2));
                    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                    r2InsertCollider(rigidBodyHandle, &collider);
                }
            }
            size_t activeCount = r2ActiveRigidBodies(world, NULL, 0);
            R2RigidBodyHandle *active = malloc(activeCount * sizeof(*active));
            if (activeCount && !active) {
                abort();
            }
            activeCount = r2ActiveRigidBodies(world, active, activeCount);
            for (size_t i = 0; i < activeCount; ++i) {
                R2Vector position = r2RigidBody_Translation(active[i]);
                if (position.y > 1) {
                    r2RigidBody_SetGravityScale(active[i], 1, 0);
                } else if (position.y < -1) {
                    r2RigidBody_SetGravityScale(active[i], -1, 0);
                }
            }
            free(active);
        }
    }
    r2FreeWorld(world);
}
