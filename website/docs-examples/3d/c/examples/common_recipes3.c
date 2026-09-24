#include "snippets.h"
#include <math.h>

// DOCUSAURUS: OneWayPlatform start
typedef struct OneWayPlatform {
    R3ColliderHandle platform;
} OneWayPlatform;

static int same_collider(R3ColliderHandle a, R3ColliderHandle b) {
    return a.world == b.world && a.index == b.index && a.generation == b.generation;
}

static void RAPIER_CALL one_way_platform(void *user_data, const R3ReadContext *read,
                                         R3ColliderHandle collider1, R3ColliderHandle collider2,
                                         R3ContactModificationContext *context) {
    (void)read;
    (void)collider2;
    const OneWayPlatform *hook = user_data;
    /* Keep only the contacts pushing along the local +y axis of the platform; the other
     * ones (the character arriving from below) are discarded. The normal is expressed in
     * the frame of the first collider of the pair, hence the flip. */
    R3Vector allowed_local_n1 =
        same_collider(collider1, hook->platform) ? r3Vector(0.0, 1.0, 0.0) : r3Vector(0.0, -1.0, 0.0);
    r3ContactModificationContext_UpdateAsOnewayPlatform(context, allowed_local_n1, 0.1);
}
// DOCUSAURUS: OneWayPlatform stop

// DOCUSAURUS: ConveyorBelt start
static void RAPIER_CALL conveyor_belt(void *user_data, const R3ReadContext *read,
                                      R3ColliderHandle collider1, R3ColliderHandle collider2,
                                      R3ContactModificationContext *context) {
    (void)user_data;
    (void)read;
    (void)collider1;
    (void)collider2;
    /* The belt drags the objects along the world-space z axis at 12 m/s. */
    r3ContactModificationContext_SetTangentVelocity(context, r3Vector(0.0, 0.0, 12.0));
}
// DOCUSAURUS: ConveyorBelt stop

int main(void) {
    snippets_init();
    R3World *world = r3NewWorld();

    /* A box falling on the platform, so the hooks have contacts to modify. */
    R3RigidBodyDesc box_body = r3DynamicRigidBodyDesc();
    box_body.position.translation = r3Vector(0.0, 1.5, 0.0);
    R3ColliderDesc box = r3CuboidColliderDesc(r3Vector(0.25, 0.25, 0.25));
    r3InsertCollider(r3InsertRigidBody(world, &box_body), &box);

    // DOCUSAURUS: MovingPlatform start
    R3RigidBodyDesc platform_body = r3KinematicPositionBasedRigidBodyDesc();
    platform_body.position.translation = r3Vector(0.0, 1.0, 0.0);
    R3RigidBodyHandle platform_handle = r3InsertRigidBody(world, &platform_body);
    R3ColliderDesc platform_collider_desc = r3CuboidColliderDesc(r3Vector(2.0, 0.1, 2.0));
    r3InsertCollider(platform_handle, &platform_collider_desc);

    for (int step = 0; step < 200; step++) {
        /* Setting the next position of the platform, once per timestep. */
        R3Real time = (R3Real)step * r3TimeStep(world);
        r3RigidBody_SetNextKinematicTranslation(platform_handle, r3Vector(sinf(time) * 2.0f, 1.0, 0.0));
        r3Step(world, NULL, NULL);
    }
    // DOCUSAURUS: MovingPlatform stop

    // DOCUSAURUS: Hooks start
    /* The hooks are only called for the colliders asking for them. */
    R3ColliderHandle platform_collider;
    r3RigidBody_Colliders(platform_handle, &platform_collider, 1);
    r3Collider_SetActiveHooks(platform_collider, R3_MODIFY_SOLVER_CONTACTS);

    OneWayPlatform platform = {platform_collider};
    R3PhysicsHooks hooks = {0};
    hooks.user_data = &platform;
    hooks.modify_solver_contacts_context = one_way_platform;
    r3Step(world, &hooks, NULL);
    // DOCUSAURUS: Hooks stop

    R3PhysicsHooks belt_hooks = {0};
    belt_hooks.modify_solver_contacts_context = conveyor_belt;
    r3Step(world, &belt_hooks, NULL);

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
