#include "snippets.h"

int main(void) {
    snippets_init();
    R2World *world = r2NewWorld();
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = r2Vector(0.0, 1.0);
    R2ColliderDesc collider = r2BallColliderDesc(0.5);
    r2InsertCollider(r2InsertRigidBody(world, &body), &collider);

    // DOCUSAURUS: Quarantine start
    r2Step(world, NULL, NULL);

    /* After the step, the objects neutralized by this step are known. */
    size_t num_quarantined = r2QuarantinedRigidBodies(world, NULL, 0);
    R2RigidBodyHandle *quarantined = malloc(num_quarantined * sizeof(R2RigidBodyHandle));
    num_quarantined = r2QuarantinedRigidBodies(world, quarantined, num_quarantined);

    for (size_t i = 0; i < num_quarantined; i++) {
        printf("The rigid-body {%u, %u} went non-finite and was disabled.\n", quarantined[i].index,
               quarantined[i].generation);
        /* Once the cause is fixed, the rigid-body is brought back into the simulation. */
        r2RigidBody_SetEnabled(quarantined[i], 1);
    }
    free(quarantined);
    // DOCUSAURUS: Quarantine stop

    r2Step(world, NULL, NULL);

    {
        R2RigidBodyHandle rigid_body;
        r2RigidBodyHandles(world, &rigid_body, 1);
        R2Vector sprite_translation;
        R2Real sprite_rotation;
        // DOCUSAURUS: PixelsPerMeter start
        R2Pose pose = r2RigidBody_Position(rigid_body);
        /* Scale the translation to convert from meters to pixels. */
        sprite_translation = r2VectorScale(pose.translation, 50.0);
        /* Rotation angles don't need to be scaled. */
        sprite_rotation = pose.rotation.angle;
        // DOCUSAURUS: PixelsPerMeter stop
        printf("Sprite at (%f, %f), angle %f\n", (double)sprite_translation.x, (double)sprite_translation.y,
               (double)sprite_rotation);
    }

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
