#include "snippets.h"
#include <string.h>

int main(void) {
    snippets_init();

    // DOCUSAURUS: BuildProfile start
    /* Warn if the Rapier library loaded by the application isn't optimized. */
    if (strcmp(r3BuildProfile(), "release") != 0) {
        fprintf(stderr, "Rapier is built in debug mode: the simulation will be very slow.\n");
    }
    // DOCUSAURUS: BuildProfile stop

    // DOCUSAURUS: FeatureDefines start
    /* Checks the version, the dimension, the precision, and the RAPIER_FEM/RAPIER_ROBOTICS definitions,
     * which change the layout of some structures (e.g. R3IntegrationParameters). */
    if (r3CheckAbi(R3_ABI_VERSION, R3_DIMENSION, sizeof(R3Real), sizeof(R3Vector), sizeof(R3Pose),
                   R3_ABI_FEATURES) != R3_OK) {
        fprintf(stderr, "Incompatible Rapier library: %s\n", r3LastError());
        exit(EXIT_FAILURE);
    }
    // DOCUSAURUS: FeatureDefines stop

    R3World *world = r3NewWorld();

    {
        // DOCUSAURUS: Descriptions start
        /* WRONG version: every field is zero, so the rigid-body would be disabled, with a zero gravity
         * scale. In 3D, its insertion even fails because a zero quaternion isn't a valid rotation. */
        R3RigidBodyDesc wrong_body = {0};

        /* CORRECT version: the constructor gives every field a meaningful default value. */
        R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
        body.position.translation = r3Vector(0.0, 1.0, 0.0);
        // DOCUSAURUS: Descriptions stop
        (void)wrong_body;
        r3InsertRigidBody(world, &body);
    }

    {
        R3ColliderDesc desc = r3BallColliderDesc(0.5);
        R3ColliderHandle collider = r3InsertColliderWithoutParent(world, &desc);
        // DOCUSAURUS: Rotations start
        /* WRONG version: a zero-initialized quaternion isn't a valid rotation, the call fails. */
        // R3Pose pose = {{1.0, 2.0, 3.0}, {0}};

        /* CORRECT version: start from the identity rotation (0, 0, 0, 1), or build a rotation from an
         * axis and an angle. */
        R3Pose pose = r3TranslationPose(r3Vector(1.0, 2.0, 3.0));
        pose.rotation = r3RotationFromAxisAngle(r3Vector(0.0, 1.0, 0.0), 0.5);
        r3Collider_SetPosition(collider, pose);
        // DOCUSAURUS: Rotations stop
    }

    r3Step(world, NULL, NULL);

    {
        // DOCUSAURUS: NotFound start
        /* WRONG version, if missing the ground is expected: a miss is an error (R3_NOT_FOUND) given
         * to the error handler, and the returned hit is meaningless. */
        // R3RayHit hit = r3CastRay(world, NULL, r3Vector(0.0, 10.0, 0.0), r3Vector(0.0, -1.0, 0.0), 1.0, 1);

        /* CORRECT version: a miss is reported by the `found` field. */
        R3OptionalRayHit hit =
            r3TryCastRay(world, NULL, r3Vector(0.0, 10.0, 0.0), r3Vector(0.0, -1.0, 0.0), 1.0, 1);
        if (!hit.found) {
            printf("Nothing below.\n");
        }
        // DOCUSAURUS: NotFound stop
    }

    {
        R3RigidBodyDesc desc = r3DynamicRigidBodyDesc();
        R3RigidBodyHandle handle = r3InsertRigidBody(world, &desc);
        // DOCUSAURUS: StaleHandles start
        r3RemoveRigidBody(handle, 1);

        /* The handle is now stale: using it (even to remove it again) fails with R3_INVALID_HANDLE. Check
         * it first if the object may have been removed by another part of the application. */
        if (r3RigidBody_Contains(handle)) {
            r3RigidBody_WakeUp(handle, 1);
        }
        // DOCUSAURUS: StaleHandles stop
    }

    // DOCUSAURUS: Ownership start
    /* The objects allocated by Rapier are freed by their own Free function, never by `free`. */
    R3EventCollector *events = r3NewEventCollector();
    r3Step(world, NULL, events);
    r3FreeEventCollector(events);

    /* Freeing the world frees everything it contains, and invalidates all its handles. */
    r3FreeWorld(world);
    world = NULL;
    // DOCUSAURUS: Ownership stop
    return EXIT_SUCCESS;
}
