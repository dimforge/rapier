/* Port of examples3d/inverse_kinematics3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbInverseKinematics3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.01, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.01, 0.2));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }

    const int numSegments = 10;
    R3RigidBodyDesc body = r3FixedRigidBodyDesc();
    R3RigidBodyHandle lastBody = r3InsertRigidBody(world, &body);
    R3MultibodyJointHandle lastLink = {NULL, UINT32_MAX, UINT32_MAX};
    for (int i = 0; i < numSegments; ++i) {
        const R3Real size = 1.0 / numSegments;
        R3RigidBodyHandle newBody;
        /* Sensors draw the links; IK does not require colliders. */
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(0, 0, 0);
            rigidBody.canSleep = 0;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(size / 8, size / 2, size / 8));
            collider.density = 0;
            collider.isSensor = 1;
            newBody = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(newBody, &collider);
        }
        R3JointDesc linkAb = r3SphericalJointDesc();
        linkAb.localFrame1.translation = r3Vector(0, size / 2 * (i != 0), 0);
        linkAb.localFrame2.translation = r3Vector(0, -size / 2, 0);
        lastLink = r3InsertMultibodyJoint(lastBody, newBody, &linkAb);

        lastBody = newBody;
    }
    tbCamera(testbed, 0, .5, 2.5, 0, .5, 0);

    tbSetWorld(testbed, world);
    R3Real *displacements = NULL;
    size_t capacity = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            if (!testbed->cursorValid) {
                continue;
            }
            size_t ndofs = r3MultibodyJoint_Ndofs(lastLink);
            if (capacity < ndofs) {
                R3Real *resized = realloc(displacements, ndofs * sizeof(*resized));
                if (!resized) {
                    abort();
                }
                displacements = resized;
                capacity = ndofs;
            }
            memset(displacements, 0, ndofs * sizeof(*displacements));
            R3InverseKinematicsOptions options = r3DefaultInverseKinematicsOptions();
            options.constrained_axes = 7; /* Linear axes only. */
            const R3Pose target = r3TranslationPose(testbed->cursor);
            r3MultibodyJoint_InverseKinematics(lastLink, &options, target, NULL, NULL,
                                              displacements, ndofs);
            r3MultibodyJoint_ApplyDisplacements(lastLink, displacements, ndofs);
        }
    }
    free(displacements);
    r3FreeWorld(world);
}
