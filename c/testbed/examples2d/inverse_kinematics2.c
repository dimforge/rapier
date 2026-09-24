/* Port of examples2d/inverse_kinematics2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbInverseKinematics2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.01);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 0.01));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }

    const int numSegments = 10;
    R2RigidBodyDesc body = r2FixedRigidBodyDesc();
    R2RigidBodyHandle lastBody = r2InsertRigidBody(world, &body);
    R2MultibodyJointHandle lastLink = {NULL, UINT32_MAX, UINT32_MAX};
    for (int i = 0; i < numSegments; ++i) {
        const R2Real size = 1.0 / numSegments;
        R2RigidBodyHandle newBody;
        /* Sensors draw the links; IK does not require colliders. */
        {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(0, 0);
            rigidBody.canSleep = 0;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(size / 8, size / 2));
            collider.density = 0;
            collider.isSensor = 1;
            newBody = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(newBody, &collider);
        }
        R2JointDesc linkAb = r2RevoluteJointDesc();
        linkAb.localFrame1.translation = r2Vector(0, size / 2 * (i != 0));
        linkAb.localFrame2.translation = r2Vector(0, -size / 2);
        lastLink = r2InsertMultibodyJoint(lastBody, newBody, &linkAb);

        lastBody = newBody;
    }
    tbCamera2(testbed, 0, 0, 300);

    tbSetWorld(testbed, world);
    R2Real *displacements = NULL;
    size_t capacity = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
            if (!testbed->cursorValid) {
                continue;
            }
            size_t ndofs = r2MultibodyJoint_Ndofs(lastLink);
            if (capacity < ndofs) {
                R2Real *resized = realloc(displacements, ndofs * sizeof(*resized));
                if (!resized) {
                    abort();
                }
                displacements = resized;
                capacity = ndofs;
            }
            memset(displacements, 0, ndofs * sizeof(*displacements));
            R2InverseKinematicsOptions options = r2DefaultInverseKinematicsOptions();
            options.constrained_axes = 3; /* Linear axes only. */
            const R2Pose target = r2TranslationPose(testbed->cursor);
            r2MultibodyJoint_InverseKinematics(lastLink, &options, target, NULL, NULL,
                                              displacements, ndofs);
            r2MultibodyJoint_ApplyDisplacements(lastLink, displacements, ndofs);
        }
    }
    free(displacements);
    r2FreeWorld(world);
}
