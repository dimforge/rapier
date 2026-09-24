#include "snippets.h"

/* Inserts a dynamic rigid-body with a ball collider at the given translation. */
static R2RigidBodyHandle insert_ball(R2World *world, R2Vector translation) {
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = translation;
    R2RigidBodyHandle handle = r2InsertRigidBody(world, &body);
    R2ColliderDesc collider = r2BallColliderDesc(0.5);
    r2InsertCollider(handle, &collider);
    return handle;
}

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc body_desc = r2DynamicRigidBodyDesc();
    R2RigidBodyHandle body_handle1 = r2InsertRigidBody(world, &body_desc);
    R2RigidBodyHandle body_handle2 = r2InsertRigidBody(world, &body_desc);

    {
        // DOCUSAURUS: FixedJoint start
        // NOTE: the local anchors are the translation parts of the local frames.
        R2JointDesc joint = r2FixedJointDesc();
        joint.localFrame1.translation = r2Vector(0.0, 1.0);
        joint.localFrame2.translation = r2Vector(0.0, -3.0);
        r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: FixedJoint stop
    }

    {
        // DOCUSAURUS: RevoluteJoint start
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = r2Vector(0.0, 1.0);
        joint.localFrame2.translation = r2Vector(0.0, -3.0);
        r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: RevoluteJoint stop
    }

    {
        // DOCUSAURUS: PrismaticJoint start
        R2Vector x = r2Vector(1.0, 0.0);
        R2JointDesc joint = r2PrismaticJointDesc(x);
        joint.localFrame1.translation = r2Vector(0.0, 1.0);
        joint.localFrame2.translation = r2Vector(0.0, -3.0);
        // The free axis of a prismatic joint is the X axis of its local frames.
        r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, -2.0, 5.0);
        r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: PrismaticJoint stop
    }

    R2ImpulseJointHandle motor_joint;
    {
        // DOCUSAURUS: Motor start
        R2Vector x = r2Vector(1.0, 0.0);
        R2JointDesc joint = r2PrismaticJointDesc(x);
        joint.localFrame1.translation = r2Vector(0.0, 1.0);
        joint.localFrame2.translation = r2Vector(0.0, -3.0);
        r2JointDesc_SetMotorVelocity(&joint, R2_AXIS_LIN_X, 1.0, 0.5);
        R2ImpulseJointHandle joint_handle = r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: Motor stop
        motor_joint = joint_handle;
        r2JointDesc_SetMotorMaxForce(&joint, R2_AXIS_LIN_X, 100.0);
        r2JointDesc_SetMotorModel(&joint, R2_AXIS_LIN_X, R2_MOTOR_FORCE_BASED);
        r2ImpulseJoint_SetDesc(joint_handle, &joint, 1);
    }

    {
        // DOCUSAURUS: GenericJoint start
        // A cartesian joint: only the rotation is locked.
        R2JointDesc joint = r2DefaultJointDesc();
        joint.lockedAxes = 1 << R2_AXIS_ANG_X;
        joint.localFrame1.translation = r2Vector(0.0, 1.0);
        joint.localFrame2.translation = r2Vector(0.0, -3.0);
        // Limit the relative translation along the X axis.
        r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, -2.0, 5.0);
        // Allow contacts between the colliders of the two rigid-bodies.
        // Default: 1
        joint.contactsEnabled = 1;
        // Make the locked axes springy instead of rigid.
        // Default: a natural frequency of 1.0e6 Hz and a damping ratio of 1.0.
        joint.softness.natural_frequency = 10.0;
        joint.softness.damping_ratio = 1.0;
        r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: GenericJoint stop
    }

    {
        // DOCUSAURUS: CoupledAxes start
        // The relative translation along all the axes is free, but its length is limited to 2.0 (like a rope).
        R2JointDesc joint = r2DefaultJointDesc();
        joint.coupledAxes = (1 << R2_AXIS_LIN_X) | (1 << R2_AXIS_LIN_Y);
        // Only the limits of the first coupled axis are used.
        r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, 0.0, 2.0);
        r2InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: CoupledAxes stop
    }

    {
        // DOCUSAURUS: RopeSpringJoint start
        // The distance between the anchors can't exceed 2.0.
        R2JointDesc rope = r2RopeJointDesc(2.0);
        r2InsertImpulseJoint(body_handle1, body_handle2, &rope);
        // A spring with a rest length of 2.0, a stiffness of 10.0, and a damping of 0.5.
        R2JointDesc spring = r2SpringJointDesc(2.0, 10.0, 0.5);
        r2InsertImpulseJoint(body_handle1, body_handle2, &spring);
        // A pin-slot joint: free rotation, and free translation along the X axis.
        R2JointDesc pin_slot = r2PinSlotJointDesc(r2Vector(1.0, 0.0));
        r2InsertImpulseJoint(body_handle1, body_handle2, &pin_slot);
        // DOCUSAURUS: RopeSpringJoint stop
    }

    r2Step(world, NULL, NULL);
    R2ImpulseJointHandle joint_handle = motor_joint;

    {
        // DOCUSAURUS: ModifyJoint start
        // Change the motor of an existing joint (the last argument wakes up its rigid-bodies).
        r2ImpulseJoint_SetMotorVelocity(joint_handle, R2_AXIS_LIN_X, 2.0, 0.5, 1);
        // Read a copy of the whole joint description, modify it, then apply it back.
        R2JointDesc desc = r2ImpulseJoint_Desc(joint_handle);
        desc.contactsEnabled = 0;
        r2ImpulseJoint_SetDesc(joint_handle, &desc, 1);
        // Disable the joint: it stays attached to its rigid-bodies but is ignored by the solver.
        r2ImpulseJoint_SetEnabled(joint_handle, 0, 1);
        // The rigid-bodies attached by the joint.
        R2JointBodies bodies = r2ImpulseJoint_Bodies(joint_handle);
        // Remove the joint, waking up its rigid-bodies.
        r2RemoveImpulseJoint(joint_handle, 1);
        // DOCUSAURUS: ModifyJoint stop
        if (bodies.body1.index != body_handle1.index || bodies.body2.index != body_handle2.index) {
            return EXIT_FAILURE;
        }
    }

    {
        /* A fresh prismatic joint, as the one above was removed. */
        R2JointDesc desc = r2PrismaticJointDesc(r2Vector(1.0, 0.0));
        R2ImpulseJointHandle joint_handle = r2InsertImpulseJoint(body_handle1, body_handle2, &desc);
        // DOCUSAURUS: JointImpulses start
        const R2Real max_impulse = 10.0;
        r2Step(world, NULL, NULL);
        // The impulses applied by the joint during the last step.
        R2JointImpulses impulses = r2ImpulseJoint_Impulses(joint_handle);
        // Break the joint if it has to pull its rigid-bodies too strongly to keep them together.
        if (r2VectorLength(impulses.linear) > max_impulse) {
            r2RemoveImpulseJoint(joint_handle, 1);
        }
        // DOCUSAURUS: JointImpulses stop
    }

    {
        // DOCUSAURUS: Multibody start
        // The root of the multibody: a fixed rigid-body.
        R2RigidBodyDesc root_desc = r2FixedRigidBodyDesc();
        R2RigidBodyHandle root = r2InsertRigidBody(world, &root_desc);
        // Three links, each attached to the previous one by a revolute multibody joint.
        R2RigidBodyHandle parent = root;
        R2MultibodyJointHandle last_joint = R2_INVALID_MULTIBODY_JOINT_HANDLE;
        for (int i = 1; i <= 3; i++) {
            R2RigidBodyHandle link = insert_ball(world, r2Vector(2.0 * i, 0.0));
            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame2.translation = r2Vector(-2.0, 0.0);
            last_joint = r2InsertMultibodyJoint(parent, link, &joint);
            parent = link;
        }
        // DOCUSAURUS: Multibody stop

        r2Step(world, NULL, NULL);

        // DOCUSAURUS: GeneralizedVelocity start
        // The number of degrees of freedom of the whole multibody (3 revolute joints: 3 DOF).
        size_t ndofs = r2MultibodyJoint_Ndofs(last_joint);
        R2Real *velocities = malloc(ndofs * sizeof(R2Real));
        // Read the generalized velocities of the multibody, i.e., the relative angular velocity of each joint.
        r2MultibodyJoint_GeneralizedVelocity(last_joint, velocities, ndofs);
        // Stop every joint of the multibody.
        for (size_t i = 0; i < ndofs; i++) {
            velocities[i] = 0.0;
        }
        r2MultibodyJoint_SetGeneralizedVelocity(last_joint, velocities, ndofs);
        free(velocities);
        // DOCUSAURUS: GeneralizedVelocity stop
        if (ndofs != 3) {
            return EXIT_FAILURE;
        }

        // DOCUSAURUS: ModifyMultibodyJoint start
        // Change the motor of an existing multibody joint.
        R2JointDesc desc = r2MultibodyJoint_Desc(last_joint);
        r2JointDesc_SetMotorVelocity(&desc, R2_AXIS_ANG_X, 1.0, 0.5);
        r2MultibodyJoint_SetDesc(last_joint, &desc, 1);
        // The rigid-bodies attached by the joint: its parent link, then its own link.
        R2JointBodies bodies = r2MultibodyJoint_Bodies(last_joint);
        // DOCUSAURUS: ModifyMultibodyJoint stop
        if (bodies.body2.index != parent.index) {
            return EXIT_FAILURE;
        }
    }

    {
        R2RigidBodyDesc root_desc = r2FixedRigidBodyDesc();
        R2RigidBodyHandle root = r2InsertRigidBody(world, &root_desc);
        R2RigidBodyHandle parent = root;
        R2MultibodyJointHandle end_effector = R2_INVALID_MULTIBODY_JOINT_HANDLE;
        for (int i = 1; i <= 3; i++) {
            R2RigidBodyHandle link = insert_ball(world, r2Vector(0.0, i - 0.5));
            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame1.translation = r2Vector(0.0, 0.5 * (i != 1));
            joint.localFrame2.translation = r2Vector(0.0, -0.5);
            end_effector = r2InsertMultibodyJoint(parent, link, &joint);
            parent = link;
        }
        /* Let the first step finalize the multibody (its fixed root then has no degree of freedom). */
        r2Step(world, NULL, NULL);

        // DOCUSAURUS: InverseKinematics start
        // Only try to reach the target translation, whatever the orientation of the last link.
        R2InverseKinematicsOptions options = r2DefaultInverseKinematicsOptions();
        options.constrained_axes = (1 << R2_AXIS_LIN_X) | (1 << R2_AXIS_LIN_Y);
        R2Pose target = r2TranslationPose(r2Vector(0.5, 1.5));

        // The displacements must be zero-initialized, with one entry per degree of freedom of the multibody.
        size_t ndofs = r2MultibodyJoint_Ndofs(end_effector);
        R2Real *displacements = calloc(ndofs, sizeof(R2Real));
        // Compute the displacements moving the link of `end_effector` toward the target, then apply them.
        r2MultibodyJoint_InverseKinematics(end_effector, &options, target, NULL, NULL, displacements, ndofs);
        r2MultibodyJoint_ApplyDisplacements(end_effector, displacements, ndofs);
        free(displacements);
        // DOCUSAURUS: InverseKinematics stop
    }

    {
        // DOCUSAURUS: LoopClosing start
        // Five pearls forming a necklace.
        R2RigidBodyHandle pearls[5];
        for (int i = 0; i < 5; i++) {
            R2Real angle = 2.0 * R2_PI * i / 5.0;
            pearls[i] = insert_ball(world, r2Vector(2.0 * cos(angle), 10.0 + 2.0 * sin(angle)));
        }
        // Each revolute joint links the centers of two consecutive pearls.
        R2JointDesc joint = r2RevoluteJointDesc();
        // The first four joints form a multibody (a tree).
        for (int i = 0; i < 4; i++) {
            R2Vector delta = r2VectorSub(r2RigidBody_Translation(pearls[i + 1]), r2RigidBody_Translation(pearls[i]));
            joint.localFrame1.translation = delta;
            r2InsertMultibodyJoint(pearls[i], pearls[i + 1], &joint);
        }
        // The fifth joint closes the loop: it has to be an impulse joint.
        joint.localFrame1.translation = r2VectorSub(r2RigidBody_Translation(pearls[0]), r2RigidBody_Translation(pearls[4]));
        r2InsertImpulseJoint(pearls[4], pearls[0], &joint);
        // DOCUSAURUS: LoopClosing stop
    }

    for (int i = 0; i < 10; i++) {
        r2Step(world, NULL, NULL);
    }

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
