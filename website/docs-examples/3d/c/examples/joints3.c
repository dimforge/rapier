#include "snippets.h"

/* Inserts a dynamic rigid-body with a ball collider at the given translation. */
static R3RigidBodyHandle insert_ball(R3World *world, R3Vector translation) {
    R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
    body.position.translation = translation;
    R3RigidBodyHandle handle = r3InsertRigidBody(world, &body);
    R3ColliderDesc collider = r3BallColliderDesc(0.5);
    r3InsertCollider(handle, &collider);
    return handle;
}

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc body_desc = r3DynamicRigidBodyDesc();
    R3RigidBodyHandle body_handle1 = r3InsertRigidBody(world, &body_desc);
    R3RigidBodyHandle body_handle2 = r3InsertRigidBody(world, &body_desc);

    {
        // DOCUSAURUS: FixedJoint start
        // NOTE: the local anchors are the translation parts of the local frames.
        R3JointDesc joint = r3FixedJointDesc();
        joint.localFrame1.translation = r3Vector(0.0, 1.0, 0.0);
        joint.localFrame2.translation = r3Vector(0.0, -3.0, 0.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: FixedJoint stop
    }

    {
        // DOCUSAURUS: SphericalJoint start
        R3JointDesc joint = r3SphericalJointDesc();
        joint.localFrame1.translation = r3Vector(0.0, 0.0, 1.0);
        joint.localFrame2.translation = r3Vector(0.0, 0.0, -3.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: SphericalJoint stop
    }

    {
        // DOCUSAURUS: RevoluteJoint start
        R3Vector x = r3Vector(1.0, 0.0, 0.0);
        R3JointDesc joint = r3RevoluteJointDesc(x);
        joint.localFrame1.translation = r3Vector(0.0, 0.0, 1.0);
        joint.localFrame2.translation = r3Vector(0.0, 0.0, -3.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: RevoluteJoint stop
    }

    {
        // DOCUSAURUS: PrismaticJoint start
        R3Vector x = r3Vector(1.0, 0.0, 0.0);
        R3JointDesc joint = r3PrismaticJointDesc(x);
        joint.localFrame1.translation = r3Vector(0.0, 0.0, 1.0);
        joint.localFrame2.translation = r3Vector(0.0, 0.0, -3.0);
        // The free axis of a prismatic joint is the X axis of its local frames.
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -2.0, 5.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: PrismaticJoint stop
    }

    R3ImpulseJointHandle motor_joint;
    {
        // DOCUSAURUS: Motor start
        R3Vector x = r3Vector(1.0, 0.0, 0.0);
        R3JointDesc joint = r3PrismaticJointDesc(x);
        joint.localFrame1.translation = r3Vector(0.0, 0.0, 1.0);
        joint.localFrame2.translation = r3Vector(0.0, 0.0, -3.0);
        r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_LIN_X, 1.0, 0.5);
        R3ImpulseJointHandle joint_handle = r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: Motor stop
        motor_joint = joint_handle;
        r3JointDesc_SetMotorMaxForce(&joint, R3_AXIS_LIN_X, 100.0);
        r3JointDesc_SetMotorModel(&joint, R3_AXIS_LIN_X, R3_MOTOR_FORCE_BASED);
        r3ImpulseJoint_SetDesc(joint_handle, &joint, 1);
    }

    {
        // DOCUSAURUS: GenericJoint start
        // A cylindrical joint: only the translation along, and the rotation around, the X axis are free.
        R3JointDesc joint = r3DefaultJointDesc();
        joint.lockedAxes = (1 << R3_AXIS_LIN_Y) | (1 << R3_AXIS_LIN_Z) | (1 << R3_AXIS_ANG_Y) | (1 << R3_AXIS_ANG_Z);
        joint.localFrame1.translation = r3Vector(0.0, 0.0, 1.0);
        joint.localFrame2.translation = r3Vector(0.0, 0.0, -3.0);
        // Limit the relative translation along the X axis.
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -2.0, 5.0);
        // Allow contacts between the colliders of the two rigid-bodies.
        // Default: 1
        joint.contactsEnabled = 1;
        // Make the locked axes springy instead of rigid.
        // Default: a natural frequency of 1.0e6 Hz and a damping ratio of 1.0.
        joint.softness.natural_frequency = 10.0;
        joint.softness.damping_ratio = 1.0;
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: GenericJoint stop
    }

    {
        // DOCUSAURUS: CoupledAxes start
        // The relative translation along all the axes is free, but its length is limited to 2.0 (like a rope).
        R3JointDesc joint = r3DefaultJointDesc();
        joint.coupledAxes = (1 << R3_AXIS_LIN_X) | (1 << R3_AXIS_LIN_Y) | (1 << R3_AXIS_LIN_Z);
        // Only the limits of the first coupled axis are used.
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, 0.0, 2.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &joint);
        // DOCUSAURUS: CoupledAxes stop
    }

    {
        // DOCUSAURUS: RopeSpringJoint start
        // The distance between the anchors can't exceed 2.0.
        R3JointDesc rope = r3RopeJointDesc(2.0);
        r3InsertImpulseJoint(body_handle1, body_handle2, &rope);
        // A spring with a rest length of 2.0, a stiffness of 10.0, and a damping of 0.5.
        R3JointDesc spring = r3SpringJointDesc(2.0, 10.0, 0.5);
        r3InsertImpulseJoint(body_handle1, body_handle2, &spring);
        // DOCUSAURUS: RopeSpringJoint stop
    }

    r3Step(world, NULL, NULL);
    R3ImpulseJointHandle joint_handle = motor_joint;

    {
        // DOCUSAURUS: ModifyJoint start
        // Change the motor of an existing joint (the last argument wakes up its rigid-bodies).
        r3ImpulseJoint_SetMotorVelocity(joint_handle, R3_AXIS_LIN_X, 2.0, 0.5, 1);
        // Read a copy of the whole joint description, modify it, then apply it back.
        R3JointDesc desc = r3ImpulseJoint_Desc(joint_handle);
        desc.contactsEnabled = 0;
        r3ImpulseJoint_SetDesc(joint_handle, &desc, 1);
        // Disable the joint: it stays attached to its rigid-bodies but is ignored by the solver.
        r3ImpulseJoint_SetEnabled(joint_handle, 0, 1);
        // The rigid-bodies attached by the joint.
        R3JointBodies bodies = r3ImpulseJoint_Bodies(joint_handle);
        // Remove the joint, waking up its rigid-bodies.
        r3RemoveImpulseJoint(joint_handle, 1);
        // DOCUSAURUS: ModifyJoint stop
        if (bodies.body1.index != body_handle1.index || bodies.body2.index != body_handle2.index) {
            return EXIT_FAILURE;
        }
    }

    {
        /* A fresh prismatic joint, as the one above was removed. */
        R3JointDesc desc = r3PrismaticJointDesc(r3Vector(1.0, 0.0, 0.0));
        R3ImpulseJointHandle joint_handle = r3InsertImpulseJoint(body_handle1, body_handle2, &desc);
        // DOCUSAURUS: JointImpulses start
        const R3Real max_impulse = 10.0;
        r3Step(world, NULL, NULL);
        // The impulses applied by the joint during the last step.
        R3JointImpulses impulses = r3ImpulseJoint_Impulses(joint_handle);
        // Break the joint if it has to pull its rigid-bodies too strongly to keep them together.
        if (r3VectorLength(impulses.linear) > max_impulse) {
            r3RemoveImpulseJoint(joint_handle, 1);
        }
        // DOCUSAURUS: JointImpulses stop
    }

    {
        // DOCUSAURUS: Multibody start
        // The root of the multibody: a fixed rigid-body.
        R3RigidBodyDesc root_desc = r3FixedRigidBodyDesc();
        R3RigidBodyHandle root = r3InsertRigidBody(world, &root_desc);
        // Three links, each attached to the previous one by a revolute multibody joint.
        R3RigidBodyHandle parent = root;
        R3MultibodyJointHandle last_joint = R3_INVALID_MULTIBODY_JOINT_HANDLE;
        for (int i = 1; i <= 3; i++) {
            R3RigidBodyHandle link = insert_ball(world, r3Vector(0.0, 0.0, -2.0 * i));
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(1.0, 0.0, 0.0));
            joint.localFrame2.translation = r3Vector(0.0, 0.0, 2.0);
            last_joint = r3InsertMultibodyJoint(parent, link, &joint);
            parent = link;
        }
        // DOCUSAURUS: Multibody stop

        r3Step(world, NULL, NULL);

        // DOCUSAURUS: GeneralizedVelocity start
        // The number of degrees of freedom of the whole multibody (3 revolute joints: 3 DOF).
        size_t ndofs = r3MultibodyJoint_Ndofs(last_joint);
        R3Real *velocities = malloc(ndofs * sizeof(R3Real));
        // Read the generalized velocities of the multibody, i.e., the relative angular velocity of each joint.
        r3MultibodyJoint_GeneralizedVelocity(last_joint, velocities, ndofs);
        // Stop every joint of the multibody.
        for (size_t i = 0; i < ndofs; i++) {
            velocities[i] = 0.0;
        }
        r3MultibodyJoint_SetGeneralizedVelocity(last_joint, velocities, ndofs);
        free(velocities);
        // DOCUSAURUS: GeneralizedVelocity stop
        if (ndofs != 3) {
            return EXIT_FAILURE;
        }

        // DOCUSAURUS: ModifyMultibodyJoint start
        // Change the motor of an existing multibody joint.
        R3JointDesc desc = r3MultibodyJoint_Desc(last_joint);
        r3JointDesc_SetMotorVelocity(&desc, R3_AXIS_ANG_X, 1.0, 0.5);
        r3MultibodyJoint_SetDesc(last_joint, &desc, 1);
        // The rigid-bodies attached by the joint: its parent link, then its own link.
        R3JointBodies bodies = r3MultibodyJoint_Bodies(last_joint);
        // DOCUSAURUS: ModifyMultibodyJoint stop
        if (bodies.body2.index != parent.index) {
            return EXIT_FAILURE;
        }
    }

    {
        R3RigidBodyDesc root_desc = r3FixedRigidBodyDesc();
        R3RigidBodyHandle root = r3InsertRigidBody(world, &root_desc);
        R3RigidBodyHandle parent = root;
        R3MultibodyJointHandle end_effector = R3_INVALID_MULTIBODY_JOINT_HANDLE;
        for (int i = 1; i <= 3; i++) {
            R3RigidBodyHandle link = insert_ball(world, r3Vector(0.0, i - 0.5, 0.0));
            R3JointDesc joint = r3SphericalJointDesc();
            joint.localFrame1.translation = r3Vector(0.0, 0.5 * (i != 1), 0.0);
            joint.localFrame2.translation = r3Vector(0.0, -0.5, 0.0);
            end_effector = r3InsertMultibodyJoint(parent, link, &joint);
            parent = link;
        }
        /* Let the first step finalize the multibody (its fixed root then has no degree of freedom). */
        r3Step(world, NULL, NULL);

        // DOCUSAURUS: InverseKinematics start
        // Only try to reach the target translation, whatever the orientation of the last link.
        R3InverseKinematicsOptions options = r3DefaultInverseKinematicsOptions();
        options.constrained_axes = (1 << R3_AXIS_LIN_X) | (1 << R3_AXIS_LIN_Y) | (1 << R3_AXIS_LIN_Z);
        R3Pose target = r3TranslationPose(r3Vector(0.5, 1.5, 0.0));

        // The displacements must be zero-initialized, with one entry per degree of freedom of the multibody.
        size_t ndofs = r3MultibodyJoint_Ndofs(end_effector);
        R3Real *displacements = calloc(ndofs, sizeof(R3Real));
        // Compute the displacements moving the link of `end_effector` toward the target, then apply them.
        r3MultibodyJoint_InverseKinematics(end_effector, &options, target, NULL, NULL, displacements, ndofs);
        r3MultibodyJoint_ApplyDisplacements(end_effector, displacements, ndofs);
        free(displacements);
        // DOCUSAURUS: InverseKinematics stop
    }

    {
        // DOCUSAURUS: LoopClosing start
        // Five pearls forming a necklace.
        R3RigidBodyHandle pearls[5];
        for (int i = 0; i < 5; i++) {
            R3Real angle = 2.0 * R3_PI * i / 5.0;
            pearls[i] = insert_ball(world, r3Vector(2.0 * cos(angle), 10.0, 2.0 * sin(angle)));
        }
        // Each spherical joint links the centers of two consecutive pearls.
        R3JointDesc joint = r3SphericalJointDesc();
        // The first four joints form a multibody (a tree).
        for (int i = 0; i < 4; i++) {
            R3Vector delta = r3VectorSub(r3RigidBody_Translation(pearls[i + 1]), r3RigidBody_Translation(pearls[i]));
            joint.localFrame1.translation = delta;
            r3InsertMultibodyJoint(pearls[i], pearls[i + 1], &joint);
        }
        // The fifth joint closes the loop: it has to be an impulse joint.
        joint.localFrame1.translation = r3VectorSub(r3RigidBody_Translation(pearls[0]), r3RigidBody_Translation(pearls[4]));
        r3InsertImpulseJoint(pearls[4], pearls[0], &joint);
        // DOCUSAURUS: LoopClosing stop
    }

    for (int i = 0; i < 10; i++) {
        r3Step(world, NULL, NULL);
    }

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
