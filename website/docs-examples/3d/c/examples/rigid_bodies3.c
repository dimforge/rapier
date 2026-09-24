/* Keep the asserts of this snippet active in release builds. */
#undef NDEBUG
#include <assert.h>

#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    // DOCUSAURUS: Creation start
    // The world that will contain our rigid-bodies.
    R3World *world = r3NewWorld();

    // Description of a fixed rigid-body.
    R3RigidBodyDesc fixed_desc = r3FixedRigidBodyDesc();
    // Description of a dynamic rigid-body.
    R3RigidBodyDesc dynamic_desc = r3DynamicRigidBodyDesc();
    // Description of a kinematic rigid-body controlled at the velocity level.
    R3RigidBodyDesc kinematic_velocity_desc = r3KinematicVelocityBasedRigidBodyDesc();
    // Description of a kinematic rigid-body controlled at the position level.
    R3RigidBodyDesc kinematic_position_desc = r3KinematicPositionBasedRigidBodyDesc();

    R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
    // The body type: R3_DYNAMIC, R3_FIXED, R3_KINEMATIC_VELOCITY_BASED, or R3_KINEMATIC_POSITION_BASED.
    // Default: the type of the constructor used to initialize the description.
    rigid_body.bodyType = R3_DYNAMIC;
    // The rigid body translation.
    // Default: zero vector.
    rigid_body.position.translation = r3Vector(0.0, 5.0, 1.0);
    // The rigid body rotation.
    // Default: no rotation.
    rigid_body.position.rotation = r3RotationFromAxisAngle(r3Vector(0.0, 0.0, 1.0), 5.0);
    // The rigid body position. Will override the translation and rotation set above.
    // Default: the identity pose.
    rigid_body.position = r3Pose(r3Vector(1.0, 3.0, 2.0), r3RotationFromAxisAngle(r3Vector(0.0, 0.0, 1.0), 0.4));
    // The linear velocity of this body.
    // Default: zero velocity.
    rigid_body.linvel = r3Vector(1.0, 3.0, 4.0);
    // The angular velocity of this body.
    // Default: zero velocity.
    rigid_body.angvel = r3Vector(3.0, 0.0, 1.0);
    // The scaling factor applied to the gravity affecting the rigid-body.
    // Default: 1.0
    rigid_body.gravityScale = 0.5;
    // Whether or not this body can sleep.
    // Default: 1
    rigid_body.canSleep = 1;
    // Whether or not CCD is enabled for this rigid-body.
    // Default: 0
    rigid_body.ccdEnabled = 0;
    // All done, actually create the rigid-body and insert it into the world.
    R3RigidBodyHandle rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
    // DOCUSAURUS: Creation stop
    r3InsertRigidBody(world, &fixed_desc);
    r3InsertRigidBody(world, &dynamic_desc);
    r3InsertRigidBody(world, &kinematic_velocity_desc);
    r3InsertRigidBody(world, &kinematic_position_desc);
    assert(r3RigidBody_GravityScale(rigid_body_handle) == 0.5);

    {
        // DOCUSAURUS: Position1 start
        /* Set the position when the rigid-body is created. */
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        // The rigid body translation.
        // Default: zero vector.
        rigid_body.position.translation = r3Vector(0.0, 5.0, 1.0);
        // The rigid body rotation.
        // Default: no rotation.
        rigid_body.position.rotation = r3RotationFromAxisAngle(r3Vector(1.0, 0.0, 0.0), 0.2);
        // The rigid body position. Will override the translation and rotation set above.
        // Default: the identity pose.
        rigid_body.position = r3Pose(r3Vector(1.0, 2.0, 3.0), r3RotationFromAxisAngle(r3Vector(1.0, 0.0, 0.0), 0.2));
        // DOCUSAURUS: Position1 stop
        // Insert the rigid-body into the world.
        rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Position2 start
        /* Set the position after the rigid-body creation. */
        R3Rotation rotation = r3RotationFromAxisAngle(r3Vector(1.0, 0.0, 0.0), 0.2);
        // The last `1` argument makes sure the rigid-body is awake.
        r3RigidBody_SetTranslation(rigid_body_handle, r3Vector(0.0, 5.0, 1.0), 1);
        r3RigidBody_SetRotation(rigid_body_handle, rotation, 1);
        R3Vector current_translation = r3RigidBody_Translation(rigid_body_handle);
        R3Rotation current_rotation = r3RigidBody_Rotation(rigid_body_handle);
        assert(current_translation.x == 0.0 && current_translation.y == 5.0 && current_translation.z == 1.0);
        assert(current_rotation.x == rotation.x && current_rotation.y == rotation.y &&
               current_rotation.z == rotation.z && current_rotation.w == rotation.w);

        R3Pose pose = r3Pose(r3Vector(1.0, 2.0, 3.0), r3RotationFromAxisAngle(r3Vector(0.0, 1.0, 0.0), 0.4));
        r3RigidBody_SetPosition(rigid_body_handle, pose, 1);
        R3Pose current_pose = r3RigidBody_Position(rigid_body_handle);
        assert(current_pose.translation.x == 1.0 && current_pose.translation.y == 2.0 &&
               current_pose.translation.z == 3.0);
        assert(current_pose.rotation.x == pose.rotation.x && current_pose.rotation.y == pose.rotation.y &&
               current_pose.rotation.z == pose.rotation.z && current_pose.rotation.w == pose.rotation.w);
        // DOCUSAURUS: Position2 stop
    }

    {
        // DOCUSAURUS: Velocity1 start
        /* Set the velocities when the rigid-body is created. */
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        // The linear velocity of this body.
        // Default: zero velocity.
        rigid_body.linvel = r3Vector(1.0, 3.0, 4.0);
        // The angular velocity of this body.
        // Default: zero velocity.
        rigid_body.angvel = r3Vector(3.0, 0.0, 0.0);
        // DOCUSAURUS: Velocity1 stop
        // Insert the rigid-body into the world.
        rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Velocity2 start
        /* Set the velocities after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r3RigidBody_SetLinvel(rigid_body_handle, r3Vector(1.0, 3.0, 4.0), 1);
        r3RigidBody_SetAngvel(rigid_body_handle, r3Vector(3.0, 0.0, 0.0), 1);
        R3Vector linvel = r3RigidBody_Linvel(rigid_body_handle);
        R3AngVector angvel = r3RigidBody_Angvel(rigid_body_handle);
        assert(linvel.x == 1.0 && linvel.y == 3.0 && linvel.z == 4.0);
        assert(angvel.x == 3.0 && angvel.y == 0.0 && angvel.z == 0.0);
        // DOCUSAURUS: Velocity2 stop
    }

    {
        // DOCUSAURUS: Forces start
        // The last `1` argument makes sure the rigid-body is awake.
        r3RigidBody_ResetForces(rigid_body_handle, 1);  // Reset the forces to zero.
        r3RigidBody_ResetTorques(rigid_body_handle, 1); // Reset the torques to zero.
        r3RigidBody_AddForce(rigid_body_handle, r3Vector(0.0, 1000.0, 0.0), 1);
        r3RigidBody_AddTorque(rigid_body_handle, r3Vector(100.0, 0.0, 0.0), 1);
        r3RigidBody_AddForceAtPoint(rigid_body_handle, r3Vector(0.0, 1000.0, 0.0), r3Vector(1.0, 2.0, 3.0), 1);

        r3RigidBody_ApplyImpulse(rigid_body_handle, r3Vector(0.0, 1000.0, 0.0), 1);
        r3RigidBody_ApplyTorqueImpulse(rigid_body_handle, r3Vector(100.0, 0.0, 0.0), 1);
        r3RigidBody_ApplyImpulseAtPoint(rigid_body_handle, r3Vector(0.0, 1000.0, 0.0), r3Vector(1.0, 2.0, 3.0), 1);
        // DOCUSAURUS: Forces stop
        assert(r3RigidBody_UserForce(rigid_body_handle).y == 2000.0);
    }

    {
        // DOCUSAURUS: Mass2 start
        /* Set the mass-properties when the rigid-body is created. */
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        rigid_body.additionalMass = 0.5;
        // Sets both the mass and angular inertia at once (this overrides `additionalMass`).
        rigid_body.useAdditionalMassProperties = 1;
        rigid_body.additionalMassProperties = (R3MassProperties){
            .local_com = r3Vector(0.0, 1.0, 0.0),
            .mass = 0.5,
            .principal_inertia = r3Vector(0.3, 0.2, 0.1),
            // The principal inertia axes are aligned with the rigid-body's local axes.
            .principal_inertia_local_frame = {0.0, 0.0, 0.0, 1.0},
        };
        // DOCUSAURUS: Mass2 stop
        r3InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Mass3 start
        /* Set the mass-properties after the rigid-body creation. */
        R3MassProperties mass_properties = {
            .local_com = r3Vector(0.0, 1.0, 0.0),
            .mass = 0.5,
            .principal_inertia = r3Vector(0.3, 0.2, 0.1),
            .principal_inertia_local_frame = {0.0, 0.0, 0.0, 1.0},
        };
        // The last `1` argument makes sure the rigid-body is awake.
        r3RigidBody_SetAdditionalMassProperties(rigid_body_handle, mass_properties, 1);
        // DOCUSAURUS: Mass3 stop
    }

    {
        // DOCUSAURUS: LockedAxes1 start
        /* Lock translations/rotations when the rigid-body is created. */
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        rigid_body.lockedAxes =
            R3_LOCK_TRANSLATION_X | R3_LOCK_TRANSLATION_Y | R3_LOCK_TRANSLATION_Z // prevent translations along all axes.
            | R3_LOCK_ROTATION_Y | R3_LOCK_ROTATION_Z; // only enable rotations along the X axis.
        // DOCUSAURUS: LockedAxes1 stop
        r3InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: LockedAxes2 start
        /* Lock translations/rotations after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r3RigidBody_SetTranslationsLocked(rigid_body_handle, 1, 1);
        r3RigidBody_SetRotationsLocked(rigid_body_handle, 1, 1);
        // Only enable rotations along the X axis.
        uint8_t locked_axes = r3RigidBody_LockedAxes(rigid_body_handle) & ~R3_LOCK_ROTATION_X;
        r3RigidBody_SetLockedAxes(rigid_body_handle, locked_axes, 1);
        // DOCUSAURUS: LockedAxes2 stop
        assert(r3RigidBody_LockedAxes(rigid_body_handle) ==
               (R3_LOCK_TRANSLATION_X | R3_LOCK_TRANSLATION_Y | R3_LOCK_TRANSLATION_Z | R3_LOCK_ROTATION_Y |
                R3_LOCK_ROTATION_Z));
    }

    {
        // DOCUSAURUS: SolverSettings start
        /* Give a rigid-body more solver accuracy than the rest of the scene. */
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        // Extra substeps run for the whole island component this body belongs to.
        rigid_body.additionalSolverIterations = 4;
        // Extra internal PGS iterations run per substep for that same component.
        rigid_body.additionalPgsIterations = 2;
        // Predictive contacts generated up to that distance ahead of the body's path: a cheaper
        // alternative to CCD for slow-but-thin or moderately fast objects.
        rigid_body.softCcdPrediction = 0.5;
        // Let the body exceed the angular speed cap, e.g. for a wheel.
        rigid_body.allowFastRotation = 1;
        // Gyroscopic forces give more realistic behaviors, e.g. the precession of a spinning top.
        rigid_body.gyroscopicForcesEnabled = 1;
        // DOCUSAURUS: SolverSettings stop
        R3RigidBodyHandle handle = r3InsertRigidBody(world, &rigid_body);
        assert(r3RigidBody_AdditionalSolverIterations(handle) == 4);
        assert(r3RigidBody_AdditionalPgsIterations(handle) == 2);
        assert(r3RigidBody_SoftCcdPrediction(handle) == 0.5);
        assert(r3RigidBody_IsFastRotationAllowed(handle));
        r3RigidBody_SetAdditionalSolverIterations(handle, 1);
        r3RigidBody_SetAdditionalPgsIterations(handle, 1);
        r3RigidBody_SetSoftCcdPrediction(handle, 0.25);
        r3RigidBody_SetAllowFastRotation(handle, 0);
        assert(r3RigidBody_AdditionalSolverIterations(handle) == 1);
        assert(r3RigidBody_AdditionalPgsIterations(handle) == 1);
        assert(r3RigidBody_SoftCcdPrediction(handle) == 0.25);
        assert(!r3RigidBody_IsFastRotationAllowed(handle));
    }

    r3Step(world, NULL, NULL);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
