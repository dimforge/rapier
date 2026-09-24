/* Keep the asserts of this snippet active in release builds. */
#undef NDEBUG
#include <assert.h>

#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    // DOCUSAURUS: Creation start
    // The world that will contain our rigid-bodies.
    R2World *world = r2NewWorld();

    // Description of a fixed rigid-body.
    R2RigidBodyDesc fixed_desc = r2FixedRigidBodyDesc();
    // Description of a dynamic rigid-body.
    R2RigidBodyDesc dynamic_desc = r2DynamicRigidBodyDesc();
    // Description of a kinematic rigid-body controlled at the velocity level.
    R2RigidBodyDesc kinematic_velocity_desc = r2KinematicVelocityBasedRigidBodyDesc();
    // Description of a kinematic rigid-body controlled at the position level.
    R2RigidBodyDesc kinematic_position_desc = r2KinematicPositionBasedRigidBodyDesc();

    R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
    // The body type: R2_DYNAMIC, R2_FIXED, R2_KINEMATIC_VELOCITY_BASED, or R2_KINEMATIC_POSITION_BASED.
    // Default: the type of the constructor used to initialize the description.
    rigid_body.bodyType = R2_DYNAMIC;
    // The rigid body translation.
    // Default: zero vector.
    rigid_body.position.translation = r2Vector(0.0, 5.0);
    // The rigid body rotation.
    // Default: no rotation.
    rigid_body.position.rotation = r2Rotation(5.0);
    // The rigid body position. Will override the translation and rotation set above.
    // Default: the identity pose.
    rigid_body.position = r2Pose(r2Vector(1.0, 2.0), r2Rotation(0.4));
    // The linear velocity of this body.
    // Default: zero velocity.
    rigid_body.linvel = r2Vector(1.0, 2.0);
    // The angular velocity of this body.
    // Default: zero velocity.
    rigid_body.angvel = 2.0;
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
    R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
    // DOCUSAURUS: Creation stop
    r2InsertRigidBody(world, &fixed_desc);
    r2InsertRigidBody(world, &dynamic_desc);
    r2InsertRigidBody(world, &kinematic_velocity_desc);
    r2InsertRigidBody(world, &kinematic_position_desc);
    assert(r2RigidBody_GravityScale(rigid_body_handle) == 0.5);

    {
        // DOCUSAURUS: Position1 start
        /* Set the position when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        // The rigid body translation.
        // Default: zero vector.
        rigid_body.position.translation = r2Vector(0.0, 5.0);
        // The rigid body rotation.
        // Default: no rotation.
        rigid_body.position.rotation = r2Rotation(5.0);
        // The rigid body position. Will override the translation and rotation set above.
        // Default: the identity pose.
        rigid_body.position = r2Pose(r2Vector(1.0, 2.0), r2Rotation(0.4));
        // DOCUSAURUS: Position1 stop
        // Insert the rigid-body into the world.
        rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Position2 start
        /* Set the position after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_SetTranslation(rigid_body_handle, r2Vector(0.0, 5.0), 1);
        r2RigidBody_SetRotation(rigid_body_handle, r2Rotation(0.2), 1);
        R2Vector translation = r2RigidBody_Translation(rigid_body_handle);
        R2Rotation rotation = r2RigidBody_Rotation(rigid_body_handle);
        assert(translation.x == 0.0 && translation.y == 5.0);
        assert(rotation.angle == (R2Real)0.2);

        r2RigidBody_SetPosition(rigid_body_handle, r2Pose(r2Vector(1.0, 2.0), r2Rotation(0.4)), 1);
        R2Pose position = r2RigidBody_Position(rigid_body_handle);
        assert(position.translation.x == 1.0 && position.translation.y == 2.0);
        assert(position.rotation.angle == (R2Real)0.4);
        // DOCUSAURUS: Position2 stop
    }

    {
        // DOCUSAURUS: Velocity1 start
        /* Set the velocities when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        // The linear velocity of this body.
        // Default: zero velocity.
        rigid_body.linvel = r2Vector(1.0, 3.0);
        // The angular velocity of this body.
        // Default: zero velocity.
        rigid_body.angvel = 3.0;
        // DOCUSAURUS: Velocity1 stop
        // Insert the rigid-body into the world.
        rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Velocity2 start
        /* Set the velocities after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_SetLinvel(rigid_body_handle, r2Vector(1.0, 3.0), 1);
        r2RigidBody_SetAngvel(rigid_body_handle, 3.0, 1);
        R2Vector linvel = r2RigidBody_Linvel(rigid_body_handle);
        assert(linvel.x == 1.0 && linvel.y == 3.0);
        assert(r2RigidBody_Angvel(rigid_body_handle) == 3.0);
        // DOCUSAURUS: Velocity2 stop
    }

    {
        // DOCUSAURUS: Gravity1 start
        /* Set the gravity scale when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        // Divide by 2 the strength of gravity for this rigid-body.
        rigid_body.gravityScale = 0.5;
        // DOCUSAURUS: Gravity1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Gravity2 start
        /* Set the gravity scale after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_SetGravityScale(rigid_body_handle, 0.5, 1);
        assert(r2RigidBody_GravityScale(rigid_body_handle) == 0.5);
        // DOCUSAURUS: Gravity2 stop
    }

    {
        // DOCUSAURUS: Forces start
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_ResetForces(rigid_body_handle, 1);  // Reset the forces to zero.
        r2RigidBody_ResetTorques(rigid_body_handle, 1); // Reset the torques to zero.
        r2RigidBody_AddForce(rigid_body_handle, r2Vector(0.0, 1000.0), 1);
        r2RigidBody_AddTorque(rigid_body_handle, 100.0, 1);
        r2RigidBody_AddForceAtPoint(rigid_body_handle, r2Vector(0.0, 1000.0), r2Vector(1.0, 2.0), 1);

        r2RigidBody_ApplyImpulse(rigid_body_handle, r2Vector(0.0, 1000.0), 1);
        r2RigidBody_ApplyTorqueImpulse(rigid_body_handle, 100.0, 1);
        r2RigidBody_ApplyImpulseAtPoint(rigid_body_handle, r2Vector(0.0, 1000.0), r2Vector(1.0, 2.0), 1);
        // DOCUSAURUS: Forces stop
        assert(r2RigidBody_UserForce(rigid_body_handle).y == 2000.0);
    }

    {
        // DOCUSAURUS: Mass1 start
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
        // The default density is 1.0, we are setting 2.0 for this example.
        R2ColliderDesc collider = r2BallColliderDesc(1.0);
        collider.density = 2.0;
        // When the collider is attached, the rigid-body's mass and angular
        // inertia is automatically updated to take the collider into account.
        r2InsertCollider(rigid_body_handle, &collider);
        // DOCUSAURUS: Mass1 stop
        assert(r2RigidBody_Mass(rigid_body_handle) > 0.0);
    }

    {
        // DOCUSAURUS: Mass2 start
        /* Set the mass-properties when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        rigid_body.additionalMass = 0.5;
        // Sets both the mass and angular inertia at once (this overrides `additionalMass`).
        rigid_body.useAdditionalMassProperties = 1;
        rigid_body.additionalMassProperties = (R2MassProperties){
            .local_com = r2Vector(0.0, 1.0),
            .mass = 0.5,
            .principal_inertia = 0.3,
        };
        // DOCUSAURUS: Mass2 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Mass3 start
        /* Set the mass-properties after the rigid-body creation. */
        R2MassProperties mass_properties = {
            .local_com = r2Vector(0.0, 1.0),
            .mass = 0.5,
            .principal_inertia = 0.3,
        };
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_SetAdditionalMassProperties(rigid_body_handle, mass_properties, 1);
        // DOCUSAURUS: Mass3 stop
    }

    {
        // DOCUSAURUS: LockedAxes1 start
        /* Lock translations/rotations when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        rigid_body.lockedAxes = R2_LOCK_TRANSLATION_X | R2_LOCK_TRANSLATION_Y // prevent translations along all axes.
                                | R2_LOCK_ROTATION_Z;                         // prevent rotations.
        // DOCUSAURUS: LockedAxes1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: LockedAxes2 start
        /* Lock translations/rotations after the rigid-body creation. */
        // The last `1` argument makes sure the rigid-body is awake.
        r2RigidBody_SetTranslationsLocked(rigid_body_handle, 1, 1);
        r2RigidBody_SetRotationsLocked(rigid_body_handle, 1, 1);
        // DOCUSAURUS: LockedAxes2 stop
        assert(r2RigidBody_LockedAxes(rigid_body_handle) & R2_LOCK_ROTATION_Z);
    }

    {
        // DOCUSAURUS: Damping1 start
        /* Set the damping coefficients when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        rigid_body.linearDamping = 0.5;
        rigid_body.angularDamping = 1.0;
        // DOCUSAURUS: Damping1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Damping2 start
        /* Set the damping coefficients after the rigid-body creation. */
        r2RigidBody_SetLinearDamping(rigid_body_handle, 0.5);
        r2RigidBody_SetAngularDamping(rigid_body_handle, 1.0);
        assert(r2RigidBody_LinearDamping(rigid_body_handle) == 0.5);
        assert(r2RigidBody_AngularDamping(rigid_body_handle) == 1.0);
        // DOCUSAURUS: Damping2 stop
    }

    {
        // DOCUSAURUS: Dominance1 start
        /* Set the dominance group when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        rigid_body.dominanceGroup = 10;
        // DOCUSAURUS: Dominance1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Dominance2 start
        /* Set the dominance group after the rigid-body creation. */
        r2RigidBody_SetDominanceGroup(rigid_body_handle, 10);
        assert(r2RigidBody_DominanceGroup(rigid_body_handle) == 10);
        // DOCUSAURUS: Dominance2 stop
    }

    {
        // DOCUSAURUS: Ccd1 start
        /* Enable CCD when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        rigid_body.ccdEnabled = 1;
        // DOCUSAURUS: Ccd1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Ccd2 start
        /* Enable CCD after the rigid-body creation. */
        r2RigidBody_SetCcdEnabled(rigid_body_handle, 1);
        assert(r2RigidBody_IsCcdEnabled(rigid_body_handle));
        // DOCUSAURUS: Ccd2 stop
    }

    {
        // DOCUSAURUS: Userdata1 start
        /* Set the user-data when the rigid-body is created. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        // The 128 bits of the user-data are split into its `low` and `high` 64 bits.
        rigid_body.userData.low = 42;
        // DOCUSAURUS: Userdata1 stop
        r2InsertRigidBody(world, &rigid_body);
    }

    {
        // DOCUSAURUS: Userdata2 start
        /* Set the user-data after the rigid-body creation. */
        R2UserData user_data = {.low = 42, .high = 0};
        r2RigidBody_SetUserData(rigid_body_handle, user_data);
        assert(r2RigidBody_UserData(rigid_body_handle).low == 42);
        // DOCUSAURUS: Userdata2 stop
    }

    {
        // DOCUSAURUS: SolverSettings start
        /* Give a rigid-body more solver accuracy than the rest of the scene. */
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        // Extra substeps run for the whole island component this body belongs to.
        rigid_body.additionalSolverIterations = 4;
        // Extra internal PGS iterations run per substep for that same component.
        rigid_body.additionalPgsIterations = 2;
        // Predictive contacts generated up to that distance ahead of the body's path: a cheaper
        // alternative to CCD for slow-but-thin or moderately fast objects.
        rigid_body.softCcdPrediction = 0.5;
        // Let the body exceed the angular speed cap, e.g. for a wheel.
        rigid_body.allowFastRotation = 1;
        // DOCUSAURUS: SolverSettings stop
        R2RigidBodyHandle handle = r2InsertRigidBody(world, &rigid_body);
        assert(r2RigidBody_AdditionalSolverIterations(handle) == 4);
        assert(r2RigidBody_AdditionalPgsIterations(handle) == 2);
        assert(r2RigidBody_SoftCcdPrediction(handle) == 0.5);
        assert(r2RigidBody_IsFastRotationAllowed(handle));
        r2RigidBody_SetAdditionalSolverIterations(handle, 1);
        r2RigidBody_SetAdditionalPgsIterations(handle, 1);
        r2RigidBody_SetSoftCcdPrediction(handle, 0.25);
        r2RigidBody_SetAllowFastRotation(handle, 0);
        assert(r2RigidBody_AdditionalSolverIterations(handle) == 1);
        assert(r2RigidBody_AdditionalPgsIterations(handle) == 1);
        assert(r2RigidBody_SoftCcdPrediction(handle) == 0.25);
        assert(!r2RigidBody_IsFastRotationAllowed(handle));
    }

    r2Step(world, NULL, NULL);
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
