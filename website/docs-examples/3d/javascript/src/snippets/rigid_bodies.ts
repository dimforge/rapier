import RAPIER from '@dimforge/rapier3d';

{
    // DOCUSAURUS: Creation start
    // The world that will contain our rigid-bodies.
    let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });

    // Builder for a fixed rigid-body.
    let example1 = RAPIER.RigidBodyDesc.fixed();
    // Builder for a dynamic rigid-body.
    let example2 = RAPIER.RigidBodyDesc.dynamic();
    // Builder for a kinematic rigid-body controlled at the velocity level.
    let example3 = RAPIER.RigidBodyDesc.kinematicVelocityBased();
    // Builder for a kinematic rigid-body controlled at the position level.
    let example4 = RAPIER.RigidBodyDesc.kinematicPositionBased();
    // Builder for a body with a status specified by an enum.
    let rigidBodyDesc = new RAPIER.RigidBodyDesc(RAPIER.RigidBodyType.Dynamic)
        // The rigid body translation.
        // Default: zero vector.
        .setTranslation(0.0, 5.0, 1.0)
        // The rigid body rotation, given as a quaternion.
        // Default: no rotation.
        .setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 })
        // The linear velocity of this body.
        // Default: zero velocity.
        .setLinvel(1.0, 3.0, 4.0)
        // The angular velocity of this body.
        // Default: zero velocity.
        .setAngvel({ x: 3.0, y: 0.0, z: 1.0 })
        // The scaling factor applied to the gravity affecting the rigid-body.
        // Default: 1.0
        .setGravityScale(0.5)
        // Whether or not this body can sleep.
        // Default: true
        .setCanSleep(true)
        // Whether or not CCD is enabled for this rigid-body.
        // Default: false
        .setCcdEnabled(false);

    // All done, actually build the rigid-body.
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // The integer handle of the rigid-body can be read from the `handle` field. 
    let rigidBodyHandle = rigidBody.handle;
    // DOCUSAURUS: Creation stop

    let position = rigidBody.translation();
    console.log("Rigid-body position: ", position.x, position.y);
}

let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
{
    // DOCUSAURUS: Position1 start
    /* Set the position when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        // The rigid body translation.
        // Default: zero vector.
        .setTranslation(0.0, 5.0, 1.0)
        // The rigid body rotation, given as a quaternion.
        // Default: no rotation.
        .setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the position after the rigid-body creation. */
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.setTranslation({ x: 0.0, y: 5.0, z: 1.0 }, true);
    rigidBody.setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 }, true);
    // DOCUSAURUS: Position2 stop
}

{
    // DOCUSAURUS: Velocity1 start
    /* Set the velocities when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        // The linear velocity of this body.
        // Default: zero velocity.
        .setLinvel(1.0, 3.0, 4.0)
        // The angular velocity of this body.
        // Default: zero velocity.
        .setAngvel({ x: 3.0, y: 0.0, z: 0.0 });
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Velocity1 stop

    // DOCUSAURUS: Velocity2 start
    /* Set the velocities after the rigid-body creation. */
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.setLinvel({ x: 1.0, y: 3.0, z: 4.0 }, true);
    rigidBody.setAngvel({ x: 3.0, y: 0.0, z: 0.0 }, true);
    // DOCUSAURUS: Velocity2 stop
}

{
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);

    // DOCUSAURUS: Forces start
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.resetForces(true);  // Reset the forces to zero.
    rigidBody.resetTorques(true); // Reset the torques to zero.
    rigidBody.addForce({ x: 0.0, y: 1000.0, z: 0.0 }, true);
    rigidBody.addTorque({ x: 100.0, y: 0.0, z: 0.0 }, true);
    rigidBody.addForceAtPoint({ x: 0.0, y: 1000.0, z: 0.0 }, { x: 1.0, y: 2.0, z: 3.0 }, true);

    rigidBody.applyImpulse({ x: 0.0, y: 1000.0, z: 0.0 }, true);
    rigidBody.applyTorqueImpulse({ x: 100.0, y: 0.0, z: 0.0 }, true);
    rigidBody.applyImpulseAtPoint({ x: 0.0, y: 1000.0, z: 0.0 }, { x: 1.0, y: 2.0, z: 3.0 }, true);
    // DOCUSAURUS: Forces stop
}

{
    // DOCUSAURUS: Mass2 start
    /* Set the mass-properties when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setAdditionalMass(0.5)
        // Sets both the mass and angular inertia at once.
        .setAdditionalMassProperties(
            0.5,                                // Mass.
            { x: 0.0, y: 1.0, z: 0.0 },         // Center of mass.
            { x: 0.3, y: 0.2, z: 0.1 },         // Principal angular inertia.
            { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }  // Principal angular inertia frame (unit quaternion).
        );
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Mass2 stop
}

{
    // DOCUSAURUS: LockedAxes1 start
    /* Lock translations/rotations when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .lockTranslations() // prevent translations along along all axes.
        .lockRotations()   // prevent rotations along all axes.
        .enabledRotations(true, false, false); // only enable rotations along the X axis.
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: LockedAxes1 stop

    // DOCUSAURUS: LockedAxes2 start
    /* Lock translations/rotations after the rigid-body creation. */
    // The last `true` argument makes sure the rigid-body is awake.
    rigidBody.lockTranslations(true, true);
    rigidBody.lockRotations(true, true);
    rigidBody.setEnabledRotations(true, false, false, true);
    // DOCUSAURUS: LockedAxes2 stop
}
