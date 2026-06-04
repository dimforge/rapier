import RAPIER from '@dimforge/rapier2d';

{
    // DOCUSAURUS: Creation start
    // The world that will contain our rigid-bodies.
    let world = new RAPIER.World({ x: 0.0, y: -9.81 });

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
        .setTranslation(0.0, 5.0)
        // The rigid body rotation.
        // Default: no rotation.
        .setRotation(5.0)
        // The linear velocity of this body.
        // Default: zero velocity.
        .setLinvel(1.0, 2.0)
        // The angular velocity of this body.
        // Default: zero velocity.
        .setAngvel(2.0)
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

let world = new RAPIER.World({ x: 0.0, y: -9.81 });
{
    // DOCUSAURUS: Position1 start
    /* Set the position when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        // The rigid body translation.
        // Default: zero vector.
        .setTranslation(0.0, 5.0)
        // The rigid body rotation.
        // Default: no rotation.
        .setRotation(5.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the position after the rigid-body creation. */
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.setTranslation({ x: 0.0, y: 5.0 }, true);
    rigidBody.setRotation(0.2, true);
    // DOCUSAURUS: Position2 stop
}

{
    // DOCUSAURUS: Velocity1 start
    /* Set the velocities when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        // The linear velocity of this body.
        // Default: zero velocity.
        .setLinvel(1.0, 3.0)
        // The angular velocity of this body.
        // Default: zero velocity.
        .setAngvel(3.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Velocity1 stop

    // DOCUSAURUS: Velocity2 start
    /* Set the velocities after the rigid-body creation. */
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.setLinvel({ x: 1.0, y: 3.0 }, true);
    rigidBody.setAngvel(3.0, true);
    // DOCUSAURUS: Velocity2 stop
}

{
    // DOCUSAURUS: Gravity1 start
    /* Set the gravity scale when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setGravityScale(2.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Gravity1 stop

    // DOCUSAURUS: Gravity2 start
    /* Set the gravity scale after the rigid-body creation. */
    rigidBody.setGravityScale(2.0, true);
    // DOCUSAURUS: Gravity2 stop

    // DOCUSAURUS: Forces start
    // The `true` argument makes sure the rigid-body is awake.
    rigidBody.resetForces(true);  // Reset the forces to zero.
    rigidBody.resetTorques(true); // Reset the torques to zero.
    rigidBody.addForce({ x: 0.0, y: 1000.0 }, true);
    rigidBody.addTorque(100.0, true);
    rigidBody.addForceAtPoint({ x: 0.0, y: 1000.0 }, { x: 1.0, y: 2.0 }, true);

    rigidBody.applyImpulse({ x: 0.0, y: 1000.0 }, true);
    rigidBody.applyTorqueImpulse(100.0, true);
    rigidBody.applyImpulseAtPoint({ x: 0.0, y: 1000.0 }, { x: 1.0, y: 2.0 }, true);
    // DOCUSAURUS: Forces stop
}

{
    // DOCUSAURUS: Mass1 start
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // The default density is 1.0, we are setting 2.0 for this example.
    let colliderDesc = RAPIER.ColliderDesc.ball(1.0).setDensity(2.0);
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Mass1 stop
}

{
    // DOCUSAURUS: Mass2 start
    /* Set the mass-properties when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setAdditionalMass(0.5)
        // Sets both the mass and angular inertia at once.
        .setAdditionalMassProperties(
            0.5,                // Mass.
            { x: 0.0, y: 1.0 }, // Center of mass.
            0.3                 // Principal angular inertia.
        );
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Mass2 stop
}

{
    // DOCUSAURUS: LockedAxes1 start
    /* Lock translations/rotations when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .lockTranslations() // prevent translations along along all axes.
        .lockRotations();   // prevent rotations.
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: LockedAxes1 stop

    // DOCUSAURUS: LockedAxes2 start
    /* Lock translations/rotations after the rigid-body creation. */
    // The last `true` argument makes sure the rigid-body is awake.
    rigidBody.lockTranslations(true, true);
    rigidBody.lockRotations(true, true);
    // DOCUSAURUS: LockedAxes2 stop
}

{
    // DOCUSAURUS: Damping1 start
    /* Set the damping coefficients when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setLinearDamping(0.5)
        .setAngularDamping(1.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Damping1 stop

    // DOCUSAURUS: Damping2 start
    /* Set the damping coefficients after the rigid-body creation. */
    rigidBody.setLinearDamping(0.5);
    rigidBody.setAngularDamping(1.0);
    // DOCUSAURUS: Damping2 stop
}

{
    // DOCUSAURUS: Dominance1 start
    /* Set the damping coefficients when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setDominanceGroup(10);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Dominance1 stop

    // DOCUSAURUS: Dominance2 start
    /* Set the damping coefficients after the rigid-body creation. */
    rigidBody.setDominanceGroup(10);
    // DOCUSAURUS: Dominance2 stop
}

{
    // DOCUSAURUS: Ccd1 start
    /* Enable CCD when the rigid-body is created. */
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setCcdEnabled(true);
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // DOCUSAURUS: Ccd1 stop

    // DOCUSAURUS: Ccd2 start
    /* Enable CCD after the rigid-body creation. */
    rigidBody.enableCcd(true);
    // DOCUSAURUS: Ccd2 stop
}