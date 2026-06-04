import RAPIER from '@dimforge/rapier2d';


let world = new RAPIER.World({ x: 0.0, y: -9.81 });
{

    // DOCUSAURUS: Setup1 start
    // The gap the controller will leave between the character and its environment.
    let offset = 0.01;
    // Create the controller.
    let characterController = world.createCharacterController(offset);
    // Remove the controller once we are done with it.
    world.removeCharacterController(characterController);
    // DOCUSAURUS: Setup1 stop
}

{
    let offset = 0.01;
    let desiredTranslation = { x: 1, y: 0 };
    let colliderDesc = new RAPIER.ColliderDesc(new RAPIER.Ball(0.5));
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Setup2 start
    let characterController = world.createCharacterController(offset);
    characterController.computeColliderMovement(
        collider,    // The collider we would like to move.
        desiredTranslation, // The movement we would like to apply if there wasn’t any obstacle.
    );
    // Read the result.
    let correctedMovement = characterController.computedMovement();
    // TODO: apply this corrected movement by following the rules described below.
    // DOCUSAURUS: Setup2 stop
}

{
    // DOCUSAURUS: Offset start
    // Here the character controller is initialized with an offset of 0.01.
    let offset = 0.01;
    let characterController = world.createCharacterController(0.01);
    // DOCUSAURUS: Offset stop
}
{
    // DOCUSAURUS: UpVector start
    let characterController = world.createCharacterController(0.01);
    // Change the character controller’s up vector to the positive X axis.
    characterController.setUp({ x: 1.0, y: 0.0 });
    // DOCUSAURUS: UpVector stop
}
{
    // DOCUSAURUS: Slopes start
    let characterController = world.createCharacterController(0.01);
    // Don’t allow climbing slopes larger than 45 degrees.
    characterController.setMaxSlopeClimbAngle(45 * Math.PI / 180);
    // Automatically slide down on slopes smaller than 30 degrees.
    characterController.setMinSlopeSlideAngle(30 * Math.PI / 180);
    // DOCUSAURUS: Slopes stop
}

{
    // DOCUSAURUS: Stairs start
    let characterController = world.createCharacterController(0.01);
    // Autostep if the step height is smaller than 0.5, its width is larger than 0.2,
    // and allow stepping on dynamic bodies.
    characterController.enableAutostep(0.5, 0.2, true);
    // Disable autostep.
    characterController.disableAutostep();
    // DOCUSAURUS: Stairs stop
}

{
    // DOCUSAURUS: Snap start
    let characterController = world.createCharacterController(0.01);
    // Snap to the ground if the vertical distance to the ground is smaller than 0.5.
    characterController.enableSnapToGround(0.5);
    // Disable snap-to-ground.
    characterController.disableSnapToGround();
    // DOCUSAURUS: Snap stop
}

{
    let colliderDesc = new RAPIER.ColliderDesc(new RAPIER.Ball(0.5));
    let collider = world.createCollider(colliderDesc);
    let desiredMovementVector = { x: 1, y: 0 };
    // DOCUSAURUS: Collisions1 start
    let characterController = world.createCharacterController(0.01);
    characterController.computeColliderMovement(collider, desiredMovementVector);

    // After the collider movement calculation is done, we can read the
    // collision events.
    for (let i = 0; i < characterController.numComputedCollisions(); i++) {
        let collision = characterController.computedCollision(i);
        // Do something with that collision information.
    }
    // DOCUSAURUS: Collisions1 stop
}

{
    // DOCUSAURUS: Collisions2 start
    let characterController = world.createCharacterController(0.01);
    // Enable the automatic application of impulses to the dynamic bodies
    // hit by the character along its path.
    characterController.setApplyImpulsesToDynamicBodies(true);
    // DOCUSAURUS: Collisions2 stop
}