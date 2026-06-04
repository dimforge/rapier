import RAPIER from '@dimforge/rapier3d';


let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });

{
    // DOCUSAURUS: UpVector start
    let characterController = world.createCharacterController(0.01);
    // Change the character controller’s up vector to the positive Z axis.
    characterController.setUp({ x: 0.0, y: 0.0, z: 1.0 });
    // DOCUSAURUS: UpVector stop
}