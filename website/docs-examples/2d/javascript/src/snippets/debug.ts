import { Graphics } from "./graphics/Graphics";

import('@dimforge/rapier2d').then(RAPIER => {
    // Use the RAPIER module here.
    let gravity = { x: 0.0, y: -9.81 };
    let world = new RAPIER.World(gravity);

    // Create the ground
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(10.0, 0.1);
    world.createCollider(groundColliderDesc);
    world.lengthUnit = 1;

    // Create a dynamic rigid-body.
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0.0, 10.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);

    // Create a cuboid collider attached to the dynamic rigidBody.
    let colliderDesc = RAPIER.ColliderDesc.cuboid(0.5, 0.5);
    let collider = world.createCollider(colliderDesc, rigidBody);


    let graphics = new Graphics();
    graphics.lookAt({ zoom: 35.0, target: { x: 0.0, y: 0.0 } });
    // Game loop. Replace by your own game loop system.
    let gameLoop = () => {
        // Step the simulation forward.  
        world.step();
        // Get and print the rigid-body's position.
        let position = rigidBody.translation();
        console.log("Rigid-body position: ", position.x, position.y);

        graphics.render(world);
        setTimeout(gameLoop, 16);
    };

    gameLoop();
})