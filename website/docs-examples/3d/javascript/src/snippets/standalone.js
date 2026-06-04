
// DOCUSAURUS: NoBundler start
import RAPIER from 'https://cdn.skypack.dev/@dimforge/rapier3d-compat';

RAPIER.init().then(() => {
    // Run the simulation.
    _run_simulation(RAPIER);
});

// OR using the await syntax:
async function run_simulation() {
    await RAPIER.init();
    // Run the simulation.
    _run_simulation(RAPIER);
}
// DOCUSAURUS: NoBundler stop
run_simulation()

function _run_simulation(RAPIER) {
    // Use the RAPIER module here.
    let gravity = { x: 0.0, y: -9.81, z: 0.0 };
    let world = new RAPIER.World(gravity);

    // Create the ground
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(10.0, 0.1, 10.0);
    world.createCollider(groundColliderDesc);

    // Create a dynamic rigid-body.
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0.0, 1.0, 1.0);
    let rigidBody = world.createRigidBody(rigidBodyDesc);

    // Create a cuboid collider attached to the dynamic rigidBody.
    let colliderDesc = RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5);
    let collider = world.createCollider(colliderDesc, rigidBody);

    // Game loop. Replace by your own game loop system.
    let gameLoop = () => {
        // Step the simulation forward.  
        world.step();

        // Get and print the rigid-body's position.
        let position = rigidBody.translation();
        console.log("Rigid-body position: ", position.x, position.y, position.z);

        setTimeout(gameLoop, 16);
    };

    gameLoop();
}




