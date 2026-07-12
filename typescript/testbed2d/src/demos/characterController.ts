import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(15.0, 0.1);
    world.createCollider(colliderDesc, body);

    // Dynamic cubes.
    let rad = 0.5;
    let num = 5;
    let i, j, k;
    let shift = rad * 2.5;
    let center = num * rad;
    let height = 5.0;

    for (i = 0; i < num; ++i) {
        for (k = i; k < num; ++k) {
            let x = (i * shift) / 2.0 + (k - i) * shift - center;
            let y = (i * shift) / 2.0 + height;

            // Create dynamic cube.
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad / 2.0);
            world.createCollider(colliderDesc, body);
        }
    }

    // Character.
    let characterDesc =
        RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
            -10.0,
            4.0,
        );
    let character = world.createRigidBody(characterDesc);
    let characterColliderDesc = RAPIER.ColliderDesc.cuboid(0.6, 1.2);
    let characterCollider = world.createCollider(
        characterColliderDesc,
        character,
    );

    let characterController = world.createCharacterController(0.1);
    characterController.enableAutostep(0.7, 0.3, true);
    characterController.enableSnapToGround(0.7);

    let speed = 0.2;
    let movementDirection = {x: 0.0, y: -speed};

    let updateCharacter = () => {
        characterController.computeColliderMovement(
            characterCollider,
            movementDirection,
        );

        let movement = characterController.computedMovement();
        let newPos = character.translation();
        newPos.x += movement.x;
        newPos.y += movement.y;
        character.setNextKinematicTranslation(newPos);
        console.log("here");
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updateCharacter);

    document.onkeydown = function (event: KeyboardEvent) {
        if (event.key == "ArrowLeft") movementDirection.x = -speed;
        if (event.key == "ArrowRight") movementDirection.x = speed;
        if (event.key == " ") movementDirection.y = speed;
    };

    document.onkeyup = function (event: KeyboardEvent) {
        if (event.key == "ArrowLeft") movementDirection.x = 0.0;
        if (event.key == "ArrowRight") movementDirection.x = 0.0;
        if (event.key == " ") movementDirection.y = -speed; // Gravity
    };

    testbed.lookAt({
        target: {x: 0.0, y: -1.0},
        zoom: 50.0,
    });
}
