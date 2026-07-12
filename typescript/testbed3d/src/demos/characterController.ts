import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(15.0, 0.1, 15.0);
    world.createCollider(colliderDesc, body);

    // Dynamic cubes.
    let rad = 0.5;
    let num = 5;
    let i, j, k;
    let shift = rad * 2.5;
    let center = num * rad;
    let height = 5.0;

    for (i = 0; i < num; ++i) {
        for (j = i; j < num; ++j) {
            for (k = i; k < num; ++k) {
                let x = (i * shift) / 2.0 + (k - i) * shift - center;
                let y = (i * shift) / 2.0 + height;
                let z = (i * shift) / 2.0 + (j - i) * shift - center;

                // Create dynamic cube.
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x,
                    y,
                    z,
                );
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(
                    rad,
                    rad / 2.0,
                    rad,
                );
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Character.
    let characterDesc =
        RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(
            -10.0,
            4.0,
            -10.0,
        );
    let character = world.createRigidBody(characterDesc);
    let characterColliderDesc = RAPIER.ColliderDesc.cylinder(1.2, 0.6);
    let characterCollider = world.createCollider(
        characterColliderDesc,
        character,
    );

    let characterController = world.createCharacterController(0.1);
    characterController.enableAutostep(0.7, 0.3, true);
    characterController.enableSnapToGround(0.7);

    let speed = 0.2;
    let movementDirection = {x: 0.0, y: -speed, z: 0.0};

    let updateCharacter = () => {
        characterController.computeColliderMovement(
            characterCollider,
            movementDirection,
        );

        let movement = characterController.computedMovement();
        let newPos = character.translation();
        newPos.x += movement.x;
        newPos.y += movement.y;
        newPos.z += movement.z;
        character.setNextKinematicTranslation(newPos);
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(updateCharacter);

    document.onkeydown = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp") movementDirection.x = speed;
        if (event.key == "ArrowDown") movementDirection.x = -speed;
        if (event.key == "ArrowLeft") movementDirection.z = -speed;
        if (event.key == "ArrowRight") movementDirection.z = speed;
        if (event.key == " ") movementDirection.y = speed;
    };

    document.onkeyup = function (event: KeyboardEvent) {
        if (event.key == "ArrowUp") movementDirection.x = 0.0;
        if (event.key == "ArrowDown") movementDirection.x = 0.0;
        if (event.key == "ArrowLeft") movementDirection.z = 0.0;
        if (event.key == "ArrowRight") movementDirection.z = 0.0;
        if (event.key == " ") movementDirection.y = -speed; // Gravity
    };

    let cameraPosition = {
        eye: {x: -40.0, y: 19.730000000000008, z: 0.0},
        target: {x: 0.0, y: -0.4126, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
