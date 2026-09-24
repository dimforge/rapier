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
    let characterDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(-10.0, 4.0, -10.0)
        .setGravityScale(10.0)
        .setSoftCcdPrediction(10.0);
    let character = world.createRigidBody(characterDesc);
    let characterColliderDesc = RAPIER.ColliderDesc.cylinder(1.2, 0.6);
    world.createCollider(characterColliderDesc, character);

    let pidController = world.createPidController(
        60.0,
        0.0,
        1.0,
        RAPIER.PidAxesMask.AllAng,
    );
    let speed = 0.2;
    let movementDirection = {x: 0.0, y: 0.0, z: 0.0};
    let targetVelocity = {x: 0.0, y: 0.0, z: 0.0};
    let targetRotation = new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0);

    let updateCharacter = () => {
        if (
            movementDirection.x == 0.0 &&
            movementDirection.y == 0.0 &&
            movementDirection.z == 0.0
        ) {
            // Only adjust the rotation, but let translation.
            pidController.setAxes(RAPIER.PidAxesMask.AllAng);
        } else if (movementDirection.y == 0.0) {
            // Don’t control the linear Y axis so the player can fall down due to gravity.
            pidController.setAxes(
                RAPIER.PidAxesMask.AllAng |
                    RAPIER.PidAxesMask.LinX |
                    RAPIER.PidAxesMask.LinZ,
            );
        } else {
            pidController.setAxes(RAPIER.PidAxesMask.All);
        }

        let targetPoint = character.translation();
        targetPoint.x += movementDirection.x;
        targetPoint.y += movementDirection.y;
        targetPoint.z += movementDirection.z;

        pidController.applyLinearCorrection(
            character,
            targetPoint,
            targetVelocity,
        );
        pidController.applyAngularCorrection(
            character,
            targetRotation,
            targetVelocity,
        );
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
        if (event.key == " ") movementDirection.y = 0.0;
    };

    let cameraPosition = {
        eye: {x: -40.0, y: 19.730000000000008, z: 0.0},
        target: {x: 0.0, y: -0.4126, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
