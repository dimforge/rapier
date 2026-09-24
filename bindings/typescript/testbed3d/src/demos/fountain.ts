import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);
    let removableBodies = new Array();

    // Create Ground.
    let groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    let groundBody = world.createRigidBody(groundBodyDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(40.0, 0.1, 40.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    let rad = 1.0;
    let j = 0;
    let spawn_interval = 5;

    let spawnBodies = (graphics: Testbed["graphics"]) => {
        j += 1;
        if (j % spawn_interval != 0) {
            return;
        }

        let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setLinvel(0.0, 15.0, 0.0)
            .setTranslation(0.0, 10.0, 0.0);
        let colliderDesc;

        switch ((j / spawn_interval) % 4) {
            case 0:
                colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
                break;
            case 1:
                colliderDesc = RAPIER.ColliderDesc.ball(rad);
                break;
            case 2:
                colliderDesc = RAPIER.ColliderDesc.roundCylinder(
                    rad,
                    rad,
                    rad / 10.0,
                );
                break;
            case 3:
                colliderDesc = RAPIER.ColliderDesc.cone(rad, rad);
                break;
        }

        let body = world.createRigidBody(bodyDesc);
        let collider = world.createCollider(colliderDesc, body);
        graphics.addCollider(RAPIER, world, collider);

        removableBodies.push(body);

        // We reached the max number, delete the oldest rigid-body.
        if (removableBodies.length > 400) {
            let rb = removableBodies[0];
            world.removeRigidBody(rb);
            graphics.removeRigidBody(rb);
            removableBodies.shift();
        }
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(spawnBodies);

    let cameraPosition = {
        eye: {
            x: -88.48024008669711,
            y: 46.911325612198354,
            z: 83.56055570254844,
        },
        target: {x: 0.0, y: 10.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
