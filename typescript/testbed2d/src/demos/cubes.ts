import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 40.0;
    let grounds = [
        {x: 0.0, y: 0.0, hx: groundSize, hy: 0.1},
        {x: -groundSize, y: groundSize, hx: 0.1, hy: groundSize},
        {x: groundSize, y: groundSize, hx: 0.1, hy: groundSize},
    ];

    grounds.forEach((ground) => {
        let bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
            ground.x,
            ground.y,
        );
        let body = world.createRigidBody(bodyDesc);
        let colliderDesc = RAPIER.ColliderDesc.cuboid(ground.hx, ground.hy);
        world.createCollider(colliderDesc, body);
    });

    // Dynamic cubes.
    let num = 20;
    let numy = 50;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;

    let i, j;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            let x = i * shift - centerx;
            let y = j * shift + centery + 3.0;

            // Create dynamic cube.
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad);
            world.createCollider(colliderDesc, body);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: -10.0, y: -30.0},
        zoom: 7.0,
    });
}
