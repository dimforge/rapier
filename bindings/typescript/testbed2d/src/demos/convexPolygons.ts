import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    /*
     * Ground
     */
    // Create Ground.
    let groundSize = 30.0;
    let grounds = [
        {x: 0.0, y: 0.0, hx: groundSize, hy: 1.2},
        {x: -groundSize, y: groundSize, hx: 1.2, hy: groundSize},
        {x: groundSize, y: groundSize, hx: 1.2, hy: groundSize},
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

    /*
     * Create the convex polygons
     */
    let num = 14;
    let scale = 4.0;

    let shift = scale;
    let centerx = (shift * num) / 2.0;
    let centery = shift / 2.0;

    let i, j, k;
    let rng = seedrandom("convexPolygon");

    for (i = 0; i < num; ++i) {
        for (j = 0; j < num * 2; ++j) {
            let x = i * shift - centerx;
            let y = j * shift * 2.0 + centery + 2.0;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            let body = world.createRigidBody(bodyDesc);

            let points = [];
            for (k = 0; k < 10; ++k) {
                points.push(rng() * scale, rng() * scale);
            }
            let colliderDesc = RAPIER.ColliderDesc.convexHull(
                new Float32Array(points),
            );
            world.createCollider(colliderDesc, body);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: -10.0, y: -30.0},
        zoom: 7.0,
    });
}
