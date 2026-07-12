import type * as RAPIER from "@dimforge/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

function buildBlock(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    halfExtents: RAPIER.Vector,
    shift: RAPIER.Vector,
    numx: number,
    numy: number,
    numz: number,
) {
    let halfExtents_yx = {x: halfExtents.y, y: halfExtents.x};
    let dimensions = [halfExtents, halfExtents_yx];
    let spacing = (halfExtents.y * numx - halfExtents.x) / (numz - 1.0);

    let i;
    let j;
    let y = 0.0;

    for (i = 0; i <= numy; ++i) {
        let dim = dimensions[i % 2];
        [numx, numz] = [numz, numx];

        for (j = 0; j < numx; ++j) {
            let x = i % 2 == 0 ? spacing * j * 2.0 : dim.x * j * 2.0;

            // Build the rigid body.
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                x + dim.x + shift.x,
                y + dim.y + shift.y,
            );
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(dim.x, dim.y);
            world.createCollider(colliderDesc, body);
        }

        y += dim.y * 2.0;
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 150.0;
    let groundHeight = 0.1;
    let bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        0.0,
        -groundHeight,
    );
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(groundSize, groundHeight);
    world.createCollider(colliderDesc, body);

    // Keva tower.
    let halfExtents = new RAPIER.Vector2(0.5, 2.0);
    let blockHeight = 0.0;
    // These should only be set to odd values otherwise
    // the blocks won't align in the nicest way.
    let numyArr = [0, 3, 5, 5, 7, 9];
    let numBlocksBuilt = 0;
    let i;

    for (i = 5; i >= 1; --i) {
        let numx = i * 15;
        let numy = numyArr[i];
        let numz = numx * 2 + 1;
        let blockWidth = numx * halfExtents.y * 2.0;
        buildBlock(
            RAPIER,
            world,
            halfExtents,
            new RAPIER.Vector2(-blockWidth / 2.0, blockHeight),
            numx,
            numy,
            numz,
        );
        blockHeight += (numy + 1) * (halfExtents.x + halfExtents.y);
        numBlocksBuilt += numx * numy;
    }

    testbed.setWorld(world);

    testbed.lookAt({
        target: {x: -10.0, y: -5.0},
        zoom: 4.0,
    });
}
