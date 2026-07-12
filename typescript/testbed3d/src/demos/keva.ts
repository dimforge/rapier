import type RAPIER from "@dimforge/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

function buildBlock(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    halfExtents: RAPIER.Vector,
    shift: RAPIER.Vector,
    numx: number,
    numy: number,
    numz: number,
) {
    let half_extents_zyx = {
        x: halfExtents.z,
        y: halfExtents.y,
        z: halfExtents.x,
    };
    let dimensions = [halfExtents, half_extents_zyx];
    let blockWidth = 2.0 * halfExtents.z * numx;
    let blockHeight = 2.0 * halfExtents.y * numy;
    let spacing = (halfExtents.z * numx - halfExtents.x) / (numz - 1.0);

    let i;
    let j;
    let k;

    for (i = 0; i < numy; ++i) {
        [numx, numz] = [numz, numx];
        let dim = dimensions[i % 2];
        let y = dim.y * i * 2.0;

        for (j = 0; j < numx; ++j) {
            let x = i % 2 == 0 ? spacing * j * 2.0 : dim.x * j * 2.0;

            for (k = 0; k < numz; ++k) {
                let z = i % 2 == 0 ? dim.z * k * 2.0 : spacing * k * 2.0;
                // Build the rigid body.
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x + dim.x + shift.x,
                    y + dim.y + shift.y,
                    z + dim.z + shift.z,
                );
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(
                    dim.x,
                    dim.y,
                    dim.z,
                );
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Close the top.
    let dim = {x: halfExtents.z, y: halfExtents.x, z: halfExtents.y};

    for (i = 0; i < blockWidth / (dim.x * 2.0); ++i) {
        for (j = 0; j < blockWidth / (dim.z * 2.0); ++j) {
            // Build the rigid body.
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                i * dim.x * 2.0 + dim.x + shift.x,
                dim.y + shift.y + blockHeight,
                j * dim.z * 2.0 + dim.z + shift.z,
            );
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(dim.x, dim.y, dim.z);
            world.createCollider(colliderDesc, body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 50.0;
    let groundHeight = 0.1;
    let bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        0.0,
        -groundHeight,
        0.0,
    );
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(
        groundSize,
        groundHeight,
        groundSize,
    );
    world.createCollider(colliderDesc, body);

    // Keva tower.
    let halfExtents = new RAPIER.Vector3(0.1, 0.5, 2.0);
    let blockHeight = 0.0;
    // These should only be set to odd values otherwise
    // the blocks won't align in the nicest way.
    let numyArr = [0, 3, 5, 5, 7, 9];
    let numBlocksBuilt = 0;
    let i;

    for (i = 5; i >= 1; --i) {
        let numx = i;
        let numy = numyArr[i];
        let numz = numx * 3 + 1;
        let blockWidth = numx * halfExtents.z * 2.0;
        buildBlock(
            RAPIER,
            world,
            halfExtents,
            new RAPIER.Vector3(
                -blockWidth / 2.0,
                blockHeight,
                -blockWidth / 2.0,
            ),
            numx,
            numy,
            numz,
        );
        blockHeight += numy * halfExtents.y * 2.0 + halfExtents.x * 2.0;
        numBlocksBuilt += numx * numy * numz;
    }

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {
            x: -70.38553832116718,
            y: 17.893810295517365,
            z: 29.34767842147597,
        },
        target: {
            x: 0.5890869353464383,
            y: 3.132044603021203,
            z: -0.2899937806661885,
        },
    };
    testbed.lookAt(cameraPosition);
}
