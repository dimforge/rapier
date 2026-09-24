import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

function generateHeightfield(nsubdivs: number) {
    let heights = [];

    let rng = seedrandom("heightfield");

    let i, j;
    for (i = 0; i <= nsubdivs; ++i) {
        for (j = 0; j <= nsubdivs; ++j) {
            heights.push(rng());
        }
    }

    return new Float32Array(heights);
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let nsubdivs = 20;
    let scale = new RAPIER.Vector3(70.0, 4.0, 70.0);
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let heights = generateHeightfield(nsubdivs);
    let colliderDesc = RAPIER.ColliderDesc.heightfield(
        nsubdivs,
        nsubdivs,
        heights,
        scale,
    );
    world.createCollider(colliderDesc, body);

    // Dynamic cubes.
    let num = 4;
    let numy = 10;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centery = shift / 2.0;

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j, k;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift + offset;
                let y = j * shift + centery + 3.0;
                let z = k * shift + offset;

                // Create dynamic cube.
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x,
                    y,
                    z,
                );
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc;

                switch (j % 5) {
                    case 0:
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad,
                            rad,
                            rad,
                        );
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
                    case 4:
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad / 2.0,
                            rad / 2.0,
                            rad / 2.0,
                        );
                        world.createCollider(colliderDesc, body);
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad / 2.0,
                            rad,
                            rad / 2.0,
                        ).setTranslation(rad, 0.0, 0.0);
                        world.createCollider(colliderDesc, body);
                        colliderDesc = RAPIER.ColliderDesc.cuboid(
                            rad / 2.0,
                            rad,
                            rad / 2.0,
                        ).setTranslation(-rad, 0.0, 0.0);
                        break;
                }

                world.createCollider(colliderDesc, body);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {
            x: -88.48024008669711,
            y: 46.911325612198354,
            z: 83.56055570254844,
        },
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
