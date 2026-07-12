import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

function generateTriMesh(nsubdivs: number, wx: number, wy: number, wz: number) {
    let vertices = [];
    let indices = [];

    let elementWidth = 1.0 / nsubdivs;
    let rng = seedrandom("trimesh");

    let i, j;
    for (i = 0; i <= nsubdivs; ++i) {
        for (j = 0; j <= nsubdivs; ++j) {
            let x = (j * elementWidth - 0.5) * wx;
            let y = rng() * wy;
            let z = (i * elementWidth - 0.5) * wz;

            vertices.push(x, y, z);
        }
    }

    for (i = 0; i < nsubdivs; ++i) {
        for (j = 0; j < nsubdivs; ++j) {
            let i1 = (i + 0) * (nsubdivs + 1) + (j + 0);
            let i2 = (i + 0) * (nsubdivs + 1) + (j + 1);
            let i3 = (i + 1) * (nsubdivs + 1) + (j + 0);
            let i4 = (i + 1) * (nsubdivs + 1) + (j + 1);

            indices.push(i1, i3, i2);
            indices.push(i3, i4, i2);
        }
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let trimesh = generateTriMesh(20, 70.0, 4.0, 70.0);
    let colliderDesc = RAPIER.ColliderDesc.trimesh(
        trimesh.vertices,
        trimesh.indices,
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
