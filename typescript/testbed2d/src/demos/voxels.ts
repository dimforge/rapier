import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

function generateVoxels(n: number) {
    let points = [];

    let i;
    for (i = 0; i <= n; ++i) {
        let y = Math.max(-0.8, Math.min(Math.sin((i / n) * 10.0), 0.8)) * 8.0;
        points.push(i - n / 2.0, y);
    }
    return {
        points: new Float32Array(points),
        voxelSize: {x: 1.0, y: 1.2},
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let voxels = generateVoxels(100);
    let colliderDesc = RAPIER.ColliderDesc.voxels(
        voxels.points,
        voxels.voxelSize,
    );
    world.createCollider(colliderDesc, body);

    // Dynamic cubes.
    let num = 10;
    let numy = 4;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centery = shift / 2.0;

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            let x = i * shift + offset;
            let y = j * shift + centery + 10.0;

            // Create dynamic cube.
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc;

            switch (j % 3) {
                case 0:
                    colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad);
                    break;
                case 1:
                    colliderDesc = RAPIER.ColliderDesc.ball(rad);
                    break;
                case 2:
                    colliderDesc = RAPIER.ColliderDesc.cuboid(
                        rad / 2.0,
                        rad / 2.0,
                    );
                    world.createCollider(colliderDesc, body);
                    colliderDesc = RAPIER.ColliderDesc.cuboid(
                        rad / 2.0,
                        rad,
                    ).setTranslation(rad, 0.0);
                    world.createCollider(colliderDesc, body);
                    colliderDesc = RAPIER.ColliderDesc.cuboid(
                        rad / 2.0,
                        rad,
                    ).setTranslation(-rad, 0.0);
                    break;
            }

            world.createCollider(colliderDesc, body);
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 0.0, y: 0.0},
        zoom: 20.0,
    });
}
