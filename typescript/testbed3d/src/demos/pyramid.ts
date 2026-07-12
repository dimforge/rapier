import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0);
    world.createCollider(colliderDesc, body);

    // Dynamic cubes.
    let rad = 0.5;
    let num = 10;
    let i, j, k;
    let shift = rad * 2.5;
    let center = num * rad;
    let height = 10.0;

    for (i = 0; i < num; ++i) {
        for (j = i; j < num; ++j) {
            for (k = i; k < num; ++k) {
                let x =
                    (i * shift) / 2.0 + (k - i) * shift - height * rad - center;
                let y = i * shift + height;
                let z =
                    (i * shift) / 2.0 + (j - i) * shift - height * rad - center;

                // Create dynamic cube.
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x,
                    y,
                    z,
                );
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: -31.96000000000001, y: 19.730000000000008, z: -27.86},
        target: {x: -0.0505, y: -0.4126, z: -0.0229},
    };
    testbed.lookAt(cameraPosition);
}
