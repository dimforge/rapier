import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let groundBody = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(5.0, 0.1, 5.0);
    world.createCollider(colliderDesc, groundBody);

    // Setup groups.
    let group1 = 0x00010001;
    let group2 = 0x00020002;

    // Add one floor that collides with the first group only.
    colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1, 1.0)
        .setTranslation(0.0, 1.0, 0.0)
        .setCollisionGroups(group1);
    world.createCollider(colliderDesc, groundBody);

    // Add one floor that collides with the second group only.
    colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1, 1.0)
        .setTranslation(0.0, 2.0, 0.0)
        .setCollisionGroups(group2);
    world.createCollider(colliderDesc, groundBody);

    // Dynamic cubes.
    let num = 8;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2);
    let centery = 2.5;
    let centerz = shift * (num / 2);
    let i, j, k;

    for (j = 0; j < 4; j++) {
        for (i = 0; i < num; i++) {
            for (k = 0; k < num; k++) {
                let x = i * shift - centerx;
                let y = j * shift + centery;
                let z = k * shift - centerz;

                // Alternate between the green and blue groups.
                let group = k % 2 == 0 ? group1 : group2;
                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x,
                    y,
                    z,
                );
                let body = world.createRigidBody(bodyDesc);

                colliderDesc = RAPIER.ColliderDesc.cuboid(
                    rad,
                    rad,
                    rad,
                ).setCollisionGroups(group);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: 10.0, y: 5.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
