import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    /*
     * The ground
     */
    let ground_size = 1.7;
    let ground_height = 0.1;

    let bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        0.0,
        -ground_height,
        0.0,
    );
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(
        ground_size,
        ground_height,
        ground_size,
    );
    world.createCollider(colliderDesc, body);

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    bodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0.0, 3.0, 0.0)
        .lockTranslations()
        .enabledRotations(true, false, false);
    body = world.createRigidBody(bodyDesc);
    colliderDesc = RAPIER.ColliderDesc.cuboid(0.2, 0.6, 2.0);
    world.createCollider(colliderDesc, body);

    /*
     * A cylinder that cannot rotate.
     */
    bodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0.2, 5.0, 0.4)
        .lockRotations();
    body = world.createRigidBody(bodyDesc);
    colliderDesc = RAPIER.ColliderDesc.cylinder(0.6, 0.4);
    world.createCollider(colliderDesc, body);

    /*
     * Setup the testbed.
     */
    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: -10.0, y: 3.0, z: 0.0},
        target: {x: 0.0, y: 3.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
