import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, 0.0, 0.0);
    let world = new RAPIER.World(gravity);

    /*
     * Create the cubes
     */
    let num = 10;
    let rad = 0.2;

    let subdiv = 1.0 / num;

    let i;
    for (i = 0; i < num; ++i) {
        let x = Math.sin(i * subdiv * Math.PI * 2.0);
        let y = Math.cos(i * subdiv * Math.PI * 2.0);

        // Build the rigid body.
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(x, y, 0.0)
            .setLinvel(x * 10.0, y * 10.0, 0.0)
            .setAngvel(new RAPIER.Vector3(0.0, 0.0, 100.0))
            .setLinearDamping((i + 1) * subdiv * 10.0)
            .setAngularDamping((num - i) * subdiv * 10.0);
        let body = world.createRigidBody(bodyDesc);

        // Build the collider.
        let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
        world.createCollider(colliderDesc, body);
    }

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: 0, y: 2.0, z: 20},
        target: {x: 0, y: 2.0, z: 0},
    };
    testbed.lookAt(cameraPosition);
}
