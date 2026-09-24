import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);
    let bodies = [];

    let rad = 0.4;
    let numi = 30; // Num vertical nodes.
    let numk = 30; // Num horizontal nodes.
    let shift = 1.0;
    let i, k;

    for (k = 0; k < numk; ++k) {
        for (i = 0; i < numi; ++i) {
            let status =
                k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0
                    ? RAPIER.RigidBodyType.Fixed
                    : RAPIER.RigidBodyType.Dynamic;

            let bodyDesc = new RAPIER.RigidBodyDesc(status).setTranslation(
                k * shift,
                -i * shift,
            );
            let child = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            if (i > 0) {
                let parent = bodies[bodies.length - 1];
                let anchor1 = new RAPIER.Vector2(0.0, 0.0);
                let anchor2 = new RAPIER.Vector2(0.0, shift);
                let JointData = RAPIER.JointData.revolute(anchor1, anchor2);
                world.createImpulseJoint(JointData, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                let parentIndex = bodies.length - numi;
                let parent = bodies[parentIndex];
                let anchor1 = new RAPIER.Vector2(0.0, 0.0);
                let anchor2 = new RAPIER.Vector2(-shift, 0.0);
                let JointData = RAPIER.JointData.revolute(anchor1, anchor2);
                world.createImpulseJoint(JointData, parent, child, true);
            }

            bodies.push(child);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 30.0, y: 30.0},
        zoom: 10.0,
    });
}
