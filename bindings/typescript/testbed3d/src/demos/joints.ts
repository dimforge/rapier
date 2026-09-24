import type RAPIER from "@dimforge/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

function createPrismaticJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector,
    num: number,
) {
    let rad = 0.4;
    let shift = 1.0;

    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        origin.x,
        origin.y,
        origin.z,
    );
    let currParent = world.createRigidBody(groundDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
    world.createCollider(colliderDesc, currParent);

    let i;
    let z;

    for (i = 0; i < num; ++i) {
        z = origin.z + (i + 1) * shift;
        let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            origin.x,
            origin.y,
            z,
        );
        let currChild = world.createRigidBody(rigidBodyDesc);
        let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
        world.createCollider(colliderDesc, currChild);

        let axis;

        if (i % 2 == 0) {
            axis = new RAPIER.Vector3(1.0, 1.0, 0.0);
        } else {
            axis = new RAPIER.Vector3(-1.0, 1.0, 0.0);
        }

        z = new RAPIER.Vector3(0.0, 0.0, 1.0);
        let prism = RAPIER.JointData.prismatic(
            new RAPIER.Vector3(0.0, 0.0, 0.0),
            new RAPIER.Vector3(0.0, 0.0, -shift),
            axis,
        );
        prism.limitsEnabled = true;
        prism.limits = [-2.0, 2.0];
        world.createImpulseJoint(prism, currParent, currChild, true);

        currParent = currChild;
    }
}

function createRevoluteJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector3,
    num: number,
) {
    let rad = 0.4;
    let shift = 2.0;

    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        origin.x,
        origin.y,
        0.0,
    );
    let currParent = world.createRigidBody(groundDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
    world.createCollider(colliderDesc, currParent);

    let i, k;
    let z;

    for (i = 0; i < num; ++i) {
        // Create four bodies.
        z = origin.z + i * shift * 2.0 + shift;

        let positions = [
            new RAPIER.Vector3(origin.x, origin.y, z),
            new RAPIER.Vector3(origin.x + shift, origin.y, z),
            new RAPIER.Vector3(origin.x + shift, origin.y, z + shift),
            new RAPIER.Vector3(origin.x, origin.y, z + shift),
        ];

        let parents = [currParent, currParent, currParent, currParent];

        for (k = 0; k < 4; ++k) {
            let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                positions[k].x,
                positions[k].y,
                positions[k].z,
            );
            let rigidBody = world.createRigidBody(rigidBodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
            world.createCollider(colliderDesc, rigidBody);

            parents[k] = rigidBody;
        }

        // Setup four joints.
        let o = new RAPIER.Vector3(0.0, 0.0, 0.0);
        let x = new RAPIER.Vector3(1.0, 0.0, 0.0);
        z = new RAPIER.Vector3(0.0, 0.0, 1.0);

        let revs = [
            RAPIER.JointData.revolute(
                o,
                new RAPIER.Vector3(0.0, 0.0, -shift),
                z,
            ),
            RAPIER.JointData.revolute(
                o,
                new RAPIER.Vector3(-shift, 0.0, 0.0),
                x,
            ),
            RAPIER.JointData.revolute(
                o,
                new RAPIER.Vector3(0.0, 0.0, -shift),
                z,
            ),
            RAPIER.JointData.revolute(
                o,
                new RAPIER.Vector3(shift, 0.0, 0.0),
                x,
            ),
        ];

        world.createImpulseJoint(revs[0], currParent, parents[0], true);
        world.createImpulseJoint(revs[1], parents[0], parents[1], true);
        world.createImpulseJoint(revs[2], parents[1], parents[2], true);
        world.createImpulseJoint(revs[3], parents[2], parents[3], true);

        currParent = parents[3];
    }
}

function createFixedJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector3,
    num: number,
) {
    let rad = 0.4;
    let shift = 1.0;
    let i, k;
    let parents = [];

    for (k = 0; k < num; ++k) {
        for (i = 0; i < num; ++i) {
            let fk = k;
            let fi = i;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let bodyType;

            if (i == 0 && ((k % 4 == 0 && k != num - 2) || k == num - 1)) {
                bodyType = RAPIER.RigidBodyType.Fixed;
            } else {
                bodyType = RAPIER.RigidBodyType.Dynamic;
            }

            let rigidBody = new RAPIER.RigidBodyDesc(bodyType).setTranslation(
                origin.x + fk * shift,
                origin.y,
                origin.z + fi * shift,
            );
            let child = world.createRigidBody(rigidBody);
            let colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            if (i > 0) {
                let parent = parents[parents.length - 1];
                let params = RAPIER.JointData.fixed(
                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                    new RAPIER.Vector3(0.0, 0.0, -shift),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                );

                world.createImpulseJoint(params, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                let parent_index = parents.length - num;
                let parent = parents[parent_index];
                let params = RAPIER.JointData.fixed(
                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                    new RAPIER.Vector3(-shift, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                );

                world.createImpulseJoint(params, parent, child, true);
            }

            parents.push(child);
        }
    }
}

function createBallJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    num: number,
) {
    let rad = 0.4;
    let shift = 1.0;
    let i, k;
    let parents = [];

    for (k = 0; k < num; ++k) {
        for (i = 0; i < num; ++i) {
            let fk = k;
            let fi = i;

            let bodyType;

            if (i == 0 && (k % 4 == 0 || k == num - 1)) {
                bodyType = RAPIER.RigidBodyType.Fixed;
            } else {
                bodyType = RAPIER.RigidBodyType.Dynamic;
            }

            let bodyDesc = new RAPIER.RigidBodyDesc(bodyType).setTranslation(
                fk * shift,
                0.0,
                fi * shift,
            );
            let child = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            let o = new RAPIER.Vector3(0.0, 0.0, 0.0);

            if (i > 0) {
                let parent = parents[parents.length - 1];
                let params = RAPIER.JointData.spherical(
                    o,
                    new RAPIER.Vector3(0.0, 0.0, -shift),
                );
                world.createImpulseJoint(params, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                let parent_index = parents.length - num;
                let parent = parents[parent_index];
                let params = RAPIER.JointData.spherical(
                    o,
                    new RAPIER.Vector3(-shift, 0.0, 0.0),
                );
                world.createImpulseJoint(params, parent, child, true);
            }

            parents.push(child);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    createPrismaticJoints(
        RAPIER,
        world,
        new RAPIER.Vector3(20.0, 10.0, 0.0),
        5,
    );
    createFixedJoints(RAPIER, world, new RAPIER.Vector3(0.0, 10.0, 0.0), 5);
    createRevoluteJoints(RAPIER, world, new RAPIER.Vector3(20.0, 0.0, 0.0), 3);
    createBallJoints(RAPIER, world, 15);

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: 15.0, y: 5.0, z: 42.0},
        target: {x: 13.0, y: 1.0, z: 1.0},
    };
    testbed.lookAt(cameraPosition);
}
