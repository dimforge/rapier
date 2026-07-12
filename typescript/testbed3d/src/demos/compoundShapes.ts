import type {Testbed} from "../Testbed";
import type {ColliderDesc} from "@dimforge/rapier3d";
import seedrandom from "seedrandom";

type RAPIER_API = typeof import("@dimforge/rapier3d");

function createLShape(RAPIER: RAPIER_API): ColliderDesc {
    const shape1 = new RAPIER.Cuboid(2.0, 0.5, 0.5);
    const shape2 = new RAPIER.Cuboid(0.5, 1.5, 0.5);

    const shapes = [shape1, shape2];
    const positions = [
        new RAPIER.Vector3(0.0, 0.0, 0.0),
        new RAPIER.Vector3(-1.5, -1.5, 0.0),
    ];
    const rotations = [
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
    ];

    return RAPIER.ColliderDesc.compound(shapes, positions, rotations);
}

function createTShape(RAPIER: RAPIER_API): ColliderDesc {
    const shape1 = new RAPIER.Cuboid(2.0, 0.5, 0.5);
    const shape2 = new RAPIER.Cuboid(0.5, 1.5, 0.5);

    const shapes = [shape1, shape2];
    const positions = [
        new RAPIER.Vector3(0.0, 1.0, 0.0),
        new RAPIER.Vector3(0.0, -1.0, 0.0),
    ];
    const rotations = [
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
    ];

    return RAPIER.ColliderDesc.compound(shapes, positions, rotations);
}

function createPlusShape(RAPIER: RAPIER_API): ColliderDesc {
    const shape1 = new RAPIER.Cuboid(2.0, 0.5, 0.5);
    const shape2 = new RAPIER.Cuboid(0.5, 2.0, 0.5);

    const shapes = [shape1, shape2];
    const positions = [
        new RAPIER.Vector3(0.0, 0.0, 0.0),
        new RAPIER.Vector3(0.0, 0.0, 0.0),
    ];
    const rotations = [
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
    ];

    return RAPIER.ColliderDesc.compound(shapes, positions, rotations);
}

function createStairsShape(RAPIER: RAPIER_API): ColliderDesc {
    const step = new RAPIER.Cuboid(0.8, 0.4, 0.5);

    const shapes = [step, step, step];
    // Bottom, middle, top steps respectively.
    const positions = [
        new RAPIER.Vector3(-1.2, -0.8, 0.0),
        new RAPIER.Vector3(0.0, 0.0, 0.0),
        new RAPIER.Vector3(1.2, 0.8, 0.0),
    ];
    const rotations = [
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
    ];

    return RAPIER.ColliderDesc.compound(shapes, positions, rotations);
}

function createDumbbellShape(RAPIER: RAPIER_API): ColliderDesc {
    const shape1 = new RAPIER.Ball(0.8); // left weight
    const shape2 = new RAPIER.Cuboid(1.5, 0.2, 0.2); // bar
    const shape3 = new RAPIER.Ball(0.8); // right weight

    const shapes = [shape1, shape2, shape3];
    const positions = [
        new RAPIER.Vector3(-2.0, 0.0, 0.0),
        new RAPIER.Vector3(0.0, 0.0, 0.0),
        new RAPIER.Vector3(2.0, 0.0, 0.0),
    ];
    const rotations = [
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
        new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
    ];

    return RAPIER.ColliderDesc.compound(shapes, positions, rotations);
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    testbed.parameters.debugRender = true;

    // Create Ground - a large platform
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let groundBody = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(15.0, 0.5, 15.0);
    world.createCollider(colliderDesc, groundBody);

    for (let i = 0; i < 3; i++) {
        const x = (i - 1) * 8;
        const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
            x,
            1.0,
            0.0,
        );
        const body = world.createRigidBody(bodyDesc);
        const colliderDesc = RAPIER.ColliderDesc.cylinder(2.0, 0.5);
        world.createCollider(colliderDesc, body);
    }

    const rng = seedrandom("compoundShapes");

    const shapeCreators = [
        createLShape,
        createTShape,
        createPlusShape,
        createStairsShape,
        createDumbbellShape,
    ];

    const gridSize = 4;
    const spacing = 6.0;
    const startHeight = 15.0;

    for (let i = 0; i < gridSize; i++) {
        for (let j = 0; j < gridSize; j++) {
            const x = (i - gridSize / 2) * spacing;
            const z = (j - gridSize / 2) * spacing;
            const y = startHeight + i * 3.0;

            const shapeCreator =
                shapeCreators[Math.floor(rng() * shapeCreators.length)];

            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                x,
                y,
                z,
            );
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = shapeCreator(RAPIER);
            world.createCollider(colliderDesc, body);
            body.setAngvel(
                new RAPIER.Vector3(
                    (rng() - 0.5) * 3,
                    (rng() - 0.5) * 3,
                    (rng() - 0.5) * 3,
                ),
                true,
            );
            body.setLinvel(
                new RAPIER.Vector3((rng() - 0.5) * 2, -1.0, (rng() - 0.5) * 2),
                true,
            );
        }
    }

    for (let i = 0; i < 3; i++) {
        const side = i % 2 === 0 ? -12 : 12;
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            side,
            20.0 + i * 4,
            0.0,
        );
        const body = world.createRigidBody(bodyDesc);
        const colliderDesc = createDumbbellShape(RAPIER);
        world.createCollider(colliderDesc, body);
        body.setLinvel(new RAPIER.Vector3(side * -1.5, 0.0, 0.0), true);
        body.setAngvel(new RAPIER.Vector3(0.0, (rng() - 0.5) * 4, 0.0), true);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 30.0, y: 25.0, z: 30.0},
        target: {x: 0.0, y: 10.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
