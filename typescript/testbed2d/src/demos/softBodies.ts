import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier2d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector2(0.0, -9.81);
    let world = new RAPIER.World(gravity);

    // Ground and walls.
    let walls = [
        {x: 0.0, y: -0.5, hx: 15.0, hy: 0.5},
        {x: -15.0, y: 5.0, hx: 0.5, hy: 5.0},
        {x: 15.0, y: 5.0, hx: 0.5, hy: 5.0},
    ];
    for (let wall of walls) {
        let bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
            wall.x,
            wall.y,
        );
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(
            RAPIER.ColliderDesc.cuboid(wall.hx, wall.hy),
            body,
        );
    }

    // Pressurized blobs of various sizes.
    for (let i = 0; i < 5; ++i) {
        let radius = 0.6 + 0.15 * i;
        let blob = RAPIER.SoftBodyDesc.disk(
            {x: -10.0 + i * 2.5, y: 2.0 + i},
            radius,
            24,
        )
            .setSoftness(20.0, 1.0)
            .setVolumeFactor(1.1)
            .setSelfContacts(true)
            .setParticleMass(0.05);
        world.createSoftBody(blob);
    }

    // Jelly squares: corotational, Neo-Hookean, and per-cell area constraints.
    let jellyMaterial = new RAPIER.SoftBodyMaterial();
    jellyMaterial.youngModulus = 3.0e3;
    jellyMaterial.poissonRatio = 0.35;
    jellyMaterial.elasticDampingRatio = 0.5;
    let models = [
        {x: 2.0, model: RAPIER.SoftBodyCellModel.Corotational},
        {x: 8.0, model: RAPIER.SoftBodyCellModel.NeoHookean},
    ];
    for (let {x, model} of models) {
        let jelly = RAPIER.SoftBodyDesc.grid(
            {x: x, y: 1.2},
            {x: 1.0, y: 1.0},
            6,
            6,
        )
            .setCellModel(model)
            .setMaterial(jellyMaterial)
            .setParticleMass(0.2);
        world.createSoftBody(jelly);
    }
    let volumeJelly = RAPIER.SoftBodyDesc.grid(
        {x: 5.0, y: 1.2},
        {x: 1.0, y: 1.0},
        6,
        6,
    )
        .setCellModel(RAPIER.SoftBodyCellModel.Volume)
        .setSoftness(20.0, 1.0)
        .setParticleMass(0.2);
    world.createSoftBody(volumeJelly);

    // A rope hanging from a fixed anchor, holding a rigid box.
    let ropeDesc = RAPIER.SoftBodyDesc.rope(
        {x: 8.0, y: 9.0},
        {x: 12.0, y: 9.0},
        25,
    )
        .setPinnedParticles([0])
        .setSoftness(40.0, 1.0)
        .setParticleMass(0.05);
    let rope = world.createSoftBody(ropeDesc);
    let last = rope.particlePosition(24);
    let weightDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        last.x,
        last.y - 0.4,
    );
    let weight = world.createRigidBody(weightDesc);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.3, 0.3).setDensity(2.0),
        weight,
    );
    rope.attachParticle(24, weight);

    // A stack of rigid boxes for the blobs to knock down.
    for (let i = 0; i < 6; ++i) {
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            -4.0,
            0.3 + 0.6 * i,
        );
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.3, 0.3), body);
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 0.0, y: -4.0},
        zoom: 25.0,
    });
}
