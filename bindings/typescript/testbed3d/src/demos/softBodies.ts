import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(
        0.0,
        -0.1,
        0.0,
    );
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(12.0, 0.1, 12.0), ground);

    // A cloth pinned by its four corners, with a box dropped on it.
    let n = 24;
    let cloth = RAPIER.SoftBodyDesc.cloth(
        {x: -3.5, y: 2.5, z: -1.2},
        {x: 0.1, y: 0.0, z: 0.0},
        {x: 0.0, y: 0.0, z: 0.1},
        n,
        n,
    )
        .setPinnedParticles([0, n - 1, n * (n - 1), n * n - 1])
        .setSoftness(30.0, 1.0)
        .setParticleMass(0.05);
    world.createSoftBody(cloth);
    let boxDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        -2.35,
        4.0,
        0.0,
    );
    let box = world.createRigidBody(boxDesc);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.3, 0.3, 0.3).setDensity(0.5),
        box,
    );

    // A balloon: a hollow sphere inflated by volume preservation.
    let balloon = RAPIER.SoftBodyDesc.sphere({x: 0.5, y: 3.0, z: 0.0}, 0.8, 2)
        .setSoftness(15.0, 1.0)
        .setVolumeFactor(1.2)
        .setParticleMass(0.05);
    world.createSoftBody(balloon);

    // Jelly cubes: corotational, Neo-Hookean, and per-cell volume constraints.
    let jellyMaterial = new RAPIER.SoftBodyMaterial();
    jellyMaterial.youngModulus = 2.0e3;
    jellyMaterial.poissonRatio = 0.35;
    jellyMaterial.elasticDampingRatio = 0.5;
    let models = [
        {z: 1.5, model: RAPIER.SoftBodyCellModel.Corotational},
        {z: 4.5, model: RAPIER.SoftBodyCellModel.NeoHookean},
    ];
    for (let {z, model} of models) {
        let jelly = RAPIER.SoftBodyDesc.cuboid(
            {x: 3.0, y: 1.0, z: z},
            {x: 0.6, y: 0.6, z: 0.6},
            5,
            5,
            5,
        )
            .setCellModel(model)
            .setMaterial(jellyMaterial)
            .setParticleMass(0.2);
        world.createSoftBody(jelly);
    }
    let volumeJelly = RAPIER.SoftBodyDesc.cuboid(
        {x: 3.0, y: 1.0, z: -1.5},
        {x: 0.6, y: 0.6, z: 0.6},
        5,
        5,
        5,
    )
        .setCellModel(RAPIER.SoftBodyCellModel.Volume)
        .setSoftness(20.0, 1.0)
        .setParticleMass(0.2);
    world.createSoftBody(volumeJelly);

    // A rope hanging from a fixed anchor, holding a rigid ball attached by its last
    // particle.
    let ropeDesc = RAPIER.SoftBodyDesc.rope(
        {x: -0.5, y: 5.0, z: 3.0},
        {x: 2.5, y: 5.0, z: 3.0},
        30,
    )
        .setPinnedParticles([0])
        .setSoftness(40.0, 1.0)
        .setParticleMass(0.05);
    let rope = world.createSoftBody(ropeDesc);
    let last = rope.particlePosition(29);
    let weightDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        last.x,
        last.y - 0.3,
        last.z,
    );
    let weight = world.createRigidBody(weightDesc);
    world.createCollider(
        RAPIER.ColliderDesc.ball(0.25).setDensity(2.0),
        weight,
    );
    rope.attachParticle(29, weight);

    testbed.setWorld(world);
    testbed.lookAt({
        eye: {x: 9.0, y: 6.0, z: 12.0},
        target: {x: 0.0, y: 1.5, z: 0.0},
    });
}
