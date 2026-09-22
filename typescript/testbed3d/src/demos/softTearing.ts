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
    world.createCollider(RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0), ground);

    // Stiff cloth springs (100 Hz) so only impacts pass the tear strain.
    let material = RAPIER.SoftBodyMaterial.uniform(100.0, 1.0);
    material.bendSoftness = {naturalFrequency: 3.0, dampingRatio: 1.0};
    material.tearStrain = 0.4;

    // A sheet pinned along its border: the heavy ball dropped on it rips through.
    let n = 40;
    let border = [];
    for (let k = 0; k < n * n; ++k) {
        let i = Math.floor(k / n);
        let j = k % n;
        if (i == 0 || j == 0 || i == n - 1 || j == n - 1) {
            border.push(k);
        }
    }
    let sheet = RAPIER.SoftBodyDesc.cloth(
        {x: -6.0, y: 3.0, z: -2.0},
        {x: 0.1, y: 0.0, z: 0.0},
        {x: 0.0, y: 0.0, z: 0.1},
        n,
        n,
    )
        .setPinnedParticles(border)
        .setMaterial(material)
        .setParticleMass(0.02)
        .setParticleRadius(0.05)
        .setSurfaceCollider(RAPIER.ColliderDesc.ball(0.05).setFriction(0.8));
    world.createSoftBody(sheet);
    let ballDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        -4.0,
        6.0,
        0.0,
    );
    let ball = world.createRigidBody(ballDesc);
    world.createCollider(RAPIER.ColliderDesc.ball(0.6).setDensity(30.0), ball);

    // A curtain pinned along its top edge, with a heavy box shot through it.
    let curtainMaterial = RAPIER.SoftBodyMaterial.uniform(100.0, 1.0);
    curtainMaterial.bendSoftness = {naturalFrequency: 3.0, dampingRatio: 1.0};
    curtainMaterial.tearStrain = 0.2;
    let top = [];
    for (let i = 0; i < 50; ++i) {
        top.push(i * 40);
    }
    let curtain = RAPIER.SoftBodyDesc.cloth(
        {x: 0.0, y: 4.5, z: 4.0},
        {x: 0.1, y: 0.0, z: 0.0},
        {x: 0.0, y: -0.1, z: 0.0},
        50,
        40,
    )
        .setPinnedParticles(top)
        .setMaterial(curtainMaterial)
        .setParticleMass(0.02);
    world.createSoftBody(curtain);
    let bulletDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(2.5, 2.5, 12.0)
        .setLinvel(0.0, 0.0, -25.0);
    let bullet = world.createRigidBody(bulletDesc);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.3, 0.3, 0.3)
            .setRotation({w: 0.8, x: 0.35, y: 0.35, z: 0.35})
            .setDensity(20.0),
        bullet,
    );

    // A jelly bar pinned at both ends, torn by pulling its right end away.
    let barMaterial = new RAPIER.SoftBodyMaterial();
    barMaterial.youngModulus = 5.0e4;
    barMaterial.poissonRatio = 0.3;
    barMaterial.elasticDampingRatio = 1.0;
    barMaterial.tearStrain = 0.4;
    let barDesc = RAPIER.SoftBodyDesc.cuboid(
        {x: 3.0, y: 1.0, z: -4.0},
        {x: 2.0, y: 0.4, z: 0.4},
        21,
        5,
        5,
    )
        .setCellModel(RAPIER.SoftBodyCellModel.Corotational)
        .setMaterial(barMaterial)
        .setParticleMass(0.05)
        .setParticleRadius(0.1)
        .setSurfaceCollider(RAPIER.ColliderDesc.ball(0.1).setFriction(0.8));
    let bar = world.createSoftBody(barDesc);
    type Vector = {x: number; y: number; z: number};
    let ends: Array<{
        body: number;
        particle: number;
        rest: Vector;
        right: boolean;
    }> = [];
    for (let i = 0; i < bar.numParticles(); ++i) {
        let p = bar.particlePosition(i);
        if (p.x < 1.01) {
            ends.push({
                body: bar.handle,
                particle: i,
                rest: p as Vector,
                right: false,
            });
        } else if (p.x > 4.99) {
            ends.push({
                body: bar.handle,
                particle: i,
                rest: p as Vector,
                right: true,
            });
        }
    }
    for (let end of ends) {
        bar.setParticlePinned(end.particle, true);
    }

    // The right end starts moving after a second, at half a meter per second, and stops
    // once the bar has doubled its length. Tears move the driven particles to new bodies
    // and indices, so the events are followed.
    let t = 0.0;
    testbed.setpreTimestepAction(() => {
        t += world.timestep;
        let shift = Math.min(Math.max(t - 1.0, 0.0) * 0.5, 4.0);
        for (let end of ends) {
            if (end.right) {
                let body = world.getSoftBody(end.body);
                if (!!body) {
                    body.setParticleKinematicTarget(end.particle, {
                        x: end.rest.x + shift,
                        y: end.rest.y,
                        z: end.rest.z,
                    });
                }
            }
        }
        testbed.events.drainSoftBodyTearEvents((event) => {
            for (let end of ends) {
                if (event.softBody() == end.body) {
                    let destination = event.particleDestination(end.particle);
                    if (!!destination) {
                        end.body = destination.softBody;
                        end.particle = destination.particle;
                    }
                }
            }
        });
    });

    testbed.setWorld(world);
    testbed.lookAt({
        eye: {x: 9.0, y: 8.0, z: 16.0},
        target: {x: 0.0, y: 1.5, z: 1.0},
    });
}
