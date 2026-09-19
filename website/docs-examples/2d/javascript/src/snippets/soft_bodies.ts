import RAPIER from '@dimforge/rapier2d';

{
    // DOCUSAURUS: Creation start
    // The world that will contain our soft bodies.
    let world = new RAPIER.World({ x: 0.0, y: -9.81 });
    world.createCollider(RAPIER.ColliderDesc.cuboid(10.0, 0.1));

    // Description of a rope of 20 particles between two points.
    let example1 = RAPIER.SoftBodyDesc.rope({ x: 0.0, y: 3.0 }, { x: 2.0, y: 3.0 }, 20);
    // Description of a grid of `nx` by `ny` particles filled with triangle cells.
    let example2 = RAPIER.SoftBodyDesc.grid({ x: 3.0, y: 1.0 }, { x: 1.0, y: 1.0 }, 6, 6);
    // Description of a disk: a ring of particles holding its area (a pressurized blob).
    let example3 = RAPIER.SoftBodyDesc.disk({ x: 0.0, y: 3.0 }, 0.8, 24);
    // Description of a closed polygon of particles holding its area.
    let example4 = RAPIER.SoftBodyDesc.polygon([5.0, 4.0, 7.0, 4.0, 7.0, 6.0, 5.0, 6.0]);
    // Description over raw particle positions; the elements are added by the setters.
    let example5 = new RAPIER.SoftBodyDesc([0.0, 1.0, 1.0, 1.0]).setEdges([0, 1]);

    let n = 20;
    let sheetDesc = RAPIER.SoftBodyDesc.grid({ x: -3.0, y: 3.0 }, { x: 1.0, y: 1.0 }, n, n)
        // Particles held in place.
        .setPinnedParticles([0, n - 1])
        // A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
        .setSoftness(30.0, 1.0)
        // The mass of each particle.
        // Default: 1.0
        .setParticleMass(0.05)
        // The thickness of the particles, for collisions.
        // Default: 0.01
        .setParticleRadius(0.05)
        // The template of the body's colliders: its shape is replaced by the deformable surface.
        .setSurfaceCollider(RAPIER.ColliderDesc.ball(0.05).setFriction(0.8))
        // Whether the body may fall asleep.
        // Default: true
        .setCanSleep(true);
    // Create the soft body: this creates its hidden root rigid body and its colliders.
    let sheet = world.createSoftBody(sheetDesc);
    // The integer handle of the soft body can be read from the `handle` field.
    let sheetHandle = sheet.handle;
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly square with corotational linear elasticity.
    let material = new RAPIER.SoftBodyMaterial();
    // Stiffness of the elastic cells.
    material.youngModulus = 3.0e3;
    material.poissonRatio = 0.35;
    material.elasticDampingRatio = 0.5;
    // Plasticity: the rest shape flows past 5% strain, at 20 per second.
    material.plasticYield = 0.05;
    material.plasticCreep = 20.0;
    // Tearing: an element past 40% strain tears.
    material.tearStrain = 0.4;
    let jellyDesc = RAPIER.SoftBodyDesc.grid({ x: 3.0, y: 1.2 }, { x: 1.0, y: 1.0 }, 6, 6)
        // The constitutive model of the cells: `Volume` (per-cell area constraints,
        // the shape is held by the edges), `Corotational` or `NeoHookean`.
        .setCellModel(RAPIER.SoftBodyCellModel.Corotational)
        .setMaterial(material)
        .setParticleMass(0.2);
    let jelly = world.createSoftBody(jellyDesc);

    // A pressurized blob: a ring of particles inflated by area preservation.
    let blobDesc = RAPIER.SoftBodyDesc.disk({ x: 0.0, y: 3.0 }, 0.8, 24)
        .setSoftness(20.0, 1.0)
        // Target area multiplier (`> 1` inflates the body); enables area preservation.
        .setVolumeFactor(1.1)
        .setSelfContacts(true);
    let blob = world.createSoftBody(blobDesc);
    // DOCUSAURUS: Material stop

    // DOCUSAURUS: Particles start
    // Read the particles.
    let position = sheet.particlePosition(0);
    let velocity = sheet.particleVelocity(0);
    let positions: Float32Array = sheet.particlePositions(); // Two floats per particle.
    console.log("The sheet has", sheet.numParticles(), "particles;", positions.length / 2);
    // Move a particle.
    sheet.setParticlePosition(1, { x: position.x, y: position.y + 0.1 });
    sheet.setParticleVelocity(1, velocity);
    // Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
    sheet.setParticlePinned(2, true);
    sheet.setParticleKinematicTarget(2, { x: -3.5, y: 3.5 });
    // The elements: edges (two indices each), cells (three) and the boundary segments (two).
    let edges: Uint32Array = sheet.edges();
    let cells: Uint32Array = sheet.cells();
    let boundary: Uint32Array = sheet.boundary();
    console.log(edges.length / 2, "edges,", cells.length / 3, "cells,", boundary.length / 2, "segments");
    // DOCUSAURUS: Particles stop

    // DOCUSAURUS: Forces start
    // The `true` argument makes sure the soft body is awake.
    sheet.resetForces(true); // Reset the forces to zero.
    sheet.addForce({ x: 0.0, y: 1.0 }, true); // Spread over the particles by mass.
    sheet.addParticleForce(3, { x: 0.0, y: 1.0 }, true);
    sheet.applyImpulse({ x: 0.0, y: 0.1 }, true);
    sheet.applyParticleImpulse(3, { x: 0.0, y: 0.1 }, true);
    // An impulse on the particles within 0.5 of a point, scaled down with the distance.
    sheet.applyImpulseAtPoint({ x: 0.0, y: 0.1 }, { x: -3.0, y: 3.0 }, 0.5, true);
    // A blast pushing the particles away from a center.
    sheet.applyRadialImpulse({ x: -3.0, y: 3.0 }, 0.1, 1.0, true);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid box, at the particle's position.
    let ropeDesc = RAPIER.SoftBodyDesc.rope({ x: 8.0, y: 9.0 }, { x: 12.0, y: 9.0 }, 25)
        .setPinnedParticles([0])
        .setSoftness(40.0, 1.0);
    let rope = world.createSoftBody(ropeDesc);
    let last = rope.particlePosition(24);
    let weight = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(last.x, last.y - 0.4));
    world.createCollider(RAPIER.ColliderDesc.cuboid(0.3, 0.3).setDensity(2.0), weight);
    rope.attachParticle(24, weight);
    // The hidden rigid body standing for the whole soft body in joints and islands.
    let rootBody = rope.rootBody();
    console.log("root body is a soft frame:", rootBody.isSoftFrame());
    // DOCUSAURUS: Attachments stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly: a rigid proxy that joints and
    // colliders can attach to.
    let top = [];
    for (let i = 0; i < jelly.numParticles(); ++i) {
        if (jelly.particlePosition(i).y > 2.0) {
            top.push(i);
        }
    }
    let cluster = world.addSoftBodyCluster(jelly, top);
    let proxy = jelly.clusterProxy(cluster);
    // A rigid plate welded onto the cluster.
    let plate = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(3.0, 2.4));
    world.createCollider(RAPIER.ColliderDesc.cuboid(1.2, 0.05).setDensity(0.4), plate);
    let weld = RAPIER.JointData.fixed({ x: 0.0, y: -0.1 }, 0.0, { x: 0.0, y: 0.0 }, 0.0);
    world.createImpulseJoint(weld, plate, proxy, true);
    // A cluster can be pinned, driven or tuned as a whole.
    jelly.setClusterStiffnessScale(cluster, 2.0);
    jelly.enableClusterShapeMatching(cluster, true);
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: DeformableColliders start
    // A deformable polyline bound to the blob: each vertex follows one particle (`direct`),
    // or is embedded in the cell holding it (`skinned`). The polyline is given in the frame
    // of the proxy it is attached to.
    let root = blob.rootBody();
    let origin = root.translation();
    let num = blob.numParticles();
    let vertices = blob.particlePositions();
    for (let i = 0; i < num; ++i) {
        vertices[i * 2] -= origin.x;
        vertices[i * 2 + 1] -= origin.y;
    }
    let indices = new Uint32Array(num * 2);
    let particles = [];
    for (let i = 0; i < num; ++i) {
        indices[i * 2] = i;
        indices[i * 2 + 1] = (i + 1) % num;
        particles.push(i);
    }
    let outlineDesc = RAPIER.ColliderDesc.polyline(vertices, indices, RAPIER.PolylineFlags.DEFORMABLE).setSensor(true);
    let outline = world.createDeformableCollider(outlineDesc, RAPIER.SoftMeshBinding.direct(particles), root);
    // The polyline follows the particles: read its current vertices back.
    let meshIndex = blob.meshOfCollider(outline);
    let outlineVertices: Float32Array = blob.meshVertices(meshIndex);
    console.log("The outline has", outlineVertices.length / 2, "vertices");
    // DOCUSAURUS: DeformableColliders stop

    // DOCUSAURUS: Tearing start
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    sheet.tearEdge(10); // Applied at the end of the next step.
    // Tear at once along edges and through cells; pieces the tear disconnects become soft
    // bodies of their own.
    let tear = world.tearSoftBody(sheet, [11, 12], []);
    if (tear) {
        console.log(tear.tornEdges().length / 2, "edges torn");
        tear.free();
    }
    // Cut along a blade (a segment in 2D), without removing material.
    let cut = world.cutSoftBody(sheet, [{ x: -3.0, y: -10.0 }, { x: -3.0, y: 10.0 }]);
    if (cut) {
        for (let i = 0; i < cut.numPieces(); ++i) {
            let piece = world.getSoftBody(cut.pieceSoftBody(i));
            console.log("piece", i, "has", piece.numParticles(), "particles");
        }
        // Where a particle of the torn body went.
        let destination = cut.particleDestination(n * n - 1);
        if (destination) {
            console.log("particle", n * n - 1, "is now particle", destination.particle, "of", destination.softBody);
        }
        cut.free();
    }
    // DOCUSAURUS: Tearing stop

    // DOCUSAURUS: Events start
    // Tears applied during a step are reported through the event queue.
    let eventQueue = new RAPIER.EventQueue(true);
    world.step(eventQueue);
    eventQueue.drainSoftBodyTearEvents((event) => {
        console.log("Soft body", event.softBody(), "tore:", event.numPieces(), "pieces split off");
    });
    // DOCUSAURUS: Events stop

    // DOCUSAURUS: Settings start
    // Settings shared by every soft body of the world.
    // Strain beyond which a constraint is re-solved after the contacts of every substep.
    // Default: 0.75
    world.integrationParameters.softBodiesResweepStrain = 0.75;
    // Extra substeps a soft body requests while it is hit fast; 0 disables them.
    // Default: 4
    world.integrationParameters.softBodiesMaxExtraSubsteps = 4;
    // Stiffening of the soft-body contacts relative to the rigid ones.
    // Default: 4.0
    world.integrationParameters.softBodiesContactStiffening = 4.0;
    // DOCUSAURUS: Settings stop

    // DOCUSAURUS: Removal start
    // Removing a soft body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    world.removeSoftBody(rope);
    // A cluster can be removed on its own.
    world.removeSoftBodyCluster(jelly, cluster);
    // DOCUSAURUS: Removal stop
}
