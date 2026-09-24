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

    // DOCUSAURUS: Volumetric start
    // Fill a closed, counter-clockwise polyline (here a rectangle) with triangle cells of about
    // 0.2 in size.
    let boxVertices = new Float32Array([-0.5, -0.25, 0.5, -0.25, 0.5, 0.25, -0.5, 0.25]);
    let boxIndices = new Uint32Array([0, 1, 1, 2, 2, 3, 3, 0]);
    let blockDesc = RAPIER.SoftBodyDesc.volumetric(boxVertices, boxIndices, 0.2);
    let block = world.createSoftBody(blockDesc.setTranslation({ x: -3.0, y: 1.0 }));
    // DOCUSAURUS: Volumetric stop

    // DOCUSAURUS: Fem start
    // A stiff beam simulated by the FEM solver: its stiffness doesn't depend on the number of
    // solver iterations.
    let beamMaterial = new RAPIER.SoftBodyMaterial();
    beamMaterial.youngModulus = 1.0e5;
    beamMaterial.poissonRatio = 0.3;
    let beamDesc = RAPIER.SoftBodyDesc.grid({ x: 0.0, y: 2.0 }, { x: 1.0, y: 0.1 }, 21, 3)
        .setSolver(RAPIER.SoftBodySolver.Fem)
        .setCellModel(RAPIER.SoftBodyCellModel.NeoHookean)
        .setMaterial(beamMaterial)
        // The particles of the side at `x = -1` are the first 3 ones.
        .setPinnedParticles(new Uint32Array([0, 1, 2]));
    let beam = world.createSoftBody(beamDesc);

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    world.integrationParameters.softBodiesFemLinearTolerance = 1.0e-5;
    world.integrationParameters.softBodiesFemMaxLinearIterations = 20;
    // DOCUSAURUS: Fem stop

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    let points = [];
    for (let i = 0; i < 9; ++i) {
        points.push((i % 3) * 0.3, Math.floor(i / 3) * 0.3 + 4.0);
    }
    let shapeMaterial = new RAPIER.SoftBodyMaterial();
    // How fast the particles are pulled back toward their rest shape.
    shapeMaterial.shapeMatchingSoftness = { naturalFrequency: 5.0, dampingRatio: 1.0 };
    let pointCloudDesc = new RAPIER.SoftBodyDesc(points)
        .setShapeMatching(true)
        .setMaterial(shapeMaterial)
        .setParticleRadius(0.1);
    let pointCloud = world.createSoftBody(pointCloudDesc);
    // DOCUSAURUS: ShapeMatching stop

    // DOCUSAURUS: Oriented start
    // A shell: a closed surface that is not oriented, so its inner side holds the bodies put
    // inside it (a bowl, a box, a container). A closed surface is oriented by default.
    let bowlDesc = RAPIER.SoftBodyDesc.disk({ x: -3.0, y: 2.0 }, 0.8, 24)
        .setOriented(false)
        .setSoftness(60.0, 1.0);
    let bowl = world.createSoftBody(bowlDesc);
    // DOCUSAURUS: Oriented stop

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
    // DOCUSAURUS: Attachments stop

    // DOCUSAURUS: RootBody start
    // The rigid body the engine created for the whole soft body, read back after its insertion.
    let rootBody = jelly.rootBody();
    console.log("root body is a soft frame:", rootBody.isSoftFrame());

    // A rigid collider attached to it follows the frame of the whole body: here a sensor
    // detecting what comes close to the jelly.
    world.createCollider(RAPIER.ColliderDesc.ball(1.6).setSensor(true), rootBody);

    // A joint attached to it acts on the soft body as a whole: this one hangs the jelly under a
    // fixed anchor by a spring.
    let jellyAnchor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(3.0, 5.0));
    let jellySpring = RAPIER.JointData.spring(2.0, 60.0, 2.0, { x: 0.0, y: 0.0 }, { x: 0.0, y: 0.0 });
    world.createImpulseJoint(jellySpring, jellyAnchor, rootBody, true);
    // DOCUSAURUS: RootBody stop

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

    // DOCUSAURUS: ClusterControl start
    // Pin every particle of the cluster, then move it along a path: the cluster behaves like a
    // kinematic rigid part dragging the rest of the body.
    jelly.setClusterPinned(cluster, true);
    jelly.setClusterKinematicTarget(cluster, { x: 3.0, y: 2.5 }, 0.0);
    // Release it: the cluster is simulated again.
    jelly.setClusterPinned(cluster, false);
    // DOCUSAURUS: ClusterControl stop

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

    // DOCUSAURUS: Skinning start
    // A detailed outline held by a coarse cage of cells (the last `true` argument): only the
    // cells are simulated, and the outline (the skin) follows their deformation.
    let numSegments = 48;
    let circleVertices = new Float32Array(numSegments * 2);
    let circleIndices = new Uint32Array(numSegments * 2);
    for (let i = 0; i < numSegments; ++i) {
        let angle = (i / numSegments) * 2.0 * Math.PI;
        circleVertices.set([Math.cos(angle) * 0.5, Math.sin(angle) * 0.5], i * 2);
        circleIndices.set([i, (i + 1) % numSegments], i * 2);
    }
    let skinnedDesc = RAPIER.SoftBodyDesc.volumetric(circleVertices, circleIndices, 0.25, true)
        // Collide through the skin instead of the boundary of the cage.
        .setSkinCollision(true)
        .setTranslation({ x: 0.0, y: 4.0 });
    let skinned = world.createSoftBody(skinnedDesc);
    // The skin is the body's collision mesh: read its vertices back to render it.
    let skinPositions: Float32Array = skinned.meshVertices(0);
    console.log("The skin has", skinPositions.length / 2, "vertices");
    // DOCUSAURUS: Skinning stop

    // DOCUSAURUS: Plasticity start
    // The jelly has elastic (corotational) cells: the plasticity of `Volume` cells has no effect.
    let plasticMaterial = jelly.material();
    // Cells: the rest shape flows toward the current one past 5% strain, at a rate of 20 per
    // second, up to a total permanent deformation of 50%.
    plasticMaterial.plasticYield = 0.05;
    plasticMaterial.plasticCreep = 20.0;
    plasticMaterial.plasticMax = 0.5;
    // Edges: the rest length flows past 10% strain, up to half the initial length, but only
    // when squeezed (a dent stays, a stretch springs back).
    plasticMaterial.edgePlasticYield = 0.1;
    plasticMaterial.edgePlasticCreep = 10.0;
    plasticMaterial.edgePlasticMax = 0.5;
    plasticMaterial.edgePlasticFlow = RAPIER.SoftEdgePlasticFlow.Compression;
    jelly.setMaterial(plasticMaterial);
    // Every permanent deformation can be undone at once.
    jelly.resetPlasticity();
    // DOCUSAURUS: Plasticity stop

    // DOCUSAURUS: TearingMaterial start
    let tearMaterial = sheet.material();
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    tearMaterial.tearStrain = 0.4;
    tearMaterial.tearForce = 50.0;
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    tearMaterial.tearSmoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    tearMaterial.interiorStrength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    tearMaterial.minPiece = 10;
    sheet.setMaterial(tearMaterial);
    // DOCUSAURUS: TearingMaterial stop

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
    // The tangle detection and recovery stack can be switched off mechanism by mechanism; the
    // getter gives back a copy, so the settings are assigned back after being changed.
    let recovery = world.integrationParameters.softBodiesRecovery;
    recovery.crossingRepulsion = true;
    world.integrationParameters.softBodiesRecovery = recovery;
    // DOCUSAURUS: Settings stop

    // DOCUSAURUS: Removal start
    // Removing a soft body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    world.removeSoftBody(rope);
    // A cluster can be removed on its own.
    world.removeSoftBodyCluster(jelly, cluster);
    // DOCUSAURUS: Removal stop
}
