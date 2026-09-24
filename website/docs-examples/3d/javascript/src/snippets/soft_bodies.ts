import RAPIER from '@dimforge/rapier3d';

{
    // DOCUSAURUS: Creation start
    // The world that will contain our soft bodies.
    let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
    world.createCollider(RAPIER.ColliderDesc.cuboid(10.0, 0.1, 10.0));

    // Description of a rope of 20 particles between two points.
    let example1 = RAPIER.SoftBodyDesc.rope({ x: 0.0, y: 3.0, z: 0.0 }, { x: 2.0, y: 3.0, z: 0.0 }, 20);
    // Description of a cloth: `nu` by `nv` particles, particle `(i, j)` at `origin + i * du + j * dv`.
    let example2 = RAPIER.SoftBodyDesc.cloth(
        { x: -1.0, y: 2.0, z: -1.0 }, { x: 0.1, y: 0.0, z: 0.0 }, { x: 0.0, y: 0.0, z: 0.1 }, 20, 20,
    );
    // Description of a box of `nx * ny * nz` particles filled with tetrahedral cells.
    let example3 = RAPIER.SoftBodyDesc.cuboid({ x: 3.0, y: 1.0, z: 0.0 }, { x: 0.5, y: 0.5, z: 0.5 }, 4, 4, 4);
    // Description of a hollow sphere holding its volume (a balloon).
    let example4 = RAPIER.SoftBodyDesc.sphere({ x: 0.0, y: 3.0, z: 3.0 }, 0.8, 2);
    // Description over raw particle positions; the elements are added by the setters.
    let example5 = new RAPIER.SoftBodyDesc([0.0, 1.0, 0.0, 1.0, 1.0, 0.0]).setEdges([0, 1]);

    let n = 20;
    let clothDesc = RAPIER.SoftBodyDesc.cloth(
        { x: -1.0, y: 2.0, z: -1.0 }, { x: 0.1, y: 0.0, z: 0.0 }, { x: 0.0, y: 0.0, z: 0.1 }, n, n,
    )
        // Particles held in place.
        .setPinnedParticles([0, n - 1, n * (n - 1), n * n - 1])
        // A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
        .setSoftness(30.0, 1.0)
        // The mass of each particle.
        // Default: 1.0
        .setParticleMass(0.05)
        // The thickness of the particles, for collisions.
        // Default: 0.01
        .setParticleRadius(0.02)
        // The template of the body's colliders: its shape is replaced by the deformable surface.
        .setSurfaceCollider(RAPIER.ColliderDesc.ball(0.05).setFriction(0.8))
        // Whether the surface collides with itself.
        // Default: false
        .setSelfContacts(true)
        // Whether the body may fall asleep.
        // Default: true
        .setCanSleep(true);
    // Create the soft body: this creates its hidden root rigid body and its colliders.
    let cloth = world.createSoftBody(clothDesc);
    // The integer handle of the soft body can be read from the `handle` field.
    let clothHandle = cloth.handle;
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: Volumetric start
    // Fill a closed, outward-oriented triangle mesh (here a box) with tetrahedral cells of
    // about 0.2 in size.
    let boxVertices = new Float32Array([
        -0.5, -0.25, -0.25, 0.5, -0.25, -0.25, 0.5, 0.25, -0.25, -0.5, 0.25, -0.25,
        -0.5, -0.25, 0.25, 0.5, -0.25, 0.25, 0.5, 0.25, 0.25, -0.5, 0.25, 0.25,
    ]);
    let boxIndices = new Uint32Array([
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
    ]);
    let blockDesc = RAPIER.SoftBodyDesc.volumetric(boxVertices, boxIndices, 0.2);
    let block = world.createSoftBody(blockDesc.setTranslation({ x: -3.0, y: 1.0, z: 0.0 }));
    // DOCUSAURUS: Volumetric stop

    // DOCUSAURUS: Fem start
    // A stiff beam simulated by the FEM solver: its stiffness doesn't depend on the number of
    // solver iterations.
    let beamMaterial = new RAPIER.SoftBodyMaterial();
    beamMaterial.youngModulus = 1.0e5;
    beamMaterial.poissonRatio = 0.3;
    let beamDesc = RAPIER.SoftBodyDesc.cuboid(
        { x: 0.0, y: 2.0, z: -3.0 }, { x: 1.0, y: 0.1, z: 0.1 }, 11, 3, 3,
    )
        .setSolver(RAPIER.SoftBodySolver.Fem)
        .setCellModel(RAPIER.SoftBodyCellModel.NeoHookean)
        .setMaterial(beamMaterial)
        // The particles of the face at `x = -1` are the first 3 × 3 ones.
        .setPinnedParticles(new Uint32Array([0, 1, 2, 3, 4, 5, 6, 7, 8]));
    let beam = world.createSoftBody(beamDesc);

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    world.integrationParameters.softBodiesFemLinearTolerance = 1.0e-5;
    world.integrationParameters.softBodiesFemMaxLinearIterations = 20;
    // DOCUSAURUS: Fem stop

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    let points = [];
    for (let i = 0; i < 27; ++i) {
        points.push((i % 3) * 0.3, (Math.floor(i / 3) % 3) * 0.3 + 4.0, Math.floor(i / 9) * 0.3);
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
    let bowlDesc = RAPIER.SoftBodyDesc.sphere({ x: -3.0, y: 2.0, z: 0.0 }, 0.8, 2)
        .setOriented(false)
        .setSoftness(60.0, 1.0);
    let bowl = world.createSoftBody(bowlDesc);
    // DOCUSAURUS: Oriented stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly cube with corotational linear elasticity.
    let material = new RAPIER.SoftBodyMaterial();
    // Stiffness of the elastic cells.
    material.youngModulus = 2.0e3;
    material.poissonRatio = 0.35;
    material.elasticDampingRatio = 0.5;
    // Plasticity: the rest shape flows past 5% strain, at 20 per second.
    material.plasticYield = 0.05;
    material.plasticCreep = 20.0;
    // Tearing: an element past 40% strain tears.
    material.tearStrain = 0.4;
    let jellyDesc = RAPIER.SoftBodyDesc.cuboid({ x: 3.0, y: 1.0, z: 0.0 }, { x: 0.5, y: 0.5, z: 0.5 }, 4, 4, 4)
        // The constitutive model of the cells: `Volume` (per-cell volume constraints,
        // the shape is held by the edges), `Corotational` or `NeoHookean`.
        .setCellModel(RAPIER.SoftBodyCellModel.Corotational)
        .setMaterial(material)
        .setParticleMass(0.2);
    let jelly = world.createSoftBody(jellyDesc);

    // A material shared by the edges, bending constraints and volume constraints.
    let clothMaterial = RAPIER.SoftBodyMaterial.uniform(30.0, 1.0);
    // Softness of the bending constraints, on top of the uniform 30 Hz softness.
    clothMaterial.bendSoftness = { naturalFrequency: 3.0, dampingRatio: 1.0 };
    cloth.setMaterial(clothMaterial);
    // DOCUSAURUS: Material stop

    // DOCUSAURUS: Particles start
    // Read the particles.
    let position = cloth.particlePosition(0);
    let velocity = cloth.particleVelocity(0);
    let positions: Float32Array = cloth.particlePositions(); // Three floats per particle.
    console.log("The cloth has", cloth.numParticles(), "particles;", positions.length / 3);
    // Move a particle.
    cloth.setParticlePosition(1, { x: position.x, y: position.y + 0.1, z: position.z });
    cloth.setParticleVelocity(1, velocity);
    // Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
    cloth.setParticlePinned(2, true);
    cloth.setParticleKinematicTarget(2, { x: -1.0, y: 2.5, z: -0.8 });
    // The elements: edges (two indices each), cells (four) and the boundary triangles (three).
    let edges: Uint32Array = cloth.edges();
    let cells: Uint32Array = cloth.cells();
    let boundary: Uint32Array = cloth.boundary();
    console.log(edges.length / 2, "edges,", cells.length / 4, "cells,", boundary.length / 3, "triangles");
    // DOCUSAURUS: Particles stop

    // DOCUSAURUS: Forces start
    // The `true` argument makes sure the soft body is awake.
    cloth.resetForces(true); // Reset the forces to zero.
    cloth.addForce({ x: 0.0, y: 1.0, z: 0.0 }, true); // Spread over the particles by mass.
    cloth.addParticleForce(3, { x: 0.0, y: 1.0, z: 0.0 }, true);
    cloth.applyImpulse({ x: 0.0, y: 0.1, z: 0.0 }, true);
    cloth.applyParticleImpulse(3, { x: 0.0, y: 0.1, z: 0.0 }, true);
    // An impulse on the particles within 0.5 of a point, scaled down with the distance.
    cloth.applyImpulseAtPoint({ x: 0.0, y: 0.1, z: 0.0 }, { x: 0.0, y: 2.0, z: 0.0 }, 0.5, true);
    // A blast pushing the particles away from a center.
    cloth.applyRadialImpulse({ x: 0.0, y: 2.0, z: 0.0 }, 0.1, 1.0, true);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid ball, at the particle's position.
    let ropeDesc = RAPIER.SoftBodyDesc.rope({ x: -0.5, y: 5.0, z: 3.0 }, { x: 2.5, y: 5.0, z: 3.0 }, 30)
        .setPinnedParticles([0])
        .setSoftness(40.0, 1.0);
    let rope = world.createSoftBody(ropeDesc);
    let last = rope.particlePosition(29);
    let ball = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(last.x, last.y - 0.3, last.z));
    world.createCollider(RAPIER.ColliderDesc.ball(0.25).setDensity(2.0), ball);
    rope.attachParticle(29, ball);
    // DOCUSAURUS: Attachments stop

    // DOCUSAURUS: RootBody start
    // The rigid body the engine created for the whole soft body, read back after its insertion.
    let rootBody = jelly.rootBody();
    console.log("root body is a soft frame:", rootBody.isSoftFrame());

    // A rigid collider attached to it follows the frame of the whole body: here a sensor
    // detecting what comes close to the jelly.
    world.createCollider(RAPIER.ColliderDesc.ball(1.0).setSensor(true), rootBody);

    // A joint attached to it acts on the soft body as a whole: this one hangs the jelly under a
    // fixed anchor by a spring.
    let jellyAnchor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(3.0, 4.0, 0.0));
    let jellySpring = RAPIER.JointData.spring(
        2.5, 60.0, 2.0,
        { x: 0.0, y: 0.0, z: 0.0 }, { x: 0.0, y: 0.0, z: 0.0 },
    );
    world.createImpulseJoint(jellySpring, jellyAnchor, rootBody, true);
    // DOCUSAURUS: RootBody stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly: a rigid proxy that joints and
    // colliders can attach to.
    let top = [];
    for (let i = 0; i < jelly.numParticles(); ++i) {
        if (jelly.particlePosition(i).y > 1.3) {
            top.push(i);
        }
    }
    let cluster = world.addSoftBodyCluster(jelly, top);
    let proxy = jelly.clusterProxy(cluster);
    // A rigid plate welded onto the cluster.
    let plate = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(3.0, 1.9, 0.0));
    world.createCollider(RAPIER.ColliderDesc.cuboid(0.7, 0.05, 0.7).setDensity(0.4), plate);
    let weld = RAPIER.JointData.fixed(
        { x: 0.0, y: -0.1, z: 0.0 }, { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
        { x: 0.0, y: 0.0, z: 0.0 }, { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
    );
    world.createImpulseJoint(weld, plate, proxy, true);
    // A cluster can be pinned, driven or tuned as a whole.
    jelly.setClusterStiffnessScale(cluster, 2.0);
    jelly.enableClusterShapeMatching(cluster, true);
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: ClusterControl start
    // Pin every particle of the cluster, then move it along a path: the cluster behaves like a
    // kinematic rigid part dragging the rest of the body.
    jelly.setClusterPinned(cluster, true);
    jelly.setClusterKinematicTarget(cluster, { x: 3.0, y: 2.0, z: 0.0 }, { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    // Release it: the cluster is simulated again.
    jelly.setClusterPinned(cluster, false);
    // DOCUSAURUS: ClusterControl stop

    // DOCUSAURUS: DeformableColliders start
    // A deformable triangle mesh bound to the jelly: each vertex is embedded in the cell
    // holding it (`skinned`), or follows one particle (`direct`). The mesh is given in the
    // frame of the proxy it is attached to.
    let root = jelly.rootBody();
    let origin = root.translation();
    let c = jelly.centerOfMass();
    let r = 1.0;
    let vertices = new Float32Array([
        c.x + r, c.y, c.z, c.x - r, c.y, c.z, c.x, c.y + r, c.z,
        c.x, c.y - r, c.z, c.x, c.y, c.z + r, c.x, c.y, c.z - r,
    ]);
    for (let i = 0; i < 6; ++i) {
        vertices[i * 3] -= origin.x;
        vertices[i * 3 + 1] -= origin.y;
        vertices[i * 3 + 2] -= origin.z;
    }
    let indices = new Uint32Array([0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5]);
    let skinDesc = RAPIER.ColliderDesc.trimesh(vertices, indices, RAPIER.TriMeshFlags.DEFORMABLE).setSensor(true);
    let skin = world.createDeformableCollider(skinDesc, RAPIER.SoftMeshBinding.skinned(), root);
    // The mesh follows the particles: read its current vertices back.
    let meshIndex = jelly.meshOfCollider(skin);
    let skinVertices: Float32Array = jelly.meshVertices(meshIndex);
    console.log("The skin has", skinVertices.length / 3, "vertices");
    // DOCUSAURUS: DeformableColliders stop

    // DOCUSAURUS: Skinning start
    // A mesh held by a cage of cells (the last `true` argument): only the cells are simulated,
    // and the mesh (the skin) follows their deformation.
    let skinnedDesc = RAPIER.SoftBodyDesc.volumetric(boxVertices, boxIndices, 0.25, true)
        // Collide through the skin instead of the boundary of the cage.
        .setSkinCollision(true)
        .setTranslation({ x: 0.0, y: 4.0, z: 3.0 });
    let skinned = world.createSoftBody(skinnedDesc);
    // The skin is the body's collision mesh: read its vertices back to render it.
    let skinPositions: Float32Array = skinned.meshVertices(0);
    console.log("The skin has", skinPositions.length / 3, "vertices");
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
    let tearMaterial = cloth.material();
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    tearMaterial.tearStrain = 0.4;
    tearMaterial.tearForce = 50.0;
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    tearMaterial.tearSmoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    tearMaterial.interiorStrength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    tearMaterial.minPiece = 10;
    cloth.setMaterial(tearMaterial);
    // DOCUSAURUS: TearingMaterial stop

    // DOCUSAURUS: Tearing start
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    cloth.tearEdge(10); // Applied at the end of the next step.
    // Tear at once along edges and through cells; pieces the tear disconnects become soft
    // bodies of their own.
    let tear = world.tearSoftBody(cloth, [11, 12], []);
    if (tear) {
        console.log(tear.tornEdges().length / 2, "edges torn");
        tear.free();
    }
    // Cut along a blade (a triangle in 3D), without removing material.
    let cut = world.cutSoftBody(cloth, [
        { x: -0.1, y: -10.0, z: -10.0 }, { x: -0.1, y: 10.0, z: 0.0 }, { x: -0.1, y: -10.0, z: 10.0 },
    ]);
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
