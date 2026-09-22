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
    // The hidden rigid body standing for the whole soft body in joints and islands.
    let rootBody = rope.rootBody();
    console.log("root body is a soft frame:", rootBody.isSoftFrame());
    // DOCUSAURUS: Attachments stop

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
    // DOCUSAURUS: Settings stop

    // DOCUSAURUS: Removal start
    // Removing a soft body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    world.removeSoftBody(rope);
    // A cluster can be removed on its own.
    world.removeSoftBodyCluster(jelly, cluster);
    // DOCUSAURUS: Removal stop
}
