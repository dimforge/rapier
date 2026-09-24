#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();

    // A ground.
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(10.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    // A sheet pinned by its two bottom corners.
    uint32_t n = 20;
    R2SoftBodyDesc sheet_desc = r2GridSoftBodyDesc(r2Vector(-3.0, 3.0), r2Vector(1.0, 1.0), n, n);
    uint32_t corners[] = {0, n - 1};
    r2SoftBodyDesc_SetPinnedParticles(&sheet_desc, (R2IndexView){corners, 2});
    sheet_desc.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30.0, 1.0});
    sheet_desc.particleMass = 0.05;
    sheet_desc.particleRadius = (R2OptionalReal){1, 0.05};
    sheet_desc.collider = r2BallColliderDesc(0.05);
    sheet_desc.collider.friction = 0.8;
    R2SoftBodyHandle sheet = r2InsertSoftBody(world, &sheet_desc);

    // A jelly square with elastic cells.
    R2SoftBodyDesc jelly_desc = r2GridSoftBodyDesc(r2Vector(3.0, 1.2), r2Vector(1.0, 1.0), 6, 6);
    jelly_desc.cellModel = R2_SOFT_CELL_COROTATIONAL;
    jelly_desc.material.youngModulus = 3.0e3;
    jelly_desc.material.poissonRatio = 0.35;
    jelly_desc.material.elasticDampingRatio = 0.5;
    jelly_desc.material.plasticYield = 0.05;
    jelly_desc.material.plasticCreep = 20.0;
    jelly_desc.material.tearStrain = (R2OptionalReal){1, 0.4};
    jelly_desc.particleMass = 0.2;
    R2SoftBodyHandle jelly = r2InsertSoftBody(world, &jelly_desc);

    // A pressurized blob: a ring of 24 particles inflated by area preservation.
    R2SoftBodyDesc blob_desc = r2DiskSoftBodyDesc(r2Vector(0.0, 3.0), 0.8, 24);
    blob_desc.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20.0, 1.0});
    blob_desc.volumePreservation = 1;
    blob_desc.volumeFactor = 1.1;
    blob_desc.selfContacts = 1;
    R2SoftBodyHandle blob = r2InsertSoftBody(world, &blob_desc);

    // DOCUSAURUS: Oriented start
    // A shell: a closed surface that is not oriented, so its inner side holds the bodies put
    // inside it (a bowl, a box, a container). A closed surface is oriented by default.
    R2SoftBodyDesc bowl = r2DiskSoftBodyDesc(r2Vector(-3.0, 2.0), 0.8, 24);
    // Default: disabled, i.e., oriented if the surface is closed.
    bowl.oriented = (R2OptionalBool){1, 0};
    bowl.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){60.0, 1.0});
    R2SoftBodyHandle bowl_handle = r2InsertSoftBody(world, &bowl);
    // DOCUSAURUS: Oriented stop
    (void)bowl_handle;

    // DOCUSAURUS: Forces start
    // The last argument set to 1 makes sure the soft-body is awake.
    r2SoftBody_ResetForces(sheet, 1); // Reset the forces to zero.
    r2SoftBody_AddForce(sheet, r2Vector(0.0, 1.0), 1); // Added to the force of each particle.
    r2SoftBody_AddParticleForce(sheet, 3, r2Vector(0.0, 1.0), 1);
    r2SoftBody_ApplyImpulse(sheet, r2Vector(0.0, 0.1), 1);
    r2SoftBody_ApplyParticleImpulse(sheet, 3, r2Vector(0.0, 0.1), 1);
    // An impulse on the particles within 0.5 of a point, scaled down with the distance.
    r2SoftBody_ApplyImpulseAtPoint(sheet, r2Vector(0.0, 0.1), r2Vector(-3.0, 3.0), 0.5, 1);
    // A blast pushing the particles away from a center.
    r2SoftBody_ApplyRadialImpulse(sheet, r2Vector(-3.0, 3.0), 0.1, 1.0, 1);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid box, at the particle's position.
    R2SoftBodyDesc rope = r2RopeSoftBodyDesc(r2Vector(8.0, 9.0), r2Vector(12.0, 9.0), 25);
    uint32_t pinned[] = {0};
    r2SoftBodyDesc_SetPinnedParticles(&rope, (R2IndexView){pinned, 1});
    rope.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){40.0, 1.0});
    R2SoftBodyHandle rope_handle = r2InsertSoftBody(world, &rope);
    R2Vector last_position = r2SoftBody_ParticlePosition(rope_handle, 24);
    R2RigidBodyDesc weight_body = r2DynamicRigidBodyDesc();
    weight_body.position.translation = r2VectorSub(last_position, r2Vector(0.0, 0.4));
    R2RigidBodyHandle weight = r2InsertRigidBody(world, &weight_body);
    R2ColliderDesc weight_collider = r2CuboidColliderDesc(r2Vector(0.3, 0.3));
    weight_collider.density = 2.0;
    r2InsertCollider(weight, &weight_collider);
    r2SoftBody_AttachParticle(rope_handle, 24, weight);
    // DOCUSAURUS: Attachments stop

    {
        // DOCUSAURUS: DeformableColliders start
        // A deformable polyline bound to the blob: each vertex follows one particle (direct),
        // or is embedded in the cell holding it (skinned). The polyline is given in the frame
        // of the proxy it is attached to.
        R2RigidBodyHandle root = r2SoftBody_RootBody(blob);
        R2Pose root_pose_inverse = r2PoseInverse(r2RigidBody_Position(root));
        size_t num = r2SoftBody_NumParticles(blob); // 24 particles.
        R2Vector vertices[24];
        R2Edge indices[24];
        uint32_t particles[24];
        r2SoftBody_ParticlePositions(blob, vertices, 24);
        for (uint32_t i = 0; i < num; i++) {
            vertices[i] = r2PoseTransformPoint(root_pose_inverse, vertices[i]);
            indices[i] = (R2Edge){i, (i + 1) % num};
            // The vertex `i` follows the particle `i`.
            particles[i] = i;
        }
        R2ColliderDesc outline = r2DefaultColliderDesc();
        r2ShapeDesc_SetPolyline(&outline.shape, (R2VectorView){vertices, num}, (R2EdgeView){indices, num},
                                R2_POLYLINE_DEFORMABLE);
        outline.isSensor = 1;
        R2SoftMeshBindingDesc binding = r2DefaultSoftMeshBindingDesc();
        binding.kind = R2_SOFT_BINDING_DIRECT;
        binding.particles = (R2IndexView){particles, num};
        R2ColliderHandle outline_handle = r2InsertDeformableCollider(&outline, &binding, root);
        // The polyline follows the particles: read its current vertices back.
        R2Vector outline_vertices[24];
        size_t num_outline_vertices = r2SoftBody_MeshVertices(blob, outline_handle, outline_vertices, 24);
        // DOCUSAURUS: DeformableColliders stop
        if (num != 24 || num_outline_vertices != num) {
            return EXIT_FAILURE;
        }
    }

    {
        // DOCUSAURUS: Skinning start
        // A detailed outline held by a coarse cage of cells: only the cells are simulated, and the
        // outline (the skin) follows their deformation.
        R2Vector vertices[48];
        R2Edge indices[48];
        for (uint32_t i = 0; i < 48; i++) {
            R2Real angle = (R2Real)i / 48 * 2.0 * R2_PI;
            vertices[i] = r2Vector(0.5 * cos(angle), 0.5 * sin(angle));
            indices[i] = (R2Edge){i, (i + 1) % 48};
        }
        R2VectorView outline_vertices = {vertices, 48};
        R2SurfaceElementView outline_segments = {indices, 48};
        // The cage: the outline filled with cells of about 0.25 in size.
        R2SoftBodyDesc skinned =
            r2VolumetricSoftBodyDesc(outline_vertices, outline_segments, r2NewVolumeMeshParameters(0.25));
        // The skin: the outline itself, following the cells holding its vertices.
        r2SoftBodyDesc_SetSkin(&skinned, outline_vertices, outline_segments);
        // Collide through the skin instead of the boundary of the cage.
        skinned.skinCollision = 1;
        skinned.translation = r2Vector(0.0, 4.0);
        R2SoftBodyHandle skinned_handle = r2InsertSoftBody(world, &skinned);

        // The skin is the body's collision mesh: read its vertices back to render it.
        R2ColliderHandle skin_collider;
        r2SoftBody_MeshColliders(skinned_handle, &skin_collider, 1);
        R2Vector skin_vertices[48];
        size_t num_skin_vertices = r2SoftBody_MeshVertices(skinned_handle, skin_collider, skin_vertices, 48);
        // DOCUSAURUS: Skinning stop
        if (num_skin_vertices != 48) {
            return EXIT_FAILURE;
        }
    }

    // DOCUSAURUS: Plasticity start
    // The jelly has elastic (corotational) cells: the plasticity of volume cells has no effect.
    R2SoftBodyMaterial plastic_material = r2SoftBody_Material(jelly);
    // Cells: the rest shape flows toward the current one past 5% strain, at a rate of 20 per
    // second, up to a total permanent deformation of 50%.
    plastic_material.plasticYield = 0.05;
    plastic_material.plasticCreep = 20.0;
    plastic_material.plasticMax = 0.5;
    // Edges: the rest length flows past 10% strain, up to half the initial length, but only
    // when squeezed (a dent stays, a stretch springs back).
    plastic_material.edgePlasticYield = 0.1;
    plastic_material.edgePlasticCreep = 10.0;
    plastic_material.edgePlasticMax = 0.5;
    plastic_material.edgePlasticFlow = R2_SOFT_EDGE_PLASTIC_FLOW_COMPRESSION;
    r2SoftBody_SetMaterial(jelly, &plastic_material);
    // Every permanent deformation can be undone at once.
    r2SoftBody_ResetPlasticity(jelly);
    // DOCUSAURUS: Plasticity stop

    // DOCUSAURUS: TearingMaterial start
    R2SoftBodyMaterial tear_material = r2SoftBody_Material(sheet);
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    tear_material.tearStrain = (R2OptionalReal){1, 0.4};
    tear_material.tearForce = (R2OptionalReal){1, 50.0};
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    tear_material.tearSmoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    tear_material.interiorStrength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    tear_material.minPiece = (R2OptionalU32){1, 10};
    r2SoftBody_SetMaterial(sheet, &tear_material);
    // DOCUSAURUS: TearingMaterial stop

    // DOCUSAURUS: Tearing start
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    r2SoftBody_TearEdge(sheet, 10); // Applied at the end of the next step.
    // Tear at once along edges and through cells; pieces the tear disconnects become soft
    // bodies of their own. The event is NULL if nothing changed.
    uint32_t torn_edges[] = {11, 12};
    R2SoftBodyTearEvent *tear = r2SoftBody_Tear(sheet, torn_edges, 2, NULL, 0);
    if (tear != NULL) {
        printf("%zu edges torn\n", r2SoftBodyTearEvent_TornEdges(tear, NULL, 0) / 2);
        r2FreeSoftBodyTearEvent(tear);
    }
    // Cut along a blade (a segment in 2D), without removing material.
    R2Vector blade[2] = {{-3.0, -10.0}, {-3.0, 10.0}};
    R2SoftBodyTearEvent *cut = r2CutSoftBody(sheet, blade);
    if (cut != NULL) {
        // The soft-bodies the sheet is now made of, the one keeping its handle first.
        size_t num_pieces = r2SoftBodyTearEvent_PieceCount(cut);
        R2SoftBodyHandle *pieces = malloc(num_pieces * sizeof(R2SoftBodyHandle));
        r2SoftBodyTearEvent_Bodies(cut, pieces, num_pieces);
        for (size_t i = 0; i < num_pieces; i++) {
            size_t num_piece_particles = r2SoftBodyTearEvent_PieceParticles(cut, i, NULL, 0);
            printf("piece %u has %zu particles\n", pieces[i].index, num_piece_particles);
        }
        free(pieces);
        // Where a particle of the torn body went.
        R2OptionalParticleDestination destination = r2SoftBodyTearEvent_TryParticleDestination(cut, n * n - 1);
        if (destination.found) {
            printf("particle %u is now particle %u of %u\n", n * n - 1, destination.index,
                   destination.body.index);
        }
        r2FreeSoftBodyTearEvent(cut);
    }
    // DOCUSAURUS: Tearing stop

    // DOCUSAURUS: Events start
    // Tears applied during a step are reported through the event collector.
    R2EventCollector *events = r2NewEventCollector();
    r2Step(world, NULL, events);
    size_t num_tear_events = r2EventCollector_TearEventCount(events);
    for (size_t i = 0; i < num_tear_events; i++) {
        // An owned copy of the event.
        R2SoftBodyTearEvent *tear_event = r2EventCollector_TearEvent(events, i);
        R2SoftBodyHandle torn = r2SoftBodyTearEvent_SoftBody(tear_event);
        printf("Soft body %u tore\n", torn.index);
        r2FreeSoftBodyTearEvent(tear_event);
    }
    // The collector keeps its events until it is cleared.
    r2EventCollector_Clear(events);
    // DOCUSAURUS: Events stop
    r2FreeEventCollector(events);

    // DOCUSAURUS: Settings start
    // Settings shared by every soft-body of the world.
    // Strain beyond which a constraint is re-solved after the contacts of every substep.
    // Default: 0.75
    r2SoftBodiesSetResweepStrain(world, 0.75);
    // Extra substeps a soft-body requests while it is hit fast; 0 disables them.
    // Default: 4
    r2SoftBodiesSetMaxExtraSubsteps(world, 4);
    // Stiffening of the soft-body contacts relative to the rigid ones.
    // Default: 4.0
    r2SoftBodiesSetContactStiffening(world, 4.0);
    // The tangle detection and recovery stack can be switched off mechanism by mechanism.
    r2RecoverySetCrossingRepulsion(world, 1);
    // The settings can also be read all at once (as a copy), modified, and written back.
    R2IntegrationParameters params = r2IntegrationParameters(world);
    params.softBodies.recovery.selfStandDown = 1;
    r2SetIntegrationParameters(world, &params);
    // DOCUSAURUS: Settings stop

    for (int i = 0; i < 10; i++) {
        r2Step(world, NULL, NULL);
    }

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
