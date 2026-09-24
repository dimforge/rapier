#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();

    // A ground.
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(10.0, 0.1, 10.0));
    r3InsertColliderWithoutParent(world, &ground);

    // A cloth pinned by its four corners.
    uint32_t n = 20;
    R3SoftBodyDesc cloth_desc =
        r3ClothSoftBodyDesc(r3Vector(-1.0, 2.0, -1.0), r3Vector(0.1, 0.0, 0.0), r3Vector(0.0, 0.0, 0.1), n, n);
    uint32_t corners[] = {0, n - 1, n * (n - 1), n * n - 1};
    r3SoftBodyDesc_SetPinnedParticles(&cloth_desc, (R3IndexView){corners, 4});
    cloth_desc.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30.0, 1.0});
    cloth_desc.particleMass = 0.05;
    cloth_desc.particleRadius = (R3OptionalReal){1, 0.02};
    cloth_desc.collider = r3BallColliderDesc(0.05);
    cloth_desc.collider.friction = 0.8;
    cloth_desc.selfContacts = 1;
    R3SoftBodyHandle cloth = r3InsertSoftBody(world, &cloth_desc);

    // A jelly cube with elastic cells.
    R3SoftBodyDesc jelly_desc = r3CuboidSoftBodyDesc(r3Vector(3.0, 1.0, 0.0), r3Vector(0.5, 0.5, 0.5), 4, 4, 4);
    jelly_desc.cellModel = R3_SOFT_CELL_COROTATIONAL;
    jelly_desc.material.youngModulus = 2.0e3;
    jelly_desc.material.poissonRatio = 0.35;
    jelly_desc.material.elasticDampingRatio = 0.5;
    jelly_desc.material.plasticYield = 0.05;
    jelly_desc.material.plasticCreep = 20.0;
    jelly_desc.material.tearStrain = (R3OptionalReal){1, 0.4};
    jelly_desc.particleMass = 0.2;
    R3SoftBodyHandle jelly = r3InsertSoftBody(world, &jelly_desc);

    // DOCUSAURUS: Oriented start
    // A shell: a closed surface that is not oriented, so its inner side holds the bodies put
    // inside it (a bowl, a box, a container). A closed surface is oriented by default.
    R3SoftBodyDesc bowl = r3SphereSoftBodyDesc(r3Vector(-3.0, 2.0, 0.0), 0.8, 2);
    // Default: disabled, i.e., oriented if the surface is closed.
    bowl.oriented = (R3OptionalBool){1, 0};
    bowl.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){60.0, 1.0});
    R3SoftBodyHandle bowl_handle = r3InsertSoftBody(world, &bowl);
    // DOCUSAURUS: Oriented stop
    (void)bowl_handle;

    // DOCUSAURUS: Forces start
    // The last argument set to 1 makes sure the soft-body is awake.
    r3SoftBody_ResetForces(cloth, 1); // Reset the forces to zero.
    r3SoftBody_AddForce(cloth, r3Vector(0.0, 1.0, 0.0), 1); // Added to the force of each particle.
    r3SoftBody_AddParticleForce(cloth, 3, r3Vector(0.0, 1.0, 0.0), 1);
    r3SoftBody_ApplyImpulse(cloth, r3Vector(0.0, 0.1, 0.0), 1);
    r3SoftBody_ApplyParticleImpulse(cloth, 3, r3Vector(0.0, 0.1, 0.0), 1);
    // An impulse on the particles within 0.5 of a point, scaled down with the distance.
    r3SoftBody_ApplyImpulseAtPoint(cloth, r3Vector(0.0, 0.1, 0.0), r3Vector(0.0, 2.0, 0.0), 0.5, 1);
    // A blast pushing the particles away from a center.
    r3SoftBody_ApplyRadialImpulse(cloth, r3Vector(0.0, 2.0, 0.0), 0.1, 1.0, 1);
    // DOCUSAURUS: Forces stop

    // DOCUSAURUS: Attachments start
    // Attach the last particle of a rope to a rigid ball, at the particle's position.
    R3SoftBodyDesc rope = r3RopeSoftBodyDesc(r3Vector(-0.5, 5.0, 3.0), r3Vector(2.5, 5.0, 3.0), 30);
    uint32_t pinned[] = {0};
    r3SoftBodyDesc_SetPinnedParticles(&rope, (R3IndexView){pinned, 1});
    rope.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){40.0, 1.0});
    R3SoftBodyHandle rope_handle = r3InsertSoftBody(world, &rope);
    R3Vector last_position = r3SoftBody_ParticlePosition(rope_handle, 29);
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3VectorSub(last_position, r3Vector(0.0, 0.3, 0.0));
    R3RigidBodyHandle ball = r3InsertRigidBody(world, &ball_body);
    R3ColliderDesc ball_collider = r3BallColliderDesc(0.25);
    ball_collider.density = 2.0;
    r3InsertCollider(ball, &ball_collider);
    r3SoftBody_AttachParticle(rope_handle, 29, ball);
    // DOCUSAURUS: Attachments stop

    {
        // DOCUSAURUS: DeformableColliders start
        // A deformable triangle mesh bound to the jelly: each vertex is embedded in the cell
        // holding it (skinned), or follows one particle (direct). The mesh is given in the
        // frame of the proxy it is attached to.
        R3RigidBodyHandle root = r3SoftBody_RootBody(jelly);
        R3Pose root_pose_inverse = r3PoseInverse(r3RigidBody_Position(root));
        R3Vector center = r3SoftBody_CenterOfMass(jelly);
        R3Real r = 1.0;
        R3Vector offsets[6] = {{r, 0.0, 0.0}, {-r, 0.0, 0.0}, {0.0, r, 0.0},
                               {0.0, -r, 0.0}, {0.0, 0.0, r}, {0.0, 0.0, -r}};
        R3Vector vertices[6];
        for (size_t i = 0; i < 6; i++) {
            vertices[i] = r3PoseTransformPoint(root_pose_inverse, r3VectorAdd(center, offsets[i]));
        }
        R3Triangle indices[8] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
                                 {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
        R3ColliderDesc skin = r3DefaultColliderDesc();
        r3ShapeDesc_SetTrimesh(&skin.shape, (R3VectorView){vertices, 6}, (R3TriangleView){indices, 8},
                               R3_TRIMESH_DEFORMABLE);
        skin.isSensor = 1;
        // Default: R3_SOFT_BINDING_SKINNED.
        R3SoftMeshBindingDesc binding = r3DefaultSoftMeshBindingDesc();
        R3ColliderHandle skin_handle = r3InsertDeformableCollider(&skin, &binding, root);
        // The mesh follows the particles: read its current vertices back.
        R3Vector skin_vertices[6];
        size_t num_skin_vertices = r3SoftBody_MeshVertices(jelly, skin_handle, skin_vertices, 6);
        // DOCUSAURUS: DeformableColliders stop
        if (num_skin_vertices != 6) {
            return EXIT_FAILURE;
        }
    }

    {
        // DOCUSAURUS: Skinning start
        // A detailed mesh held by a coarse cage of cells: only the cells are simulated, and the mesh
        // (the skin) follows their deformation.
        R3SharedShape *ball_shape = r3BallSharedShape(0.5);
        R3TriMeshData *ball_mesh = r3SharedShape_ToTrimesh(ball_shape, 24, 24);
        size_t num_vertices = r3TriMeshData_Vertices(ball_mesh, NULL, 0);
        size_t num_indices = r3TriMeshData_Indices(ball_mesh, NULL, 0);
        R3Vector *vertices = malloc(num_vertices * sizeof(R3Vector));
        uint32_t *indices = malloc(num_indices * sizeof(uint32_t));
        r3TriMeshData_Vertices(ball_mesh, vertices, num_vertices);
        r3TriMeshData_Indices(ball_mesh, indices, num_indices);
        r3FreeTriMeshData(ball_mesh);
        r3FreeSharedShape(ball_shape);

        R3VectorView mesh_vertices = {vertices, num_vertices};
        R3SurfaceElementView mesh_triangles = {(const R3Triangle *)indices, num_indices / 3};
        // The cage: the mesh filled with cells of about 0.25 in size.
        R3SoftBodyDesc skinned =
            r3VolumetricSoftBodyDesc(mesh_vertices, mesh_triangles, r3NewVolumeMeshParameters(0.25));
        // The skin: the mesh itself, following the cells holding its vertices.
        r3SoftBodyDesc_SetSkin(&skinned, mesh_vertices, mesh_triangles);
        // Collide through the skin instead of the boundary of the cage.
        skinned.skinCollision = 1;
        skinned.translation = r3Vector(0.0, 4.0, 3.0);
        R3SoftBodyHandle skinned_handle = r3InsertSoftBody(world, &skinned);
        // The mesh arrays are only borrowed until the insertion.
        free(vertices);
        free(indices);

        // The skin is the body's collision mesh: read its vertices back to render it.
        R3ColliderHandle skin_collider;
        r3SoftBody_MeshColliders(skinned_handle, &skin_collider, 1);
        size_t num_skin_vertices = r3SoftBody_MeshVertices(skinned_handle, skin_collider, NULL, 0);
        R3Vector *skin_vertices = malloc(num_skin_vertices * sizeof(R3Vector));
        r3SoftBody_MeshVertices(skinned_handle, skin_collider, skin_vertices, num_skin_vertices);
        // DOCUSAURUS: Skinning stop
        free(skin_vertices);
        if (num_skin_vertices != num_vertices) {
            return EXIT_FAILURE;
        }
    }

    // DOCUSAURUS: Plasticity start
    // The jelly has elastic (corotational) cells: the plasticity of volume cells has no effect.
    R3SoftBodyMaterial plastic_material = r3SoftBody_Material(jelly);
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
    plastic_material.edgePlasticFlow = R3_SOFT_EDGE_PLASTIC_FLOW_COMPRESSION;
    r3SoftBody_SetMaterial(jelly, &plastic_material);
    // Every permanent deformation can be undone at once.
    r3SoftBody_ResetPlasticity(jelly);
    // DOCUSAURUS: Plasticity stop

    // DOCUSAURUS: TearingMaterial start
    R3SoftBodyMaterial tear_material = r3SoftBody_Material(cloth);
    // An edge tears past 40% of stretch, or past a force of 50 along its direction.
    tear_material.tearStrain = (R3OptionalReal){1, 0.4};
    tear_material.tearForce = (R3OptionalReal){1, 50.0};
    // The load is smoothed over 0.1 second, so a single impact spike doesn't tear.
    tear_material.tearSmoothing = 0.1;
    // Undamaged interior elements are twice as tough: tears start from the surface.
    tear_material.interiorStrength = 2.0;
    // A tear never splits off a piece smaller than 10 elements.
    tear_material.minPiece = (R3OptionalU32){1, 10};
    r3SoftBody_SetMaterial(cloth, &tear_material);
    // DOCUSAURUS: TearingMaterial stop

    // DOCUSAURUS: Tearing start
    // Elements tear on their own past the material's thresholds; a tear can also be requested.
    r3SoftBody_TearEdge(cloth, 10); // Applied at the end of the next step.
    // Tear at once along edges and through cells; pieces the tear disconnects become soft
    // bodies of their own. The event is NULL if nothing changed.
    uint32_t torn_edges[] = {11, 12};
    R3SoftBodyTearEvent *tear = r3SoftBody_Tear(cloth, torn_edges, 2, NULL, 0);
    if (tear != NULL) {
        printf("%zu edges torn\n", r3SoftBodyTearEvent_TornEdges(tear, NULL, 0) / 2);
        r3FreeSoftBodyTearEvent(tear);
    }
    // Cut along a blade (a triangle in 3D), without removing material.
    R3Vector blade[3] = {{-0.1, -10.0, -10.0}, {-0.1, 10.0, 0.0}, {-0.1, -10.0, 10.0}};
    R3SoftBodyTearEvent *cut = r3CutSoftBody(cloth, blade);
    if (cut != NULL) {
        // The soft-bodies the cloth is now made of, the one keeping its handle first.
        size_t num_pieces = r3SoftBodyTearEvent_PieceCount(cut);
        R3SoftBodyHandle *pieces = malloc(num_pieces * sizeof(R3SoftBodyHandle));
        r3SoftBodyTearEvent_Bodies(cut, pieces, num_pieces);
        for (size_t i = 0; i < num_pieces; i++) {
            size_t num_piece_particles = r3SoftBodyTearEvent_PieceParticles(cut, i, NULL, 0);
            printf("piece %u has %zu particles\n", pieces[i].index, num_piece_particles);
        }
        free(pieces);
        // Where a particle of the torn body went.
        R3OptionalParticleDestination destination = r3SoftBodyTearEvent_TryParticleDestination(cut, n * n - 1);
        if (destination.found) {
            printf("particle %u is now particle %u of %u\n", n * n - 1, destination.index,
                   destination.body.index);
        }
        r3FreeSoftBodyTearEvent(cut);
    }
    // DOCUSAURUS: Tearing stop

    // DOCUSAURUS: Events start
    // Tears applied during a step are reported through the event collector.
    R3EventCollector *events = r3NewEventCollector();
    r3Step(world, NULL, events);
    size_t num_tear_events = r3EventCollector_TearEventCount(events);
    for (size_t i = 0; i < num_tear_events; i++) {
        // An owned copy of the event.
        R3SoftBodyTearEvent *tear_event = r3EventCollector_TearEvent(events, i);
        R3SoftBodyHandle torn = r3SoftBodyTearEvent_SoftBody(tear_event);
        printf("Soft body %u tore\n", torn.index);
        r3FreeSoftBodyTearEvent(tear_event);
    }
    // The collector keeps its events until it is cleared.
    r3EventCollector_Clear(events);
    // DOCUSAURUS: Events stop
    r3FreeEventCollector(events);

    // DOCUSAURUS: Settings start
    // Settings shared by every soft-body of the world.
    // Strain beyond which a constraint is re-solved after the contacts of every substep.
    // Default: 0.75
    r3SoftBodiesSetResweepStrain(world, 0.75);
    // Extra substeps a soft-body requests while it is hit fast; 0 disables them.
    // Default: 4
    r3SoftBodiesSetMaxExtraSubsteps(world, 4);
    // Stiffening of the soft-body contacts relative to the rigid ones.
    // Default: 4.0
    r3SoftBodiesSetContactStiffening(world, 4.0);
    // The tangle detection and recovery stack can be switched off mechanism by mechanism.
    r3RecoverySetCrossingRepulsion(world, 1);
    // The settings can also be read all at once (as a copy), modified, and written back.
    R3IntegrationParameters params = r3IntegrationParameters(world);
    params.softBodies.recovery.selfStandDown = 1;
    r3SetIntegrationParameters(world, &params);
    // DOCUSAURUS: Settings stop

    for (int i = 0; i < 10; i++) {
        r3Step(world, NULL, NULL);
    }

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
