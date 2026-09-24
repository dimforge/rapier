/* Keep the asserts of this snippet active in release builds. */
#undef NDEBUG
#include <assert.h>

#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    // DOCUSAURUS: Creation start
    // A world with a ground.
    R3World *world = r3NewWorld();
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(10.0, 0.1, 10.0));
    r3InsertColliderWithoutParent(world, &ground);

    // Description of a rope of 20 particles between two points.
    R3SoftBodyDesc rope = r3RopeSoftBodyDesc(r3Vector(0.0, 3.0, 0.0), r3Vector(2.0, 3.0, 0.0), 20);
    // Description of a cloth: `nu` by `nv` particles, particle `(i, j)` at `origin + i * du + j * dv`.
    const uint32_t n = 20;
    R3SoftBodyDesc cloth =
        r3ClothSoftBodyDesc(r3Vector(-1.0, 2.0, -1.0), r3Vector(0.1, 0.0, 0.0), r3Vector(0.0, 0.0, 0.1), n, n);
    // Description of a box of `nx * ny * nz` particles filled with tetrahedral cells.
    R3SoftBodyDesc box = r3CuboidSoftBodyDesc(r3Vector(3.0, 1.0, 0.0), r3Vector(0.5, 0.5, 0.5), 4, 4, 4);
    // Description of a hollow sphere holding its volume (a balloon).
    R3SoftBodyDesc balloon = r3SphereSoftBodyDesc(r3Vector(0.0, 3.0, 3.0), 0.8, 2);

    // Any field of a description can be modified before its insertion.
    // Particles held in place.
    const uint32_t pinned[] = {0, n - 1, n * (n - 1), n * n - 1};
    cloth.pinned = (R3IndexView){pinned, 4};
    // A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
    cloth.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30.0, 1.0});
    // The mass of each particle.
    // Default: 1.0
    cloth.particleMass = 0.05;
    // The thickness of the particles, for collisions.
    // Default: disabled, i.e., the radius computed by the constructor.
    cloth.particleRadius = (R3OptionalReal){1, 0.02};
    // The template of the body's colliders: its shape is replaced by the deformable surface.
    cloth.collider = r3BallColliderDesc(0.05);
    cloth.collider.friction = 0.8;
    // Whether the surface collides with itself.
    // Default: 0
    cloth.selfContacts = 1;
    // Whether the body may fall asleep.
    // Default: 1
    cloth.canSleep = 1;
    // Insert the soft-body: this creates its hidden root rigid-body and its colliders.
    R3SoftBodyHandle cloth_handle = r3InsertSoftBody(world, &cloth);
    // DOCUSAURUS: Creation stop
    (void)rope;
    (void)box;
    (void)balloon;

    // DOCUSAURUS: Volumetric start
    // Fill a closed, outward-oriented triangle mesh with tetrahedral cells of about 0.2 in size.
    // The triangle mesh of a cuboid (the subdivision counts only matter for curved shapes).
    R3SharedShape *cuboid = r3CuboidSharedShape(r3Vector(0.5, 0.25, 0.25));
    R3TriMeshData *mesh = r3SharedShape_ToTrimesh(cuboid, 0, 0);
    size_t num_vertices = r3TriMeshData_Vertices(mesh, NULL, 0);
    size_t num_indices = r3TriMeshData_Indices(mesh, NULL, 0);
    R3Vector *vertices = malloc(num_vertices * sizeof(R3Vector));
    R3Triangle *triangles = malloc(num_indices * sizeof(uint32_t));
    r3TriMeshData_Vertices(mesh, vertices, num_vertices);
    r3TriMeshData_Indices(mesh, (uint32_t *)triangles, num_indices);

    R3SoftBodyDesc block = r3VolumetricSoftBodyDesc((R3VectorView){vertices, num_vertices},
                                                    (R3TriangleView){triangles, num_indices / 3},
                                                    r3NewVolumeMeshParameters(0.2));
    block.translation = r3Vector(-3.0, 1.0, 0.0);
    // The mesh is only read during the insertion, which fails if it isn't closed.
    R3SoftBodyHandle block_handle = r3InsertSoftBody(world, &block);
    free(vertices);
    free(triangles);
    r3FreeTriMeshData(mesh);
    r3FreeSharedShape(cuboid);
    // DOCUSAURUS: Volumetric stop
    assert(r3SoftBody_Contains(block_handle));

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    R3Vector points[27];
    for (int i = 0; i < 27; i++) {
        points[i] = r3VectorScale(r3Vector(i % 3, i / 3 % 3 + 4.0, i / 9), 0.3);
    }
    R3SoftBodyDesc cloud = r3DefaultSoftBodyDesc();
    r3SoftBodyDesc_SetParticles(&cloud, (R3VectorView){points, 27});
    cloud.shapeMatching = (R3OptionalBool){1, 1};
    // How fast the particles are pulled back toward their rest shape.
    cloud.material.shapeMatchingSoftness = (R3SpringCoefficients){5.0, 1.0};
    cloud.particleRadius = (R3OptionalReal){1, 0.1};
    R3SoftBodyHandle cloud_handle = r3InsertSoftBody(world, &cloud);
    // DOCUSAURUS: ShapeMatching stop
    assert(r3SoftBody_NumParticles(cloud_handle) == 27);

    // DOCUSAURUS: Sets start
    // The world owns every soft-body: their number and their handles can be read at any time.
    R3SoftBodyDesc rope_desc = r3RopeSoftBodyDesc(r3Vector(0.0, 3.0, 0.0), r3Vector(2.0, 3.0, 0.0), 20);
    R3SoftBodyHandle rope_handle = r3InsertSoftBody(world, &rope_desc);
    size_t num_soft_bodies = r3SoftBodyCount(world);
    R3SoftBodyHandle *soft_bodies = malloc(num_soft_bodies * sizeof(R3SoftBodyHandle));
    r3SoftBodyHandles(world, soft_bodies, num_soft_bodies);
    for (size_t i = 0; i < num_soft_bodies; i++) {
        printf("Soft-body %u has %zu particles.\n", soft_bodies[i].index,
               r3SoftBody_NumParticles(soft_bodies[i]));
    }
    free(soft_bodies);
    // Whether a handle still refers to a soft-body of the world.
    assert(r3SoftBody_Contains(rope_handle));
    // The soft-body a rigid-body stands for (its root body, or the proxy of one of its clusters).
    R3SoftBodyHandle owner = r3RigidBody_SoftBody(r3SoftBody_RootBody(rope_handle));
    assert(owner.index == rope_handle.index && owner.generation == rope_handle.generation);
    // DOCUSAURUS: Sets stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly cube with corotational linear elasticity.
    R3SoftBodyDesc jelly = r3CuboidSoftBodyDesc(r3Vector(3.0, 1.0, 0.0), r3Vector(0.5, 0.5, 0.5), 4, 4, 4);
    // The constitutive model of the cells: R3_SOFT_CELL_VOLUME (per-cell volume constraints,
    // the shape is held by the edges), R3_SOFT_CELL_COROTATIONAL or R3_SOFT_CELL_NEO_HOOKEAN.
    jelly.cellModel = R3_SOFT_CELL_COROTATIONAL;
    // Stiffness of the elastic cells.
    jelly.material.youngModulus = 2.0e3;
    jelly.material.poissonRatio = 0.35;
    jelly.material.elasticDampingRatio = 0.5;
    // Plasticity: the rest shape flows past 5% strain, at 20 per second.
    jelly.material.plasticYield = 0.05;
    jelly.material.plasticCreep = 20.0;
    // Tearing: an element past 40% strain tears.
    jelly.material.tearStrain = (R3OptionalReal){1, 0.4};
    jelly.particleMass = 0.2;
    R3SoftBodyHandle jelly_handle = r3InsertSoftBody(world, &jelly);

    // A material shared by the edges, bending constraints and volume constraints.
    R3SoftBodyMaterial material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30.0, 1.0});
    // Softness of the bending constraints, on top of a uniform 30 Hz softness.
    material.bendSoftness = (R3SpringCoefficients){3.0, 1.0};
    r3SoftBody_SetMaterial(cloth_handle, &material);
    // DOCUSAURUS: Material stop

    // DOCUSAURUS: Fem start
    // A stiff beam simulated by the FEM solver (requires the `fem` feature): its stiffness
    // doesn't depend on the number of solver iterations.
    R3SoftBodyDesc beam = r3CuboidSoftBodyDesc(r3Vector(0.0, 2.0, -3.0), r3Vector(1.0, 0.1, 0.1), 11, 3, 3);
    beam.solver = R3_SOFT_SOLVER_FEM;
    beam.cellModel = R3_SOFT_CELL_NEO_HOOKEAN;
    beam.material.youngModulus = 1.0e5;
    beam.material.poissonRatio = 0.3;
    // The particles of the face at `x = -1` are the first 3 × 3 ones.
    const uint32_t beam_pinned[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    beam.pinned = (R3IndexView){beam_pinned, 9};
    R3SoftBodyHandle beam_handle = r3InsertSoftBody(world, &beam);

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    r3FemSetLinearTolerance(world, 1.0e-5);
    r3FemSetMaxLinearIterations(world, 20);
    // DOCUSAURUS: Fem stop
    assert(r3SoftBody_Contains(beam_handle));

    // DOCUSAURUS: Particles start
    // Read the particles.
    R3Vector position = r3SoftBody_ParticlePosition(cloth_handle, 0);
    R3Vector velocity = r3SoftBody_ParticleVelocity(cloth_handle, 0);
    size_t num_particles = r3SoftBody_NumParticles(cloth_handle);
    R3Vector *positions = malloc(num_particles * sizeof(R3Vector));
    r3SoftBody_ParticlePositions(cloth_handle, positions, num_particles);
    // Move a particle.
    r3SoftBody_SetParticlePosition(cloth_handle, 1, r3VectorAdd(position, r3Vector(0.0, 0.1, 0.0)));
    r3SoftBody_SetParticleVelocity(cloth_handle, 1, velocity);
    // Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
    r3SoftBody_SetParticlePinned(cloth_handle, 2, 1);
    r3SoftBody_SetParticleKinematicTarget(cloth_handle, 2, r3Vector(-1.0, 2.5, -0.8));
    // The elements: edges, cells and the boundary triangles, as flat arrays of particle indices
    // (2, 4, and 3 indices per element). A NULL buffer with a zero capacity gives their length.
    size_t num_edges = r3SoftBody_Edges(cloth_handle, NULL, 0) / 2;
    size_t num_cells = r3SoftBody_Cells(cloth_handle, NULL, 0) / 4;
    size_t boundary_len = r3SoftBody_Boundary(cloth_handle, NULL, 0);
    uint32_t *boundary = malloc(boundary_len * sizeof(uint32_t));
    r3SoftBody_Boundary(cloth_handle, boundary, boundary_len);
    assert(num_edges > 0 && num_cells == 0 && boundary_len > 0);
    free(positions);
    free(boundary);
    // DOCUSAURUS: Particles stop

    // DOCUSAURUS: RootBody start
    // The rigid-body the engine created for the whole soft-body, read back after its insertion.
    R3RigidBodyHandle root = r3SoftBody_RootBody(jelly_handle);
    assert(r3RigidBody_IsSoftFrame(root));

    // A rigid collider attached to it follows the frame of the whole body: here a sensor
    // detecting what comes close to the jelly.
    R3ColliderDesc sensor = r3BallColliderDesc(1.0);
    sensor.isSensor = 1;
    r3InsertCollider(root, &sensor);

    // A joint attached to it acts on the soft-body as a whole: this one hangs the jelly under a
    // fixed anchor by a spring.
    R3RigidBodyDesc anchor_desc = r3FixedRigidBodyDesc();
    anchor_desc.position.translation = r3Vector(3.0, 4.0, 0.0);
    R3RigidBodyHandle anchor = r3InsertRigidBody(world, &anchor_desc);
    R3JointDesc spring = r3SpringJointDesc(2.5, 60.0, 2.0);
    r3InsertImpulseJoint(anchor, root, &spring);
    // DOCUSAURUS: RootBody stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly: a rigid proxy that joints and
    // colliders can attach to.
    size_t num_jelly_particles = r3SoftBody_NumParticles(jelly_handle);
    uint32_t *top = malloc(num_jelly_particles * sizeof(uint32_t));
    size_t num_top = 0;
    for (uint32_t i = 0; i < num_jelly_particles; i++) {
        if (r3SoftBody_ParticlePosition(jelly_handle, i).y > 1.3) {
            top[num_top++] = i;
        }
    }
    uint32_t cluster = r3SoftBody_AddCluster(jelly_handle, top, num_top);
    free(top);
    R3RigidBodyHandle proxy = r3SoftBody_ClusterProxy(jelly_handle, cluster);
    // A rigid plate welded onto the cluster.
    R3RigidBodyDesc plate_desc = r3DynamicRigidBodyDesc();
    plate_desc.position.translation = r3Vector(3.0, 1.9, 0.0);
    R3RigidBodyHandle plate = r3InsertRigidBody(world, &plate_desc);
    R3ColliderDesc plate_collider = r3CuboidColliderDesc(r3Vector(0.7, 0.05, 0.7));
    plate_collider.density = 0.4;
    r3InsertCollider(plate, &plate_collider);
    R3JointDesc weld = r3FixedJointDesc();
    weld.localFrame1.translation = r3Vector(0.0, -0.1, 0.0);
    r3InsertImpulseJoint(plate, proxy, &weld);
    // A cluster can be pinned, driven or tuned as a whole.
    r3SoftBody_SetClusterStiffnessScale(jelly_handle, cluster, 2.0);
    r3SoftBody_SetClusterShapeMatchingEnabled(jelly_handle, cluster, 1);
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: ClusterControl start
    // Pin every particle of the cluster, then move it along a path: the cluster behaves like a
    // kinematic rigid part dragging the rest of the body.
    r3SoftBody_SetClusterPinned(jelly_handle, cluster, 1);
    r3SoftBody_SetClusterKinematicTarget(jelly_handle, cluster, r3TranslationPose(r3Vector(3.0, 2.0, 0.0)));
    // Release it: the cluster is simulated again.
    r3SoftBody_SetClusterPinned(jelly_handle, cluster, 0);
    // DOCUSAURUS: ClusterControl stop

    // DOCUSAURUS: Removal start
    // Removing a soft-body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    r3RemoveSoftBody(rope_handle);
    // A cluster can be removed on its own.
    r3SoftBody_RemoveCluster(jelly_handle, cluster);
    // DOCUSAURUS: Removal stop
    assert(!r3SoftBody_Contains(rope_handle));

    for (int i = 0; i < 10; i++) {
        r3Step(world, NULL, NULL);
    }

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
