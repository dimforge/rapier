/* Keep the asserts of this snippet active in release builds. */
#undef NDEBUG
#include <assert.h>

#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    // DOCUSAURUS: Creation start
    // A world with a ground.
    R2World *world = r2NewWorld();
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(10.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    // Description of a rope of 20 particles between two points.
    R2SoftBodyDesc rope = r2RopeSoftBodyDesc(r2Vector(0.0, 3.0), r2Vector(2.0, 3.0), 20);
    // Description of a grid of `nx` by `ny` particles filled with triangle cells.
    R2SoftBodyDesc grid = r2GridSoftBodyDesc(r2Vector(3.0, 1.0), r2Vector(1.0, 1.0), 6, 6);
    // Description of a disk: a ring of particles holding its area (a pressurized blob).
    R2SoftBodyDesc disk = r2DiskSoftBodyDesc(r2Vector(0.0, 3.0), 0.8, 24);
    // Description of a closed polygon of particles holding its area.
    const R2Vector polygon_points[] = {
        r2Vector(5.0, 4.0),
        r2Vector(7.0, 4.0),
        r2Vector(7.0, 6.0),
        r2Vector(5.0, 6.0),
    };
    R2SoftBodyDesc polygon = r2PolygonSoftBodyDesc((R2VectorView){polygon_points, 4});

    const uint32_t n = 20;
    R2SoftBodyDesc sheet = r2GridSoftBodyDesc(r2Vector(-3.0, 3.0), r2Vector(1.0, 1.0), n, n);
    // Particles held in place.
    const uint32_t pinned[] = {0, n - 1};
    sheet.pinned = (R2IndexView){pinned, 2};
    // A uniform softness (natural frequency in Hz, damping ratio) for every constraint.
    sheet.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30.0, 1.0});
    // The mass of each particle.
    // Default: 1.0
    sheet.particleMass = 0.05;
    // The thickness of the particles, for collisions.
    // Default: disabled, i.e., the radius computed by the constructor.
    sheet.particleRadius = (R2OptionalReal){1, 0.05};
    // The template of the body's colliders: its shape is replaced by the deformable surface.
    sheet.collider = r2BallColliderDesc(0.05);
    sheet.collider.friction = 0.8;
    // Whether the body may fall asleep.
    // Default: 1
    sheet.canSleep = 1;
    // Insert the soft-body: this creates its hidden root rigid-body and its colliders.
    R2SoftBodyHandle sheet_handle = r2InsertSoftBody(world, &sheet);
    // DOCUSAURUS: Creation stop
    (void)polygon;
    (void)rope;
    (void)grid;
    (void)disk;

    // DOCUSAURUS: Volumetric start
    // Fill a closed, counter-clockwise polyline with triangle cells of about 0.2 in size.
    const R2Vector vertices[] = {
        r2Vector(-0.5, -0.25),
        r2Vector(0.5, -0.25),
        r2Vector(0.5, 0.25),
        r2Vector(-0.5, 0.25),
    };
    const R2Edge indices[] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    R2SoftBodyDesc block = r2VolumetricSoftBodyDesc((R2VectorView){vertices, 4}, (R2EdgeView){indices, 4},
                                                    r2NewVolumeMeshParameters(0.2));
    block.translation = r2Vector(-3.0, 1.0);
    // The polyline is only read during the insertion, which fails if it isn't closed.
    R2SoftBodyHandle block_handle = r2InsertSoftBody(world, &block);
    // DOCUSAURUS: Volumetric stop
    assert(r2SoftBody_Contains(block_handle));

    // DOCUSAURUS: ShapeMatching start
    // A cloud of particles without any element: shape matching alone pulls them back toward
    // their rest shape, placed where it best fits the current one.
    R2Vector points[9];
    for (int i = 0; i < 9; i++) {
        points[i] = r2VectorScale(r2Vector(i % 3, i / 3 + 4.0), 0.3);
    }
    R2SoftBodyDesc cloud = r2DefaultSoftBodyDesc();
    r2SoftBodyDesc_SetParticles(&cloud, (R2VectorView){points, 9});
    cloud.shapeMatching = (R2OptionalBool){1, 1};
    // How fast the particles are pulled back toward their rest shape.
    cloud.material.shapeMatchingSoftness = (R2SpringCoefficients){5.0, 1.0};
    cloud.particleRadius = (R2OptionalReal){1, 0.1};
    R2SoftBodyHandle cloud_handle = r2InsertSoftBody(world, &cloud);
    // DOCUSAURUS: ShapeMatching stop
    assert(r2SoftBody_NumParticles(cloud_handle) == 9);

    // DOCUSAURUS: Sets start
    // The world owns every soft-body: their number and their handles can be read at any time.
    R2SoftBodyDesc rope_desc = r2RopeSoftBodyDesc(r2Vector(0.0, 3.0), r2Vector(2.0, 3.0), 20);
    R2SoftBodyHandle rope_handle = r2InsertSoftBody(world, &rope_desc);
    size_t num_soft_bodies = r2SoftBodyCount(world);
    R2SoftBodyHandle *soft_bodies = malloc(num_soft_bodies * sizeof(R2SoftBodyHandle));
    r2SoftBodyHandles(world, soft_bodies, num_soft_bodies);
    for (size_t i = 0; i < num_soft_bodies; i++) {
        printf("Soft-body %u has %zu particles.\n", soft_bodies[i].index,
               r2SoftBody_NumParticles(soft_bodies[i]));
    }
    free(soft_bodies);
    // Whether a handle still refers to a soft-body of the world.
    assert(r2SoftBody_Contains(rope_handle));
    // The soft-body a rigid-body stands for (its root body, or the proxy of one of its clusters).
    R2SoftBodyHandle owner = r2RigidBody_SoftBody(r2SoftBody_RootBody(rope_handle));
    assert(owner.index == rope_handle.index && owner.generation == rope_handle.generation);
    // DOCUSAURUS: Sets stop

    // DOCUSAURUS: Material start
    // Elastic cells: a jelly square with corotational linear elasticity.
    R2SoftBodyDesc jelly = r2GridSoftBodyDesc(r2Vector(3.0, 1.2), r2Vector(1.0, 1.0), 6, 6);
    // The constitutive model of the cells: R2_SOFT_CELL_VOLUME (per-cell area constraints,
    // the shape is held by the edges), R2_SOFT_CELL_COROTATIONAL or R2_SOFT_CELL_NEO_HOOKEAN.
    jelly.cellModel = R2_SOFT_CELL_COROTATIONAL;
    // Stiffness of the elastic cells.
    jelly.material.youngModulus = 3.0e3;
    jelly.material.poissonRatio = 0.35;
    jelly.material.elasticDampingRatio = 0.5;
    // Plasticity: the rest shape flows past 5% strain, at 20 per second.
    jelly.material.plasticYield = 0.05;
    jelly.material.plasticCreep = 20.0;
    // Tearing: an element past 40% strain tears.
    jelly.material.tearStrain = (R2OptionalReal){1, 0.4};
    jelly.particleMass = 0.2;
    R2SoftBodyHandle jelly_handle = r2InsertSoftBody(world, &jelly);

    // A pressurized blob: a ring of particles inflated by area preservation.
    R2SoftBodyDesc blob = r2DiskSoftBodyDesc(r2Vector(0.0, 3.0), 0.8, 24);
    blob.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20.0, 1.0});
    // Target area multiplier (`> 1` inflates the body), for the area preservation enabled by
    // the disk constructor (`volumePreservation`).
    blob.volumeFactor = 1.1;
    blob.selfContacts = 1;
    R2SoftBodyHandle blob_handle = r2InsertSoftBody(world, &blob);
    // DOCUSAURUS: Material stop
    assert(r2SoftBody_VolumeFactor(blob_handle) > 1.0);

    // DOCUSAURUS: Fem start
    // A stiff beam simulated by the FEM solver (requires the `fem` feature): its stiffness
    // doesn't depend on the number of solver iterations.
    R2SoftBodyDesc beam = r2GridSoftBodyDesc(r2Vector(0.0, 2.0), r2Vector(1.0, 0.1), 21, 3);
    beam.solver = R2_SOFT_SOLVER_FEM;
    beam.cellModel = R2_SOFT_CELL_NEO_HOOKEAN;
    beam.material.youngModulus = 1.0e5;
    beam.material.poissonRatio = 0.3;
    // The particles of the side at `x = -1` are the first 3 ones.
    const uint32_t beam_pinned[] = {0, 1, 2};
    beam.pinned = (R2IndexView){beam_pinned, 3};
    R2SoftBodyHandle beam_handle = r2InsertSoftBody(world, &beam);

    // The tuning of the linear solves of the FEM solver, shared by every body using it.
    r2FemSetLinearTolerance(world, 1.0e-5);
    r2FemSetMaxLinearIterations(world, 20);
    // DOCUSAURUS: Fem stop
    assert(r2SoftBody_Contains(beam_handle));

    // DOCUSAURUS: Particles start
    // Read the particles.
    R2Vector position = r2SoftBody_ParticlePosition(sheet_handle, 0);
    R2Vector velocity = r2SoftBody_ParticleVelocity(sheet_handle, 0);
    size_t num_particles = r2SoftBody_NumParticles(sheet_handle);
    R2Vector *positions = malloc(num_particles * sizeof(R2Vector));
    r2SoftBody_ParticlePositions(sheet_handle, positions, num_particles);
    // Move a particle.
    r2SoftBody_SetParticlePosition(sheet_handle, 1, r2VectorAdd(position, r2Vector(0.0, 0.1)));
    r2SoftBody_SetParticleVelocity(sheet_handle, 1, velocity);
    // Pin (or release) a particle; a pinned particle can be driven like a kinematic body.
    r2SoftBody_SetParticlePinned(sheet_handle, 2, 1);
    r2SoftBody_SetParticleKinematicTarget(sheet_handle, 2, r2Vector(-3.5, 3.5));
    // The elements: edges, cells and the boundary segments, as flat arrays of particle indices
    // (2, 3, and 2 indices per element). A NULL buffer with a zero capacity gives their length.
    size_t num_edges = r2SoftBody_Edges(sheet_handle, NULL, 0) / 2;
    size_t num_cells = r2SoftBody_Cells(sheet_handle, NULL, 0) / 3;
    size_t boundary_len = r2SoftBody_Boundary(sheet_handle, NULL, 0);
    uint32_t *boundary = malloc(boundary_len * sizeof(uint32_t));
    r2SoftBody_Boundary(sheet_handle, boundary, boundary_len);
    assert(num_edges > 0 && num_cells > 0 && boundary_len > 0);
    free(positions);
    free(boundary);
    // DOCUSAURUS: Particles stop

    // DOCUSAURUS: RootBody start
    // The rigid-body the engine created for the whole soft-body, read back after its insertion.
    R2RigidBodyHandle root = r2SoftBody_RootBody(jelly_handle);
    assert(r2RigidBody_IsSoftFrame(root));

    // A rigid collider attached to it follows the frame of the whole body: here a sensor
    // detecting what comes close to the jelly.
    R2ColliderDesc sensor = r2BallColliderDesc(1.6);
    sensor.isSensor = 1;
    r2InsertCollider(root, &sensor);

    // A joint attached to it acts on the soft-body as a whole: this one hangs the jelly under a
    // fixed anchor by a spring.
    R2RigidBodyDesc anchor_desc = r2FixedRigidBodyDesc();
    anchor_desc.position.translation = r2Vector(3.0, 5.0);
    R2RigidBodyHandle anchor = r2InsertRigidBody(world, &anchor_desc);
    R2JointDesc spring = r2SpringJointDesc(2.0, 60.0, 2.0);
    r2InsertImpulseJoint(anchor, root, &spring);
    // DOCUSAURUS: RootBody stop

    // DOCUSAURUS: Clusters start
    // A cluster over the top particles of the jelly: a rigid proxy that joints and
    // colliders can attach to.
    size_t num_jelly_particles = r2SoftBody_NumParticles(jelly_handle);
    uint32_t *top = malloc(num_jelly_particles * sizeof(uint32_t));
    size_t num_top = 0;
    for (uint32_t i = 0; i < num_jelly_particles; i++) {
        if (r2SoftBody_ParticlePosition(jelly_handle, i).y > 2.0) {
            top[num_top++] = i;
        }
    }
    uint32_t cluster = r2SoftBody_AddCluster(jelly_handle, top, num_top);
    free(top);
    R2RigidBodyHandle proxy = r2SoftBody_ClusterProxy(jelly_handle, cluster);
    // A rigid plate welded onto the cluster.
    R2RigidBodyDesc plate_desc = r2DynamicRigidBodyDesc();
    plate_desc.position.translation = r2Vector(3.0, 2.4);
    R2RigidBodyHandle plate = r2InsertRigidBody(world, &plate_desc);
    R2ColliderDesc plate_collider = r2CuboidColliderDesc(r2Vector(1.2, 0.05));
    plate_collider.density = 0.4;
    r2InsertCollider(plate, &plate_collider);
    R2JointDesc weld = r2FixedJointDesc();
    weld.localFrame1.translation = r2Vector(0.0, -0.1);
    r2InsertImpulseJoint(plate, proxy, &weld);
    // A cluster can be pinned, driven or tuned as a whole.
    r2SoftBody_SetClusterStiffnessScale(jelly_handle, cluster, 2.0);
    r2SoftBody_SetClusterShapeMatchingEnabled(jelly_handle, cluster, 1);
    // DOCUSAURUS: Clusters stop

    // DOCUSAURUS: ClusterControl start
    // Pin every particle of the cluster, then move it along a path: the cluster behaves like a
    // kinematic rigid part dragging the rest of the body.
    r2SoftBody_SetClusterPinned(jelly_handle, cluster, 1);
    r2SoftBody_SetClusterKinematicTarget(jelly_handle, cluster, r2TranslationPose(r2Vector(3.0, 2.5)));
    // Release it: the cluster is simulated again.
    r2SoftBody_SetClusterPinned(jelly_handle, cluster, 0);
    // DOCUSAURUS: ClusterControl stop

    // DOCUSAURUS: Removal start
    // Removing a soft-body removes its root body, its proxies, its colliders and the joints
    // attached to them.
    r2RemoveSoftBody(rope_handle);
    // A cluster can be removed on its own.
    r2SoftBody_RemoveCluster(jelly_handle, cluster);
    // DOCUSAURUS: Removal stop
    assert(!r2SoftBody_Contains(rope_handle));

    for (int i = 0; i < 10; i++) {
        r2Step(world, NULL, NULL);
    }

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
