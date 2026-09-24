/* Port of examples3d/soft_meshes3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct Face {
    uint32_t key[3], vertices[3];
    R3Vector centroid;
} Face;

static int compareFaces(const void *a, const void *b) {
    const Face *fa = a, *fb = b;
    for (size_t i = 0; i < 3; ++i) {
        if (fa->key[i] != fb->key[i]) {
            return fa->key[i] < fb->key[i] ? -1 : 1;
        }
    }
    return 0;
}

/* Closed boundary of cells owned by a cluster, capped over the cut. */
static void clusterSurface(R3SoftBodyHandle soft, const uint32_t *particles,
                           size_t count, R3Vector **vertices, uint32_t **indices,
                           size_t *triangleCount) {
    size_t n, cellIndexCount;
    n = r3SoftBody_NumParticles(soft);
    uint32_t *vertexOf = malloc(n * sizeof(*vertexOf));
    *vertices = malloc(count * sizeof(**vertices));
    if (!vertexOf || !*vertices) {
        abort();
    }
    for (size_t i = 0; i < n; ++i) {
        vertexOf[i] = UINT32_MAX;
    }
    for (size_t i = 0; i < count; ++i) {
        vertexOf[particles[i]] = i;
        (*vertices)[i] = r3SoftBody_ParticlePosition(soft, particles[i]);
    }
    cellIndexCount = r3SoftBody_Cells(soft, NULL, 0);
    uint32_t *cells = malloc(cellIndexCount * sizeof(*cells));
    Face *faces = malloc(cellIndexCount * sizeof(*faces));
    if (!cells || !faces) {
        abort();
    }
    cellIndexCount = r3SoftBody_Cells(soft, cells, cellIndexCount);
    size_t faceCount = 0;
    for (size_t i = 0; i < cellIndexCount; i += 4) {
        const uint32_t *cell = &cells[i];
        int owned = 1;
        R3Vector centroid = {0};
        for (size_t j = 0; j < 4; ++j) {
            if (vertexOf[cell[j]] == UINT32_MAX) {
                owned = 0;
                break;
            }
            centroid = r3VectorAdd(centroid, (*vertices)[vertexOf[cell[j]]]);
        }
        if (!owned) {
            continue;
        }
        centroid = r3VectorScale(centroid, .25);
        for (size_t k = 0; k < 4; ++k) {
            Face *face = &faces[faceCount++];
            face->centroid = centroid;
            for (size_t j = 0; j < 3; ++j) {
                face->key[j] = face->vertices[j] = cell[(k + j + 1) % 4];
            }
            for (size_t a = 0; a < 3; ++a) {
                for (size_t b = a + 1; b < 3; ++b) {
                    if (face->key[a] > face->key[b]) {
                        uint32_t tmp = face->key[a];
                        face->key[a] = face->key[b];
                        face->key[b] = tmp;
                    }
                }
            }
        }
    }
    qsort(faces, faceCount, sizeof(*faces), compareFaces);
    *indices = malloc(faceCount * 3 * sizeof(**indices));
    if (faceCount && !*indices) {
        abort();
    }
    *triangleCount = 0;
    for (size_t i = 0; i < faceCount;) {
        size_t next = i + 1;
        while (next < faceCount && !compareFaces(&faces[i], &faces[next])) {
            ++next;
        }
        if (next == i + 1) {
            const Face *face = &faces[i];
            uint32_t mapped[3];
            R3Vector p[3];
            for (size_t j = 0; j < 3; ++j) {
                mapped[j] = vertexOf[face->vertices[j]];
                p[j] = (*vertices)[mapped[j]];
            }
            const R3Vector normal = r3VectorCross(r3VectorSub(p[1], p[0]), r3VectorSub(p[2], p[0]));
            const R3Vector center =
                r3VectorScale(r3VectorAdd(r3VectorAdd(p[0], p[1]), p[2]), 1.0 / 3);
            if (r3VectorDot(normal, r3VectorSub(center, face->centroid)) < 0) {
                uint32_t tmp = mapped[1];
                mapped[1] = mapped[2];
                mapped[2] = tmp;
            }
            memcpy(&(*indices)[3 * (*triangleCount)++], mapped, sizeof(mapped));
        }
        i = next;
    }
    free(faces);
    free(cells);
    free(vertexOf);
}

static R3SoftBodyDesc jelly(R3Vector center, R3Real halfExtents, size_t n, R3Real young) {
    R3SoftBodyDesc builder =
        r3CuboidSoftBodyDesc(center, r3Vector(halfExtents, halfExtents, halfExtents), n, n, n);
    builder.cellModel = R3_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .1;
    builder.particleRadius = (R3OptionalReal){1, .05};
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = young;
        material.poissonRatio = .4;
        material.elasticDampingRatio = .5;
        builder.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.05);
        surface.friction = .7;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftMeshes3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(12, 0.1, 12));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }

    /* Skinned jelly. */
    R3SoftBodyHandle skinned;
    {
        const R3Vector center = {-3, 1.2, 0};
        R3SoftBodyDesc builder = jelly(center, .6, 4, 200);
        builder.canSleep = !testbed->noSleep;
        skinned = r3InsertSoftBody(world, &builder);

        R3SharedShape *ball = NULL;
        R3TriMeshData *mesh = NULL;
        ball = r3BallSharedShape(1.2);
        mesh = r3SharedShape_ToTrimesh(ball, 20, 20);
        r3FreeSharedShape(ball);
        size_t vertexCount, indexCount;
        vertexCount = r3TriMeshData_Vertices(mesh, NULL, 0);
        indexCount = r3TriMeshData_Indices(mesh, NULL, 0);
        R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
        uint32_t *indices = malloc(indexCount * sizeof(*indices));
        if (!vertices || !indices) {
            abort();
        }
        vertexCount = r3TriMeshData_Vertices(mesh, vertices, vertexCount);
        indexCount = r3TriMeshData_Indices(mesh, indices, indexCount);
        r3FreeTriMeshData(mesh);
        for (size_t i = 0; i < vertexCount; ++i) {
            vertices[i] = r3VectorAdd(vertices[i], center);
        }
        R3RigidBodyHandle proxy = r3SoftBody_RootBody(skinned);
        R3Pose pose = r3RigidBody_Position(proxy);
        R3ColliderDesc collider = r3DefaultColliderDesc();
        r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){vertices, vertexCount},
                              (R3TriangleView){(const R3Triangle *)indices, indexCount / 3},
                              R3_TRIMESH_DEFORMABLE);

        collider.position = r3PoseInverse(pose);
        collider.isSensor = 1;
        R3SoftMeshBindingDesc binding = r3DefaultSoftMeshBindingDesc();
        r3InsertDeformableCollider(&collider, &binding, proxy);
        free(vertices);
        free(indices);
    }
    /* Plated jelly. */
    R3SoftBodyHandle plated;
    {
        const R3Vector center = {0, 1.2, 0};
        R3SoftBodyDesc builder = jelly(center, .6, 4, 400);
        builder.canSleep = !testbed->noSleep;
        plated = r3InsertSoftBody(world, &builder);

        R3Vector positions[64];
        size_t count = r3SoftBody_ParticlePositions(plated, positions, 64);
        uint32_t top[64];
        size_t topCount = 0;
        for (size_t i = 0; i < count; ++i) {
            if (positions[i].y > center.y + .3) {
                top[topCount++] = i;
            }
        }
        uint32_t cluster = r3SoftBody_AddCluster(plated, top, topCount);

        R3RigidBodyHandle proxy = r3SoftBody_ClusterProxy(plated, cluster);
        R3ColliderDesc plate = r3CuboidColliderDesc(r3Vector(.7, .05, .7));
        plate.position.translation = r3Vector(0, .65, 0);
        r3InsertCollider(proxy, &plate);
    }
    /* Split jelly. */
    R3SoftBodyHandle split;
    {
        const R3Vector center = {3, 1.2, 0};
        R3SoftBodyDesc builder = jelly(center, .6, 4, 400);
        builder.canSleep = !testbed->noSleep;
        builder.collisionEnabled = 0;
        split = r3InsertSoftBody(world, &builder);

        R3Vector positions[64];
        size_t count = r3SoftBody_ParticlePositions(split, positions, 64);
        uint32_t halves[2][64];
        size_t counts[2] = {0};
        for (size_t i = 0; i < count; ++i) {
            size_t side = positions[i].x < center.x ? 0 : 1;
            halves[side][counts[side]++] = i;
        }
        for (size_t side = 0; side < 2; ++side) {
            uint32_t cluster = r3SoftBody_AddCluster(split, halves[side], counts[side]);

            R3RigidBodyHandle proxy = r3SoftBody_ClusterProxy(split, cluster);
            R3Vector *vertices;
            uint32_t *indices;
            size_t triangleCount;
            clusterSurface(split, halves[side], counts[side], &vertices, &indices,
                           &triangleCount);
            R3Pose pose = r3RigidBody_Position(proxy);
            R3ColliderDesc collider = r3DefaultColliderDesc();
            r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){vertices, counts[side]},
                                  (R3TriangleView){(const R3Triangle *)indices, triangleCount},
                                  R3_TRIMESH_DEFORMABLE);

            collider.position = r3PoseInverse(pose);
            collider.friction = .7;
            R3SoftMeshBindingDesc binding = r3DefaultSoftMeshBindingDesc();
            binding.kind = R3_SOFT_BINDING_DIRECT;
            binding.particles = (R3IndexView){halves[side], counts[side]};
            binding.selfContacts = 1;
            r3InsertDeformableCollider(&collider, &binding, proxy);
            free(vertices);
            free(indices);
        }
    }
    for (int i = 0; i < 3; ++i) {
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(-3 + 3 * i, 4 + i, 0);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.3, 0.3, 0.3));
            collider.density = 4;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    tbCamera(testbed, -6, 4, 8, 0, 1, 0);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
