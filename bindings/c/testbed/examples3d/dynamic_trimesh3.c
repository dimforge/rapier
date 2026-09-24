/* Port of examples3d/dynamic_trimesh3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/obj.h"

void dynamicTrimesh3RunImpl(Testbed *testbed, int useConvexDecomposition) {
    R3World *world = r3NewWorld();
    /* Floor made of a wavy mesh. */
    R3Real heights[101 * 101];
    for (size_t j = 0; j <= 100; ++j) {
        for (size_t i = 0; i <= 100; ++i) {
            heights[i + j * 101] = -cos(i * .2) - cos(j * .2);
        }
    }
    R3SharedShape *heightfield = r3HeightfieldSharedShape((R3RealView){heights, (101) * (101)}, 101,
                                                          101, r3Vector(100, 2, 100));
    R3TriMeshData *floor = r3SharedShape_ToTrimesh(heightfield, 3, 2);
    r3FreeSharedShape(heightfield);
    size_t vertexCount = 0, indexCount = 0;
    vertexCount = r3TriMeshData_Vertices(floor, NULL, 0);
    indexCount = r3TriMeshData_Indices(floor, NULL, 0);
    R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
    uint32_t *indices = malloc(indexCount * sizeof(*indices));
    if (!vertices || !indices) {
        abort();
    }
    vertexCount = r3TriMeshData_Vertices(floor, vertices, vertexCount);
    indexCount = r3TriMeshData_Indices(floor, indices, indexCount);
    R3ColliderDesc collider = r3DefaultColliderDesc();
    r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){vertices, vertexCount},
                          (R3TriangleView){(const R3Triangle *)indices, indexCount / 3},
                          R3_TRIMESH_FIX_INTERNAL_EDGES);
    r3InsertColliderWithoutParent(world, &collider);

    free(vertices);
    free(indices);
    r3FreeTriMeshData(floor);

    const char *geoms[] = {"camel_decimated.obj",       "chair.obj",
                           "cup_decimated.obj",         "dilo_decimated.obj",
                           "tstTorusModel2.obj",        "feline_decimated.obj",
                           "genus3_decimated.obj",      "hornbug.obj",
                           "tstTorusModel.obj",         "octopus_decimated.obj",
                           "rabbit_decimated.obj",      "rust_logo_simplified.obj",
                           "screwdriver_decimated.obj", "table.obj",
                           "tstTorusModel3.obj"};
    const size_t width = (size_t)sqrt(TB_COUNT(geoms));
    const int numDuplications = 4;
    const R3Real shiftY = 8, shiftXz = 9;
    for (size_t igeom = 0; igeom < TB_COUNT(geoms); ++igeom) {
        ObjMesh mesh;
        if (!loadObj(testbed->assetRoot, geoms[igeom], &mesh)) {
            r3FreeWorld(world);
            return;
        }
        R3Vector mins = mesh.vertices[0], maxs = mins;
        for (size_t i = 1; i < mesh.vertexCount; ++i) {
            R3Vector p = mesh.vertices[i];
            mins = r3Vector(fmin(mins.x, p.x), fmin(mins.y, p.y), fmin(mins.z, p.z));
            maxs = r3Vector(fmax(maxs.x, p.x), fmax(maxs.y, p.y), fmax(maxs.z, p.z));
        }
        const R3Vector center = r3VectorScale(r3VectorAdd(mins, maxs), .5);
        const R3Real diag = r3VectorLength(r3VectorSub(maxs, mins));
        for (size_t i = 0; i < mesh.vertexCount; ++i) {
            mesh.vertices[i] = r3VectorScale(r3VectorSub(mesh.vertices[i], center), 10 / diag);
        }
        R3SharedShape *decomposedShape = NULL;
        if (useConvexDecomposition) {
            decomposedShape = r3ConvexDecompositionSharedShape(
                (R3VectorView){mesh.vertices, mesh.vertexCount},
                (R3SurfaceElementView){(const R3Triangle *)mesh.indices, mesh.indexCount / 3});
        } else {
            decomposedShape = r3TrimeshSharedShapeWithFlags(
                (R3VectorView){mesh.vertices, mesh.vertexCount},
                (R3TriangleView){(const R3Triangle *)mesh.indices, mesh.indexCount / 3},
                R3_TRIMESH_FIX_INTERNAL_EDGES);
        }
        freeObj(&mesh);
        for (int k = 1; k <= numDuplications; ++k) {
            const R3Real x = (igeom % width) * shiftXz - numDuplications * shiftXz / 2;
            const R3Real y = (igeom / width) * shiftY + 7;
            const R3Real z = k * shiftXz - numDuplications * shiftXz / 2;
            R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
            body.position.translation = r3Vector(x, y, z);
            body.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3DefaultColliderDesc();
            collider.shape.kind = R3_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = decomposedShape;
            collider.contactSkin = .1;
            R3RigidBodyHandle bodyHandle = r3InsertRigidBody(world, &body);
            r3InsertCollider(bodyHandle, &collider);
        }
        r3FreeSharedShape(decomposedShape);
    }
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}

void tbDynamicTrimesh3(Testbed *testbed) {
    dynamicTrimesh3RunImpl(testbed, 0);
}
