/* Port of examples3d/soft_trimesh3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/obj.h"

void tbSoftTrimesh3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    tbLabel(testbed, "Tetrahedra", "...");
    static const char *const modelNames[] = {
        "All",   "Camel",   "Chair",  "Cup",       "Dinosaur",    "Torus 2", "Feline",  "Genus 3",
        "Torus", "Octopus", "Rabbit", "Rust logo", "Screwdriver", "Table",   "Torus 3", "Hornbug"};
    const int selectedModel =
        (int)tbChoice(testbed, "Model", 0, modelNames, TB_COUNT(modelNames), 0, 0);
    R3VolumeMeshParameters meshing =
        r3NewVolumeMeshParameters(tbSetting(testbed, "Cell size", 2, .4, 4, 0));
    meshing.enclosure = (uint32_t)tbSetting(testbed, "Crust (surface shell only)", 0, 0, 1, 1);
    meshing.cover_smoothing = (uint32_t)tbSetting(testbed, "Cover smoothing", 20, 0, 50, 1);
    meshing.cover_guard = tbSetting(testbed, "Cover guard", .15, .02, .5, 0);
    meshing.cover_subdivisions = (uint32_t)tbSetting(testbed, "Cover subdivision", 1, 0, 3, 1);
    const int wearSkin = (int)tbSetting(testbed, "Wear the model as a skin", 1, 0, 1, 1);
    const int skinCollision = wearSkin ? (int)tbSetting(testbed, "Skin collisions", 0, 0, 1, 1) : 0;
#ifdef RAPIER_FEM
    const int femSolver = (int)tbSetting(testbed, "FEM solver", 0, 0, 1, 1);
#endif
    const int cellModel =
        (int)tbSetting(testbed, "Cell model: 0 Corotational, 1 Neo-Hookean, 2 Volume", 0, 0, 2, 1);
    const R3Real youngModulus =
        cellModel == 2 ? 1e5 : tbSetting(testbed, "Young modulus", 1e5, 1e4, 1e6, 0);
    const int selfContacts = (int)tbSetting(testbed, "Self contacts", 0, 0, 1, 1);
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

    const char *models[] = {"camel_decimated.obj",
                            "chair.obj",
                            "cup_decimated.obj",
                            "dilo_decimated.obj",
                            "tstTorusModel2.obj",
                            "feline_decimated.obj",
                            "genus3_decimated.obj",
                            "tstTorusModel.obj",
                            "octopus_decimated.obj",
                            "rabbit_decimated.obj",
                            "rust_logo_simplified.obj",
                            "screwdriver_decimated.obj",
                            "table.obj",
                            "tstTorusModel3.obj",
                            "hornbug.obj"};
    const size_t ngeoms = selectedModel == 0 ? TB_COUNT(models) : 1;
    const size_t width = (size_t)fmax(ceil(sqrt(ngeoms)), 1);
    size_t totalCells = 0, totalBodies = 0, igeom = 0;
    for (size_t modelIndex = 0; modelIndex < TB_COUNT(models); ++modelIndex) {
        if (selectedModel && modelIndex + 1 != (size_t)selectedModel) {
            continue;
        }
        const size_t slot = igeom++;
        ObjMesh mesh;
        if (!loadObj(testbed->assetRoot, models[modelIndex], &mesh)) {
            continue;
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
        R3SoftBodyDesc filled = r3VolumetricSoftBodyDesc(
            (R3VectorView){mesh.vertices, mesh.vertexCount},
            (R3SurfaceElementView){(const R3Triangle *)mesh.indices, mesh.indexCount / 3}, meshing);
        const R3Real x = (slot % width) * 9.0 - (width - 1) * 9.0 / 2;
        const R3Real y = (slot / width) * 8.0 + 7;
        if (wearSkin) {
            r3SoftBodyDesc_SetSkin(
                &filled, (R3VectorView){mesh.vertices, mesh.vertexCount},
                (R3SurfaceElementView){(const R3Triangle *)mesh.indices, mesh.indexCount / 3});
            filled.skinCollision = skinCollision;
        }
        filled.translation = r3Vector(x, y, 0);
        filled.cellModel = cellModel == 1   ? R3_SOFT_CELL_NEO_HOOKEAN
                           : cellModel == 2 ? R3_SOFT_CELL_VOLUME
                                            : R3_SOFT_CELL_COROTATIONAL;
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = youngModulus;
        material.poissonRatio = .35;
        material.elasticDampingRatio = .5;
        material.deformationDamping = 2.5;
        filled.material = material;
        filled.particleMass = .05;
        filled.particleRadius =
            (R3OptionalReal){1, meshing.cell_size / (1u << meshing.cover_subdivisions) * .25};
        filled.selfContacts = selfContacts;
        filled.canSleep = !testbed->noSleep;
        {
            R3ColliderDesc surface = r3BallColliderDesc(.1);
            surface.friction = .6;
            filled.collider = surface;
        }
#ifdef RAPIER_FEM
        filled.solver = femSolver ? 1 : 0;
#endif
        R3SoftBodyHandle handle;
        /* As in the Rust demo, skip models the volume mesher cannot fill. */
        R3ErrorHandler handler = r3SetErrorHandler((R3ErrorHandler){0});
        handle = r3InsertSoftBody(world, &filled);
        R3Status status = r3LastStatus();
        r3SetErrorHandler(handler);
        freeObj(&mesh);
        if (status != R3_OK) {
            continue;
        }

        size_t cellIndices = r3SoftBody_Cells(handle, NULL, 0);
        totalCells += cellIndices / 4;
        ++totalBodies;

        R3RigidBodyHandle proxy = r3SoftBody_RootBody(handle);
        tbBodyColor(testbed, proxy, .85, .35, .3, 1);
    }
    char label[128];
    snprintf(label, sizeof(label), "%zu in %zu bodies", totalCells, totalBodies);
    tbLabel(testbed, "Tetrahedra", label);
    tbCamera(testbed, 60, 40, 60, 0, 5, 0);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
