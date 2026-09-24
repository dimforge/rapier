/* Port of examples3d/stress_tests/trimesh3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsTrimesh3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    const R3Vector groundSize = r3Vector(200, 1, 200);
    const size_t nsubdivs = 20;
    R3Real heights[21 * 21];
    for (size_t j = 0; j <= nsubdivs; ++j) {
        for (size_t i = 0; i <= nsubdivs; ++i) {
            const R3Real x = i * groundSize.x / nsubdivs;
            const R3Real z = j * groundSize.z / nsubdivs;
            heights[i + j * 21] =
                i == 0 || i == nsubdivs || j == 0 || j == nsubdivs ? 10 : sin(x) + cos(z);
        }
    }
    /* Build the triangle mesh from the native heightfield's mesh representation. */
    R3SharedShape *heightfield =
        r3HeightfieldSharedShape((R3RealView){heights, (21) * (21)}, 21, 21, groundSize);
    R3TriMeshData *mesh = r3SharedShape_ToTrimesh(heightfield, 3, 2);
    r3FreeSharedShape(heightfield);
    size_t vertexCount = 0, indexCount = 0;
    vertexCount = r3TriMeshData_Vertices(mesh, NULL, 0);
    indexCount = r3TriMeshData_Indices(mesh, NULL, 0);
    R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
    uint32_t *indices = malloc(indexCount * sizeof(*indices));
    if (!vertices || !indices) {
        abort();
    }
    vertexCount = r3TriMeshData_Vertices(mesh, vertices, vertexCount);
    indexCount = r3TriMeshData_Indices(mesh, indices, indexCount);
    R3ColliderDesc collider = r3DefaultColliderDesc();
    r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){vertices, vertexCount},
                          (R3TriangleView){(const R3Triangle *)indices, indexCount / 3}, 0);

    r3FreeTriMeshData(mesh);
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    free(vertices);
    free(indices);

    for (int j = 0; j < 47; ++j) {
        for (int i = 0; i < 8; ++i) {
            for (int k = 0; k < 8; ++k) {
                R3ColliderDesc collider;
                if (j % 2 == 0) {
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                } else {
                    collider = r3BallColliderDesc(1);
                }
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 3 - 12, j * 3 + 4.5, k * 3 - 12);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
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
