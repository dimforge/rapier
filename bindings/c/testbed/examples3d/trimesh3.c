/* Port of examples3d/trimesh3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbTrimesh3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    const R3Vector groundSize = r3Vector(100, 1, 100);
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
                          (R3TriangleView){(const R3Triangle *)indices, indexCount / 3},
                          R3_TRIMESH_MERGE_DUPLICATE_VERTICES);

    r3FreeTriMeshData(mesh);
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    free(vertices);
    free(indices);

    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                R3ColliderDesc collider;
                switch (j % 6) {
                case 0:
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    break;
                case 1:
                    collider = r3BallColliderDesc(1);
                    break;
                case 2:
                    collider = r3RoundCylinderColliderDesc(1, 1, 0.1);
                    break;
                case 3:
                    collider = r3ConeColliderDesc(1, 1);
                    break;
                case 4:
                    collider = r3CapsuleYColliderDesc(1, 1);
                    break;
                default: {
                    R3Pose poses[] = {r3TranslationPose(r3Vector(0, 0, 0)),
                                      r3TranslationPose(r3Vector(1, 0, 0)),
                                      r3TranslationPose(r3Vector(-1, 0, 0))};
                    R3SharedShape *shapes[3] = {NULL};
                    shapes[0] = r3CuboidSharedShape(r3Vector(1.0, 0.5, 0.5));
                    shapes[1] = r3CuboidSharedShape(r3Vector(0.5, 1.0, 0.5));
                    shapes[2] = r3CuboidSharedShape(r3Vector(0.5, 1.0, 0.5));
                    R3CompoundShapeDesc colliderParts[TB_COUNT(shapes)];
                    for (size_t part = 0; part < TB_COUNT(shapes); ++part) {
                        colliderParts[part].pose = poses[part];
                        colliderParts[part].shape = r3DefaultShapeDesc();
                        colliderParts[part].shape.kind = R3_SHAPE_DESC_SHARED;
                        colliderParts[part].shape.sharedShape =
                            ((const R3SharedShape *const *)shapes)[part];
                    }
                    collider = r3DefaultColliderDesc();
                    collider.shape.kind = R3_SHAPE_DESC_COMPOUND;
                    collider.shape.children =
                        (R3CompoundShapeView){colliderParts, TB_COUNT(shapes)};
                    R3SharedShape *compoundShape = r3ShapeDesc_Build(&collider.shape);
                    collider.shape.kind = R3_SHAPE_DESC_SHARED;
                    collider.shape.sharedShape = compoundShape;
                    for (size_t part = 0; part < TB_COUNT(shapes); part++) {
                        r3FreeSharedShape(shapes[part]);
                    }
                    break;
                }
                }
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 3 - 12, j * 3 + 4.5, k * 3 - 12);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
                if (collider.shape.kind == R3_SHAPE_DESC_SHARED) {
                    r3FreeSharedShape((R3SharedShape *)collider.shape.sharedShape);
                }
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
