#include "rapier_helpers.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OK(call)                                                                                   \
    do {                                                                                           \
        RAPIER_TYPE(Status) status = (call);                                                       \
        if (status != RAPIER_CONST(OK)) {                                                          \
            fprintf(stderr, "%s: %s\n", #call, RAPIER_FN(LastError)());                            \
            abort();                                                                               \
        }                                                                                          \
    } while (0)

/* Aggregate initialization is shared by C11 and C++17. */
static void test_meshes(RAPIER_TYPE(World) *world) {
    RAPIER_TYPE(Vector) points[4];
    memset(points, 0, sizeof(points));
    points[1].x = 1;
    points[2].y = 1;
    points[3].x = points[3].y = 1;
    RAPIER_TYPE(Triangle) triangles[] = {{0, 1, 2}, {1, 3, 2}};
    RAPIER_TYPE(Edge) edges[] = {{0, 1}, {1, 2}, {2, 3}};
    RAPIER_TYPE(VectorView) vertices = {points, 4};
    RAPIER_TYPE(TriangleView) faces = {triangles, 2};
    RAPIER_TYPE(EdgeView) segments = {edges, 3};
    RAPIER_TYPE(RigidBodyDesc) body = RAPIER_FN(FixedRigidBodyDesc)();
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(DefaultColliderDesc)();
    OK(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, faces, 0));
    assert(collider.shape.triangles.count == 2 && collider.shape.vertices.count == 4);
    assert(collider.shape.vertices.data == points); /* The setter borrows; insertion copies. */
    RAPIER_TYPE(RigidBodyHandle) bodyHandle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(bodyHandle, &collider);
    OK(RAPIER_FN(LastStatus)());
    OK(RAPIER_FN(ShapeDesc_SetPolyline)(&collider.shape, vertices, segments, 0));
    assert(collider.shape.edges.count == 3 &&
           collider.shape.kind == RAPIER_CONST(SHAPE_DESC_POLYLINE));
    bodyHandle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(bodyHandle, &collider);
    OK(RAPIER_FN(LastStatus)());
#ifdef RAPIER_DIM3
    points[3].z = 1;
#endif
    OK(RAPIER_FN(ShapeDesc_SetConvexHull)(&collider.shape, vertices));
    assert(collider.shape.triangles.count == 0 && collider.shape.triangles.data == NULL);
    bodyHandle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(bodyHandle, &collider);
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(ShapeDesc) original;
    memcpy(&original, &collider.shape, sizeof(original));
    RAPIER_TYPE(TriangleView) null_faces = {NULL, 2};
    assert(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, null_faces, 0) ==
           RAPIER_CONST(NULL_POINTER));
    assert(memcmp(&original, &collider.shape, sizeof(original)) == 0);
    RAPIER_TYPE(TriangleView) huge_faces = {triangles, SIZE_MAX};
    assert(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, huge_faces, 0) ==
           RAPIER_CONST(INVALID_ARGUMENT));
    assert(memcmp(&original, &collider.shape, sizeof(original)) == 0);
    RAPIER_TYPE(TriangleView)
    misaligned = {(const RAPIER_TYPE(Triangle) *)((const char *)triangles + 1), 1};
    assert(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, misaligned, 0) ==
           RAPIER_CONST(INVALID_ARGUMENT));
    assert(memcmp(&original, &collider.shape, sizeof(original)) == 0);
    assert(RAPIER_FN(ShapeDesc_SetTrimesh)(NULL, vertices, faces, 0) == RAPIER_CONST(NULL_POINTER));

    /* Values and index bounds are checked when building, not when assigning views. */
    triangles[1].c = 100;
    OK(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, faces, 0));
    bodyHandle = RAPIER_FN(InsertRigidBody)(world, &body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_FN(InsertCollider)(bodyHandle, &collider);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(INVALID_ARGUMENT));
    RAPIER_TYPE(TriangleView) empty = {NULL, 0};
    OK(RAPIER_FN(ShapeDesc_SetTrimesh)(&collider.shape, vertices, empty, 0));
    assert(collider.shape.triangles.count == 0);
}

static RAPIER_TYPE(SoftBodyHandle) test_soft(RAPIER_TYPE(World) *world) {
    RAPIER_TYPE(Vector) points[4];
    memset(points, 0, sizeof(points));
    points[0].y = 4;
    points[1].x = 1;
    points[1].y = 4;
    points[2].y = 5;
    points[3].x = 1;
    points[3].y = 5;
    RAPIER_TYPE(VectorView) vertices = {points, 4};
    RAPIER_TYPE(Edge) edges[] = {{0, 1}, {1, 2}, {2, 3}};
    RAPIER_TYPE(EdgeView) segments = {edges, 3};
    RAPIER_TYPE(Real) masses[] = {1, 2, 3, 4};
    RAPIER_TYPE(RealView) mass_view = {masses, 4};
    uint32_t pins[] = {0};
    RAPIER_TYPE(IndexView) pinned = {pins, 1};
    RAPIER_TYPE(SoftBodyDesc) soft = RAPIER_FN(DefaultSoftBodyDesc)();
    soft.gravityScale = 0.5;
    OK(RAPIER_FN(SoftBodyDesc_SetParticles)(&soft, vertices));
    OK(RAPIER_FN(SoftBodyDesc_SetEdges)(&soft, segments));
    OK(RAPIER_FN(SoftBodyDesc_SetMasses)(&soft, mass_view));
    OK(RAPIER_FN(SoftBodyDesc_SetPinnedParticles)(&soft, pinned));
    assert(soft.gravityScale == 0.5 && soft.edges.count == 3);
    RAPIER_TYPE(EdgeView) empty_edges = {NULL, 0};
    OK(RAPIER_FN(SoftBodyDesc_SetBendEdges)(&soft, empty_edges));
    RAPIER_TYPE(IndexView) empty_indices = {NULL, 0};
    OK(RAPIER_FN(SoftBodyDesc_SetTensionOnlyEdges)(&soft, empty_indices));
#ifdef RAPIER_DIM3
    points[3].x = 0;
    points[3].y = 4;
    points[3].z = 1;
    RAPIER_TYPE(Tetrahedron) cells[] = {{0, 1, 2, 3}};
    RAPIER_TYPE(Triangle) surface[] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    RAPIER_TYPE(SurfaceElementView) boundary = {surface, 4};
    RAPIER_TYPE(DihedralView) empty_dihedrals = {NULL, 0};
    OK(RAPIER_FN(SoftBodyDesc_SetDihedrals)(&soft, empty_dihedrals));
    OK(RAPIER_FN(SoftBodyDesc_SetWire)(&soft, empty_edges));
#else
    RAPIER_TYPE(Triangle) cells[] = {{0, 1, 2}, {1, 3, 2}};
    RAPIER_TYPE(Edge) surface[] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}};
    RAPIER_TYPE(SurfaceElementView) boundary = {surface, 4};
#endif
    RAPIER_TYPE(CellView) volume = {cells, sizeof(cells) / sizeof(cells[0])};
    OK(RAPIER_FN(SoftBodyDesc_SetCells)(&soft, volume));
    OK(RAPIER_FN(SoftBodyDesc_SetSurface)(&soft, boundary));
    OK(RAPIER_FN(SoftBodyDesc_SetSkin)(&soft, vertices, boundary));
    assert(soft.cells.count == volume.count && soft.surface.count == 4 &&
           soft.skinIndices.count == 4);
    RAPIER_TYPE(SoftBodyHandle) handle = RAPIER_FN(InsertSoftBody)(world, &soft);
    OK(RAPIER_FN(LastStatus)());

    RAPIER_TYPE(SoftBodyDesc) surface_body = RAPIER_FN(DefaultSoftBodyDesc)();
    OK(RAPIER_FN(SoftBodyDesc_SetSurfaceMesh)(&surface_body, vertices, boundary));
    assert(surface_body.kind == RAPIER_CONST(SOFT_DESC_SURFACE) && surface_body.surface.count == 4);
    RAPIER_FN(InsertSoftBody)(world, &surface_body);
    OK(RAPIER_FN(LastStatus)());
    RAPIER_TYPE(SoftBodyDesc) original;
    memcpy(&original, &soft, sizeof(original));
    RAPIER_TYPE(RealView) bad_masses = {NULL, 4};
    assert(RAPIER_FN(SoftBodyDesc_SetMasses)(&soft, bad_masses) == RAPIER_CONST(NULL_POINTER));
    assert(memcmp(&soft, &original, sizeof(soft)) == 0);
    RAPIER_TYPE(VectorView) bad_vertices = {NULL, 4};
    assert(RAPIER_FN(SoftBodyDesc_SetSurfaceMesh)(&soft, bad_vertices, boundary) ==
           RAPIER_CONST(NULL_POINTER));
    assert(memcmp(&soft, &original, sizeof(soft)) == 0);

    /* Neither descriptor copies nor views own memory. Native storage is independent. */
    points[0].y = 999;
    RAPIER_TYPE(Vector) position = RAPIER_FN(SoftBody_ParticlePosition)(handle, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(position.y == 4);
    return handle;
}

int main(void) {
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    OK(RAPIER_FN(LastStatus)());
    test_meshes(world);
    RAPIER_TYPE(SoftBodyHandle) soft = test_soft(world);
    /* All input arrays have left scope by the time physics reads its copies. */
    OK(RAPIER_FN(Step)(world, NULL, NULL));
    RAPIER_TYPE(Vector) position = RAPIER_FN(SoftBody_ParticlePosition)(soft, 0);
    OK(RAPIER_FN(LastStatus)());
    assert(isfinite(position.y));
    OK(RAPIER_FN(FreeWorld)(world));
    return 0;
}
