/* Port of examples2d/voxels2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbVoxels2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    const int fallingObjects = (int)tbSetting(
        testbed, "Falling objects: 0 Ball, 1 Cuboid, 2 Capsule, 3 Mixed", 3, 0, 3, 1);
    const R2Real voxelSizeY = tbSetting(testbed, "Voxel size y", 1, .5, 2, 0);
    const R2Vector voxelSize = r2Vector(1, voxelSizeY);
    const int testCcd = (int)tbSetting(testbed, "Test CCD", 0, 0, 1, 1);
    const int nx = 50;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < 10; ++j) {
            R2RigidBodyDesc rb = r2DynamicRigidBodyDesc();
            rb.position.translation = r2Vector(i * 2.0 - nx / 2.0, 20 + j * 2);
            rb.canSleep = !testbed->noSleep;
            if (testCcd) {
                rb.linvel = r2Vector(0, -1000);
                rb.ccdEnabled = 1;
            }
            const int type = fallingObjects == 3 ? j % 3 : fallingObjects;
            R2ColliderDesc co;
            switch (type) {
            case 0:
                co = r2BallColliderDesc(.5);
                break;
            case 1:
                co = r2CuboidColliderDesc(r2Vector(.5, .5));
                break;
            case 2:
                co = r2CapsuleYColliderDesc(.5, .5);
                break;
            }
            R2RigidBodyHandle rbHandle = r2InsertRigidBody(world, &rb);
            r2InsertCollider(rbHandle, &co);
        }
    }
    const R2Vector polyline[] = {{0, 0},  {0, 10}, {7, 4}, {14, 10},
                                 {14, 0}, {13, 7}, {7, 2}, {1, 7}};
    uint32_t indices[16];
    for (uint32_t i = 0; i < 8; ++i) {
        indices[2 * i] = i;
        indices[2 * i + 1] = (i + 1) % 8;
    }
    R2SharedShape *shape =
        r2VoxelizedMeshSharedShape((R2VectorView){polyline, TB_COUNT(polyline)},
                                   (R2SurfaceElementView){(const R2Edge *)indices, 8}, .2);
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-20, -10);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    r2FreeSharedShape(shape);
    R2Vector voxels[300];
    for (size_t i = 0; i < TB_COUNT(voxels); ++i) {
        const R2Real y = fmax(-.5, fmin(.5, sin(i / 20.0))) * 20;
        voxels[i] = r2Vector((i - 125.0) * voxelSize.x / 2, y * voxelSize.y);
    }
    shape = r2VoxelsSharedShapeFromPoints(voxelSize, (R2VectorView){voxels, TB_COUNT(voxels)});
    R2ColliderDesc collider = r2DefaultColliderDesc();
    collider.shape.kind = R2_SHAPE_DESC_SHARED;
    collider.shape.sharedShape = shape;
    r2InsertColliderWithoutParent(world, &collider);

    r2FreeSharedShape(shape);
    tbCamera2(testbed, 0, 20, 17);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
