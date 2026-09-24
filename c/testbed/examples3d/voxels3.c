/* Port of examples3d/voxels3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include <float.h>

static R3Bool RAPIER_CALL isVoxel(void *userData, const R3ReadContext *read,
                                  R3ColliderHandle handle) {
    (void)userData;
    R3Bool result = r3ReadCollider_IsVoxels(read, handle);
    return result;
}

void tbVoxels3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    const int fallingObjects = (int)tbSetting(
        testbed, "Falling objects: 0 Ball, 1 Cuboid, 2 Cylinder, 3 Cone, 4 Capsule, 5 Mixed", 5, 0,
        5, 1);
    const R3Real voxelSizeY = tbSetting(testbed, "Voxel size y", 1, .5, 2, 0);
    const R3Vector voxelSize = {1, voxelSizeY, 1};
    const int testCcd = (int)tbSetting(testbed, "Test CCD", 0, 0, 1, 1);
    /* The Rust example leaves its optional OBJ block disabled too. */
    const int n = 200;
    R3Vector *samples = malloc((200 * 200 + 4 * 4 * 200) * sizeof(*samples));
    if (!samples) {
        abort();
    }
    size_t sampleCount = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const R3Real y = fmax(-.8, fmin(.8, sin(i / (R3Real)n * 10))) *
                             fmax(-.8, fmin(.8, cos(j / (R3Real)n * 10))) * 16;
            samples[sampleCount++] = r3Vector(i, y * voxelSizeY, j);
            if (i == 0 || i == n - 1 || j == 0 || j == n - 1) {
                for (int k = 0; k < 4; ++k) {
                    samples[sampleCount++] = r3Vector(i, (y + k) * voxelSizeY, j);
                }
            }
        }
    }
    R3SharedShape *shape =
        r3VoxelsSharedShapeFromPoints(voxelSize, (R3VectorView){samples, sampleCount});
    free(samples);
    R3Aabb floorAabb = r3SharedShape_ComputeAabb(shape, r3TranslationPose(r3Vector(0, 0, 0)));
    R3ColliderDesc floor = r3DefaultColliderDesc();
    floor.shape.kind = R3_SHAPE_DESC_SHARED;
    floor.shape.sharedShape = shape;

    r3InsertColliderWithoutParent(world, &floor);
    r3FreeSharedShape(shape);

    const R3Vector size = r3VectorSub(floorAabb.maxs, floorAabb.mins);
    const R3Vector extents = r3VectorScale(size, .75);
    const R3Vector margin = r3VectorScale(r3VectorSub(size, extents), .5);
    const int nik = 30;
    for (int i = 0; i < nik; ++i) {
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < nik; ++k) {
                R3RigidBodyDesc rb = r3DynamicRigidBodyDesc();
                rb.position.translation = r3Vector(
                    floorAabb.mins.x + margin.x + i * extents.x / nik, floorAabb.maxs.y + j * 2,
                    floorAabb.mins.z + margin.z + k * extents.z / nik);
                rb.canSleep = !testbed->noSleep;
                if (testCcd) {
                    rb.linvel = r3Vector(0, -1000, 0);
                    rb.ccdEnabled = 1;
                }
                R3ColliderDesc co;
                switch (fallingObjects == 5 ? j % 5 : fallingObjects) {
                case 0:
                    co = r3BallColliderDesc(.5);
                    break;
                case 1:
                    co = r3CuboidColliderDesc(r3Vector(.5, .5, .5));
                    break;
                case 2:
                    co = r3CylinderColliderDesc(.5, .5);
                    break;
                case 3:
                    co = r3ConeColliderDesc(.5, .5);
                    break;
                case 4:
                    co = r3CapsuleYColliderDesc(.5, .5);
                    break;
                }
                R3RigidBodyHandle rbHandle = r3InsertRigidBody(world, &rb);
                r3InsertCollider(rbHandle, &co);
            }
        }
    }
    R3ColliderHandle hitIndicatorHandle, hitHighlightHandle;
    R3ColliderDesc indicator = r3BallColliderDesc(.1);
    indicator.collisionGroups = (R3InteractionGroups){0, 0, 0};
    hitIndicatorHandle = r3InsertColliderWithoutParent(world, &indicator);

    R3ColliderDesc highlight = r3CuboidColliderDesc(r3Vector(.51, .51, .51));
    highlight.collisionGroups = (R3InteractionGroups){0, 0, 0};
    hitHighlightHandle = r3InsertColliderWithoutParent(world, &highlight);

    tbColliderColor(testbed, hitIndicatorHandle, .5, .5, .1, 1);
    tbColliderColor(testbed, hitHighlightHandle, .1, .5, .1, 1);
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);
    testbed->snapshotSupported = 0;
    tbLabel(testbed, "Voxel editing:", "Space: add | Left Shift + Space: remove");
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            if (!testbed->rayValid) {
                continue;
            }
            R3QueryOptions query = r3DefaultQueryOptions();

            query.predicate = isVoxel;
            R3RayHit hit;
            R3Bool found;
            R3OptionalRayHit tryCastRayResult =
                r3TryCastRay(world, &query, testbed->rayOrigin, testbed->rayDirection, FLT_MAX, 1);
            hit = tryCastRayResult.hit;
            found = tryCastRayResult.found;

            if (found) {
                R3Pose hitPos = r3Collider_Position(hit.collider);
                const R3Vector hitLocalNormal =
                    r3RotationTransformVector(r3RotationInverse(hitPos.rotation), hit.normal);
                R3VoxelKey voxelKey;
                R3Vector voxelCenterLocal, size;
                R3Bool voxelFound;
                R3VoxelQuery colliderVoxelAtFlatIdResult2 =
                    r3Collider_VoxelAtFlatId(hit.collider, hit.feature_id);
                voxelKey = colliderVoxelAtFlatIdResult2.key;
                voxelCenterLocal = colliderVoxelAtFlatIdResult2.center;
                size = colliderVoxelAtFlatIdResult2.size;
                voxelFound = colliderVoxelAtFlatIdResult2.found;
                if (!voxelFound) {
                    continue;
                }
                const R3Vector voxelCenter = r3PoseTransformPoint(hitPos, voxelCenterLocal);
                r3Collider_SetTranslation(hitHighlightHandle, voxelCenter);
                R3SharedShape *shape = r3CuboidSharedShape(
                    r3VectorAdd(r3VectorScale(size, .5), r3Vector(.001, .001, .001)));
                r3Collider_SetShape(hitHighlightHandle, shape);
                r3FreeSharedShape(shape);
                const R3Vector hitPt = r3VectorAdd(
                    testbed->rayOrigin, r3VectorScale(testbed->rayDirection, hit.time_of_impact));
                r3Collider_SetTranslation(hitIndicatorHandle, hitPt);
                shape = r3BallSharedShape(r3VectorLength(size) / 3.5);
                r3Collider_SetShape(hitIndicatorHandle, shape);
                r3FreeSharedShape(shape);
                if (testbed->jump) {
                    R3VoxelKey affectedKey = voxelKey;
                    if (!testbed->removeVoxel) {
                        const R3Vector a = {fabs(hitLocalNormal.x), fabs(hitLocalNormal.y),
                                            fabs(hitLocalNormal.z)};
                        if (a.x >= a.y && a.x >= a.z) {
                            affectedKey.x += hitLocalNormal.x >= 0 ? 1 : -1;
                        } else if (a.y >= a.z) {
                            affectedKey.y += hitLocalNormal.y >= 0 ? 1 : -1;
                        } else {
                            affectedKey.z += hitLocalNormal.z >= 0 ? 1 : -1;
                        }
                    }

                    r3Collider_SetVoxel(hit.collider, affectedKey, !testbed->removeVoxel);
                }
            } else {
                const R3Vector behindCamera =
                    r3VectorSub(testbed->rayOrigin, r3VectorScale(testbed->rayDirection, 1000));
                r3Collider_SetTranslation(hitIndicatorHandle, behindCamera);
                r3Collider_SetTranslation(hitHighlightHandle, behindCamera);
            }
        }
    }
    r3FreeWorld(world);
}
