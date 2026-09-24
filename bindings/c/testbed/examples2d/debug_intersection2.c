/* Port of examples2d/debug_intersection2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugIntersection2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    const R2Real rad = 1;
    R2ColliderDesc collider = r2BallColliderDesc(rad);
    const int count = 100;
    R2RigidBodyHandle handles[100 * 100];
    for (int x = 0; x < count; ++x) {
        for (int y = 0; y < count; ++y) {
            R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
            rigidBody.position.translation =
                r2Vector((x - count / 2.0) * rad * 3, (y - count / 2.0) * rad * 3);
            R2RigidBodyHandle handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);

            handles[x * count + y] = handle;
            tbBodyColor(testbed, handle, x / (float)count, (count - y) / (float)count, .5, 1);
        }
    }

    tbCamera2(testbed, 0, 0, 50);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    size_t stepId = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
            ++stepId;
            const R2Real slowTime = stepId / 3.0;

            R2SharedShape *ball = r2BallSharedShape(rad / 2);
            const R2Pose pose = r2TranslationPose(r2Vector(cos(slowTime) * 10, sin(slowTime) * 10));
            size_t intersectionCount = r2IntersectShape(world, NULL, pose, ball, NULL, 0);
            R2ColliderHandle *intersections = malloc(intersectionCount * sizeof(*intersections));
            if (intersectionCount && !intersections) {
                abort();
            }
            intersectionCount =
                r2IntersectShape(world, NULL, pose, ball, intersections, intersectionCount);
            r2FreeSharedShape(ball);

            for (size_t i = 0; i < intersectionCount; ++i) {
                for (size_t j = 0; j < TB_COUNT(handles); ++j) {
                    tbBodyColor(testbed, handles[j], .5, .5, .5, 1);
                }

                R2RigidBodyHandle bodyHandle = r2Collider_Parent(intersections[i]);
                tbBodyColor(testbed, bodyHandle, 1, 0, 0, 1);
            }
            free(intersections);
        }
    }
    r2FreeWorld(world);
}
