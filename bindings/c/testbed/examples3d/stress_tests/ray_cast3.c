/* Port of examples3d/stress_tests/ray_cast3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsRayCast3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(200.1, 0.1, 200.1));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3Real offset = -10;
    for (int j = 0; j < 10; ++j) {
        for (int i = 0; i < 10; ++i) {
            for (int k = 0; k < 10; ++k) {
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r3Vector(i * 2 - 10 + offset, j * 2 + 1, k * 2 - 10 + offset);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
        offset -= .45;
    }
    const R3Real rayBallRadius = 100;
    R3SharedShape *rayBall = r3BallSharedShape(rayBallRadius);
    R3TriMeshData *mesh = r3SharedShape_ToTrimesh(rayBall, 100, 100);
    r3FreeSharedShape(rayBall);
    size_t rayCount = r3TriMeshData_Vertices(mesh, NULL, 0);
    R3Vector *rayOrigins = malloc(rayCount * sizeof(*rayOrigins));
    R3Vector *directions = malloc(rayCount * sizeof(*directions));
    R3Vector *centeredRays = malloc(rayCount * sizeof(*centeredRays));
    if (!rayOrigins || !directions || !centeredRays) {
        abort();
    }
    rayCount = r3TriMeshData_Vertices(mesh, rayOrigins, rayCount);
    r3FreeTriMeshData(mesh);
    for (size_t i = 0; i < rayCount; ++i) {
        directions[i] = r3VectorScale(r3VectorNormalize(rayOrigins[i]), -1);
    }
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);

            size_t bodyCount = r3RigidBodyCount(world);
            R3RigidBodyHandle *handles = malloc(bodyCount * sizeof(*handles));
            if (!handles) {
                abort();
            }
            bodyCount = r3RigidBodyHandles(world, handles, bodyCount);
            R3Vector center = {0};
            for (size_t i = 0; i < bodyCount; ++i) {
                R3Vector translation = r3RigidBody_Translation(handles[i]);
                center = r3VectorAdd(center, translation);
            }
            free(handles);
            center = r3VectorScale(center, 1.0 / bodyCount);
            for (size_t i = 0; i < rayCount; ++i) {
                centeredRays[i] = r3VectorAdd(center, rayOrigins[i]);
            }
            const double t1 = tbClock();

            size_t hits = 0;
            for (size_t i = 0; i < rayCount; ++i) {
                R3RayToi hit =
                    r3CastRayToi(world, NULL, centeredRays[i], directions[i], rayBallRadius - 1, 1);
                hits += hit.found != 0;
            }
            const double mainCheckTime = tbClock() - t1;

            char label[64];
            snprintf(label, sizeof(label), "%zu", rayCount);
            tbLabel(testbed, "Ray count:", label);
            snprintf(label, sizeof(label), "%zu", hits);
            tbLabel(testbed, "Ray hits:", label);
            snprintf(label, sizeof(label), "%.2f ms", mainCheckTime * 1000);
            tbLabel(testbed, "Ray-cast time:", label);
        }
    }
    free(rayOrigins);
    free(directions);
    free(centeredRays);
    r3FreeWorld(world);
}
